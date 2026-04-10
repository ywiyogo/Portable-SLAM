# GTSAM SLAM Implementation Details

This document contains the detailed implementation notes for the GTSAM factor graph SLAM node (`gtsam_slam_node`). The architectural rationale and factor design are in `03_gtsam_architecture.md`.

---

## Implementation Status

| Phase | Status | Description |
|-------|--------|-------------|
| 1 | ✅ Complete | LPS22HB barometer driver + Z constraint fix |
| 1.5 | ✅ Complete | GTSAM built from source on SBC |
| 2a | ✅ Complete | IMU preintegration + rf2o odometry + barometric altitude |
| 2b | ✅ Complete | Gravity tilt + magnetometer heading factors |
| 2c | ✅ Complete | Occupancy grid map generation |
| 2d | 🔲 Deferred | Loop closure (ScanContext + ICP) — not needed for typical sessions |
| 3 | 🔲 Pending | Platform parity, cleanup, remove deprecated configs |

---

## Phase 2a: IMU Preintegration + Odometry + Altitude

### Factors Implemented

- `CombinedImuFactor` — IMU preintegration between keyframes (replaces Madgwick + EKF)
- `BetweenFactor<Pose3>` — rf2o scan-matching odometry (high Z/roll/pitch uncertainty)
- `PriorFactor<Pose3>` — barometric altitude from LPS22HB (Z constraint)

### Key Design Decisions

- **Single MutuallyExclusive callback group**: All subscribers (`/imu/data_raw`, `/odom_rf2o`, `/pressure`) share one callback group to prevent race conditions on shared state (`pim_`, `new_factors_`, `new_values_`).
- **ISAM2 via `unique_ptr`**: `isam2_` is constructed in `initGtsam()` (after `loadParameters()`), not in the member initializer list. This ensures YAML parameters are used for ISAM2 configuration rather than hardcoded defaults.
- **`PriorFactor<Pose3>` for altitude**: Uses `PriorFactor<Pose3>` with noise `(10, 10, 10, 10, 10, altitude_sigma)` instead of `PriorFactor<double>` with a disconnected `Symbol('Z', idx)` variable. The disconnected variable would create an orphan subgraph that ISAM2 cannot optimize.
- **GTSAM develop branch API differences**:
  - Header path: `<gtsam/navigation/ImuBias.h>` (not `<gtsam/imuBias/ConstantBias.h>`)
  - `PreintegrationCombinedParams::biasAccOmegaInt` (not `biasAccOmegaInit`)
  - `Rot3::Quaternion()` is a static factory (4 args) — use `Rot3::toQuaternion()` which returns `Eigen::Quaterniond`

### GTSAM State per Keyframe

Each keyframe `i` has three state variables:
- `X(i)` — `gtsam::Pose3` (position + orientation)
- `V(i)` — `gtsam::Velocity3` (3D velocity)
- `B(i)` — `gtsam::imuBias::ConstantBias` (accelerometer + gyroscope biases)

### Keyframe Selection

A new keyframe is triggered when the rf2o odometry reports motion exceeding a threshold:
- Translation > `keyframe_trans_thresh` (default 0.3 m)
- Rotation > `keyframe_rot_thresh` (default 0.26 rad ≈ 15°)

Between keyframes, IMU data is preintegrated into `pim_`. When a keyframe is added:
1. `CombinedImuFactor` connects `(X(i), V(i), B(i))` to `(X(i+1), V(i+1), B(i+1))`
2. `BetweenFactor<Pose3>` connects `X(i)` to `X(i+1)` with the rf2o relative transform
3. Optional altitude, gravity, and magnetometer priors are added
4. ISAM2 update is called, and the bias estimate is extracted for the next integration cycle

---

## Phase 2b: Gravity Tilt + Magnetometer Heading

### Factors Added

- `PriorFactor<Rot3>` — gravity tilt constraint (roll/pitch from accelerometer, yaw unconstrained)
- `PriorFactor<Rot3>` — magnetometer heading (adaptive trust based on field consistency)

### Subscribes Additionally

- `/imu/mag` (`sensor_msgs/MagneticField`) from `sense_hat_node`

### Gravity Tilt Factor

At each keyframe, the latest accelerometer reading is used to extract roll and pitch:

```
roll  = atan2(ay, az)      // rotation around X
pitch = atan2(-ax, sqrt(ay² + az²))  // rotation around Y
```

The gravity vector provides no yaw information, so the noise model uses sigma=10.0 rad for yaw (effectively unconstrained):

```cpp
auto tilt_noise = Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 10.0).finished());
//                                              roll  pitch  yaw
```

If the accelerometer norm is below 1e-6 (free-fall / shaking), the gravity factor is skipped for that keyframe.

### Magnetometer Heading Factor

The magnetometer reading is rotated from body frame to the horizontal plane using the current tilt estimate, then yaw is extracted via `atan2(East_component, North_component)` in ENU convention.

**Adaptive trust**: The magnetic field norm is compared against a reference value (~50 µT for Earth). If the deviation exceeds `mag_consistency_thresh` (default 0.3), the environment is classified as indoor and yaw_sigma is set to `mag_yaw_sigma_indoor` (5.0 rad = essentially unconstrained). Otherwise, yaw_sigma is set to `mag_yaw_sigma_outdoor` (0.1 rad = useful heading reference).

```cpp
double yaw_sigma = isMagConsistent(mx, my, mz)
                      ? mag_yaw_sigma_outdoor_   // 0.1 rad outdoors
                      : mag_yaw_sigma_indoor_;   // 5.0 rad indoors
auto mag_noise = Diagonal::Sigmas((Vector(3) << 10.0, 10.0, yaw_sigma).finished());
//                                                 roll  pitch  yaw
```

---

## Phase 2c: Occupancy Grid Map Generation

### Implementation

Replaces slam_toolbox's map generation. The GTSAM node now subscribes to `/scan` and builds a 2D occupancy grid using Bresenham ray-tracing with log-odds updates.

#### Pipeline

1. Each incoming `LaserScan` is stored in `latest_scan_`
2. On each keyframe addition, the optimized Pose3 is projected to Pose2 (x, y, yaw)
3. For each valid scan beam:
   - Compute endpoint in map frame using sensor pose + beam angle + range
   - Bresenham ray-trace from sensor origin to endpoint: clear cells along ray (`log_odds_miss = -0.2`)
   - Mark endpoint as occupied (`log_odds_hit = +0.7`)
4. Log-odds values are clamped to `[-5.0, +5.0]` to prevent saturation
5. The `/map` topic is published at `map_update_interval` (default 2.0 s) as `nav_msgs/OccupancyGrid`
6. TF `map → odom` is published as identity (will be updated by loop closure in a future phase)

### Subscribes Additionally

- `/scan` (`sensor_msgs/LaserScan`) from YDLidar

### Map Parameters (from `config/gtsam_slam_config.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_resolution` | 0.05 m | Cell size (same as slam_toolbox) |
| `map_size` | 100.0 m | Total width/height (centered at origin) → 2000×2000 cells |
| `log_odds_hit` | 0.7 | Log-odds increment for occupied cells |
| `log_odds_miss` | -0.2 | Log-odds increment for free cells along ray |
| `log_odds_clamp_max` | 5.0 (hardcoded) | Maximum log-odds value |
| `log_odds_clamp_min` | -5.0 (hardcoded) | Minimum log-odds value |
| `min_laser_range` | 0.5 m | Ignore returns closer than this |
| `max_laser_range` | 12.0 m | Ignore returns farther than this |
| `map_update_interval` | 2.0 s | How often to publish `/map` |
| `map_frame` | `"map"` | Frame ID for the occupancy grid |

### Methods

| Method | Description |
|--------|-------------|
| `scanCallback()` | Store latest laser scan |
| `initMap()` | Allocate log-odds grid (`map_width_ × map_height_`), initialize to 0.0 |
| `updateMap(pose_2d, scan)` | Ray-trace all valid beams, update log-odds grid |
| `publishMap(stamp)` | Convert log-odds to probabilities (0-100), publish as `OccupancyGrid` |
| `worldToGrid(wx, wy, gx, gy)` | Convert world coordinates to grid indices |
| `gridToWorld(gx, gy, wx, wy)` | Convert grid indices to world coordinates |
| `rayTrace(x0, y0, x1, y1)` | Bresenham line algorithm — clear cells along ray |

---

## Phase 2d: Loop Closure (Deferred)

Loop closure is not implemented. The current factor graph constrains drift well for typical handheld SLAM sessions through:

- IMU preintegration (bias-corrected interoceptive measurements)
- rf2o odometry (continuous 10 Hz BetweenFactor constraints)
- Gravity tilt (prevents roll/pitch drift)
- Barometric altitude (prevents Z drift)
- Magnetometer heading (soft constraint on yaw outdoors)

If drift becomes a problem in practice (long sessions, large loops), loop closure can be added as a future enhancement. The approach would be:

1. ScanContext descriptor computation for each keyframe scan
2. Descriptor matching against past keyframes for place recognition
3. ICP alignment for precise relative pose between matched keyframes
4. `BetweenFactor<Pose3>` added to the factor graph, triggering ISAM2 re-optimization

---

## Configuration

All parameters are in `config/gtsam_slam_config.yaml`:

```yaml
gtsam_slam_node:
  ros__parameters:
    # Keyframe selection
    keyframe_trans_thresh: 0.3       # meters
    keyframe_rot_thresh: 0.26        # radians (~15 deg)

    # IMU preintegration noise
    accel_noise_sigma: 0.4905         # m/s^2
    gyro_noise_sigma: 0.01745         # rad/s
    accel_bias_rw_sigma: 0.001        # m/s^2/sqrt(Hz)
    gyro_bias_rw_sigma: 0.0001        # rad/s/sqrt(Hz)
    integration_cov_sigma: 1.0e-8
    bias_acc_cov_sigma: 0.001
    bias_gyro_cov_sigma: 0.0001
    bias_init_sigma: 1.0e-3

    # Gravity
    gravity: [0.0, 0.0, -9.81]

    # BetweenFactor<Pose3> noise (rf2o odometry)
    # Order: [roll, pitch, yaw, x, y, z]
    odom_noise_sigmas: [0.1, 0.1, 0.01, 0.05, 0.05, 10.0]

    # Barometric altitude
    altitude_sigma: 1.0              # meters

    # Gravity tilt factor
    gravity_tilt_roll_sigma: 0.01    # radians
    gravity_tilt_pitch_sigma: 0.01   # radians
    gravity_tilt_yaw_sigma: 10.0     # radians (unconstrained)

    # Magnetometer heading factor
    mag_yaw_sigma_indoor: 5.0        # radians
    mag_yaw_sigma_outdoor: 0.1        # radians
    mag_consistency_thresh: 0.3      # relative deviation
    mag_reference_norm: 50.0          # µT

    # Occupancy grid map
    map_resolution: 0.05             # m/cell
    map_size: 100.0                   # meters (total width/height)
    log_odds_hit: 0.7
    log_odds_miss: -0.2
    min_laser_range: 0.5             # meters
    max_laser_range: 12.0            # meters
    map_update_interval: 2.0          # seconds
    scan_topic: "scan"
    map_frame: "map"

    # ISAM2 optimizer
    isam2_relinearize_thresh: 0.1
    isam2_relinearize_skip: 1.0

    # Prior factors (first keyframe)
    prior_pose_sigma: 0.01
    prior_vel_sigma: 0.1
    prior_bias_sigma: 1.0e-3

    # Frame IDs and topics
    odom_frame: "odom"
    base_frame: "base_link"
    imu_frame: "imu_link"
    imu_topic: "imu/data_raw"
    odom_topic: "odom_rf2o"
    pressure_topic: "pressure"
    mag_topic: "imu/mag"
    output_odom_topic: "odometry/filtered"
```

---

## Node Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/imu/data_raw` | `sensor_msgs/Imu` | Raw IMU from sense_hat_node |
| Subscribe | `/imu/mag` | `sensor_msgs/MagneticField` | Raw magnetometer |
| Subscribe | `/scan` | `sensor_msgs/LaserScan` | LiDAR scans (for map) |
| Subscribe | `/pressure` | `sensor_msgs/FluidPressure` | Barometric pressure |
| Subscribe | `/odom_rf2o` | `nav_msgs/Odometry` | rf2o laser odometry |
| Publish | `/odometry/filtered` | `nav_msgs/Odometry` | Optimized odometry |
| Publish | `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy grid |
| Publish | TF: `odom → base_link` | `tf2` | Real-time pose |
| Publish | TF: `map → odom` | `tf2` | Map-frame correction |

---

## Launch Files

Separate launch files for each platform run the GTSAM pipeline:

- `launch/launch_gtsam_rpi4.py` — GTSAM pipeline on Raspberry Pi 4
- `launch/launch_gtsam_opi5.py` — GTSAM pipeline on Orange Pi 5

Both launch files start:
1. `sense_hat_node` — publishes `/imu/data_raw` and `/imu/mag`
2. `lps22hb_node` — publishes `/pressure`
3. `rf2o_laser_odometry` — publishes `/odom_rf2o`
4. `ydlidar_ros2_driver_node` — publishes `/scan`
5. `gtsam_slam_node` — subscribes to all above, publishes `/odometry/filtered`, `/map`, and TF
6. Static TF publishers for `base_link → imu_link` and `base_link → laser_frame`

The original EKF + Madgwick + slam_toolbox launch files (`launch_rpi4.py`, `launch_opi5.py`) remain untouched for backward compatibility.

---

## Build Notes (GCC 13 on aarch64)

GTSAM build requires two compiler flags on ARM64 with GCC 13:

| Flag | Reason |
|------|--------|
| `-DEIGEN_DONT_VECTORIZE` | Avoids NEON `array-bounds` false positive in Eigen vector ops |
| `-Wno-maybe-uninitialized` | Suppresses GCC 13 false-positive in `Gal3.cpp` and similar |

These are set in `CMakeLists.txt` via `target_compile_options(gtsam_slam_node PRIVATE -DEIGEN_DONT_VECTORIZE)`.

GTSAM is installed to `/usr/local` from the `develop` branch. Verify with:
```bash
pkg-config --variable=prefix gtsam   # → /usr/local
ldconfig -p | grep gtsam             # → libgtsam.so
```

---

## Known Limitations

1. **`map → odom` is identity**: Without loop closure, there is no map-frame correction. The map is built assuming the odometry frame is the map frame.
2. **No scan deskewing**: Laser scans are projected using the keyframe pose without motion compensation during the scan. At walking speeds this is acceptable, but fast rotation may cause streaking.
3. **Single callback group**: All subscribers run in a single MutuallyExclusive group to avoid race conditions. This means callbacks are serialized, which may add latency at high data rates. Future optimization: add mutexes and use separate groups.
4. **2D map from 3D pose**: The occupancy grid ignores Z. Staircases will overlay on the same map layer. This is acceptable for single-floor indoor SLAM.
5. **Magnetometer threshold**: The indoor/outdoor classification uses a fixed 30% deviation from 50 µT. In practice, the threshold may need tuning for specific environments.