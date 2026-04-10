# Portable-SLAM Architecture Migration: EKF + slam_toolbox → GTSAM Factor Graph

## Current Architecture

### Data Flow

```
ICM-20948 IMU ──→ sense_hat_node (/imu/data_raw, /imu/mag)
                        │
                        ▼
                 imu_filter_madgwick (/imu/data)
                        │
                        ▼
                 robot_localization EKF (/odometry/filtered → /odom)
                        │                                    ↑
                        │                              rf2o_laser_odometry
                        │                              (/odom_rf2o)
                        ▼                                    │
                 slam_toolbox (/map)  ◄──────────────────── /scan
                   │
                   ▼
            2D Occupancy Grid Map
```

### Components

| Component | Package | Role | Config File |
|-----------|---------|------|-------------|
| IMU Driver | `portable_slam/sense_hat_node` | Reads ICM-20948, publishes raw IMU + magnetometer | `launch_rpi4.py:61-74` |
| Madgwick Filter | `imu_filter_madgwick` | 9-axis orientation fusion (gyro + accel + mag) | `config/imu_filter_madgwick.yaml` |
| EKF | `robot_localization/ekf_node` | 15-state EKF fusing IMU orientation + rf2o odometry | `config/imu_odom_config.yaml` |
| Laser Odometry | `rf2o_laser_odometry` | 2D scan-matching odometry from `/scan` | `launch_rpi4.py:188-203` |
| SLAM | `slam_toolbox` | 2D occupancy grid mapping + loop closure + Ceres pose graph | `config/mapper_params_online_async.yaml` |

### Fundamental Problems

#### Problem 1: Loose Coupling — EKF and SLAM Fight Each Other

The system has two independent estimators connected only through TF transforms:

- **EKF** publishes `odom → base_link` transform
- **slam_toolbox** publishes `map → odom` transform

When slam_toolbox detects a loop closure and corrects the `map → odom` transform, the EKF's internal state is **not retroactively updated**. This creates:

1. Map jumps/shifts at loop closure events
2. Inconsistent state between EKF and SLAM
3. Temporary mapping failures during large corrections (hence `transform_timeout: 0.8` in `config/mapper_params_online_async.yaml:36`)

#### Problem 2: Forward-Only Filtering — No Backward Correction

The EKF is a **sequential forward-only filter**. Once a measurement is processed, past state estimates cannot be revised. This means:

- Drift accumulates linearly between loop closures
- After a loop closure, the EKF pose estimate is inconsistent with the corrected map
- The EKF's covariance estimate becomes overly optimistic over time

#### Problem 3: Cascaded Filters — Double Linearization

The current pipeline has two cascaded filters:

```
Raw IMU → Madgwick (filter 1) → EKF (filter 2)
```

Each filter performs its own linearization of the nonlinear dynamics. The Madgwick filter produces a quaternion orientation estimate, which is then treated as a measurement by the EKF. Errors from the first linearization propagate into the second, with no joint optimization.

#### Problem 4: No IMU Bias Estimation

The current system cannot estimate and update IMU biases online:

- Gyroscope biases are only calibrated at startup (`src/icm20948.cpp:158-172`, via `calibrateGyro()`)
- Accelerometer biases are computed during 6-position calibration but never saved (`src/icm20948.cpp:299-301` only saves gyro biases)
- Biases drift with temperature and motion, causing growing estimation errors

#### Problem 5: Z-Axis Underconstrained

As documented in `docs/01_constraint_analysis.md`, the Z dimension has no absolute reference. The EKF fuses meaningless Z data from rf2o (`imu_odom_config.yaml:24`) while the LPS22HB barometric pressure sensor on the Sense HAT B (`README.md:52`) remains unused.

---

## Proposed Architecture: GTSAM Factor Graph with ISAM2

### Overview

Replace the Madgwick + EKF + slam_toolbox pipeline with a **single GTSAM factor graph** using ISAM2 incremental optimizer. This eliminates all five fundamental problems above.

### Data Flow

```
ICM-20948 IMU ──→ sense_hat_node (/imu/data_raw)
                        │
                        ▼
                 IMU Preintegration
                 (Between keyframes in GTSAM)
                        │
                        ▼
              ┌─────────────────────────────────┐
              │      GTSAM Factor Graph          │
              │                                  │
               │  BetweenFactor<Pose3>           │  ← rf2o scan-matching odometry
               │  (x, y, z, roll, pitch, yaw)     │
              │                                  │
              │  CombinedImuFactor              │  ← raw accel + gyro between keyframes
              │  (handles bias estimation)      │
              │                                  │
              │  PriorFactor<Rot3>               │  ← gravity vector → roll, pitch
              │  (tilt constraint, high conf)    │
              │                                  │
              │  PriorFactor<Rot3>               │  ← magnetometer → yaw
              │  (heading, LOW confidence)      │
              │                                  │
               │  BetweenFactor<Pose3>           │  ← loop closure (ScanContext + ICP)
               │  (long-range pose constraint)    │
              │                                  │
              │  PriorFactor<double>             │  ← barometric altitude → Z
              │  (altitude from LPS22HB)         │
              │                                  │
              │     ISAM2 Optimizer              │
              │  (incremental, corrects ALL      │
              │   past poses on loop closure)    │
              └──────────┬───────────────────────┘
                         │
                   ┌─────┴──────┐
                   ▼             ▼
            Optimized Pose    2D Occupancy
              Graph (3D)        Grid Map
```

### GTSAM Factors — Detailed Specification

#### Factor 1: BetweenFactor<Pose3> — Scan Matching Odometry

**Replaces**: rf2o odometry → EKF position fusion

**What it does**: Each time rf2o produces a scan-matching result (relative pose between two consecutive keyframes), add a `BetweenFactor<Pose3>` constraining the relative transform. Since rf2o is a 2D LiDAR, it provides reliable (x, y, yaw) but unreliable (z, roll, pitch), so high uncertainty sigmas are assigned to those dimensions.

```cpp
// Pseudocode
NonlinearFactorGraph graph;
Pose3 odometry_delta = Pose3(
    Rot3::RzRyRx(droll, dpitch, dyaw),   // mostly yaw from 2D scan
    Point3(dx, dy, dz)                     // mostly x,y from 2D scan
);
// High confidence in x, y, yaw; low confidence in z, roll, pitch
noiseModel::Diagonal::shared_ptr odom_noise =
    noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.1, 0.1, 0.01, 0.05, 0.05, 10.0).finished());
        //              roll  pitch yaw     x     y     z
graph.add(BetweenFactor<Pose3>(key_prev, key_curr, odometry_delta, odom_noise));
```

**Advantage over current**: rf2o's covariance estimate can be used directly as the factor noise model. Currently, the EKF treats rf2o as an absolute odometry source with fixed covariance, ignoring varying scan-matching quality. In GTSAM, poor matches naturally get higher variance.

#### Factor 2: CombinedImuFactor — IMU Preintegration

**Replaces**: Madgwick filter + EKF orientation + acceleration fusion

**What it does**: Between consecutive keyframes, pre-integrate all IMU measurements (accelerometer + gyroscope) into a single relative constraint. Bias states are estimated jointly as part of the factor graph.

```cpp
// Pseudocode
PreintegratedImuMeasurements imu_params;
// Set noise from ICM-20948 datasheet specifications
imu_params.setAccNoise(0.05);   // from icm20948.hpp:194 ACCEL_VARIANCE
imu_params.setGyroNoise(0.017); // from icm20948.hpp:204 GYRO_VARIANCE sqrt
imu_params.setIntegrateCovariance(...);

// Between keyframe times t_i and t_j:
CombinedImuFactor imu_factor(
    pose_key_i, vel_key_i,       // previous state
    pose_key_j, vel_key_j,       // current state
    bias_key_i, bias_key_j,      // bias states (optimized!)
    preintegrated_measurements    // accumulated IMU data
);
graph.add(imu_factor);
```

**Key advantages**:
1. **Bias estimation**: Accelerometer and gyroscope biases are **state variables** optimized jointly, not assumed constant. This eliminates the need for separate calibration persistence (`icm20948.cpp:287-308`).
2. **Single linearization**: No cascaded filters. All IMU data is integrated once between keyframes.
3. **Proper uncertainty propagation**: The preintegration covariance is computed analytically, not approximated through two sequential filters.

#### Factor 3: PriorFactor<Rot3> — Gravity Constraint (Roll, Pitch)

**Replaces**: Madgwick filter's accelerometer-based tilt estimation

**What it does**: At each keyframe, extract roll and pitch from the accelerometer's gravity vector. Add a prior factor constraining the orientation's tilt components to match the measured gravity direction.

```cpp
// Pseudocode
// Extract roll, pitch from accelerometer (gravity vector)
Rot3 measured_tilt = gravityToOrientation(accel_measurement);

// High confidence in roll/pitch from gravity (stable reference)
noiseModel::Diagonal::shared_ptr tilt_noise =
    noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 10.0));
//                                           roll   pitch  yaw (unconstrained)

graph.add(PriorFactor<Rot3>(pose_key, measured_tilt, tilt_noise));
```

Note the noise model: low noise for roll and pitch (0.01 rad ≈ 0.6 deg), but very high noise for yaw (10.0 rad) — indicating we trust gravity for tilt but **do not constrain yaw** through this factor. This is precisely what the current system gets wrong: the Madgwick filter mixes gravity-based tilt with magnetometer-based yaw, and indoor magnetometer corruption affects both.

**Advantage**: Decouples tilt estimation from heading estimation. Roll and pitch are well-constrained by gravity; yaw is left to other factors.

#### Factor 4: BetweenFactor<Pose3> — Loop Closure

**Replaces**: slam_toolbox's built-in loop closure

**What it does**: When the robot revisits a previously mapped area, detect the loop and add a pose constraint between the current and past keyframes.

**Loop detection** (replaces slam_toolbox's correlation-based search):
- Use **ScanContext** descriptors for place recognition (computationally efficient on SBCs)
- When a candidate match is found, perform ICP scan alignment to get the precise relative pose

```cpp
// Pseudocode — when loop closure detected
Pose3 loop_closure_delta = Pose3(
    Rot3::RzRyRx(droll, dpitch, dyaw),
    Point3(dx, dy, dz)
);
// Loop closures are less certain than odometry, especially in Z/roll/pitch
noiseModel::Diagonal::shared_ptr loop_noise =
    noiseModel::Diagonal::Sigmas(
        (Vector(6) << 0.2, 0.2, 0.02, 0.15, 0.15, 5.0).finished());
        //              roll  pitch yaw     x     y     z
graph.add(BetweenFactor<Pose3>(
    current_key, matched_key, loop_closure_delta, loop_noise));

// ISAM2 re-optimizes affected portion of the graph
isam2.update(graph, initial_estimates);
```

**Advantage over slam_toolbox loop closure**:
- ISAM2 performs incremental Bayes tree optimization — only the affected portion of the graph is re-optimized
- All past poses are corrected consistently, not just the `map → odom` transform offset
- No inconsistency between front-end and back-end (they're the same graph)

#### Factor 5: PriorFactor<double> — Barometric Altitude (Z Constraint)

**Replaces**: Nothing (currently Z is underconstrained)

**What it does**: Read the LPS22HB barometric pressure sensor (already on the Sense HAT B, I2C address `0x5C`, listed in `README.md:52`) and convert to altitude. Add a prior on the Z state variable.

```cpp
// Pseudocode
double altitude = pressureToAltitude(lps22hb_reading);
noiseModel::Diagonal::shared_ptr alt_noise =
    noiseModel::Diagonal::Sigmas(Vector1(1.0)); // ±1m altitude accuracy from barometer

graph.add(PriorFactor<double>(z_key, altitude, alt_noise));
```

**Practical note**: Barometric pressure sensors provide altitude accuracy of approximately ±1m, which is much better than unconstrained drift (10+ meters over minutes). This converts Z from an **underconstrained** dimension to a **constrained** one.

**Required new code**: A driver for the LPS22HB sensor needs to be written, similar to `src/sense_hat_node.cpp` and `src/icm20948.cpp`. The I2C address is `0x5C` (from `README.md:52`). This driver would publish `sensor_msgs/FluidPressure` messages that the GTSAM node subscribes to.

#### Factor 6: PriorFactor<Rot3> — Magnetometer Heading (Low Confidence)

**Replaces**: Madgwick filter's magnetometer heading (used indoors with high confidence)

**What it does**: When magnetometer data is available and appears reliable (low local distortion), add a low-confidence prior on yaw.

```cpp
// Pseudocode
double yaw_from_mag = getMagneticHeading(mag_measurement);
Rot3 mag_orientation = Rot3::Yaw(yaw_from_mag);

// Very high noise — we trust this only as a soft suggestion, not a constraint
// Indoors: yaw_noise ≈ 5.0 rad (basically unconstrained)
// Outdoors: yaw_noise ≈ 0.1 rad (more trustworthy)
noiseModel::Diagonal::shared_ptr mag_noise =
    noiseModel::Diagonal::Sigmas(Vector3(10.0, 10.0, mag_yaw_noise));

graph.add(PriorFactor<Rot3>(pose_key, mag_orientation, mag_noise));
```

**Advantage**: Adaptive magnetometer trust. The current system enables the magnetometer globally (`use_mag: true` in `config/imu_filter_madgwick.yaml:5`), which causes heading corruption near ferromagnetic objects. In the GTSAM approach, the magnetometer noise model can be dynamically adjusted based on magnetic field consistency checks, or disabled entirely indoors.

---

## ISAM2 Incremental Optimization

### Why ISAM2 (not full batch optimization)

ISAM2 (Incremental Smoothing and Mapping 2) is an incrementalBayes tree algorithm that:

1. **Only re-optimizes affected variables** when new factors are added — not the entire graph
2. Runs in **O(log n) amortized time** per update, where n is the number of variables
3. Is suitable for **real-time operation on SBCs** (RPi4, Orange Pi 5)

For this 2D SLAM system with ~100-500 keyframes per session, ISAM2 updates typically take <10ms — well within the 100ms scan interval (10 Hz LiDAR).

### Comparison: Current vs. Proposed Back-End

| Aspect | Current (slam_toolbox Ceres) | Proposed (GTSAM ISAM2) |
|--------|------------------------------|------------------------|
| Optimization type | Batch Ceres solver on pose graph | Incremental Bayes tree |
| Optimization scope | Only `map → odom` transform adjusted | All past poses re-optimized |
| IMU integration | Cascaded Madgwick → EKF | PreintegratedCombinedImuFactor |
| Loop closure effect | Map offset adjusted, EKF unchanged | Entire graph re-optimized consistently |
| Bias handling | Static calibration, manually saved | Online estimation as state variable |
| Computational cost | Ceres full solve on loop closure | ISAM2 incremental update |

---

## Implementation Plan

### Phase 1: Add Barometric Pressure Sensor Driver (Fix Z Constraint)

**Goal**: Make Z observable before the architectural migration.

1. Write LPS22HB I2C driver (similar to `src/icm20948.cpp`)
2. Publish `sensor_msgs/FluidPressure` on a new topic
3. Add altitude conversion: `altitude = 44330 * (1 - (pressure/101325)^0.1903)`
4. In the current EKF, add a height sensor input:

```yaml
# In imu_odom_config.yaml, add:
odom1: "/altitude"
odom1_config: [false, false, true, false, false, false, false, false, false, ...]
#                         ^Z only
odom1_differential: false
```

This is a minimal change that immediately addresses the Z underconstraint issue.

### Phase 1.5: Install GTSAM on SBC

**Goal**: Build GTSAM from source on the target SBC (RPi4 / Orange Pi 5) and make it available for Phase 2.

GTSAM is not available as a ROS2 package and must be built from source. Build time is approximately **2-3 hours on RPi4** and **1-2 hours on Orange Pi 5**.

#### Prerequisites

```bash
sudo apt update && sudo apt install -y \
  cmake g++ git libeigen3-dev \
  libboost-all-dev libtbb-dev \
  python3-pip python3-dev
```

#### Clone GTSAM (outside the ROS2 workspace)

Clone into `~/gtsam`, **not** inside `~/ros2_ws/src/`. GTSAM is built with plain CMake and `make install`, not as a colcon package.

```bash
cd ~
git clone --single-branch --branch develop https://github.com/borglab/gtsam.git
cd gtsam
git submodule update --init --recursive
```

#### Configure and Build

```bash
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGTSAM_BUILD_EXAMPLES=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=ON \
  -DGTSAM_THROW_ON_EMPTY_HANDLER=ON \
  -DCMAKE_CXX_FLAGS="-DEIGEN_DONT_VECTORIZE -Wno-maybe-uninitialized"

# Use -j3 to avoid OOM on 4GB RPi4; use -j4 on 8GB RPi4 or OPI5
make -j3
sudo make install
sudo ldconfig
```

#### Key CMake Flags Explained

| Flag | Reason |
|------|--------|
| `BUILD_EXAMPLES=OFF` | Saves ~30-40% build time, not needed at runtime |
| `BUILD_TESTS=OFF` | Saves ~30-40% build time, not needed at runtime |
| `BUILD_UNSTABLE=ON` | **Required** — `CombinedImuFactor` lives in the unstable module |
| `WITH_TBB=OFF` | TBB can cause issues on ARM64; GTSAM will use OpenMP instead |
| `USE_SYSTEM_EIGEN=ON` | Uses the system Eigen (installed with ROS2 Jazzy) to avoid ABI conflicts |
| `THROW_ON_EMPTY_HANDLER=ON` | Safer runtime error handling |

#### Verify Installation

```bash
# Should print /usr/local
pkg-config --variable=prefix gtsam

# Should list libgtsam.so
ldconfig -p | grep gtsam
```

#### Known Build Issues: GCC 13 on aarch64

Building GTSAM on **aarch64 (ARM64)** with **GCC 13+** (default on Ubuntu 24.04 / ROS2 Jazzy) triggers two false-positive compiler errors due to GTSAM's `-Werror`:

**Issue 1: NEON array-bounds error**
```
error: array subscript '__Float64x2_t[0]' is partly outside array bounds
of 'gtsam::ProductLieGroup<...>::Jacobian2 [1]' {aka 'Eigen::Matrix<double, 1, 1> [1]'}
```

GCC 13's stricter `-Werror=array-bounds` catches Eigen's NEON vectorization loading a 2-element `float64x2_t` from a 1-element `Matrix<double,1,1>`. The extra element is never used — this is a false positive.

**Issue 2: maybe-uninitialized error**
```
error: 'Rt.Eigen::Matrix<double, 3, 3>::...::array[6]' may be used uninitialized
   [-Werror=maybe-uninitialized]
```

GCC 13 incorrectly flags local Eigen matrices as potentially uninitialized in `Gal3.cpp` and similar files. Again a false positive — the matrices are always written before being read.

**Fix**: Add both `-DEIGEN_DONT_VECTORIZE` and `-Wno-maybe-uninitialized` to `CMAKE_CXX_FLAGS`. The full cmake command is shown in the Configure and Build section above. Performance impact is negligible for our SLAM workload.

#### Troubleshooting

| Issue | Fix |
|-------|-----|
| CMake can't find Eigen | `sudo apt install libeigen3-dev` (comes with ROS2 Jazzy) |
| Out of memory during `make` | Use `make -j1` (much slower but safe) |
| `undefined reference to TBB` | Ensure `-DGTSAM_WITH_TBB=OFF` was set |
| CMake version too old | ROS2 Jazzy on Ubuntu 24.04 has CMake 3.28+, which is sufficient |
| GCC 13 aarch64 NEON array-bounds error | Add `-DCMAKE_CXX_FLAGS="-DEIGEN_DONT_VECTORIZE -Wno-maybe-uninitialized"` (see above) |
| GCC 13 aarch64 maybe-uninitialized error | Same fix as above — `-Wno-maybe-uninitialized` suppresses the false positive |

#### After Installation: Build System Changes for Phase 2

Once GTSAM is installed, the `CMakeLists.txt` will need these additions (done in Phase 2):

```cmake
find_package(GTSAM REQUIRED)
# In target:
ament_target_dependencies(gtsam_slam_node ... GTSAM)
```

And `package.xml` will need:

```xml
<depend>gtsam</depend>
```

### Phase 2: Replace EKF + Madgwick with GTSAM Factor Graph

**Goal**: Eliminate cascaded filters and bias handling issues by building a GTSAM-based SLAM node.

See **`docs/04_gtsam_implementation.md`** for detailed implementation notes, configuration, and status.

This phase is broken into sub-phases for incremental testing on hardware:

#### Phase 2a: Node skeleton + IMU preintegration + odometry + altitude ✅ COMPLETE

#### Phase 2b: Gravity + magnetometer orientation factors ✅ COMPLETE

#### Phase 2c: Occupancy grid map generation ✅ COMPLETE

#### Phase 2d: Loop closure (ScanContext + ICP) 🔲 DEFERRED

Not currently needed — the factor graph (IMU + rf2o + altitude + gravity + magnetometer) constrains drift well for typical handheld SLAM sessions. Can be added later if long-session drift becomes problematic.

### Phase 3: Platform Parity and Cleanup

**Goal**: Ensure both SBC platforms have identical functionality and clean up deprecated config.

1. Remove the Madgwick filter dependency from the RPi4 launch (it's now handled by GTSAM)
2. Ensure the OPI5 launch has the same GTSAM configuration
3. Fix the static transform for OPI5 (`launch/launch_opi5.py:167-171`) to match physical sensor geometry
4. Move IMU calibration to online bias estimation (no more separate calibration step)
5. Remove deprecated config files: `imu_odom_config.yaml`, `imu_filter_madgwick.yaml`

---

## Node Structure

```
portable_slam/
├── src/
│   ├── sense_hat_node.cpp         # EXISTING — publish raw IMU + mag
│   ├── icm20948.cpp               # EXISTING — ICM-20948 driver
│   ├── lps22hb_node.cpp           # Phase 1 — LPS22HB barometer ROS2 node
│   ├── lps22hb.cpp                # Phase 1 — LPS22HB I2C driver
│   ├── gtsam_slam_node.cpp         # Phase 2 — main GTSAM SLAM node
│   └── static_transform_node.cpp  # EXISTING
├── include/portable_slam/
│   ├── icm20948.hpp               # EXISTING
│   ├── lps22hb.hpp                # Phase 1
│   └── gtsam_slam.hpp             # Phase 2 — keyframe selection, factor graph state
├── config/
│   ├── gtsam_slam_config.yaml     # Phase 2 — ISAM2 parameters, noise models, keyframe thresholds, map params
│   ├── lps22hb_config.yaml        # Phase 1 — LPS22HB parameters
│   ├── imu_odom_config.yaml       # DEPRECATED after Phase 3 (kept for backward compat)
│   ├── imu_filter_madgwick.yaml   # DEPRECATED after Phase 3 (kept for backward compat)
│   └── mapper_params_online_async.yaml  # DEPRECATED after Phase 3
└── launch/
    ├── launch_rpi4.py              # EXISTING — EKF + Madgwick + slam_toolbox pipeline
    ├── launch_opi5.py              # EXISTING — EKF + Madgwick + slam_toolbox pipeline
    ├── launch_gtsam_rpi4.py        # Phase 2 — GTSAM pipeline for RPi4
    └── launch_gtsam_opi5.py        # Phase 2 — GTSAM pipeline for OPI5
```

For topic details, see **`docs/04_gtsam_implementation.md` → Node Topics**.

---

## Noise Model Parameters

See **`docs/04_gtsam_implementation.md` → Configuration** for the full parameter reference.

---

## Migration Risks and Mitigations

### Risk 1: GTSAM Computational Cost on SBC

**Mitigation**: For Pose3 + IMU factor graphs, ISAM2 updates are lightweight. Testing on similar SBCs shows ISAM2 updates for graphs with ~500 keyframes take <10ms. The main bottleneck will be scan matching (rf2o), which runs at 10Hz as before.

### Risk 2: Scan Matching Quality Degradation

**Mitigation**: Keep rf2o as the front-end scan matcher. It's already proven to work well for this system. The change is only in how its output is used — as a factor in a graph rather than an EKF input.

### Risk 3: Magnetometer Interference Detection

**Mitigation**: Implement a simple magnetic field consistency check:
```cpp
double mag_norm = sqrt(mag_x² + mag_y² + mag_z²);
double expected_norm = 50.0; // ~50 μT typical Earth field
double mag_consistency = abs(mag_norm - expected_norm) / expected_norm;
// If consistency < 0.3, trust magnetometer (outdoors)
// If consistency >= 0.3, increase yaw noise model (indoors/interference)
```

### Risk 4: Real-Time Performance

**Mitigation**: Use ISAM2's incremental mode and limit optimization frequency:
- Run ISAM2 update at keyframe rate (every 0.3m or 15° rotation)
- Use rf2o odometry for real-time TF between keyframe updates
- Limit loop closure search to last N keyframes for real-time performance

---

## What Gets Removed

After full migration:

| Component | Status | Reason |
|-----------|--------|--------|
| `robot_localization/ekf_node` | **Removed** | Replaced by GTSAM ISAM2 optimizer |
| `imu_filter_madgwick_node` | **Removed** | Replaced by GTSAM IMU preintegration |
| `slam_toolbox` | **Removed** | Replaced by GTSAM factor graph + custom map builder |
| `config/imu_odom_config.yaml` | **Removed** | No longer needed |
| `config/imu_filter_madgwick.yaml` | **Removed** | No longer needed |
| `config/mapper_params_online_async.yaml` | **Removed** | Replaced by `gtsam_slam_config.yaml` |
| `sense_hat_node` | **Kept** | Still needed for raw IMU data |
| `rf2o_laser_odometry` | **Kept** | Still needed for scan-matching odometry |

---

## Summary

The current architecture suffers from five fundamental problems: loose coupling between EKF and SLAM, forward-only filtering, cascaded filter double-linearization, no online bias estimation, and an underconstrained Z dimension. The GTSAM factor graph approach addresses all five:

1. **Single optimization** instead of two independent estimators — no TF inconsistency
2. **Full smoothing** — ISAM2 corrects all past poses on loop closure
3. **IMU preintegration** — single integration path, no cascaded filters
4. **Online bias estimation** — accelerometer and gyroscope biases are state variables
5. **Z constraint** — barometric pressure sensor provides absolute altitude reference

The migration is phased to allow incremental testing: first add the barometer to fix Z, then replace the EKF with GTSAM, then replace slam_toolbox with GTSAM-driven map generation.