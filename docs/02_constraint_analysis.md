# Portable-SLAM Constraint Analysis

## Overview

This document analyzes the observability and constraint quality for each degree of freedom in the Portable-SLAM system. A state variable is **constrained** when it has enough independent measurement sources to keep its estimate bounded and accurate. A state variable is **underconstrained** (unobservable) when fewer measurements exist than unknowns, causing the estimate to drift without bound.

---

## System State Vector

The EKF in `robot_localization` tracks a **15-dimensional state vector**:

```
State: [X, Y, Z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
```

Configured in `config/imu_odom_config.yaml:59-73` via the `process_noise_covariance` diagonal:

| Index | State | Process Noise | Description |
|-------|-------|--------------|-------------|
| 0 | X | 0.12 | Position X |
| 1 | Y | 0.12 | Position Y |
| 2 | Z | 0.15 | Position Z |
| 3 | roll | 0.005 | Rotation around X |
| 4 | pitch | 0.005 | Rotation around Y |
| 5 | yaw | 0.005 | Rotation around Z |
| 6 | vx | 0.04 | Velocity X |
| 7 | vy | 0.04 | Velocity Y |
| 8 | vz | 0.05 | Velocity Z |
| 9 | vroll | 0.04 | Angular velocity X |
| 10 | vpitch | 0.04 | Angular velocity Y |
| 11 | vyaw | 0.05 | Angular velocity Z |
| 12 | ax | 0.04 | Linear acceleration X |
| 13 | ay | 0.04 | Linear acceleration Y |
| 14 | az | 0.05 | Linear acceleration Z |

---

## Sensor Constraint Map

### IMU Input (9-axis: accelerometer + gyroscope + magnetometer)

Configured in `config/imu_odom_config.yaml:15-19`:

```yaml
imu0: "imu/data"
imu0_config: [true, true, true, true, true, true, true, true, true, false, false, false, false, false, false]
#              X     Y     Z     roll   pitch  yaw    vx    vy    vz
```

The IMU provides fused orientation (roll, pitch, yaw) and position/velocity estimates derived from the Madgwick filter output. Angular velocity and linear acceleration are intentionally excluded from the EKF fusion (`false` at indices 9-14).

### rf2o Laser Odometry Input

Configured in `config/imu_odom_config.yaml:21-27`:

```yaml
odom0: "/odom_rf2o"
odom0_config: [true, true, true, false, false, false, true, true, true, false, false, false, false, false, false]
#              X     Y     Z     roll   pitch  yaw    vx    vy    vz
```

rf2o provides position (X, Y, Z) and velocity (vx, vy, vz) from 2D scan matching. Orientation from rf2o is excluded (`false` at indices 3-5).

---

## Degree-of-Freedom Constraint Analysis

### X, Y Position — WELL CONSTRAINED ✅

**Independent measurement sources**:
1. rf2o laser odometry — scan matching provides (x, y) relative to the environment
2. IMU orientation + acceleration — Madgwick filter produces position via integration

**How it works**: The rf2o node (`launch/launch_rpi4.py:188-203`) performs correlation-based scan matching between consecutive LiDAR scans. Each successful match ties the position estimate to physical features in the environment. The EKF fuses this with IMU-derived position at `imu0_config[0,1] = true` and `odom0_config[0,1] = true`.

**Residual drift**: ~1-3% of distance traveled (per `README.md:265`), primarily from scan matching error accumulation.

**Weakness**: Long featureless corridors cause rf2o degeneracy — scan matching produces poor matches when scans look similar from different positions.

---

### Yaw (Heading) — MODERATELY CONSTRAINED ⚠️

**Independent measurement sources**:
1. rf2o scan matching — relative yaw change between scans (good in feature-rich areas)
2. Magnetometer — absolute heading relative to magnetic north
3. Gyroscope Z-axis — relative yaw rate (integrates, drifts over time)

**How it works**: The Madgwick filter (`config/imu_filter_madgwick.yaml:5`) fuses gyroscope + accelerometer + magnetometer to produce an orientation quaternion. This is then fed to the EKF as `imu0_config[5] = true` (yaw).

**Critical problem — indoor magnetometer corruption**:
The magnetometer (`use_mag: true` at `config/imu_filter_madgwick.yaml:5`) provides an absolute heading reference, but indoors this reference is **corrupted** by ferromagnetic materials in building structures (rebar, electrical wiring, HVAC ducts). The magnetometer variance is set to `0.09` (`icm20948.hpp:211`), which the filter treats as relatively high confidence, causing it to trust corrupted magnetic readings.

```cpp
// include/portable_slam/icm20948.hpp:210-211
static constexpr double MAG_TOLERANCE_UT = 0.3;
static constexpr double MAG_VARIANCE = MAG_STDEV * MAG_STDEV; // 0.09
```

**Observed symptom**: <5 deg/min heading drift (`README.md:266`), but sudden heading jumps near ferromagnetic structures.

**Recommendation**: Use magnetometer with very low confidence indoors, or disable entirely and rely on scan-matching yaw + GTSAM loop closure for global heading consistency.

---

### Roll, Pitch (Tilt) — CONSTRAINED BUT IMU-ONLY ⚠️

**Independent measurement sources**:
1. Accelerometer gravity vector — provides absolute roll and pitch relative to gravity
2. Gyroscope X/Y axes — relative roll/pitch rate (integrates, drifts over time)

**How it works**: The Madgwick filter extracts roll and pitch from the accelerometer's gravity reference. The EKF fuses this at `imu0_config[3,4] = true`.

**The 2D LiDAR is completely blind to roll and pitch.** When the device tilts during walking, the LiDAR scan plane tilts, but `slam_toolbox` assumes horizontal scans. This causes map projection errors.

**Walking acceleration corruption**: The accelerometer measures:
```
accelerometer_measured = gravity + walking_acceleration
```

During walking, foot impacts and body sway add transient accelerations that corrupt the gravity reference. The Madgwick beta was reduced from 0.3 to 0.1 (`config/imu_filter_madgwick.yaml:12`) to prevent over-trusting the accelerometer:

```yaml
# CRITICAL FIX: Reduced from 0.3 to 0.1 to avoid overshooting during fast movements
# High beta (0.3) causes filter to over-trust accelerometer, leading to tilt errors
# during walking accelerations (foot impacts, body sway)
beta: 0.1
```

The orientation standard deviation was also increased (`config/imu_filter_madgwick.yaml:23`):

```yaml
# CRITICAL FIX: Increased from 0.03 to 0.1 to account for walking vibrations
orientation_stddev: 0.1
```

**Static transform inconsistency**: The RPi4 launch accounts for the LiDAR's physical tilt with a -90 degree X rotation (`launch/launch_rpi4.py:183`):

```python
arguments=["0", "0", "0.4", "-1.5708", "0", "0", "base_link", "laser_frame"]
```

But the Orange Pi 5 launch uses an identity transform (`launch/launch_opi5.py:167-171`), which may not reflect the actual sensor geometry — potentially causing scan misalignment.

**No scan deskewing using IMU tilt**: Although `config/mapper_params_online_async.yaml:79` enables `use_scan_deskewing: true`, slam_toolbox's deskewing only compensates for translational motion distortion, not tilt-induced scan plane rotation.

---

### Z (Height) — UNDERCONSTRAINED ❌

**This is the critical weakness of the system.**

**Measurement sources claiming Z observation**:

| Source | Claimed constraint | Actual quality | Code location |
|--------|--------------------|---------------|---------------|
| IMU accelerometer Z | Absolute position via double integration | **Diverges quadratically** | `sense_hat_node.cpp:130-135` → EKF |
| rf2o odometry Z | Position + velocity | **Meaningless — 2D scanner has no Z information** | `imu_odom_config.yaml:24` `odom0_config[2] = true` |

**Why Z is underconstrained**:

1. **IMU double integration drift**: Even a tiny accelerometer bias of 0.01 m/s² produces a position error of:
   ```
   error = 0.5 * bias * t²  →  0.5 * 0.01 * 60² = 18 meters in 60 seconds
   ```
   The accelerometer variance is set to `(0.05 * 9.81)² ≈ 0.24 m/s²` (`icm20948.hpp:194-195`), which the EKF treats as measurement noise — but the real problem is **bias drift**, not noise.

2. **rf2o produces no real Z**: The YDLidar Tmini Pro is a 2D spinning LiDAR. It measures distances in a single horizontal plane. The rf2o node publishes an Odometry message with Z fields, but these are effectively zero or noise — the algorithm has no mechanism to observe vertical motion. Yet the EKF configures `odom0_config[2] = true` (`imu_odom_config.yaml:24`), fusing this non-information as if it were a real Z measurement.

3. **Higher Z process noise acknowledges the problem**: The process noise for Z is 0.15, higher than X/Y's 0.12 (`imu_odom_config.yaml:61`), with the comment:
   ```yaml
   # Walking introduces more variability in position estimates due to:
   #   - Vertical bobbing (2-5 cm per step)
   ```

4. **Unused barometric altitude sensor**: The Sense HAT B includes an LPS22HB barometric pressure sensor (I2C address `0x5C`, listed in `README.md:52`) that could provide absolute altitude, but it has **no driver code** in this project.

**Consequence**: The Z estimate in the state vector drifts without bound. Over a 5-minute walk, Z can easily drift several meters from truth. For a 2D occupancy grid map this doesn't directly corrupt the map (slam_toolbox projects to 2D), but it causes problems when:
- The EKF's Z estimate feeds back into the `odom->base_link` transform, causing vertical oscillation
- The scan deskewing algorithm uses the incorrect Z velocity

---

## What Fully Constrained Would Require

For every degree of freedom to be well-constrained, each needs **at least one independent measurement source that provides an absolute (not just relative) reference**:

| DoF | Current absolute reference | Required |
|-----|---------------------------|----------|
| X, Y | LiDAR scan matching (environment-relative) | ✅ Good — add GTSAM loop closure for global consistency |
| Yaw | Magnetometer (corrupted indoors) | Scan-matching yaw + loop closure (disable magnetometer indoors or use low weight) |
| Z | None | Barometric pressure (LPS22HB already on board) or downward-facing rangefinder |
| Roll, Pitch | Gravity vector (corrupted by walking accel) | Zero-velocity updates during foot planting, or scan-based tilt correction |

---

## Specific Code Issues Affecting Constraints

### Issue 1: rf2o Z Fused as Real Data

**File**: `config/imu_odom_config.yaml:24`
```yaml
odom0_config: [true, true, true, false, false, false, true, true, true, ...]
#                          ^Z            ^vz
```

rf2o laser odometry from a 2D LiDAR cannot observe Z or vz. Fusing these as `true` injects zero-valued "measurements" into the EKF, incorrectly pulling the Z estimate toward zero.

**Fix**: Set `odom0_config[2] = false` and `odom0_config[8] = false` to stop fusing Z and vz from rf2o.

### Issue 2: Incomplete IMU Calibration Persistence

**File**: `src/icm20948.cpp:287-301`

Only gyroscope biases are saved:
```cpp
file << "gyro_bias_x: " << calibration_.gyroBias.x << "\n";
file << "gyro_bias_y: " << calibration_.gyroBias.y << "\n";
file << "gyro_bias_z: " << calibration_.gyroBias.z << "\n";
```

Accelerometer and magnetometer calibration data (bias + scale, computed in `calibrateAccel()` at lines 174-207 and `calibrateMag()` at lines 209-244) are lost on restart. `loadCalibration()` at lines 254-284 only parses `gyro_bias_x/y/z`.

**Fix**: Save and load all 15 calibration parameters (3x bias + 3x scale for accel, gyro, mag).

### Issue 3: Orientation Covariance Set to Unknown

**File**: `src/sense_hat_node.cpp:181-184`
```cpp
// Orientation covariance (set to -1 if orientation not provided)
for (size_t i = 0; i < 9; ++i) {
    message.orientation_covariance[i] = -1;
}
```

The raw IMU node sets orientation covariance to -1 (unknown), which is correct since the raw node doesn't compute orientation. The Madgwick filter fills this in, providing the EKF with orientation data with proper covariance estimates.

### Issue 4: Static Transform Inconsistency

**RPi4** (`launch/launch_rpi4.py:183`):
```python
arguments=["0", "0", "0.4", "-1.5708", "0", "0", "base_link", "laser_frame"]
#                          0.4m Z offset, -90deg X rotation
```

**Orange Pi 5** (`launch/launch_opi5.py:167-171`):
```python
arguments=["0", "0", "0", "0", "0", "0", "base_link", "laser_frame"]
#                          No offset, no rotation
```

The OPI5 transform assumes the LiDAR is co-located and co-oriented with `base_link`, which is unlikely given the physical sensor arrangement.

**Fix**: Calibrate and set the correct LiDAR-to-base_link transform for the OPI5 configuration.

---

## Summary: Constraint Quality Per Degree of Freedom

| DoF | Constraint Quality | Primary Issue | Priority |
|-----|-------------------|---------------|----------|
| X, Y | ✅ Well constrained | Featureless corridor degeneracy | Medium — addressed by loop closure |
| Yaw | ⚠️ Moderately constrained | Magnetometer corruption indoors | High — use scan matching + loop closure instead |
| Z | ❌ Underconstrained | No absolute altitude reference | **Critical** — add barometric pressure or disable Z fusion |
| Roll, Pitch | ⚠️ IMU-only constrained | Walking acceleration corrupts gravity vector | Medium — ZUPT or scan-based correction |