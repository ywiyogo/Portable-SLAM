# Portable-SLAM Initial Design: Architecture Overview

## Implementation

The portable SLAM system is configured for handheld walking operation without wheel encoders, GPS, or visual-inertial odometry (VIO). Instead, it relies on:

- **Laser Odometry**: rf2o_laser_odometry provides scan-matching odometry from LiDAR data
- **IMU-based Orientation**: 9-axis IMU with magnetometer for heading stabilization
- **Sensor Fusion**: EKF combines laser odometry + IMU for robust pose estimation

**Key Configuration:**

- EKF Configuration for `robot_localization`:
  - EKF: 20Hz processing rate
  - IMU: 20Hz to match EKF
  - rf2o Laser Odometry: 10Hz scan-matching
  - **Optimized for walking dynamics**:
    - Increased position process noise (0.12-0.15) to account for vertical bobbing and lateral sway
    - Increased velocity process noise (0.04-0.05) for variable walking speeds
    - Moderate trust in IMU orientation (stddev: 0.1) accounting for walking vibrations
  - Gravity compensation enabled

- `slam_toolbox` Configuration:
  - **Optimized for handheld walking SLAM**:
    1.  Minimum laser range: 0.5m (captures nearby obstacles during walking)
    2.  Scan processing at 10Hz (minimum_time_interval: 0.1)
    3.  Transform timeout: 0.8s (accommodates SBC processing delays)
    4.  Motion model max angle: 1.2 rad (~69°) for natural head rotations
    5.  Scan deskewing enabled for motion compensation

  - IMU Integration:
    1.  Madgwick filter with reduced beta (0.1) for stable orientation during walking
    2.  Magnetometer enabled for absolute heading reference
    3.  Quick transform updates (transform_publish_period: 0.02)
    4.  Balanced angle/distance penalties

  - QoS Settings:
    1.  Best effort reliability (matching YDLidar)
    2.  History: Keep last 10 messages
    3.  Configured through parameters file for better compatibility

Our integration strategies are:

    * LiDAR as primary source for mapping (using scan matching)
    * rf2o laser odometry for short-term position tracking
    * IMU for orientation and motion detection
    * EKF for sensor fusion combining rf2o + IMU
    * Proper message handling between components

---

## System Design

The portable SLAM system is configured for handheld walking operation without wheel encoders, GPS, or visual-inertial odometry (VIO). It relies on:

- **Laser Odometry**: rf2o_laser_odometry provides scan-matching odometry from LiDAR data
- **IMU-based Orientation**: 9-axis IMU with magnetometer for heading stabilization
- **Sensor Fusion**: EKF combines laser odometry + IMU for robust pose estimation

**Key Configuration:**

- EKF Configuration for `robot_localization`:
  - EKF: 20Hz processing rate
  - IMU: 20Hz to match EKF
  - rf2o Laser Odometry: 10Hz scan-matching
  - **Optimized for walking dynamics**:
    - Increased position process noise (0.12-0.15) to account for vertical bobbing and lateral sway
    - Increased velocity process noise (0.04-0.05) for variable walking speeds
    - Moderate trust in IMU orientation (stddev: 0.1) accounting for walking vibrations
  - Gravity compensation enabled

- `slam_toolbox` Configuration:
  - **Optimized for handheld walking SLAM**:
    1.  Minimum laser range: 0.5m (captures nearby obstacles during walking)
    2.  Scan processing at 10Hz (minimum_time_interval: 0.1)
    3.  Transform timeout: 0.8s (accommodates SBC processing delays)
    4.  Motion model max angle: 1.2 rad (~69°) for natural head rotations
    5.  Scan deskewing enabled for motion compensation

  - IMU Integration:
    1.  Madgwick filter with reduced beta (0.1) for stable orientation during walking
    2.  Magnetometer enabled for absolute heading reference
    3.  Quick transform updates (transform_publish_period: 0.02)
    4.  Balanced angle/distance penalties

  - QoS Settings:
    1.  Best effort reliability (matching YDLidar)
    2.  History: Keep last 10 messages
    3.  Configured through parameters file for better compatibility

Integration strategies:

- LiDAR as primary source for mapping (using scan matching)
- rf2o laser odometry for short-term position tracking
- IMU for orientation and motion detection
- EKF for sensor fusion combining rf2o + IMU
- Proper message handling between components

---

## Data Flow

```
Raw IMU data → Calibration → Madgwick filter (with magnetometer) → Filtered IMU with orientation → EKF (fusing IMU + rf2o odometry) → Odometry estimate → SLAM toolbox for mapping
```

Detailed data flow diagram:

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

---

## Processing Pipeline

- Lidar (10Hz) → SLAM (10Hz with optimized processing) → IMU (20Hz) → Madgwick (50Hz) → EKF (20Hz)
- Hardware Integration: Well-structured with proper I2C configuration, transform publishing, and calibration workflow

---

## Key Components

| Component | Package | Role | Config File |
|-----------|---------|------|-------------|
| IMU Driver | `portable_slam/sense_hat_node` | Reads ICM-20948, publishes raw IMU + magnetometer at 20Hz | `launch_rpi4.py:61-74` |
| Madgwick Filter | `imu_filter_madgwick` | 9-axis orientation fusion (gyro + accel + mag) | `config/imu_filter_madgwick.yaml` |
| EKF | `robot_localization/ekf_node` | 15-state EKF fusing IMU orientation + rf2o odometry | `config/imu_odom_config.yaml` |
| Laser Odometry | `rf2o_laser_odometry` | 2D scan-matching odometry from `/scan` | `launch_rpi4.py:188-203` |
| SLAM | `slam_toolbox` | 2D occupancy grid mapping + loop closure + Ceres pose graph | `config/mapper_params_online_async.yaml` |

---

## Transform Hierarchy

```
map
└── odom
    └── base_link
        ├── imu_link
        └── laser_frame
```

- **EKF** publishes `odom → base_link` transform
- **slam_toolbox** publishes `map → odom` transform
- **Static transforms**: `base_link → imu_link`, `base_link → laser_frame`

---

## Sensor Fusion Strategy

- IMU provides orientation and motion detection through Madgwick filtering with magnetometer stabilization
- rf2o provides odometry from lidar scan matching for short-term accuracy
- EKF fuses IMU and rf2o odometry for robust long-term pose estimation
- SLAM toolbox uses fused odometry for scan deskewing and pose extrapolation
- Lidar scan matching provides the primary mapping capability with enhanced stability

### EKF Sensor Inputs

**IMU** (`imu0: /imu/data`):
- Fuses: position XYZ, orientation RPY, linear velocity XYZ
- Excludes: angular velocity, linear acceleration (indices 9-14 set to `false`)

**Odometry** (`odom0: /odom_rf2o`):
- Fuses: position XYZ, linear velocity XYZ
- Excludes: orientation, angular velocity, linear acceleration

Both sensors use absolute (non-differential) mode. The EKF publishes the `odom → base_link` transform at 20Hz.

---

## Performance Characteristics

### Hardware Optimization

- Processing frequencies optimized for Raspberry Pi 4B and Orange Pi 5
- Conservative noise models based on ICM20948 datasheet specifications
- Automatic IMU calibration workflow in launch sequence
- Enhanced sensor fusion with rf2o odometry for better drift reduction
- Motion compensation ensures scan quality during movement with magnetometer heading stabilization

### Walking Motion Optimization

- Process noise tuned for human walking dynamics (0.5-1.5 m/s speed range)
- Madgwick filter beta reduced to 0.1 to prevent orientation errors during accelerations
- Motion model supports fast rotations up to 69° (1.2 rad) for natural head movements
- Minimum laser range 0.5m to capture nearby obstacles within arm's reach
- Transform timeout increased to 0.8s to handle SBC processing during intensive operations

### Expected Performance

- **Suitable for**: Slow walking (< 1.0 m/s), indoor environments, short sessions (< 10 min)
- **Position drift**: ~1-3% of distance traveled (with rf2o + IMU fusion)
- **Heading drift**: < 5° per minute (with magnetometer)
- **Limitations**: Fast walking/running, long corridors, outdoor magnetic interference

### Known Issues

See [Constraint Analysis](01_constraint_analysis.md) for detailed analysis of degree-of-freedom constraint quality and [GTSAM Architecture Migration](02_gtsam_architecture.md) for the proposed solution to these issues.

- Z-axis is underconstrained (no absolute altitude reference, rf2o provides no Z data)
- Indoor magnetometer corruption affects heading stability
- EKF and slam_toolbox are loosely coupled, causing map shifts on loop closure
- Walking acceleration corrupts gravity-based roll/pitch estimation
- IMU calibration only persists gyroscope biases (accelerometer and magnetometer calibration lost on restart)