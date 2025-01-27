# Portable SLAM

A ROS2 Jazzy package for SLAM using the IMU of Waveshare Sense Hat B and YDLidar.

## Requirements

### Hardware

- OrangePi 5
- Waveshare Sense Hat B
- YDLidar Tmini Pro (can be switch to another YDLidar device)
- Dupont Connector

### Software
- Install ROS2 Jazzy on your Ubuntu 24.04 system, see https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html.
- Create a new ROS2 workspace: `mkdir -p ~/ros2_ws_slam/src && cd ros2_ws_slam`.
- Clone and install the YD Lidar SDK outside the ROS2 workspace `git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/YDLidar-SDK`.
- Clone the YD Lidar driver for ROS2 inside the ROS2 workspace: `cd ~/ros2_ws_slam/src && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git`.


## Getting Started

- Clone this repository inside the src folder.
- Build the package: `colcon build --symlink-install`.
- If the build is successful, source the local setup `source ./install/setup.bash`.
- Run the launch script `ros2 launch portable_slam launch.py`.

## Implementation

The portable SLAM system is configured like following:

* EKF Configuration for `robot_localization`:

   * EKF: 20Hz with 0.5s timeout for stable processing
   * IMU: 20Hz to match EKF
   * High trust in IMU orientation
   * Conservative velocity estimation
   * Gravity compensation enabled

* `slam_toolbox` Configuration:

   * LiDAR-primary settings:
      1. Frequent scan processing 10 Hz (minimum_time_interval: 0.1)
      2. Optimized correlation parameters for scan matching
      3. Conservative loop closure parameters

   * IMU Integration:
      1. Uses filtered IMU data for orientation
      2. Quick transform updates (transform_publish_period: 0.02)
      3. Balanced angle/distance penalties

   * QoS Settings:
      1. Best effort reliability (matching YDLidar)
      2. History: Keep last 10 messages
      3. Configured through parameters file for better compatibility

Our integration strategies are:

   * LiDAR as primary source for mapping (using scan matching)
   * IMU for orientation and motion detection
   * EKF for sensor fusion at reduced rates
   * Proper message handling between components