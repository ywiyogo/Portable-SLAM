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
