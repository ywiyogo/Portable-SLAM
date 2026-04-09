# Portable SLAM

A ROS2 Jazzy package for SLAM using the IMU of Waveshare Sense Hat B and YDLidar.

![portable SLAM prototype](./docs/prototype1.jpeg)

## Requirements

### Hardware

- Single Board Computer: Raspberry Pi or OrangePi 5
- [Waveshare Sense HAT B](<https://www.waveshare.com/wiki/Sense_HAT_(B)>)
- YDLidar Tmini Pro (can be switch to another YDLidar device)
- Dupont Connector

See [I2C Setup Guide](docs/01_i2c_setup.md) for instructions on enabling and verifying I2C on both Orange Pi 5 and Raspberry Pi 4.

### Software

- Install ROS2 Jazzy on your Ubuntu 24.04 system, see https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html.
- Create a new ROS2 workspace: `mkdir -p ~/ros2_ws_slam/src && cd ros2_ws_slam`.
- Clone and install the YD Lidar SDK outside the ROS2 workspace `git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/YDLidar-SDK`.
- Clone the YD Lidar driver for ROS2 from the `humble` brunch, inside the ROS2 workspace: `cd ~/ros2_ws_slam/src && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble`.
- Install robot_localization package `sudo apt install ros-${ROS_DISTRO}-robot-localization`

## Getting Started

- Clone this repository inside the src folder of the ROS workspace root folder.
- Build the package from the workspace root folder: `colcon build --symlink-install`.
- If the build is successful, source the local setup: `source ./install/setup.bash`.
- Run the test script to validate setup: `./scripts/test_slam_setup.sh`
- Run the launch script `ros2 launch portable_slam launch_opi5.py` for Orange Pi 5 or `ros2 launch portable_slam launch_rpi4.py` for Raspberry Pi 4B.

## Development Workflow

### Local Development with Remote Testing

This project supports a hybrid development workflow that allows editing code locally in your IDE while testing builds on the target Raspberry Pi hardware. This eliminates the need for constant remote access during development.

#### Prerequisites

- SSH key-based authentication set up between your local machine and Raspberry Pi
- Rsync installed on your local machine
- Raspberry Pi accessible via hostname/IP
- Docker installed on host (for visualization, optional)

#### Visualization with Docker (Optional)

To visualize SLAM mapping without installing ROS2 on your host, use Docker:

1. **Build the ROS2 Jazzy desktop container:**

   ```bash
   git clone https://github.com/ywiyogo/ubuntu-gui-container.git
   cd ubuntu-gui-container
   ./build_podman.sh -d ros2_jazzy_desktop_dev.dockerfile -n ros2-jazzy-dev
   ```

2. **Run the container with network access:**

   ```bash
   ./run_podman_for_gui.sh ros2-jazzy-dev
   ```

   If your host utilizes `ufw`, allow UDP Multicast by running:

   ```bash
   sudo ufw allow in proto udp from 192.168.8.0/24 to any
   ```

3. **Inside the container, connect to your target:**

   ```bash
   export ROS_DOMAIN_ID=8  # Same ID as your target
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   rviz2 -d /root/portable_slam.rviz
   ```

4. **On your target device, launch SLAM with the same ROS_DOMAIN_ID:**
   ```bash
   export ROS_DOMAIN_ID=8
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ros2 launch portable_slam launch_rpi4.py
   ```

#### Development Phase: Testing Uncommitted Changes

1. Edit code locally in your IDE (e.g., VSCode)
2. Set the target environment variable:
   ```bash
   export TARGET="user@host"  # e.g., "pi@raspberrypi.local" or "user@192.168.1.100"
   ```
   Or run in one line: `TARGET=pi@raspberrypi.local ./scripts/sync_and_build.sh`
3. Run `./scripts/sync_and_build.sh` to:
   - Rsync your local changes to the Pi (excluding .git to avoid conflicts)
   - Automatically build the package on the Pi with `colcon build --symlink-install`
4. Test the functionality on the Pi
5. Iterate: make more local changes, sync, test, repeat

#### Commit Phase: Version Control

When changes are stable and tested:

1. Commit locally: `git add . && git commit -m "your message"`
2. Push to remote: `git push`
3. Sync Pi to official version: SSH to Pi and run `cd ~/ros2_slam_ws/src/portable_slam && git pull`

#### Benefits

- **Rapid iteration**: Test changes immediately without committing
- **Safe commits**: Only push thoroughly tested code
- **No remote editing**: Full IDE features locally
- **Git integration**: Maintains version control workflow

## Technical Documentation

- **[Initial Design & Architecture](docs/00_initial_design.md)** — System architecture, data flow, key components, transform hierarchy, sensor fusion strategy, implementation configuration, and performance characteristics.
- **[Constraint Analysis](docs/02_constraint_analysis.md)** — Analysis of observability and constraint quality for each degree of freedom (X, Y, Z, roll, pitch, yaw). Identifies the Z-axis as underconstrained, yaw as moderately constrained (indoors), and roll/pitch as IMU-only constrained. Includes specific code issues affecting constraint quality and recommended fixes.
- **[GTSAM Architecture Migration](docs/03_gtsam_architecture.md)** — Proposed migration from the current EKF + slam_toolbox pipeline to a GTSAM factor graph with ISAM2 incremental optimizer. Documents the five fundamental problems with the current architecture, specifies six GTSAM factor types, and provides a phased implementation plan.
