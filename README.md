# Portable SLAM

A ROS2 Jazzy package for SLAM using the IMU of Waveshare Sense Hat B and YDLidar.

## Requirements

### Hardware

- Single Board Computer: Raspberry Pi or OrangePi 5
- [Waveshare Sense HAT B](https://www.waveshare.com/wiki/Sense_HAT_(B))
- YDLidar Tmini Pro (can be switch to another YDLidar device)
- Dupont Connector


#### Enabling I2C for the Waveshare Sense HAT B

On Orange Pi 5 with ArmbianOS, the physical pins 3 (SDA) and 5 (SCL) for the 26-pin header are actually connected to I2C-5 controller in the RK3588 SoC, and are configured in mux mode 3.

* Add user overlay of /boot/ArmbianEnv.txt: `user_overlays= rk3588-i2c5-m3`
* Run `sudo armbian-add-overlay ./hw/rk3588/rk3588-i2c5-m3.dts`
* Reboot and recheck if there are some entries with `sudo i2cdetect -y 5`.
* Another alternative to list all of the i2c-bus by running `ls /dev/i2c* | while read line; do id="$(echo $line | cut -d '-' -f 2)"; echo -e "\\n## Detecting i2c ID: $id"; sudo i2cdetect -y $id; done`

See other dts overlay files in https://github.com/orangepi-xunlong/linux-orangepi/tree/orange-pi-5.10-rk3588/arch/arm64/boot/dts/rockchip/overlay

On RaspberryPi with Ubuntu 24.04 Server, the sensor is connected to I2C-1 follow these steps:
1. `sudo raspi-config`
2. Choose "3. Interface Options" -> "I5 I2C" -> select "yes".
3. Reboot and check that `sudo i2cdetect -y 1` returns device addresses on 0x29, 0x48, 0x5c, 0x68, and 0x70.

Overall `sudo i2cdetect -y 5` on Orange Pi 5 or `sudo i2cdetect -y 1` on Raspberry Pi 4 should returns:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- 5c -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: 70 -- -- -- -- -- -- --     
```
Where

* 0x29 represents the Color recognition sensor TCS34725
* 0x48 represents the AD conversion ADS1015.
* 0x68 represents the IMU 9-axis sensor ICM-20948.
* 0x5C represents the Air pressure sensor LPS22HB
* 0x70 represents the Temperature and humidity sensor SHTC3

Add user to the group permission of i2c and dialout:

```
sudo usermod -aG i2c $USER
sudo usermod -aG dialout $USER
```

### Software
- Install ROS2 Jazzy on your Ubuntu 24.04 system, see https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html.
- Create a new ROS2 workspace: `mkdir -p ~/ros2_ws_slam/src && cd ros2_ws_slam`.
- Clone and install the YD Lidar SDK outside the ROS2 workspace `git clone https://github.com/YDLIDAR/YDLidar-SDK.git ~/YDLidar-SDK`.
- Clone the YD Lidar driver for ROS2 from the `humble` brunch, inside the ROS2 workspace: `cd ~/ros2_ws_slam/src && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble`.
- Install robot_localization package `sudo apt install ros-${ROS_DISTRO}-robot-localization`


## Getting Started

- Clone this repository inside the src folder.
- Build the package: `colcon build --symlink-install`.
- If the build is successful, source the local setup `source ./install/setup.bash`.
- Run the launch script `ros2 launch portable_slam launch_opi5.py` for Orange Pi 5 or `ros2 launch portable_slam launch_opi5.py` for Raspberry Pi 4B.

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