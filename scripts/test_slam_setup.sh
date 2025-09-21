#!/bin/bash

# Test script for portable SLAM setup validation
echo "=== Portable SLAM Setup Test ==="
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced. Please source ROS2 setup."
    exit 1
fi
echo "✅ ROS2 $ROS_DISTRO detected"

# Check I2C permissions
echo ""
echo "Checking I2C permissions..."
if groups $USER | grep -q i2c; then
    echo "✅ User has I2C permissions"
else
    echo "❌ User does not have I2C permissions. Run: sudo usermod -aG i2c $USER"
fi

# Check I2C bus
echo ""
echo "Checking I2C bus..."
if sudo i2cdetect -y 1 | grep -q "68"; then
    echo "✅ ICM20948 detected on I2C bus 1"
else
    echo "❌ ICM20948 not detected on I2C bus 1"
fi

# Test IMU data publication
echo ""
echo "Testing IMU data publication..."
timeout 5 ros2 topic echo /imu/data_raw --once 2>/dev/null | head -5
if [ $? -eq 0 ]; then
    echo "✅ IMU data topic working"
else
    echo "❌ IMU data topic not working"
fi

# Test magnetometer data
echo ""
echo "Testing magnetometer data..."
timeout 5 ros2 topic echo /imu/mag --once 2>/dev/null | head -5
if [ $? -eq 0 ]; then
    echo "✅ Magnetometer topic working"
else
    echo "❌ Magnetometer topic not working"
fi

# Test calibration service
echo ""
echo "Testing calibration service..."
ros2 service list | grep -q calibrate_imu
if [ $? -eq 0 ]; then
    echo "✅ Calibration service available"
else
    echo "❌ Calibration service not available"
fi

# Test Lidar
echo ""
echo "Testing Lidar..."
timeout 5 ros2 topic echo /scan --once 2>/dev/null | head -5
if [ $? -eq 0 ]; then
    echo "✅ Lidar scan topic working"
else
    echo "❌ Lidar scan topic not working"
fi

echo ""
echo "=== Test Complete ==="
echo "If all checks passed, the setup should be ready for mapping."
echo "Run: ros2 launch portable_slam launch_rpi4.py"