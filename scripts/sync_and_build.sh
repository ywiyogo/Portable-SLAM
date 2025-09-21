#!/bin/bash

# sync_and_build.sh - Sync local changes to Raspberry Pi and build

# Configuration - Set this environment variable before running
# export TARGET="user@host"  # e.g., "pi@raspberrypi.local" or "user@192.168.1.100"

PI_WS_PATH="~/ros2_slam_ws"  # Path to ROS2 workspace on target

# Function to display usage
usage() {
    echo "Usage: $0"
    echo "Please set the required environment variable:"
    echo "  export TARGET='user@host'  # e.g., 'pi@raspberrypi.local'"
    exit 1
}

# Check if TARGET is set
if [ -z "$TARGET" ]; then
    usage
fi

echo "Syncing local directory to ${TARGET}:${PI_WS_PATH}/src/portable_slam/ ..."

# Rsync local directory to target, excluding .git to avoid conflicts
rsync -avz --exclude='.git' --exclude='.git/' --delete . ${TARGET}:${PI_WS_PATH}/src/portable_slam/

if [ $? -eq 0 ]; then
    echo "Rsync completed successfully."
else
    echo "Rsync failed."
    exit 1
fi

echo "Building on target..."

# SSH to target and build
ssh ${TARGET} "cd ${PI_WS_PATH} && source install/setup.bash && colcon build --symlink-install"

if [ $? -eq 0 ]; then
    echo "Build completed successfully."
else
    echo "Build failed."
    exit 1
fi

echo "Sync and build process completed."