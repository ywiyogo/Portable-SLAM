#!/bin/bash
set -e

# Source the ros2 environment setup
source /opt/ros/humble/setup.bash

# Change the permission to run the lidar driver through the serial USB converter
sudo chmod 666 /dev/ttyUSB0

# It's worth noting that exec "$@" is often used in combination with the -e option,
# which sets the exit code of the script to the exit code of the last command executed.
# This allows you to propagate errors from the command line to the script and handle them appropriately.
echo "Provided arguments: $@"
exec "$@"
