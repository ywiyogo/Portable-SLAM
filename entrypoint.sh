#!/bin/bash
set -e

# Source the ros2 environment setup
source /opt/ros/humble/setup.bash

# It's worth noting that exec "$@" is often used in combination with the -e option,
# which sets the exit code of the script to the exit code of the last command executed.
# This allows you to propagate errors from the command line to the script and handle them appropriately.
echo "Provided arguments: $@"
exec "$@"