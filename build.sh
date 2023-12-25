#!/bin/bash
# Author: Yongkie Wiyogo
# Description: Docker builder for Gazebo simulator

container_name="ros2_portable_slam"
docker_file="ros2_portable_slam.dockerfile"

# Add $1 to add any additional argument such as --no-cache
docker build \
  -t $container_name . \
  -f $docker_file $1
