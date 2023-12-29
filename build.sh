#!/bin/bash
# Author: Yongkie Wiyogo
# Description: Docker builder for Gazebo simulator

# Assign the first argument to a variable for architecture
# Default to 'amd64' if no architecture argument is provided. Pass "osrf/ros:humble-desktop-full" if you want to build it on x86_64
# E.g.: /build.sh osrf/ros:humble-desktop-full

ARCH=${1:-arm64v8/ros:humble-perception}
shift # Shift the positional parameters to the left, so $2 becomes $1, $3 becomes $2, etc.

CONTAINER_NAME=${1:-portable_slam_humble}
shift

DOCKERFILE="${CONTAINER_NAME}.dockerfile"

# Add $@ to add any additional argument such as --no-cache
docker build \
  --build-arg ROS_ARCH=$ARCH \
  -t $CONTAINER_NAME . \
  -f $DOCKERFILE \
  "$@"
