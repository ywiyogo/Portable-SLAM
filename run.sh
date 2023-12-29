#!/bin/bash
# Author: Yongkie Wiyogo
# Description: Docker runner for ROS2 and Gazebo simulator

workspace_folder=""
if [ -z "$1" ]; then
  container_name="portable_slam_humble"
else
  container_name=$1
fi
USER=ros
USER_UID=1000
USER_GID=$USER_UID
HOME=/home/$USER

xauth=/tmp/docker.xauth
if [ ! -f "$xauth" ]; then
  touch $xauth
  echo "$xauth is created"
fi

# To work with the GUI we need to set DISPLAY, QT_X11_NO_MITSHM, and mount /tmp/.X11-unix
# Mount the home user and the passwd to allow working as the same user
# Mount the share memory /dev/shm to allow the ROS2 communication between the container.
docker run \
  --rm \
  --net=host \
  --user $USER \
  --tmpfs /tmp \
  -e USER=$USER \
  -e HOME=$HOME \
  -e ROS_DISTRO=humble -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$xauth \
  -e "TERM=xterm-256color" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
  -v /dev/shm:/dev/shm \
  --device=/dev/ttyUSB0 \
  -h $HOSTNAME \
  -w /home/$USER \
  -it $container_name /bin/bash
