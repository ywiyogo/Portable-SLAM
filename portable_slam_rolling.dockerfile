# Author: Yongkie Wiyogo
# Descrp: Deployment environment for the ROS2 Humble on Raspi4 using YDLidar Tmini
# Note: Add rm -rf /var/lib/apt/lists/* each after apt-get update and install to minimize the image size

# Pass  --build arg ARCH=osrf/ros if you want to build it on x86_64
ARG ROS_ARCH=arm64v8/ros:rolling-perception

FROM ${ROS_ARCH}

# Create a non-root user, uid 1000 is normally reserved for the default user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Create the ros user and group with the default home folder (-m), and create a config folder
RUN groupadd --gid ${USER_GID} ${USERNAME} \
  && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
  && mkdir /home/${USERNAME}/.config \
  && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && rm -rf /var/lib/apt/lists/*

# Example of installing programs
RUN apt-get update \
  && apt-get install -y \
  build-essential \
  nano \
  swig \
  && rm -rf /var/lib/apt/lists/*

# Install the YD lidar driver
RUN cd /opt \
  && git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
  && cd YDLidar-SDK && mkdir build && cd build \
  && cmake .. && make \
  && make install

# make bash default to run the setup.bash
SHELL ["/bin/bash", "-c"]
# change user to install the ros2 ydlidar driver
USER ${USER_UID}:${USER_GID}

RUN mkdir -p /home/${USERNAME}/ros2_ws/src && cd /home/${USERNAME}/ros2_ws/src \
  && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b master\
  && cd .. && . /opt/ros/rolling/setup.bash \
  && rosdep update && rosdep install -y -r --from-paths src --ignore-src \
  && colcon build --symlink-install

# Copy the entrypoint and bashrc scripts to start ROS2 without manual sourcing
COPY entrypoint.sh /entrypoint.sh
# COPY bashrc /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]