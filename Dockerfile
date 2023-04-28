FROM osrf/ros:humble-desktop-full

# variable version ROS
ENV ROS_DISTRO=humble

# Set the locale
ARG DEBIAN_FRONTEND=noninteractive 

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c"]

# Setup Environment
RUN apt-get update && apt-get install -y \
  cmake \
  g++ \
  git \
  gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-pip \
  python3-tk \
  wget \
  vim

# Create workspace
RUN mkdir -p /ws_navtech/src/3rd
RUN mkdir -p /ws_navtech/src/navtech

# Copy Entry Point and Makefile
COPY entrypoint.sh ./entrypoint.sh
COPY Makefile ./Makefile

# Copy package ros
COPY ./src/robot_nav ./src/navtech/robot_nav
COPY ./src/robot_driver ./src/navtech/robot_driver
COPY ./src/robot_description ./src/navtech/robot_description

# Clone lidar package
RUN cd /ws_navtech/src/3rd \
 && git clone https://github.com/Slamtec/sllidar_ros2.git

# Install Dependencies Navigation
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    ros-$ROS_DISTRO-rqt-reconfigure \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-cartographer-ros

# Install Dependencies Description
RUN apt-get install -y --no-install-recommends \
    apt-utils \
    ros-$ROS_DISTRO-rqt-reconfigure \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-tf-transformations \
    xterm

# Install Dependencies with pip3
RUN sudo pip3 install transforms3d \
 && setuptools==58.2.0

# Source ROS and Build
RUN source /opt/ros/humble/setup.bash 
#&& colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Set workdir
WORKDIR /ws_navtech
ENTRYPOINT [ "/entrypoint.sh" ]
