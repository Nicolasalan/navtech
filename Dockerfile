#FROM arm64v8/ros:noetic-ros-base
FROM osrf/ros:noetic-desktop-full

# Set the locale
ARG DEBIAN_FRONTEND=noninteractive 

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c"]

# Setup Environment
RUN apt-get update 

RUN apt-get install -q -y --no-install-recommends \
  build-essential \
  apt-utils \
  cmake \
  g++ \
  git \
  gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-catkin-tools \
  python3-pip \
  python3-tk \
  python3-rosdep \
  apt-transport-https \
  wget \
  xterm \
  curl 

# ROS packages
RUN apt-get install -y --no-install-recommends \
  ros-noetic-py-trees-ros \
  ros-noetic-py-trees \
  ros-noetic-smach-ros \
  ros-noetic-rviz \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-pkgs 

# Install Dependencies Robot Navigation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-navigation \
    ros-noetic-slam-gmapping \
    ros-noetic-base-local-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-teb-local-planner \
    ros-noetic-ros-controllers \
    ros-noetic-robot-localization 

# Install Dependencies Robot Description
RUN apt-get install -y --no-install-recommends \
    ros-noetic-gazebo-ros \
    ros-noetic-xacro \
    ros-noetic-robot-state-controller \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher \
    ros-noetic-driver-base \
    ros-noetic-rosserial-arduino \
    ros-noetic-rplidar-ros 

# Install Dependencies with pip3
RUN pip3 install transforms3d setuptools==58.2.0 pyserial smbus trimesh scipy pandas
 
# Create Colcon workspace
RUN mkdir -p /ws_navtech
WORKDIR /ws_navtech

# Create srec directory and navtech
RUN mkdir -p src/3rd
RUN mkdir -p src/navtech

# Copy Entry Point and Makefile
COPY entrypoint.sh entrypoint.sh
COPY ./Makefile Makefile

# Copy package ros
COPY ./src/robot_nav /src/navtech/robot_nav
COPY ./src/robot_description /src/navtech/robot_description

# Clone lidar package
RUN cd /ws_navtech/src/3rd \
 && git clone https://github.com/Slamtec/rplidar_ros.git 

# Source ROS and Build
RUN cd /ws_navtech && source /opt/ros/noetic/setup.bash && catkin build

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
ENV NO_AT_BRIDGE 1

# Run command entrypoint
ENTRYPOINT [ "./entrypoint.sh" ] 
