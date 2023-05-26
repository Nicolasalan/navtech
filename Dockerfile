FROM osrf/ros:humble-desktop-full

# Set the locale
ARG DEBIAN_FRONTEND=noninteractive 

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c"]

# Setup Environment
RUN apt-get update \
 && apt-get install -yq python3-pip apt-utils git vim python3-colcon-common-extensions

# Install Dependencies Robot Navigation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cartographer-ros \
    ros-humble-ament-package \
    ros-humble-robot-localization 

# Install Dependencies Robot Description
RUN apt-get install -y --no-install-recommends \
    apt-utils \
    ros-humble-rqt-reconfigure \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rplidar-ros \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf-transformations \
    ros-humble-joint-state-publisher-gui \
    xterm

# Install Dependencies with pip3
RUN pip3 install transforms3d setuptools==58.2.0 pyserial smbus trimesh scipy pandas 

# Create Colcon workspace
RUN mkdir /ws_navtech

# Copy Entry Point and Makefile
COPY entrypoint.sh /ws_navtech/entrypoint.sh
COPY ./Makefile /ws_navtech/Makefile

# Copy package ros
COPY src/navtech/robot_description /ws_navtech/src/navtech/robot_description

# Source ROS and Build
RUN cd /ws_navtech && source /opt/ros/humble/setup.bash && colcon build --symlink-install

RUN echo "source /ws_navtech/install/setup.bash" >> ~/.bashrc

WORKDIR /ws_navtech

# Run command entrypoint
ENTRYPOINT [ "./entrypoint.sh" ] 