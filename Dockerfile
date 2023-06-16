FROM osrf/ros:foxy-desktop

# Set the locale
ARG DEBIAN_FRONTEND=noninteractive 

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c"]

# Setup Environment
RUN apt-get update \
 && apt-get install -yq python3-pip apt-utils git vim python3-colcon-common-extensions

# Install Dependencies "Robot Navigation"
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-cartographer-ros \
    ros-foxy-ament-package \
    ros-foxy-robot-localization \
    ros-foxy-slam-toolbox 

# Install Dependencies "Robot Description"
RUN apt-get install -y --no-install-recommends \
    apt-utils \
    ros-foxy-rqt-reconfigure \
    ros-foxy-rviz2 \
    ros-foxy-gazebo-ros \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-xacro \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher \
    ros-foxy-rplidar-ros \
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-tf-transformations \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-gazebo-plugins \
    xterm
    
# Install Dependencies with pip
RUN pip3 install transforms3d setuptools==58.2.0 pyserial smbus trimesh scipy pandas pytest

# Create Colcon workspace
RUN mkdir /ws_navtech

# Copy Entry Point and Makefile
COPY entrypoint.sh /ws_navtech/entrypoint.sh
COPY ./Makefile /ws_navtech/Makefile

# Copy package ROS
COPY src/navtech/robot /ws_navtech/src/navtech/robot

# Source ROS and Build
RUN cd /ws_navtech && source /opt/ros/foxy/setup.bash && colcon build --symlink-install

# Source ROS and Build
RUN echo "source /ws_navtech/install/setup.bash" >> ~/.bashrc

# Set Workdir
WORKDIR /ws_navtech

# Run command entrypoint
ENTRYPOINT [ "./entrypoint.sh" ] 