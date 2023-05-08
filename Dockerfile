FROM arm64v8/ros:humble-ros-base

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
    ros-humble-cartographer \
    ros-humble-ament-package

# Install Dependencies Robot Description
RUN apt-get install -y --no-install-recommends \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    ros-humble-xacro \
    xterm

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
 && git clone https://github.com/Slamtec/sllidar_ros2.git 

# Source ROS and Build
RUN cd /ws_navtech && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Run command entrypoint
ENTRYPOINT [ "./entrypoint.sh" ] 