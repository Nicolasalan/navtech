FROM ros:humble AS base
SHELL ["/bin/bash", "-c"]

ARG UID=1000
ARG GID=1000

# install libraries and ros packages 
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# User: robot (password: robot) with sudo power
RUN useradd -ms /bin/bash robot && echo "robot:robot" | chpasswd && adduser robot sudo
RUN usermod -u $UID robot && groupmod -g $GID robot

###### USER robot ######

USER robot
RUN echo "set -g mouse on" > $HOME/.tmux.conf 
RUN touch ~/.sudo_as_admin_successful

# Set up .bashrc

RUN echo "" >> $HOME/.bashrc
RUN echo "source $HOME/ros/ws/devel/setup.bash" >> $HOME/.bashrc
RUN echo "" >> $HOME/.bashrc

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-gazebo-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rmw-cyclonedds-cpp

# Use Cyclone DDS as middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Downgrade mesa packages because of Ubuntu bug
# https://bugs.launchpad.net/ubuntu/+source/mesa/+bug/2004649
RUN apt-get update && apt-get install -q -y --allow-downgrades --no-install-recommends \
    libegl-mesa0=22.0.1-1ubuntu2 \
    libgbm1=22.0.1-1ubuntu2 \
    libgbm-dev=22.0.1-1ubuntu2 \
    libgl1-mesa-dri=22.0.1-1ubuntu2 \
    libglapi-mesa=22.0.1-1ubuntu2 \
    libglx-mesa0=22.0.1-1ubuntu2

# Create Colcon workspace with external dependencies
RUN mkdir -p /ws/src
WORKDIR /ws

RUN source /opt/ros/humble/setup.bash \
  && apt-get update -y \
  && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
RUN source /opt/ros/humble/setup.bash \
  && colcon build --symlink-install

CMD /usr/bin/tmux