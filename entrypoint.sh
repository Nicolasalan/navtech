#!/bin/bash
# Source ROS and Catkin workspaces
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo-11/setup.sh

rm -rf build/ devel/

# Execute the command passed into this entrypoint
catkin build

source devel/setup.bash

exec "$@"