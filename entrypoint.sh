#!/bin/bash
# Source ROS and Catkin workspaces
source /usr/share/gazebo-11/setup.sh

# Execute the command passed into this entrypoint
colcon build

source /opt/ros/humble/setup.bash 
source install/setup.bash 

exec "$@"