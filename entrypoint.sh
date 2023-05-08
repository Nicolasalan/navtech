#!/bin/bash

# Execute the command passed into this entrypoint
#source /opt/ros/humble/setup.bash && source /ws_navtech/install/setup.bash && xacro /ws_navtech/install/robot_description/share/robot_description/config/robot/robot.urdf.xacro -o /ws_navtech/install/robot_description/share/robot_description/config/robot/robot.urdf

colcon build

source /opt/ros/humble/setup.bash 
source install/setup.bash 

exec "$@"