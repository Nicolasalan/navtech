#!/bin/bash

# Execute the command passed into this entrypoint
colcon build --symlink-install

source /opt/ros/humble/setup.bash 
source install/setup.bash

exec "$@"