#!/bin/bash

# Execute the command passed into this entrypoint
colcon build --symlink-install

source /opt/ros/foxy/setup.bash 
source install/setup.bash

exec "$@"