#!/bin/bash

# Execute the command passed into this entrypoint
echo "Building workspace"
colcon build --symlink-install

echo "Sourcing Humble"
source /opt/ros/humble/setup.bash 
source install/setup.bash

exec "$@"