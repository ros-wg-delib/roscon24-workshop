#!/bin/bash

# Source ROS and the deliberation workspace
source /opt/ros/jazzy/setup.bash
if [ ! -f /delib_ws/install/setup.bash ]
then
  colcon build
fi
source /delib_ws/install/setup.bash

# Execute the command passed into this entrypoint
exec "$@"
