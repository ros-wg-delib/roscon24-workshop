#!/bin/bash

# Source ROS and the deliberation workspace.
source /opt/ros/jazzy/setup.bash
if [ ! -f /delib_ws/install/setup.bash ]
then
  # TODO(matthias-mayr): Figure out how to more cleanly handle this.
  colcon build --packages-ignore skiros2_task
fi
source /delib_ws/install/setup.bash

# Execute the command passed into this entrypoint.
exec "$@"
