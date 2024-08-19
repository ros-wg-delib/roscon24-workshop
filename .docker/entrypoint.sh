#!/bin/bash

# Source ROS and the deliberation workspace.
source /opt/ros/jazzy/setup.bash
if [ ! -f /delib_ws/install/setup.bash ]
then
  # TODO(matthias-mayr): This ignored package is still being ported.
  colcon build --symlink-install --packages-ignore skiros2_task
fi
source /delib_ws/install/setup.bash

# Execute the command passed into this entrypoint.
exec "$@"
