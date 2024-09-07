#!/bin/bash

# Source ROS and the deliberation workspace.
source /opt/ros/jazzy/setup.bash

# Ignores additional output because of deprecated Python tooling in ament_python
export PYTHONWARNINGS="ignore:setup.py install is deprecated,ignore:easy_install command is deprecated"

export ROS_WS=/delib_ws
export ROS_DOMAIN_ID=0

function print_ros_variables () {
        echo -e "ROS Distro: \t" $ROS_DISTRO
        echo -e "ROS Domain ID: \t" $ROS_DOMAIN_ID
        echo -e "ROS Workspace: \t" $ROS_WS
}
print_ros_variables

# Convenience functions
alias delib_src='cd $ROS_WS/src'
alias delib_ws='cd $ROS_WS'
# TODO(matthias-mayr): This ignored package is still being ported.
function delib_build() {
        cwd=$(pwd)
        delib_ws
        colcon build --symlink-install --continue-on-error --packages-ignore skiros2_task
        cd $cwd
}

function delib_build_packages() {
        cwd=$(pwd)
        delib_ws
        colcon build --symlink-install --continue-on-error --packages-ignore skiros2_task --packages-select "$@"
        cd $cwd
}

function delib_build_packages_up_to() {
        cwd=$(pwd)
        delib_ws
        colcon build --symlink-install --continue-on-error --packages-ignore skiros2_task --packages-up-to "$@"
        cd $cwd
}

function delib_clean () {
        cwd=$(pwd)
        delib_ws
        local _ret=$?
        if [ $_ret -ne 0 ] ; then
                echo "Could not switch dirs. Aborting delib_clean"
                return $_ret
        fi
        rm -r build/*
        rm -r install/*
        rm -r log/*
        cd $cwd
}

# Automatic build when entering the container
if [ ! -f /delib_ws/install/setup.bash ]
then
  delib_build
fi
source /delib_ws/install/setup.bash

# Execute the command passed into this entrypoint.
exec "$@"
