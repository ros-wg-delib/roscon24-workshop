import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # include default bt_py launch file
    bt_py_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_bt_py"), "launch/ros_bt_py.launch.py"
            )
        ),
        launch_arguments={
            "enable_web_interface": "True",
            "node_modules": "['ros_bt_py.nodes','ros_bt_py.ros_nodes','ros_bt_py_pyrobosim.nodes']",
        }.items(),
    )

    return LaunchDescription([bt_py_include])
