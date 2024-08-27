#!/usr/bin/env python3

"""
__Goal__:
Banana on table_sink.

__Initial State__:
Banana on table_source.

__Available Actions__:

- Pick object
- Place object
- Move robot

__Available Conditions__:

- Robot location
    dining_room, banana_farm
- Object location
    table_sink, table_source
- Object in hand
    true, false

"""
import os
import rclpy
import threading

from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim_ros.ros_interface import WorldROSWrapper
from ament_index_python.packages import get_package_share_directory


def create_world_from_yaml():
    world_file = os.path.join(
        get_package_share_directory("delib_ws_p3"), "data", "world.yaml"
    )
    return WorldYamlLoader().from_yaml(world_file)


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)

    # Set the world
    world = create_world_from_yaml()
    node.set_world(world)

    return node


def main():
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)


if __name__ == "__main__":
    main()
