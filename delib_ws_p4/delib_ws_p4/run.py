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
import numpy as np

from pyrobosim.core import Robot, World, WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim.navigation import ConstantVelocityExecutor, PathPlanner
from pyrobosim.utils.pose import Pose
from pyrobosim_ros.ros_interface import WorldROSWrapper
from ament_index_python.packages import get_package_share_directory


data_folder = os.path.join(get_package_share_directory("delib_ws_p4"), "data")


def create_world():
    """Create a test world"""
    # world = World()
    world = WorldYamlLoader().from_yaml(
        os.path.join(data_folder, "world.yaml"))

    # Set the location and object metadata
    world.set_metadata(
        locations=os.path.join(data_folder, "location_data.yaml"),
        objects=os.path.join(data_folder, "object_data.yaml"),
    )

    return world


def create_world_from_yaml(world_file):
    return WorldYamlLoader().from_yaml(os.path.join(data_folder, world_file))


def create_ros_node():
    """Initializes ROS node"""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)
    node.declare_parameter("world_file", value="")

    # Set the world
    world_file = node.get_parameter("world_file").get_parameter_value().string_value
    if world_file == "":
        node.get_logger().info("Creating demo world programmatically.")
        world = create_world()
    else:
        node.get_logger().info(f"Using world file {world_file}.")
        world = create_world_from_yaml(world_file)

    node.set_world(world)

    return node


def main():
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)


if __name__ == '__main__':
    main()
