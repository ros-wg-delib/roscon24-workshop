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


data_folder = os.path.join(get_package_share_directory("delib_ws_p2"), "data")


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

    print(f'rooms {world.get_room_names()}')

    # world.add_room(name="banana_farm")
    # world.add_room(name="dining_room")

    # # Add rooms
    # farm_coords = [(-2, 1), (-1, 1), (-1, -1), (-2, -1)]
    # world.add_room(name="banana_farm", footprint=farm_coords, color=[1, 0, 0])
    # dining_coords = [(2, 1), (1, 1), (1, -1), (2, -1)]
    # world.add_room(name="dining_room", footprint=dining_coords, color=[0, 1, 0])

    # # Add hallways between the rooms
    # world.add_hallway(
    #     room_start="banana_farm", room_end="dining_room", width=0.3)

    # Add locations
    table_source = world.add_location(
        category="table",
        parent="banana_farm",
        name="table_source",
        pose=Pose(x=-1.5, y=0.5)
    )

    table = world.add_location(
        category="table",
        parent="dining_room",
        name="table_sink",
        pose=Pose(x=1.5, y=0.5)
    )

    # Add objects
    world.add_object(
        category="banana", parent=table_source, pose=Pose(x=-1.5, y=0.5, yaw=np.pi / 4.0)
    )

    # Add a robot
    # Create path planner
    planner_config = {
        "world": world,
        "bidirectional": True,
        "rrt_connect": False,
        "rrt_star": True,
        "collision_check_step_dist": 0.025,
        "max_connection_dist": 0.5,
        "rewire_radius": 1.5,
        "compress_path": False,
    }
    path_planner = PathPlanner("rrt", **planner_config)
    robot = Robot(
        name="robot",
        radius=0.1,
        path_executor=ConstantVelocityExecutor(),
        path_planner=path_planner,
    )
    world.add_robot(robot, loc="dining_room")

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
