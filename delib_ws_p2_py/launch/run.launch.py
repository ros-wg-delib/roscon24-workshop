from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Problem node
    problem_node = Node(package="delib_ws_p2", executable="run", name="run")
    # Solution node
    solution_node = Node(package="delib_ws_p2_py", executable="run", name="run")

    return LaunchDescription([problem_node, solution_node])
