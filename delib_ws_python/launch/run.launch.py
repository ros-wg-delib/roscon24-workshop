from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Problem number argument
    problem_number_arg = DeclareLaunchArgument("problem_number", default_value="1")
    # Problem node
    problem_node = Node(
        package="delib_ws_worlds",
        executable="run",
        name="run_world",
        parameters=[{"problem_number": LaunchConfiguration("problem_number")}],
    )
    # Solution node
    solution_node = Node(
        package="delib_ws_python",
        executable="run",
        name="run_solution",
        parameters=[{"problem_number": LaunchConfiguration("problem_number")}],
    )

    return LaunchDescription([problem_number_arg, problem_node, solution_node])
