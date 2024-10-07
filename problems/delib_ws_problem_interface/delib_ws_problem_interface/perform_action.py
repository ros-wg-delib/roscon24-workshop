## @file
# Copyright (c) 2024, Christian Henkel, Robert Bosch GmbH
# SPDX-License-Identifier: BSD-3-Clause
##

"""
Python wrapper to perform a high-level action in a PyRoboSim simulation.
"""

from enum import Enum
import rclpy
from rclpy.action import ActionClient
from typing import List, Optional, Tuple

from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import TaskAction


ACTIONS = Enum("ACTIONS", "PICK PLACE NAVIGATE OPEN CLOSE DETECT")
ACTION_NAME = "execute_action"


class PerformAction:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("perform_action")
        self._action_client = ActionClient(self.node, ExecuteTaskAction, ACTION_NAME)
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(
                f"action server {ACTION_NAME} not available, waiting again..."
            )
        self.result: Optional[bool] = None
        self.expected_state: List[Tuple[str, str]] = []
        self.done = False
        self._get_result_future = None

    def do_action(self, command: ACTIONS, target: str) -> None:
        """
        Call the service to get the current world state.

        :param command: The command to execute.
        available commands: 'pick', 'place', 'navigate', 'open', 'close', 'detect'
        :param target: The target of the command.
        """
        self.node.get_logger().info(f"Performing action {command} on {target}")
        action = TaskAction()
        action.robot = "robot"
        action.type = command.name.lower()
        if command in (
            ACTIONS.PICK,
            ACTIONS.PLACE,
            ACTIONS.OPEN,
            ACTIONS.CLOSE,
            ACTIONS.DETECT,
        ):
            action.object = target
        elif command == ACTIONS.NAVIGATE:
            action.target_location = target
        else:
            raise ValueError(f"Unknown command: {command}")

        action_goal = ExecuteTaskAction.Goal()
        action_goal.action = action
        future = self._action_client.send_goal_async(action_goal)
        future.add_done_callback(self.goal_response_callback)
        self.done = False

        while rclpy.ok() and not self.done:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(self.node, timeout_sec=0.5)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected :(")
            self.done = True
            return

        self.node.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        try:
            result = future.result().result
            status = future.result().status
            self.node.get_logger().info(
                f"Action finished with result {result} and status {status}"
            )
        except Exception as e:
            self.node.get_logger().error(f"Exception in done_callback: {e}")
        self.done = True
