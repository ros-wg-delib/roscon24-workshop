## @file
# Copyright (c) 2024, Christian Henkel, Robert Bosch GmbH
# SPDX-License-Identifier: BSD-3-Clause
##

"""
Python wrapper to get the world state from a PyRoboSim simulation.
"""

import rclpy
import logging
from typing import List, Optional, Tuple

from pyrobosim_msgs.srv import RequestWorldState


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WorldState:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("world_state")
        self.node.declare_parameter("problem_number", 1)
        self._client = self.node.create_client(
            RequestWorldState, "/request_world_state"
        )
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("service not available, waiting again...")
        self.result: Optional[bool] = None
        self.wait: bool = False
        self.expected_state: List[Tuple[str, str]] = []

    def get_problem_number(self) -> int:
        """
        Returns the problem number configured via ROS parameter.
        """
        return self.node.get_parameter("problem_number").value

    def check_state(self, expected_state) -> bool:
        """
        Call the service to get the current world state and check it against an expected state.

        :param expected_state: The expected state of the world.
        List of tuples of the form (key, value).
        All list elements must be true for the condition to be true.
        (e.g. [('objects.banana0.parent', 'table_sink_tabletop')]
        tests if the banana is on the table_sink_tabletop)

        """
        self.expected_state = expected_state
        request = RequestWorldState.Request()
        future = self._client.call_async(request)
        future.add_done_callback(self.callback)
        self.wait = True
        self.result = None

        while rclpy.ok() and self.result is None and self.wait:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.result

    def callback(self, future):
        try:
            response = future.result()
            # self.node.get_logger().info(f'Response: {response}')
        except Exception as e:
            self.wait = False
            self.node.get_logger().info(f"World state evaluation failed: {e}")

        atomic_results = []
        for key, value in self.expected_state:
            obj = response.state
            for k in key.split("."):
                # logger.info(f'k: {k}')
                if isinstance(obj, list):
                    obj_of_name = [x for x in obj if x.name == k]
                    assert len(obj_of_name) == 1, (
                        f"Expected 1 object with name {k}, got " f"{len(obj_of_name)}"
                    )
                    obj = obj_of_name[0]
                else:
                    assert hasattr(obj, k), f"{obj} has no attribute {k}"
                    obj = getattr(obj, k)
                # logger.info(f'o: {obj}')
            atomic_results.append(obj == value)

        self.node.get_logger().info("Atomic results:")
        for atomic_result, (key, value) in zip(atomic_results, self.expected_state):
            self.node.get_logger().info(f"{key} == {value}: {atomic_result}")

        self.result = all(atomic_results)
        if len(atomic_results) > 1:
            self.node.get_logger().info(f"Final result: {self.result}")


def main():
    node = WorldState()
    node.get_state([("objects.banana0.parent", "table_sink_tabletop")])

    node.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
