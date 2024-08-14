

import rclpy
import numpy as np
import sys

from pyrobosim_msgs.srv import RequestWorldState

class IsAtGoalNode():
    def __init__(self):
        self.node = rclpy.create_node('is_at_goal_node')
        self._client = self.node.create_client(
            RequestWorldState, '/request_world_state')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.got_response = False
        self.call_service()

    def call_service(self):
        request = RequestWorldState.Request()
        future = self._client.call_async(request)
        future.add_done_callback(self.callback)
        
    def callback(self, future):
        self.got_response = True
        try:
            response = future.result()
            # self.node.get_logger().info(f'Response: {response}')
            assert len(response.state.robots) == 1, \
                f'Expected 1 robot, got {len(response.robots)}'
            assert response.state.robots[0].name == 'robot', \
                f'Expected robot name "robot", got "{response.robots[0].name}"'
            pos = response.state.robots[0].pose.position
            table_sink_pos = [
                loc for loc in response.state.locations
                if loc.name == 'table_sink'
            ][0].pose.position

            distance = np.linalg.norm(
                np.array([pos.x, pos.y, pos.z]) -
                np.array([table_sink_pos.x, table_sink_pos.y, table_sink_pos.z])
            )
            self.node.get_logger().info(f'Robot distance to goal: {distance}')

            banana = [
                obj for obj in response.state.objects
                if obj.name == 'banana0'
            ][0]
            is_on_table_sink = banana.parent == 'table_sink_tabletop'
            self.node.get_logger().info(f'Is banana on table_sink: {is_on_table_sink}')

            if is_on_table_sink:
                sys.exit(0)
            else:
                sys.exit(1)
        except Exception as e:
            self.node.get_logger().info(f'Service call failed: {e}')

def main():
    rclpy.init()
    node = IsAtGoalNode()

    WAITING_TIME_S = 10

    for i in range(WAITING_TIME_S * 10):
        rclpy.spin_once(node.node, timeout_sec=0.1)
        if node.got_response:
            break

    node.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
