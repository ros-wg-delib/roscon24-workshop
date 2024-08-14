

import rclpy
from rclpy.action import ActionClient

from pyrobosim_msgs.action import ExecuteTaskAction

class IsAtGoalNode():
    def __init__(self):
        self.node = rclpy.create_node('is_at_goal_node')
        self._action_client = ActionClient(self.node, ExecuteTaskAction, '/execute_action')
        self._action_client.wait_for_server()

    def call_action(self):
        goal_msg = ExecuteTaskAction.Goal()
        goal_msg.action.robot = 'robot'
        goal_msg.action.target_location = 'table_source'
        goal_msg.action.type = 'navigate'
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg}')

def main():
    rclpy.init()
    node = IsAtGoalNode()
    node.call_action()

    node.node.spin()

    node.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
