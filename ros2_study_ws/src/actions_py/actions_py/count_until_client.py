#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

class CountUntilActionClient(Node):
    def __init__(self):
        super().__init__('count_until_action_client')
        self.count_until_client_ = ActionClient(self, CountUntil, 'count_until')
        
    def send_goal(self, target_number, period):
        self.count_until_client_.wait_for_server()

        # Create the goal
        goal_msg = CountUntil.Goal()
        goal_msg.target_number = target_number
        goal_msg.period = period

        # Send goal asynchronously and register the callback for the response
        self.get_logger().info(f'Sending goal: count until {target_number} with period {period}.')
        self.count_until_client_.send_goal_async(goal_msg)
        # self._send_goal_future = self.count_until_client_.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # self._send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)
    action_client = CountUntilActionClient()
    action_client.send_goal(target_number=5, period=1.0)
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()