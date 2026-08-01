#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil

class CountUntilActionServer(Node):
    def __init__(self):
        super().__init__('count_until_action_server')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            self.execute_callback
        )
        self.get_logger().info('CountUntil Action Server has been started.')

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Implementation for handling the action execution goes here
        # get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        self.get_logger().info(f'Request received for counting until {target_number} with period {period}.')

        # Excecute the action counting logic
        self.get_logger().info('Executing count until action.')
        counter = 0
        for i in range(target_number):
            counter += 1
            self.get_logger().info(f'Current count: {counter}')
            time.sleep(period)

        # Once the counting is done, set the result and mark the goal as succeeded
        goal_handle.succeed()
        result = CountUntil.Result()
        result.reached_number = counter
        self.get_logger().info(f'Counting completed. Final count: {result.reached_number}')
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = CountUntilActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()