#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed

# use 4 sec full - empty, 6 sec empty - full
class BatteryNode(Node):
    
    def __init__(self):
        super().__init__("battery")
        self.battery_statue_ = "full"
        self.last_time_battery_change_ = self.get_current_time_seconds()
        self.create_timer(0.1, self.check_battery_state)
        self.set_led_client_ = self.create_client(SetLed, "set_led")
        self.get_logger().info("Battery node has been started.")

    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0

    # battery state [0, 0, 0] - empty   => [1, 1, 1] -full
    def check_battery_state(self):
        time_now = self.get_current_time_seconds()
        if self.battery_statue_ == "full" and time_now - self.last_time_battery_change_  > 4.0:
            self.battery_statue_ = "empty"
            self.get_logger().info("The battery is died, plz charge it...")
            self.call_set_led(2, 1)
            self.last_time_battery_change_ = self.get_current_time_seconds()
        if self.battery_statue_ == "empty" and time_now - self.last_time_battery_change_  > 6.0:
            self.battery_statue_ = "full"
            self.get_logger().info("The battery is full")
            self.call_set_led(2, 0)
            self.last_time_battery_change_ = self.get_current_time_seconds()
        
    def call_set_led(self, led_index, state): 
        while not self.set_led_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Set led Service......")

        request = SetLed.Request()
        request.led_index = led_index
        request.state = state

        future = self.set_led_client_.call_async(request)
        future.add_done_callback(self.callback_call_set_led)

        # self.get_logger().info(str(request.led_index) + str(request.state) + str(future.result))

    def callback_call_set_led(self, future):
        response: SetLed.Response = future.result()
        if response.success:
            self.get_logger().info("LED state was changed")
        else:
            self.get_logger().info("LED not changed")


def main(args = None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()