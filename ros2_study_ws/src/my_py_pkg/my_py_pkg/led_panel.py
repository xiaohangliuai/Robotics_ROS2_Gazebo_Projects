#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed

class LEDPanelNode(Node):
    
    def __init__(self):
        super().__init__("led_panel")
        self.led_states_ = [0, 0, 0]
        self.led_state_pub_ = self.create_publisher(LedStateArray, "led_panel_state", 10)
        self.create_timer(5.0, self.publish_led_state)
        self.create_service(SetLed, "set_led", self.callback_set_led)
        self.get_logger().info("LED panel node has been started")
    
    def publish_led_state(self):
        msg = LedStateArray()
        msg.led_states = self.led_states_
        self.led_state_pub_.publish(msg)
    
    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        led_index = request.led_index
        led_state = request.state

        # Validate the input values
        if led_index < 0 or led_index >= len(self.led_states_):
            response.success = False
            return response
        
        if led_state not in [0, 1]:
            response.success = False
            return response
        
        self.led_states_[led_index] = led_state
        self.publish_led_state()
        response.success = True
        return response
    
def main(args = None):
    rclpy.init(args=args)
    node = LEDPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()