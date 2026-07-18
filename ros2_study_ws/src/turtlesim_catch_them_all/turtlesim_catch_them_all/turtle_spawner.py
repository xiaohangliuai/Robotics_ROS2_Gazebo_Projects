#!/usr/bin/env python3
import rclpy
import random
import math
from collections import deque
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_name_ = "turtle_V"
        self.counter_ = 0
        self.alive_turtles_ = []
        # self.queue = deque()
        self.turtle_client_ = self.create_client(Spawn, "spawn")
        self.create_timer(5.0, self.spawn_new_turtle)
        self.alive_turtle_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtle_publisher_.publish(msg)

    def spawn_new_turtle(self):
        self.counter_ += 1
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        name = self.turtle_name_ + str(self.counter_)

        self.call_spawn_service(x, y, theta, name)


    def call_spawn_service(self, x, y, theta, name):
        while not self.turtle_client_.wait_for_service(1.0):
            self.get_logger().info("waiting for spawn service...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        # self.queue.append((x, y, theta, name))

        future = self.turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_spawner_response, request=request))

    def callback_spawner_response(self, future, request: Spawn.Request):

        try:
            response: Spawn.Response = future.result()
            self.get_logger().info(f"New alive turtles: {response.name}")
            
            new_turtle = Turtle()
            new_turtle.name = request.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


    
def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()