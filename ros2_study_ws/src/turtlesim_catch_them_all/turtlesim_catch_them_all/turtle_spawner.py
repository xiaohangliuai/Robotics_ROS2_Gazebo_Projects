#!/usr/bin/env python3
import rclpy
import random
import math
from collections import deque
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
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
        self.kill_client_ = self.create_client(Kill, "kill")
        self.create_timer(0.8, self.spawn_new_turtle)
        self.alive_turtle_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10)
        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle)

    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        # call kill service
        self.call_kill_service(request.name)
        response.success = True
        return response
        
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


    def call_kill_service(self, turtle_name):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().info("waiting for kill service...")
        
        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_service, turtle_name = turtle_name)
        )
    
    def callback_call_kill_service(self, future, turtle_name):
        # romve the turtle from alive_turtles
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.name == turtle_name:
                del self.alive_turtles_[i]
                self.publish_alive_turtles()
                break
            

    
def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()