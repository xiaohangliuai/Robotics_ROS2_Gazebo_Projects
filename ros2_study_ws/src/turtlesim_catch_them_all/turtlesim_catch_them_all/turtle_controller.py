#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from functools import partial
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle

# Given a coordinate point(x, y) then control turtle move to the destination
class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.cur_pose_: Pose = None
        self.turtle_to_catch_: Turtle = None
        self.turtle_cmdVel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(
            Pose, "/turtle1/pose", self.callback_turtle_pose, 10)  #run the callback as fast as we receive the message
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.create_timer(0.01, self.control_loop)

        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")

    # Save the current pose
    def callback_turtle_pose(self, pose: Pose):
        self.cur_pose_ = pose

    def callback_alive_turtles(self, msg: TurtleArray):
        if len(msg.turtles) > 0:
            self.turtle_to_catch_ = msg.turtles[0]
            

    # calculate the distance and orientation and then pushlish cmd_vel topic
    def control_loop(self):    
        if self.cur_pose_ == None or self.turtle_to_catch_ == None:
            return 
        
        dis_x = self.turtle_to_catch_.x - self.cur_pose_.x
        dis_y = self.turtle_to_catch_.y - self.cur_pose_.y
        dis = math.sqrt(dis_x**2 + dis_y**2)

        x_angular = Twist()

        if dis > 0.5:
            # position
            x_angular.linear.x = dis
            # orientation
            goal_theta = math.atan2(dis_y, dis_x)
            diff = goal_theta - self.cur_pose_.theta
            # Need to within [-180,180] degree
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            x_angular.angular.z = diff
        else:
            # target reached 
            x_angular.linear.x = 0.0
            x_angular.angular.z = 0.0    

            self.call_catch_turtle_service(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
        
        self.turtle_cmdVel_publisher_.publish(x_angular)

    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().info("waiting for catch turtle service...")
        
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle_service, turtle_name = turtle_name)
        )
    
    def callback_catch_turtle_service(self, future, turtle_name):
        reponse: CatchTurtle.Response = future.result()
        if not reponse.success:
            self.get_logger().error("turtle " + turtle_name + " could not be removed")


    
def main(args = None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()