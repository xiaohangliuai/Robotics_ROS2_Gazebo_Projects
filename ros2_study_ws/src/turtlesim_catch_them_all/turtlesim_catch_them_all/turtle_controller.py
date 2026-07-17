#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# Given a coordinate point(x, y) then control turtle move to the destination
class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.target_x_ = 9.0
        self.target_y_ = 9.0
        self.cur_pose_: Pose = None
        self.turtle_cmdVel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(
            Pose, "/turtle1/pose", self.callback_turtle_pose, 10)  #run the callback as fast as we receive the message
        self.create_timer(0.01, self.control_loop)

    # Save the current pose
    def callback_turtle_pose(self, pose: Pose):
        self.cur_pose_ = pose

    # calculate the distance and orientation and then pushlish cmd_vel topic
    def control_loop(self):    
        if self.cur_pose_ == None:
            return 
        
        dis_x = self.target_x_ - self.cur_pose_.x
        dis_y = self.target_y_ - self.cur_pose_.y
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
            x_angular.linear.x = 0
            x_angular.angular.z = 0    
        
        self.turtle_cmdVel_publisher_.publish(x_angular)


    
def main(args = None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()