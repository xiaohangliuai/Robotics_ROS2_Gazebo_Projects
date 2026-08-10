#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Locks the detected color coordinates before starting the motion.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        self.declare_parameter("approach_offset", 0.31)
        self.approach_offset = float(
            self.get_parameter("approach_offset").value
        )
        # Flags
        self.already_moved = False
        self.target_coords = None  # Stores the locked coordinates

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

    def coords_callback(self, msg):
        if self.already_moved:
            return  # Ignore messages once motion starts

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                # Lock coordinates immediately
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
                )
                self.already_moved = True

                # main() starts the blocking MoveIt sequence after this callback
                # returns. Calling it here would try to spin an executor from
                # inside an executor callback.

        except (ValueError, AttributeError) as error:
            self.get_logger().error(
                f"Invalid /color_coordinates message '{msg.data}': {error}"
            )

    def run_pick_and_place(self, target_coords):
        """Run the blocking MoveIt sequence outside the ROS subscription callback."""
        try:
            # Use the locked coordinates
            pick_position = [target_coords[0], target_coords[1], target_coords[2] - 0.60]
            quat_xyzw = [0.0, 1.0, 0.0, 0.0]

            # --- Pick-and-place sequence ---

            # 1. Move to home joint configuration
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            # 2. Move above target (Cartesian)
            self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
            self.moveit2.wait_until_executed()

            # 3. Open gripper
            self.gripper.open()
            self.gripper.wait_until_executed()

            # 4. Move down to approach object
            approach_position = [
                pick_position[0],
                pick_position[1],
                pick_position[2] - self.approach_offset
            ]

            self.moveit2.move_to_pose(
                position=approach_position,
                quat_xyzw=quat_xyzw,
                cartesian=True
            )
            self.moveit2.wait_until_executed()

            # 5. Close gripper
            self.gripper.close()
            self.gripper.wait_until_executed()

            # 6. Lift up back to pick_position
            # self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
            # self.moveit2.wait_until_executed()

            # 7. Move to home joint configuration
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            # 8. Move to drop joint configuration
            self.moveit2.move_to_configuration(self.drop_joints)
            self.moveit2.wait_until_executed()

            # 9. Open gripper to release
            self.gripper.open()
            self.gripper.wait_until_executed()

            # 10. Close gripper
            self.gripper.close()
            self.gripper.wait_until_executed()

            # 11. Return to start joint configuration
            self.moveit2.move_to_configuration(self.start_joints)
            self.moveit2.wait_until_executed()

            self.get_logger().info("Pick-and-place sequence complete.")

        except Exception as e:
            self.get_logger().error(f"Pick-and-place sequence failed: {e}")


def main():
    rclpy.init()
    node = PickAndPlace()

    try:
        # Wait for one matching detection. Once it is locked, run the sequence
        # in this main thread so pymoveit2 can safely use rclpy.spin_once() to
        # receive planning and action-result callbacks.
        while rclpy.ok() and node.target_coords is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.target_coords is not None:
            node.run_pick_and_place(node.target_coords)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
