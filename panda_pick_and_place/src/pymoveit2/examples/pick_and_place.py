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
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda

import math
import time


# Container centers expressed in the MoveIt planning frame (panda_link0).
# The corresponding world positions are defined in pick_and_place_world.sdf.
CONTAINER_POSITIONS = {
    "R": [-0.56, 0.35],
    "G": [-0.56, 0.05],
    "B": [-0.56, -0.25],
}


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()
        if self.target_color not in CONTAINER_POSITIONS:
            raise ValueError(
                "target_color must be R, G, or B; "
                f"received '{self.target_color}'"
            )

        # The detector estimates horizontal position from the camera image, but
        # its fixed depth value is not a reliable grasp height. These heights
        # are panda_hand positions measured in panda_link0 for this table.
        self.declare_parameter("pick_hover_height", 0.50)
        self.pick_hover_height = float(
            self.get_parameter("pick_hover_height").value
        )
        self.declare_parameter("grasp_height", 0.14)
        self.grasp_height = float(
            self.get_parameter("grasp_height").value
        )
        self.declare_parameter("container_approach_height", 0.45)
        self.container_approach_height = float(
            self.get_parameter("container_approach_height").value
        )
        self.declare_parameter("container_release_height", 0.25)
        self.container_release_height = float(
            self.get_parameter("container_release_height").value
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

        # The project configures gripper_controller as a JointTrajectoryController.
        # Command it directly rather than asking MoveIt to plan a gripper motion.
        self.gripper_command_pub = self.create_publisher(
            JointTrajectory,
            "/gripper_controller/joint_trajectory",
            10,
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]

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
            # Only X/Y come from vision. The detector uses an assumed camera
            # depth, so deriving Z from it made the fingers stop at the top of
            # the box. At grasp_height=0.14 m the fingers extend from about
            # z=0.028 to z=0.082 and surround the 0.08 m-high box.
            pick_position = [
                target_coords[0],
                target_coords[1],
                self.pick_hover_height,
            ]
            grasp_position = [
                target_coords[0],
                target_coords[1],
                self.grasp_height,
            ]
            container_xy = CONTAINER_POSITIONS[self.target_color]
            container_approach_position = [
                container_xy[0],
                container_xy[1],
                self.container_approach_height,
            ]
            container_release_position = [
                container_xy[0],
                container_xy[1],
                self.container_release_height,
            ]
            quat_xyzw = [0.0, 1.0, 0.0, 0.0]

            self.get_logger().info(
                f"Picking {self.target_color} at "
                f"[{grasp_position[0]:.3f}, {grasp_position[1]:.3f}, "
                f"{grasp_position[2]:.3f}] in panda_link0"
            )
            self.get_logger().info(
                f"Placing {self.target_color} in its container at "
                f"[{container_xy[0]:.3f}, {container_xy[1]:.3f}] "
                "in panda_link0"
            )

            # --- Pick-and-place sequence ---

            # 1. Move to home joint configuration
            self.moveit2.move_to_configuration(self.home_joints)
            self.wait_for_arm_motion("move to home")

            # 2. Move above target (Cartesian)
            self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
            self.wait_for_arm_motion("move above the object")

            # 3. Open gripper
            self.command_gripper(open_gripper=True)

            # 4. Descend until the fingers surround the object
            self.moveit2.move_to_pose(
                position=grasp_position,
                quat_xyzw=quat_xyzw,
                cartesian=True
            )
            self.wait_for_arm_motion("approach the object")

            # 5. Close gripper
            self.command_gripper(open_gripper=False)

            # 6. Lift up back to pick_position
            self.moveit2.move_to_pose(
                position=pick_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
            )
            self.wait_for_arm_motion("lift the object")

            # 7. Move to home joint configuration
            self.moveit2.move_to_configuration(self.home_joints)
            self.wait_for_arm_motion("lift to home")

            # 8. Move above the matching open-top container
            self.moveit2.move_to_pose(
                position=container_approach_position,
                quat_xyzw=quat_xyzw,
            )
            self.wait_for_arm_motion("move above the container")

            # 9. Descend vertically into the container opening
            self.moveit2.move_to_pose(
                position=container_release_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
            )
            self.wait_for_arm_motion("lower the object into the container")

            # 10. Open gripper to release the box
            self.command_gripper(open_gripper=True)

            # 11. Retreat vertically so the fingers clear the container walls
            self.moveit2.move_to_pose(
                position=container_approach_position,
                quat_xyzw=quat_xyzw,
                cartesian=True,
            )
            self.wait_for_arm_motion("retreat from the container")

            # 12. Return to start joint configuration with the gripper open.
            # Closing it here can immediately clamp the released box again.
            self.moveit2.move_to_configuration(self.start_joints)
            self.wait_for_arm_motion("return to start")

            self.get_logger().info("Pick-and-place sequence complete.")

        except Exception as e:
            self.get_logger().error(f"Pick-and-place sequence failed: {e}")

    def wait_for_arm_motion(self, motion_name):
        """Wait for a planned arm motion and fail fast if MoveIt rejected it."""
        if not self.moveit2.wait_until_executed():
            raise RuntimeError(f"MoveIt could not {motion_name}.")

    def command_gripper(self, open_gripper):
        """Open or close the simulated gripper through its trajectory controller."""
        positions = (
            panda.OPEN_GRIPPER_JOINT_POSITIONS
            if open_gripper
            else panda.CLOSED_GRIPPER_JOINT_POSITIONS
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = panda.gripper_joint_names()

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=1)
        trajectory.points.append(point)

        action = "Opening" if open_gripper else "Closing"
        self.get_logger().info(f"{action} gripper through gripper_controller")
        self.gripper_command_pub.publish(trajectory)
        time.sleep(1.2)


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
