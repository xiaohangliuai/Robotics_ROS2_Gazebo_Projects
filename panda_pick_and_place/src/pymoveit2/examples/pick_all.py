#!/usr/bin/env python3
"""Sort all boxes in red, green, blue order.

Start the Panda simulation first, then run:

    ros2 run pymoveit2 pick_all.py
"""

from pick_and_place import DEFAULT_GRASP_HEIGHTS, PickAndPlace
import rclpy


PICK_ORDER = ('R', 'G', 'B')
COLOR_NAMES = {
    'R': 'red',
    'G': 'green',
    'B': 'blue',
}


class PickAll(PickAndPlace):
    """Capture all detections before sorting the boxes sequentially."""

    def __init__(self):
        # MoveIt can spin this node during the parent constructor, so create
        # the detection store before the subscription is registered.
        self.target_coords_by_color = {}
        super().__init__()
        self.get_logger().info(
            'Waiting for red, green, and blue coordinates before sorting...'
        )

    def coords_callback(self, msg):
        """Lock the first valid coordinates received for each color."""
        try:
            color_id, x, y, z = msg.data.split(',')
            color_id = color_id.strip().upper()

            if (
                color_id in PICK_ORDER
                and color_id not in self.target_coords_by_color
            ):
                coordinates = [float(x), float(y), float(z)]
                self.target_coords_by_color[color_id] = coordinates
                self.get_logger().info(
                    f'Locked {COLOR_NAMES[color_id]} at '
                    f'[{coordinates[0]:.3f}, {coordinates[1]:.3f}, '
                    f'{coordinates[2]:.3f}]'
                )

        except (ValueError, AttributeError) as error:
            self.get_logger().error(
                f"Invalid /color_coordinates message '{msg.data}': {error}"
            )

    @property
    def all_targets_locked(self):
        """Return whether every required box position has been captured."""
        return all(
            color_id in self.target_coords_by_color
            for color_id in PICK_ORDER
        )

    def run_sequence(self):
        """Pick red, then green, then blue into matching containers."""
        for step, color_id in enumerate(PICK_ORDER, start=1):
            self.target_color = color_id
            self.grasp_height = DEFAULT_GRASP_HEIGHTS[color_id]

            self.get_logger().info(
                f'Step {step}/3: sorting the {COLOR_NAMES[color_id]} box'
            )
            if not self.run_pick_and_place(
                self.target_coords_by_color[color_id]
            ):
                self.get_logger().error(
                    f'Stopping because the {COLOR_NAMES[color_id]} step failed.'
                )
                return False

        self.get_logger().info(
            'Sorting complete: red, green, and blue are in their containers.'
        )
        return True


def main():
    rclpy.init()
    node = PickAll()

    try:
        # Store every initial position before moving the first box. Otherwise,
        # later camera frames may no longer contain boxes already transported.
        while rclpy.ok() and not node.all_targets_locked:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.all_targets_locked:
            node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
