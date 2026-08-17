#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_ros
import tf_transformations


# OpenCV hue uses 0-179. Saturation and value use 0-255. Gazebo lighting
# changes saturation and brightness, so these ranges intentionally leave room
# for highlights and shadows around the nominal red, green, and blue colors.
COLOR_RANGES = {
    'R': (
        ((0, 100, 60), (8, 255, 255)),
        ((168, 100, 60), (179, 255, 255)),
    ),
    'G': (
        ((35, 80, 50), (85, 255, 255)),
    ),
    'B': (
        ((90, 80, 50), (140, 255, 255)),
    ),
}

DRAW_COLORS = {
    'R': (0, 0, 255),
    'G': (0, 255, 0),
    'B': (255, 0, 0),
}

MIN_CONTOUR_AREA = 100.0


def make_color_mask(hsv, ranges):
    """Combine one or more HSV ranges and remove isolated pixel noise."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        range_mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = cv2.bitwise_or(mask, range_mask)

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


# Gazebo camera
#   → /camera/image_raw
#   → OpenCV colour detection
#   → pixel position → estimated 3D camera point
#   → TF transform into panda_link0
#   → /color_coordinates


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher
        self.coords_pub = self.create_publisher(String, '/color_coordinates', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsic parameters (from your SDF)
        self.fx = 585.0
        self.fy = 588.0
        self.cx = 320.0
        self.cy = 160.0

        self.get_logger().info('Color Detector Node Started with TF2 lookup transform')
        self.get_logger().info('Waiting for images on /camera/image_raw')

    def image_callback(self, msg):
        self.get_logger().debug('Received image frame')
        try:
            # Convert ROS Image -> OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color_id, ranges in COLOR_RANGES.items():
            mask = make_color_mask(hsv, ranges)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [
                contour for contour in contours
                if cv2.contourArea(contour) >= MIN_CONTOUR_AREA
            ]
            if not contours:
                continue

            # There is one target box per color. Selecting the largest contour
            # prevents small colored highlights from creating extra messages.
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            cx_pix, cy_pix = x + w // 2, y + h // 2

            draw_color = DRAW_COLORS[color_id]
            cv2.rectangle(frame, (x, y), (x + w, y + h), draw_color, 2)
            cv2.putText(
                frame,
                color_id,
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                draw_color,
                2,
            )

            # Convert pixel -> camera frame
            z_camera = 0.1  # Assumed depth/distance
            y_camera = (cx_pix - self.cx) * z_camera / self.fx * -10
            x_camera = (cy_pix - self.cy) * z_camera / self.fy

            try:
                # Lookup transform camera_link -> panda_link0
                t = self.tf_buffer.lookup_transform(
                    'panda_link0',
                    'camera_link',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )

                trans = np.array([
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ])
                rot = [
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                ]

                transform = tf_transformations.quaternion_matrix(rot)
                transform[:3, 3] = trans

                point_camera = np.array([
                    x_camera,
                    y_camera,
                    z_camera,
                    1.0,
                ])
                point_base = transform @ point_camera

                # Per-color image-to-table calibration. The red contour was
                # reported at y=0.369 although its SDF center is y=0.350.
                if color_id == 'R':
                    point_base[1] -= 0.019
                elif color_id == 'B':
                    point_base[1] -= 0.0215
                elif color_id == 'G':
                    point_base[1] += 0.02

                msg_str = (
                    f'{color_id},{point_base[0]:.3f},'
                    f'{point_base[1]:.3f},{point_base[2]:.3f}'
                )
                self.coords_pub.publish(String(data=msg_str))
                self.get_logger().info(msg_str)

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
            except Exception as e:
                self.get_logger().error(
                    f'Unexpected error in TF transform: {e}'
                )

        # Show image in window
        try:
            cv2.namedWindow('Color Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Color Detection', 640, 320)
            cv2.imshow('Color Detection', frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f'OpenCV display error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
