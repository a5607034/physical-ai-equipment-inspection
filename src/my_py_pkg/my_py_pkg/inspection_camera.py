#!/usr/bin/env python3
"""Save the freshest ROS camera frame when an inspection capture is requested."""

import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class InspectionCamera(Node):
    def __init__(self):
        super().__init__('inspection_camera')

        self.declare_parameter(
            'save_dir', str(Path.home() / 'ros2_ws/data/captures')
        )
        self.declare_parameter('max_image_age_sec', 1.0)
        self.save_dir = Path(
            self.get_parameter('save_dir').value
        ).expanduser()
        self.max_image_age_sec = float(
            self.get_parameter('max_image_age_sec').value
        )
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = None
        self.latest_frame_id = ''
        self.latest_stamp = None

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.request_sub = self.create_subscription(
            String, '/inspection/capture_request',
            self.request_callback, 10
        )
        self.done_pub = self.create_publisher(
            String, '/inspection/capture_done', 10
        )

        self.get_logger().info(
            f'Inspection Camera started. Save directory: {self.save_dir}'
        )
        self.get_logger().info(
            'Waiting for camera images and capture requests...'
        )

    def image_callback(self, msg):
        # Cache the most recent complete message; decode only when requested.
        self.latest_image = msg
        self.latest_image_time = time.monotonic()
        self.latest_frame_id = msg.header.frame_id
        self.latest_stamp = (
            msg.header.stamp.sec, msg.header.stamp.nanosec
        )

    def publish_result(self, result):
        message = String()
        message.data = json.dumps(result, ensure_ascii=False)
        self.done_pub.publish(message)

    def request_callback(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValueError('Request must be a JSON object')
            equipment = request.get('equipment')
            if equipment not in ('A', 'B', 'C', 'HOME'):
                raise ValueError('equipment must be A, B, C, or HOME')
        except (json.JSONDecodeError, ValueError, TypeError) as exc:
            self.get_logger().error(f'Invalid capture request: {exc}')
            self.publish_result({
                'equipment': None, 'success': False,
                'error': f'INVALID_REQUEST: {exc}'
            })
            return

        if self.latest_image is None or self.latest_image_time is None:
            self.get_logger().error(f'Capture failed ({equipment}): no image received')
            self.publish_result({
                'equipment': equipment, 'success': False,
                'error': 'NO_IMAGE'
            })
            return

        age = time.monotonic() - self.latest_image_time
        if age > self.max_image_age_sec:
            self.get_logger().error(
                f'Capture failed ({equipment}): stale image ({age:.2f}s)'
            )
            self.publish_result({
                'equipment': equipment, 'success': False,
                'error': 'STALE_IMAGE', 'image_age_sec': round(age, 3)
            })
            return

        try:
            # ROS image encodings such as rgb8 are converted into OpenCV BGR.
            cv_image = self.bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='bgr8'
            )
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            path = self.save_dir / f'equipment_{equipment}_{timestamp}.png'
            if not cv2.imwrite(str(path), cv_image):
                raise OSError('cv2.imwrite returned False')
        except (CvBridgeError, RuntimeError, ValueError, OSError) as exc:
            self.get_logger().error(f'Capture failed ({equipment}): {exc}')
            self.publish_result({
                'equipment': equipment, 'success': False,
                'error': f'SAVE_FAILED: {exc}'
            })
            return

        result = {
            'equipment': equipment,
            'success': True,
            'image_path': str(path),
            'frame_id': self.latest_frame_id,
            'image_stamp': {
                'sec': self.latest_stamp[0],
                'nanosec': self.latest_stamp[1],
            },
            'image_age_sec': round(age, 3),
        }
        self.publish_result(result)
        self.get_logger().info(f'Image saved ({equipment}): {path}')


def main(args=None):
    rclpy.init(args=args)
    node = InspectionCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
