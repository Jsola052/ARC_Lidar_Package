#!/usr/bin/env python3
"""
camera_rotate.py
Subscribes to front and back camera decompressed feeds,
rotates each image 180 degrees, and republishes on new topics.
Nothing subscribes to these topics; they are for optional RViz inspection only.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraRotateNode(Node):
    def __init__(self):
        super().__init__('camera_rotate')
        self.bridge = CvBridge()

        # Front camera
        self.front_sub = self.create_subscription(
            Image,
            '/camera/front/front_camera/image_decompressed',
            self.front_cb,
            10
        )
        self.front_pub = self.create_publisher(
            Image,
            '/camera/front/front_camera/image_rotated',
            10
        )

        # Back camera
        self.back_sub = self.create_subscription(
            Image,
            '/camera/back/back_camera/image_decompressed',
            self.back_cb,
            10
        )
        self.back_pub = self.create_publisher(
            Image,
            '/camera/back/back_camera/image_rotated',
            10
        )

        self.get_logger().info('Camera rotate node started — publishing rotated feeds.')

    def front_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            rotated = cv2.rotate(cv_img, cv2.ROTATE_180)
            out_msg = self.bridge.cv2_to_imgmsg(rotated, encoding=msg.encoding)
            out_msg.header = msg.header
            self.front_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'Front rotate error: {e}')

    def back_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            rotated = cv2.rotate(cv_img, cv2.ROTATE_180)
            out_msg = self.bridge.cv2_to_imgmsg(rotated, encoding=msg.encoding)
            out_msg.header = msg.header
            self.back_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f'Back rotate error: {e}')


def main():
    rclpy.init()
    node = CameraRotateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
