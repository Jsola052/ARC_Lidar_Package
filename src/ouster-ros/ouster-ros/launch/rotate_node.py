#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2

class RotateNode(Node):
    def __init__(self):
        super().__init__('rotate_node')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'in/image', self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Image, 'out/image', qos_profile_sensor_data)
    
    def callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Rotate image 180 degrees for visualization purposes ONLY
            cv_img = cv2.rotate(cv_img, cv2.ROTATE_180)
            
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Rotation error: {e}")

def main():
    rclpy.init()
    node = RotateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
