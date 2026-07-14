#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
import argparse

class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose')
        
        parser = argparse.ArgumentParser()
        parser.add_argument('--odom_topic', type=str, default='/vins/odometry')
        # Use parse_known_args because ROS 2 passes its own arguments too
        args, _ = parser.parse_known_args(sys.argv[1:])
        
        self.sub = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.callback,
            10
        )
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/ext_poses',
            10
        )
        self.get_logger().info(f"OdomToPose converter node started. Subscribed to {args.odom_topic} -> Publishing to /ext_poses")

    def callback(self, msg):
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        # Propagate pose and covariance
        out.pose = msg.pose
        self.pub.publish(out)

def main():
    rclpy.init()
    node = OdomToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
