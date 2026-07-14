#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.sub = self.create_subscription(Odometry, '/slam_odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Path, '/slam_path', 10)
        self.path = Path()
        
    def odom_callback(self, msg):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        
        # Keep path from growing infinitely
        if len(self.path.poses) > 5000:
            self.path.poses.pop(0)
            
        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
