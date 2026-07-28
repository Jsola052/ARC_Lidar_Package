#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math

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
        # Only append if the robot has moved more than 5cm
        # This prevents the path from decaying/vanishing while sitting still
        if len(self.path.poses) > 0:
            last_pose = self.path.poses[-1].pose.position
            curr_pose = msg.pose.pose.position
            dist = math.sqrt(
                (curr_pose.x - last_pose.x)**2 + 
                (curr_pose.y - last_pose.y)**2 + 
                (curr_pose.z - last_pose.z)**2
            )
            if dist < 0.05:
                return
                
        self.path.poses.append(pose)
        
        # Keep path from growing infinitely (50000 points is huge but RViz handles it fine)
        if len(self.path.poses) > 50000:
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
