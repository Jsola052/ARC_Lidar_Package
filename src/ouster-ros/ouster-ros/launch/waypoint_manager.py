#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import ComputePathToPose

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered_vision', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 10)
        self.path_pub = self.create_publisher(Path, '/drawn_path', 10)
        
        # Action Client for Nav2 Planner
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # State
        self.current_pose = None
        self.waypoints = []
        self.distance_threshold = 2.0  # Save a waypoint every 2 meters
        
        self.get_logger().info("Waypoint Manager started. Waiting for odometry...")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
        # Check if we should save a new breadcrumb
        if not self.waypoints:
            self.save_waypoint(self.current_pose)
        else:
            last_wp = self.waypoints[-1]
            dx = self.current_pose.position.x - last_wp.position.x
            dy = self.current_pose.position.y - last_wp.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist >= self.distance_threshold:
                self.save_waypoint(self.current_pose)

    def save_waypoint(self, pose):
        self.waypoints.append(pose)
        self.get_logger().info(f"Saved waypoint #{len(self.waypoints)} at x={pose.position.x:.2f}, y={pose.position.y:.2f}")
        self.publish_markers()

    def publish_markers(self):
        msg = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = wp
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            msg.markers.append(marker)
        self.marker_pub.publish(msg)

    def goal_callback(self, msg):
        if not self.current_pose:
            self.get_logger().warn("Cannot compute path: No odometry received yet.")
            return
            
        self.get_logger().info("Received goal! Asking Nav2 planner to draw a path...")
        
        if not self.planner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Planner Server not available!")
            return
            
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = msg
        
        # Start pose
        start = PoseStamped()
        start.header.frame_id = "odom"
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose = self.current_pose
        goal_msg.start = start
        
        goal_msg.use_start = True
        
        self._send_goal_future = self.planner_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 Planner rejected the goal.")
            return
            
        self.get_logger().info("Nav2 Planner accepted goal, computing path...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.path.poses:
            self.get_logger().info(f"Path computed successfully! {len(result.path.poses)} points.")
            # Ensure frame_id is correct for RViz
            result.path.header.frame_id = "odom"
            self.path_pub.publish(result.path)
        else:
            self.get_logger().warn("Planner returned an empty path (could not find a way to the goal).")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
