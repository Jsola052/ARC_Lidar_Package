#!/usr/bin/env python3
"""
Saved Map Loader for RViz
=========================
Loads previously saved PCD map files from /home/robotics/maps/ and
publishes them as PointCloud2 topics so the saved map is always
visible in RViz, regardless of whether SLAM is actively running.

Published topics:
  /saved_map/edges  — edge keypoints (cyan in RViz)
  /saved_map/planes — planar surfaces (yellow in RViz)

The maps are republished every 5 seconds with latched QoS so that
RViz always has the data, even if it connects late.
"""
import os
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

MAP_DIR = '/home/robotics/maps/'
EDGES_FILE = os.path.join(MAP_DIR, 'slam_map_edges.pcd')
PLANES_FILE = os.path.join(MAP_DIR, 'slam_map_planes.pcd')
REPUBLISH_INTERVAL = 5.0  # seconds


def load_pcd_to_pointcloud2(filepath, frame_id='odom'):
    """
    Read a PCD file (ASCII or binary) and convert it to a
    sensor_msgs/PointCloud2 message with x, y, z fields.
    """
    if not os.path.isfile(filepath):
        return None

    points = []
    with open(filepath, 'rb') as f:
        # Read header
        is_binary = False
        num_points = 0
        fields = []
        header_lines = 0
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header_lines += 1
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_type = line.split()[1].lower()
                is_binary = (data_type == 'binary' or data_type == 'binary_compressed')
                break

        if num_points == 0:
            return None

        # Find x, y, z field indices
        try:
            xi = fields.index('x')
            yi = fields.index('y')
            zi = fields.index('z')
        except ValueError:
            # Fallback: assume first three fields are x, y, z
            xi, yi, zi = 0, 1, 2

        if is_binary:
            # For binary PCD, each point is len(fields) * 4 bytes (float32)
            point_size = len(fields) * 4
            raw = f.read()
            for i in range(min(num_points, len(raw) // point_size)):
                offset = i * point_size
                vals = struct.unpack_from(f'<{len(fields)}f', raw, offset)
                points.append((vals[xi], vals[yi], vals[zi]))
        else:
            # ASCII format
            for line_bytes in f:
                line = line_bytes.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        x = float(parts[xi])
                        y = float(parts[yi])
                        z = float(parts[zi])
                        points.append((x, y, z))
                    except (ValueError, IndexError):
                        continue

    if not points:
        return None

    # Build PointCloud2 message
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12  # 3 x float32
    msg.height = 1
    msg.width = len(points)
    msg.row_step = msg.point_step * msg.width
    msg.is_bigendian = False
    msg.is_dense = True

    # Pack point data
    buf = bytearray(msg.row_step)
    for i, (x, y, z) in enumerate(points):
        struct.pack_into('<fff', buf, i * 12, x, y, z)
    msg.data = bytes(buf)

    return msg


class SavedMapLoader(Node):
    def __init__(self):
        super().__init__('saved_map_loader')

        # Use transient local durability so late-joining subscribers
        # (like RViz) still get the data
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.edges_pub = self.create_publisher(
            PointCloud2, '/saved_map/edges', latched_qos
        )
        self.planes_pub = self.create_publisher(
            PointCloud2, '/saved_map/planes', latched_qos
        )

        # Load and publish immediately
        self.edges_msg = None
        self.planes_msg = None
        self._load_and_publish()

        # Republish periodically so newly saved maps get picked up
        self.timer = self.create_timer(REPUBLISH_INTERVAL, self._load_and_publish)

    def _load_and_publish(self):
        """Load PCD files from disk and publish them."""
        now = self.get_clock().now().to_msg()

        # Load edges
        edges_msg = load_pcd_to_pointcloud2(EDGES_FILE, frame_id='odom')
        if edges_msg:
            edges_msg.header.stamp = now
            self.edges_pub.publish(edges_msg)
            if self.edges_msg is None:
                self.get_logger().info(
                    f'✓ Loaded saved map edges: {edges_msg.width} points'
                )
            self.edges_msg = edges_msg
        elif self.edges_msg is None:
            self.get_logger().warn(f'No saved edges map at {EDGES_FILE}')

        # Load planes
        planes_msg = load_pcd_to_pointcloud2(PLANES_FILE, frame_id='odom')
        if planes_msg:
            planes_msg.header.stamp = now
            self.planes_pub.publish(planes_msg)
            if self.planes_msg is None:
                self.get_logger().info(
                    f'✓ Loaded saved map planes: {planes_msg.width} points'
                )
            self.planes_msg = planes_msg
        elif self.planes_msg is None:
            self.get_logger().warn(f'No saved planes map at {PLANES_FILE}')


def main():
    rclpy.init()
    node = SavedMapLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
