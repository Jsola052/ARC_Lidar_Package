#!/usr/bin/env python3
"""
Dense Map Accumulator for Persistent SLAM
==========================================
Accumulates LiDAR scans into a dense voxel-downsampled point cloud
in the 'odom' frame, providing a full 3D environment map.

Published topics:
  /dense_map — accumulated dense point cloud (PointCloud2, transient-local)

Subscribed topics:
  /ouster/points — raw LiDAR scans
  /dense_map/save — trigger message (std_msgs/Empty) to save to disk

On startup, loads /home/robotics/maps/dense_map.pcd if it exists.
"""
import os
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty, Header
from lidar_slam.msg import SlamCommand
import open3d as o3d

# ─── Configuration ───────────────────────────────────────────
MAP_DIR = '/home/robotics/maps/'
DENSE_MAP_FILE = os.path.join(MAP_DIR, 'dense_map.pcd')
VOXEL_SIZE = 0.10           # 10cm voxel resolution
PUBLISH_INTERVAL = 1.0      # Republish accumulated map every 1 second
ACCUMULATE_EVERY_N = 3      # Only accumulate every Nth scan (reduce CPU)
MAX_POINTS = 5_000_000      # Safety cap on total points
MIN_Z = -0.6                # Crop points below this height (removes deep floor reflections/multipath)
MAX_Z = 5.0                 # Crop points above this height (keeps map clean)
# ─────────────────────────────────────────────────────────────


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract x, y, z from a PointCloud2 message as Nx3 float32 array."""
    # Find field offsets
    x_off = y_off = z_off = None
    for field in msg.fields:
        if field.name == 'x':
            x_off = field.offset
        elif field.name == 'y':
            y_off = field.offset
        elif field.name == 'z':
            z_off = field.offset

    if x_off is None or y_off is None or z_off is None:
        return np.empty((0, 3), dtype=np.float32)

    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    n = data.shape[0]
    points = np.empty((n, 3), dtype=np.float32)
    points[:, 0] = np.frombuffer(data[:, x_off:x_off+4].tobytes(), dtype=np.float32)
    points[:, 1] = np.frombuffer(data[:, y_off:y_off+4].tobytes(), dtype=np.float32)
    points[:, 2] = np.frombuffer(data[:, z_off:z_off+4].tobytes(), dtype=np.float32)

    # Filter out NaN/inf
    valid = np.isfinite(points).all(axis=1)
    return points[valid]


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample point cloud using Open3D voxel grid (extremely fast)."""
    if points.shape[0] == 0:
        return points

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return np.asarray(down_pcd.points, dtype=np.float32)


def xyz_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Convert Nx3 float32 array to PointCloud2 message."""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.height = 1
    msg.width = points.shape[0]
    msg.row_step = 12 * points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = True
    msg.data = points.astype(np.float32).tobytes()

    return msg


def save_pcd_binary(points: np.ndarray, filepath: str):
    """Save Nx3 float32 array as binary PCD file."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    n = points.shape[0]
    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS x y z\n"
        f"SIZE 4 4 4\n"
        f"TYPE F F F\n"
        f"COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())


def load_pcd_binary(filepath: str) -> np.ndarray:
    """Load a PCD file (binary or ASCII) and return Nx3 float32 array."""
    if not os.path.isfile(filepath):
        return np.empty((0, 3), dtype=np.float32)

    with open(filepath, 'rb') as f:
        num_points = 0
        fields = []
        is_binary = False
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_type = line.split()[1].lower()
                is_binary = data_type in ('binary', 'binary_compressed')
                break

        if num_points == 0:
            return np.empty((0, 3), dtype=np.float32)

        try:
            xi = fields.index('x')
            yi = fields.index('y')
            zi = fields.index('z')
        except ValueError:
            xi, yi, zi = 0, 1, 2

        nfields = len(fields)

        if is_binary:
            raw = f.read()
            point_size = nfields * 4
            actual_points = min(num_points, len(raw) // point_size)
            all_data = np.frombuffer(raw[:actual_points * point_size], dtype=np.float32).reshape(actual_points, nfields)
            points = np.stack([all_data[:, xi], all_data[:, yi], all_data[:, zi]], axis=1)
        else:
            points_list = []
            for line_bytes in f:
                line = line_bytes.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        points_list.append([float(parts[xi]), float(parts[yi]), float(parts[zi])])
                    except (ValueError, IndexError):
                        continue
            points = np.array(points_list, dtype=np.float32) if points_list else np.empty((0, 3), dtype=np.float32)

    valid = np.isfinite(points).all(axis=1)
    return points[valid]


class DenseMapAccumulator(Node):
    def __init__(self):
        super().__init__('dense_map_accumulator')

        # Accumulated points in odom frame (Nx3 float32)
        self.accumulated = np.empty((0, 3), dtype=np.float32)
        self.scan_count = 0

        # Load existing map from disk
        if os.path.isfile(DENSE_MAP_FILE):
            loaded = load_pcd_binary(DENSE_MAP_FILE)
            if loaded.shape[0] > 0:
                self.accumulated = loaded
                self.get_logger().info(f'✓ Loaded saved dense map: {loaded.shape[0]} points')
            else:
                self.get_logger().info('Saved dense map file exists but is empty.')
        else:
            self.get_logger().info('No saved dense map found — starting fresh.')

        # Publisher with transient-local QoS
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(PointCloud2, '/dense_map', latched_qos)

        # Subscribe to SLAM-registered scans (already in odom frame, undistorted)
        scan_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.scan_sub = self.create_subscription(
            PointCloud2,
            '/slam_registered_points',
            self._scan_cb,
            scan_qos
        )

        # Subscribe to save trigger
        self.save_sub = self.create_subscription(
            Empty,
            '/dense_map/save',
            self._save_cb,
            10
        )

        # Subscribe to SLAM commands to clear map on reset
        self.cmd_sub = self.create_subscription(
            SlamCommand,
            '/slam_command',
            self._cmd_cb,
            10
        )

        # Periodic republish
        self.pub_timer = self.create_timer(PUBLISH_INTERVAL, self._publish_map)

        # Publish immediately if we loaded a map
        if self.accumulated.shape[0] > 0:
            self._publish_map()

        self.get_logger().info('─' * 50)
        self.get_logger().info('Dense Map Accumulator Started')
        self.get_logger().info(f'  Voxel size: {VOXEL_SIZE}m')
        self.get_logger().info(f'  Accumulate every {ACCUMULATE_EVERY_N} scans')
        self.get_logger().info(f'  Max points: {MAX_POINTS:,}')
        self.get_logger().info('─' * 50)

    def _scan_cb(self, msg: PointCloud2):
        """Process each incoming registered (odom-frame) LiDAR scan."""
        self.scan_count += 1
        if self.scan_count % ACCUMULATE_EVERY_N != 0:
            return

        # Registered points are already in odom frame — extract directly
        scan_points = pointcloud2_to_xyz(msg)
        if scan_points.shape[0] == 0:
            return

        # Crop points outside Z range (removes noise below floor)
        valid_z = (scan_points[:, 2] >= MIN_Z) & (scan_points[:, 2] <= MAX_Z)
        scan_points = scan_points[valid_z]

        if scan_points.shape[0] == 0:
            return

        # Downsample the incoming scan first (huge speedup)
        scan_points = voxel_downsample(scan_points, VOXEL_SIZE)

        # Merge with accumulated
        if self.accumulated.shape[0] == 0:
            self.accumulated = scan_points
        else:
            self.accumulated = np.vstack([self.accumulated, scan_points])

        # Safety cap
        if self.accumulated.shape[0] > MAX_POINTS:
            self.accumulated = voxel_downsample(self.accumulated, VOXEL_SIZE * 1.5)
            self.get_logger().warn(
                f'Dense map exceeded {MAX_POINTS:,} points, '
                f'increased voxel size. Now: {self.accumulated.shape[0]:,} points'
            )

    def _publish_map(self):
        """Publish the accumulated map."""
        if self.accumulated.shape[0] == 0:
            return

        # Clean up duplicates across the entire map periodically
        self.accumulated = voxel_downsample(self.accumulated, VOXEL_SIZE)

        stamp = self.get_clock().now().to_msg()
        msg = xyz_to_pointcloud2(self.accumulated, 'odom', stamp)
        self.map_pub.publish(msg)

    def _save_cb(self, msg: Empty):
        """Save the accumulated map to disk."""
        self.save_map()

    def _cmd_cb(self, msg: SlamCommand):
        """Listen for SLAM resets to clear the dense map."""
        # 10 = ENABLE_SLAM_MAP_UPDATE (used by Map Manager to clear map)
        # 13 = RESET_ODOM
        if msg.command in (10, 13):
            self.get_logger().info('Received SLAM reset command — clearing dense map in memory.')
            self.accumulated = np.empty((0, 3), dtype=np.float32)

    def save_map(self):
        """Write the accumulated map to a PCD file."""
        if self.accumulated.shape[0] == 0:
            self.get_logger().warn('No dense map data to save.')
            return

        try:
            save_pcd_binary(self.accumulated, DENSE_MAP_FILE)
            self.get_logger().info(
                f'✓ Dense map saved: {self.accumulated.shape[0]:,} points → {DENSE_MAP_FILE}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to save dense map: {e}')


def main():
    rclpy.init()
    node = DenseMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown — performing final dense map save...')
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
