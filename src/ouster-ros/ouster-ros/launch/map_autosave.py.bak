#!/usr/bin/env python3
"""
Map Auto-Save Node for Persistent SLAM
=======================================
This node runs alongside the SLAM system and periodically saves
the current keypoint maps to disk. On the next boot, the SLAM
node will automatically reload these maps via the 'initial_maps'
parameter in slam_config_indoor.yaml.

Saved files (written to /home/robotics/maps/):
  - slam_map_edges.pcd
  - slam_map_intensity_edges.pcd
  - slam_map_planes.pcd
  - slam_map_blobs.pcd

How it works:
  1. Every SAVE_INTERVAL_SEC (default 120s / 2 minutes), this node
     calls the /lidar_slam/save_pc service to dump the current maps.
  2. On shutdown (Ctrl+C), it performs one final save attempt.
  3. Maps are saved with 'fixed: true' so loaded keypoints won't
     decay before the robot revisits them.
  4. Maps are saved with 'filtered: true' to remove dynamic objects
     (people, moving furniture) from the persistent map.

The save uses binary compressed format (format: 2) for smallest
file size on disk.
"""
import subprocess
import os
import rclpy
from rclpy.node import Node

# ─── Configuration ───────────────────────────────────────────
SAVE_PATH = '/home/robotics/maps/slam_map'       # File prefix for .pcd maps
SAVE_INTERVAL_SEC = 120.0                         # Save every 2 minutes
INITIAL_DELAY_SEC = 60.0                          # Wait 60s before first save
# ─────────────────────────────────────────────────────────────


class MapAutoSave(Node):
    """
    ROS 2 node that periodically calls the /lidar_slam/save_pc
    service to persist the SLAM keypoint maps to disk.
    """

    def __init__(self):
        super().__init__('map_autosave')

        # Ensure the output directory exists
        os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)

        # Start with a one-shot delay timer so the SLAM node has time
        # to initialize and (optionally) load an existing map before
        # we overwrite it with an empty save.
        self.startup_timer = self.create_timer(INITIAL_DELAY_SEC, self._start_periodic_save)
        self.periodic_timer = None

        self.get_logger().info('─' * 50)
        self.get_logger().info('Map Auto-Save Node Started')
        self.get_logger().info(f'  Save path:     {SAVE_PATH}')
        self.get_logger().info(f'  Save interval: {SAVE_INTERVAL_SEC/60:.0f} minutes')
        self.get_logger().info(f'  First save in: {INITIAL_DELAY_SEC:.0f} seconds')
        self.get_logger().info('─' * 50)

    def _start_periodic_save(self):
        """Called once after the initial delay. Cancels the startup timer
        and begins the repeating periodic save."""
        # Cancel the one-shot startup timer
        self.startup_timer.cancel()
        self.startup_timer.destroy()

        # Perform the first save immediately
        self.save_map()

        # Then schedule periodic saves
        self.periodic_timer = self.create_timer(SAVE_INTERVAL_SEC, self.save_map)
        self.get_logger().info(
            f'Periodic auto-save engaged (every {SAVE_INTERVAL_SEC/60:.0f} min).'
        )

    def save_map(self):
        """
        Call the /lidar_slam/save_pc service via subprocess.

        Using subprocess (ros2 service call) instead of a native
        service client ensures maximum reliability — no issues with
        Python binding imports, nested spins, or shutdown races.
        """
        cmd = [
            'ros2', 'service', 'call',
            '/lidar_slam/save_pc',
            'lidar_slam/srv/SavePc',
            '{output_prefix_path: ' + SAVE_PATH + ', format: 2, filtered: true, fixed: true}'
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0 and 'success=true' in result.stdout.lower():
                self.get_logger().info(f'✓ Map auto-saved to {SAVE_PATH}')
            elif result.returncode == 0:
                # Service responded but maybe success=false
                self.get_logger().warn(
                    f'Map save service responded but may have failed: '
                    f'{result.stdout.strip()}'
                )
            else:
                self.get_logger().warn(
                    f'Map save returned non-zero exit code. '
                    f'SLAM node may not be ready yet.'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Map save timed out (30s). SLAM may be busy.')
        except Exception as e:
            self.get_logger().warn(f'Map save failed: {e}')


def main():
    rclpy.init()
    node = MapAutoSave()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown received — performing final map save...')
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
