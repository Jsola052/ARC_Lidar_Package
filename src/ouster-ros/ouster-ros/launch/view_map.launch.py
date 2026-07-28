from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    edges_pcd = '/home/robotics/maps/slam_map_edges.pcd'
    planes_pcd = '/home/robotics/maps/slam_map_planes.pcd'
    
    # Node to publish the edge map
    edges_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='edges_publisher',
        arguments=[edges_pcd, '1.0'],
        parameters=[{'frame_id': 'map'}],
        remappings=[('cloud_pcd', '/slam_map_edges')],
        output='screen'
    )
    
    # Node to publish the plane map
    planes_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='planes_publisher',
        arguments=[planes_pcd, '1.0'],
        parameters=[{'frame_id': 'map'}],
        remappings=[('cloud_pcd', '/slam_map_planes')],
        output='screen'
    )
    
    # Launch RViz
    rviz_config = '/home/robotics/sensor_ws/src/ouster-ros/ouster-ros/config/slam_viz.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        edges_node,
        planes_node,
        rviz_node
    ])
