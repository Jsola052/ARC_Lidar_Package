from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ouster_share = get_package_share_directory('ouster_ros')
    sensor_pkg_share = get_package_share_directory('sensor_pkg')
    
    # Declare run_slam launch argument
    run_slam_arg = DeclareLaunchArgument(
        'run_slam',
        default_value='true',
        description='Whether to run KISS-ICP SLAM alongside the LiDAR driver'
    )
    
    # Path to the xml launch file
    sensor_launch_xml = os.path.join(ouster_share, 'launch', 'sensor.launch.xml')
    # Path to our new rviz config file
    rviz_config_path = os.path.join(ouster_share, 'config', 'viz_dual_cam.rviz')
    # Path to the URDF file in the sensor_pkg
    urdf_path = os.path.join(sensor_pkg_share, 'urdf', 'sensor_pkg.urdf')
    # Path to our custom path publisher script
    path_pub_script = os.path.join(ouster_share, 'launch', 'path_publisher.py')
    
    # Read URDF file contents
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
        
    # Include the main sensor launch
    # Pass timestamp_mode:=TIME_FROM_ROS_TIME to sync clocks and prevent TF jitter
    lidar_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(sensor_launch_xml),
        launch_arguments={
            'sensor_hostname': '10.42.0.58',
            'viz': 'true',
            'rviz_config': rviz_config_path,
            'timestamp_mode': 'TIME_FROM_ROS_TIME'
        }.items()
    )
    
    # Robot State Publisher node for URDF geometry
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )
    
    # Static TF: Link driver frame 'os_sensor' to URDF root 'base_link'
    tf_os_sensor_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_os_sensor_to_base',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '-0.04926',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'os_sensor',
            '--child-frame-id', 'base_link'
        ]
    )
    
    # Static TF: Link URDF camera 'cam2_link' to optical frame 'camera_front_optical_frame'
    tf_front_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_front_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
            '--frame-id', 'cam2_link',
            '--child-frame-id', 'camera_front_optical_frame'
        ]
    )
    
    # Static TF: Link URDF camera 'cam1_link' to optical frame 'camera_back_optical_frame'
    tf_back_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_back_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
            '--frame-id', 'cam1_link',
            '--child-frame-id', 'camera_back_optical_frame'
        ]
    )
    
    # Helper Node: Listen to /kiss/odometry and publish /kiss/path
    path_publisher_node = Node(
        executable=path_pub_script,
        name='path_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_slam'))
    )
    
    # KISS-ICP SLAM Launch inclusion
    try:
        kiss_share = get_package_share_directory('kiss_icp')
        kiss_launch_py = os.path.join(kiss_share, 'launch', 'odometry.launch.py')
        slam_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kiss_launch_py),
            launch_arguments={
                'topic': '/ouster/points',
                'visualize': 'false',
                'use_sim_time': 'false'
            }.items(),
            condition=IfCondition(LaunchConfiguration('run_slam'))
        )
    except Exception:
        slam_node = None
    
    nodes = [
        run_slam_arg,
        lidar_driver,
        robot_state_publisher,
        tf_os_sensor_to_base,
        tf_front_optical,
        tf_back_optical,
        path_publisher_node
    ]
    if slam_node is not None:
        nodes.append(slam_node)
        
    return LaunchDescription(nodes)
