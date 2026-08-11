from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ouster_share = get_package_share_directory('ouster_ros')
    sensor_pkg_share = get_package_share_directory('sensor_pkg')
    lidar_slam_share = get_package_share_directory('lidar_slam')
    lidar_conversion_share = get_package_share_directory('lidar_conversions')
    
    # Declare launch argument to toggle outdoor/indoor config
    outdoor_arg = DeclareLaunchArgument(
        'outdoor',
        default_value='false',
        description='Whether to run LidarSLAM in outdoor mode (default: false / indoor)'
    )
    
    # Path to the xml launch file
    sensor_launch_xml = os.path.join(ouster_share, 'launch', 'sensor.launch.xml')
    # Path to our new rviz config file
    rviz_config_path = os.path.join(ouster_share, 'config', 'slam_viz.rviz')
    # Path to the URDF file in the sensor_pkg
    urdf_path = os.path.join(sensor_pkg_share, 'urdf', 'sensor_pkg.urdf')
    
    # Read URDF file contents
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
        
    lidar_driver = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(sensor_launch_xml),
        launch_arguments={
            'sensor_hostname': '10.42.0.58',
            'viz': 'true',
            'rviz_config': rviz_config_path,
            'timestamp_mode': 'TIME_FROM_ROS_TIME',
            'lidar_mode': '512x10',
            'udp_dest': '10.42.0.1'
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
            '--frame-id', 'base_link',
            '--child-frame-id', 'os_sensor'
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
    # 1.5 Image Transport Decompression (Network Bandwidth Optimization)
    # Subscribes to the compressed streams from the Pi over the network,
    # decompresses them locally on the laptop, and publishes to VINS.
    front_decompress_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'image_transport', 'republish', 'compressed', 'raw',
            '--ros-args',
            '--remap', 'in/compressed:=/camera/front/front_camera/image_raw/compressed',
            '--remap', 'out:=/camera/front/front_camera/image_decompressed'
        ],
        output='screen'
    )

    # back_decompress_node removed for single-camera operation
    back_decompress_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'image_transport', 'republish', 'compressed', 'raw',
            '--ros-args',
            '--remap', 'in/compressed:=/camera/back/back_camera/image_raw/compressed',
            '--remap', 'out:=/camera/back/back_camera/image_decompressed'
        ],
        output='screen'
    )

    # 2. VINS-Fusion Node (Front)
    vins_config_front = '/home/robotics/sensor_ws/src/vins_fusion_ros2/config/ouster_dual_cam/front_mono_imu_config.yaml'
    vins_front_node = ExecuteProcess(
        cmd=[
            '/home/robotics/sensor_ws/install/vins/lib/vins/vins_node',
            vins_config_front,
            '--ros-args', '-r', '__node:=vins_front', '-r', '__ns:=/vins_front'
        ],
        output='screen'
    )

    # 2.5 VINS-Fusion Node (Back)
    vins_config_back = '/home/robotics/sensor_ws/src/vins_fusion_ros2/config/ouster_dual_cam/back_mono_imu_config.yaml'
    vins_back_node = ExecuteProcess(
        cmd=[
            '/home/robotics/sensor_ws/install/vins/lib/vins/vins_node',
            vins_config_back,
            '--ros-args', '-r', '__node:=vins_back', '-r', '__ns:=/vins_back'
        ],
        output='screen'
    )


    # 2.75 EKF Node for fusing VINS Front and VINS Back
    ekf_config_path = os.path.join(ouster_share, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', 'odometry/filtered_vision')]
    )

    # 3. Odom-to-Pose Converter helper node (bridge VINS-Fusion to LidarSLAM)
    # We will remap the EKF fused output to ext_pose here!
    odom_to_pose_script = os.path.join(ouster_share, 'launch', 'odom_to_pose.py')
    odom_to_pose_node = ExecuteProcess(
        cmd=['python3', odom_to_pose_script, '--odom_topic', '/odometry/filtered_vision'],
        output='screen'
    )

    # 3.5. Odom-to-Path Converter helper node (for clean RViz line)
    path_publisher_script = os.path.join(ouster_share, 'launch', 'path_publisher.py')
    path_publisher_node = ExecuteProcess(
        cmd=['python3', path_publisher_script],
        output='screen'
    )

    # 4. LidarSLAM Point Cloud Conversion node
    conversion_config_path = os.path.join(lidar_conversion_share, 'params', 'conversion_config.yaml')
    ouster_conversion_node = Node(
        name='ouster_conversion',
        package='lidar_conversions',
        executable='ouster_conversion_node',
        output='screen',
        parameters=[conversion_config_path]
    )

    # 5. Kitware LidarSLAM Node (subscribes to /lidar_points and /ext_poses)
    slam_config_indoor_path = os.path.join(lidar_slam_share, 'params', 'slam_config_indoor.yaml')
    slam_config_outdoor_path = os.path.join(lidar_slam_share, 'params', 'slam_config_outdoor.yaml')
    
    # Indoor Lidar Slam node (default)
    slam_indoor_node = Node(
        name='lidar_slam',
        package='lidar_slam',
        executable='lidar_slam_node',
        output='screen',
        parameters=[slam_config_indoor_path],
        remappings=[('imu', '/ouster/imu')],
        condition=UnlessCondition(LaunchConfiguration('outdoor'))
    )
    
    # Outdoor Lidar Slam node
    slam_outdoor_node = Node(
        name='lidar_slam',
        package='lidar_slam',
        executable='lidar_slam_node',
        output='screen',
        parameters=[slam_config_outdoor_path],
        remappings=[('imu', '/ouster/imu')],
        condition=IfCondition(LaunchConfiguration('outdoor'))
    )
    
    # 6. Nav2 Planner (Proof of Concept - No Motors)
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(ouster_share, 'config', 'nav2_planner_params.yaml')]
    )
    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['planner_server']}]
    )

    # 7. Waypoint Manager (Custom Python Node)
    waypoint_script = os.path.join(ouster_share, 'launch', 'waypoint_manager.py')
    waypoint_manager_node = ExecuteProcess(
        cmd=['python3', waypoint_script],
        output='screen'
    )

    # 8. Map Auto-Save Node (Persistent SLAM)
    # Saves the SLAM keypoint maps to /home/robotics/maps/ every 2 minutes.
    # On next boot, the SLAM node auto-loads these maps via initial_maps param.
    map_autosave_script = os.path.join(ouster_share, 'launch', 'map_autosave.py')
    map_autosave_node = ExecuteProcess(
        cmd=['python3', map_autosave_script],
        output='screen'
    )

    # 9. Map Manager GUI (Floating Toolbar)
    # Small always-on-top window with two buttons:
    #   "New Map"  — wipes saved maps & resets SLAM for a new deployment site
    #   "Save Now" — manual save trigger
    map_manager_script = os.path.join(ouster_share, 'launch', 'map_manager.py')
    map_manager_node = ExecuteProcess(
        cmd=['python3', map_manager_script],
        output='screen'
    )

    # 10.5 Camera Rotate Node (180° rotated feeds for optional RViz inspection)
    # Publishes /camera/front/front_camera/image_rotated
    #       and /camera/back/back_camera/image_rotated
    # Nothing subscribes to these — they are for manual RViz use only.
    camera_rotate_script = os.path.join(ouster_share, 'launch', 'camera_rotate.py')
    camera_rotate_node = ExecuteProcess(
        cmd=['python3', camera_rotate_script],
        additional_env={'CYCLONEDDS_URI': ''},
        output='screen'
    )

    # 10. Saved Map Loader (Always-Visible Map in RViz)
    # Reads saved PCD files from /home/robotics/maps/ and publishes
    # them as latched PointCloud2 topics so the map overlay is visible
    # immediately on startup, even before SLAM starts processing.
    saved_map_loader_script = os.path.join(ouster_share, 'launch', 'saved_map_loader.py')
    saved_map_loader_node = ExecuteProcess(
        cmd=['python3', saved_map_loader_script],
        output='screen'
    )

    # 10.6 Dense Map Accumulator (Full 3D Environment Map)
    # Accumulates LiDAR scans into a voxel-downsampled point cloud in odom frame.
    # Publishes /dense_map and saves/loads /home/robotics/maps/dense_map.pcd
    dense_map_script = os.path.join(ouster_share, 'launch', 'dense_map_accumulator.py')
    dense_map_node = ExecuteProcess(
        cmd=['python3', dense_map_script],
        output='screen'
    )
    
    nodes = [
        outdoor_arg,
        lidar_driver,
        robot_state_publisher,
        tf_os_sensor_to_base,
        tf_front_optical,
        tf_back_optical,

        front_decompress_node,
        back_decompress_node,
        vins_front_node,
        vins_back_node,

        ekf_node,
        odom_to_pose_node,
        path_publisher_node,
        ouster_conversion_node,
        slam_indoor_node,
        slam_outdoor_node,
        nav2_planner,
        nav2_lifecycle,
        waypoint_manager_node,
        map_autosave_node,
        map_manager_node,
        saved_map_loader_node,
        dense_map_node,
        camera_rotate_node
    ]
    
    return LaunchDescription(nodes)
