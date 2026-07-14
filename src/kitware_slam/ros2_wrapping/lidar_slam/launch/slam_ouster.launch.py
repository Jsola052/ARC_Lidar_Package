from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("replay", default_value="true", description="Whether to process live or replayed data"),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("os_driver", default_value="false", description="If true, activate os_node."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    DeclareLaunchArgument("driver_parameter_file", default_value=os.path.join(lidar_slam_share_path, 'params', "ouster_driver_parameters.yaml"),
                          description="Path to the file containing Ouster driver parameters"),
    DeclareLaunchArgument("metadata_in", default_value=os.path.join(lidar_slam_share_path, 'params', "metadata_OS1_64_1024x10.json"), description="Configuration file for Ouster data to replay"),
    DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
    DeclareLaunchArgument("domain_id", default_value="0", description="Set to different value to avoid interference when several computers running ROS2 on the same network."),
    SetEnvironmentVariable(name='ROS_DOMAIN_ID',value=LaunchConfiguration('domain_id')),
  ])

  ##########
  ## Rviz ##
  ##########
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')],
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    condition = IfCondition(LaunchConfiguration("rviz")),
  )

  #####################
  ### Ouster driver ###
  #####################

  #! For now ouster packages are not ported on Windows 10
  if os.name != 'nt':
    ouster_driver_path = get_package_share_directory("ouster_ros")
    ouster_parameters = os.path.join(lidar_slam_share_path, 'params', "ouster_driver_parameters.yaml")

    group_ouster = GroupAction(
      actions=[
        # Replay
        IncludeLaunchDescription(
          XMLLaunchDescriptionSource([os.path.join(ouster_driver_path, "launch", "replay.launch.xml")]),
          launch_arguments={
            "timestamp_mode"  : "TIME_FROM_INTERNAL_OSC",
            "bag_file"        : "b",
            "metadata"        : LaunchConfiguration("metadata_in"),
            "sensor_frame"    : "laser_sensor_frame",
            "laser_frame"     : "laser_data_frame",
            "imu_frame"       : "imu_data_frame",
            "viz"             : "False",
        }.items(),
          condition=IfCondition(LaunchConfiguration("replay")),
        ),
        # Live
        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(ouster_driver_path, "launch", "driver.launch.py")]),
          launch_arguments={
            "params_file"     : LaunchConfiguration("driver_parameter_file"),
            "viz"             : "False",
        }.items(),
          condition=UnlessCondition(LaunchConfiguration("replay")),
        ),
      ],
      condition=IfCondition(LaunchConfiguration("os_driver"))
    )

  ##########
  ## Slam ##
  ##########

  # Ouster points conversion
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
    params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("replay")

  ouster_conversion_node = Node(
    name="ouster_conversion",
    package="lidar_conversions",
    executable="ouster_conversion_node",
    output="screen",
    parameters=[params_conversion]
  )

  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_out['use_sim_time'] = LaunchConfiguration("replay")

  slam_outdoor_node = Node(
    name="lidar_slam",
    package="lidar_slam",
    executable="lidar_slam_node",
    output="screen",
    parameters=[params_slam_out],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                ("camera", LaunchConfiguration("camera_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),],
    condition=IfCondition(LaunchConfiguration("outdoor"))
  )

  # Indoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
    params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_in['use_sim_time'] = LaunchConfiguration("replay")

  slam_indoor_node = Node(
    name="lidar_slam",
    package="lidar_slam",
    executable="lidar_slam_node",
    output="screen",
    parameters=[params_slam_in],
    remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                ("camera", LaunchConfiguration("camera_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),],
    condition= UnlessCondition(LaunchConfiguration("outdoor"))
  )

  # Aggregation node
  with open(os.path.join(lidar_slam_share_path, 'params', "aggregation_config.yaml"), 'r') as f:
    params_aggregation = yaml.safe_load(f)['/aggregation']['ros__parameters']
  params_aggregation['use_sim_time'] = LaunchConfiguration("replay")

  aggregation_node = Node(
    name="aggregation",
    package="lidar_slam",
    executable="aggregation_node",
    output="screen",
    parameters=[params_aggregation],
    condition=IfCondition(LaunchConfiguration("aggregate"))
  )

  ##########
  ##  TF  ##
  ##########

  tf_nodes = GroupAction(
    actions=[
      # Static TF base to Os sensor (for compatibility with ROS1 bags)
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_os_sensor",
        parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "os_sensor"]
      ),
      # Static TF base to laser sensor
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_laser_sensor_frame",
        parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "laser_sensor_frame"]
      ),
      # Static TF base to wheel encoder
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_wheel",
        parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "wheel"]
      ),
      # Static TF base to INS
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_ins",
        parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "ins"]
      ),
      # Static TF base to GPS
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_gps",
        parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "gps"]
      )
    ],
  )

  # Static TF laser to Os lidar (when the driver is not launched)
  # To be read from the json config file in replay
  # default are for OS1 64
  # cf. https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
  tf_laser_to_os_lidar = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_laser_sensor_frame_to_os_lidar",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0.0036",
               "--roll", "0", "--pitch", "0", "--yaw", "3.14",
               "--frame-id", "laser_sensor_frame", "--child-frame-id", "os_lidar"]
  )

  # Static TF laser to IMU (when the driver is not launched)
  # To be read from the json config file in replay
  # default are for OS1 64
  # cf. https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
  tf_laser_to_imu = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_laser_sensor_frame_to_imu_data_frame",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0.006253", "--y", "-0.011775", "--z", "0.007645",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "laser_sensor_frame", "--child-frame-id", "imu_data_frame"]
  )

  ld.add_action(rviz_node)
  ld.add_action(ouster_conversion_node)
  if os.name != 'nt':
    ld.add_action(group_ouster)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(aggregation_node)
  # TF
  ld.add_action(tf_nodes)
  if LaunchConfiguration('os_driver') == 'false':
    ld.add_action(tf_laser_to_os_lidar)
    ld.add_action(tf_laser_to_imu)
  return (ld)
