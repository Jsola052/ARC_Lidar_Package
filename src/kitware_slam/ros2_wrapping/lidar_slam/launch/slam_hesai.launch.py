from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
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
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
    condition = IfCondition(LaunchConfiguration("rviz"))
  )

  ##########
  ## Slam ##
  ##########

  # Hesai points conversion
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
    params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("use_sim_time")

  hesai_conversion_node = Node(
    package="lidar_conversions",
    executable="hesai_conversion_node",
    name="hesai_conversion",
    output="screen",
    parameters=[params_conversion]
  )

  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  params_slam_out['use_sim_time'] = LaunchConfiguration("use_sim_time")

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
  params_slam_in['use_sim_time'] = LaunchConfiguration("use_sim_time")

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

  # Aggregate points
  print( os.path.join(lidar_slam_share_path, 'params', "aggregation_config.yaml"))
  with open(os.path.join(lidar_slam_share_path, 'params', "aggregation_config.yaml"), 'r') as f:
    params_aggregation = yaml.safe_load(f)['/aggregation']['ros__parameters']
  params_aggregation['use_sim_time'] = LaunchConfiguration("use_sim_time")

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
      # Static TF base to Hesai sensor
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_lidar",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "hesai_lidar"]
      ),
      # Static TF base to wheel encoder
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_wheel",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "wheel"]
      ),
      # Static TF base to INS
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_ins",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "ins"]
      ),
      # Static TF base to GPS
      Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_gps",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                  "--roll", "0", "--pitch", "0", "--yaw", "0",
                  "--frame-id", "base_link", "--child-frame-id", "gps"]
      )
    ]
  )

  ld.add_action(rviz_node)
  ld.add_action(hesai_conversion_node)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(aggregation_node)
  # TF
  ld.add_action(tf_nodes)

  return (ld)
