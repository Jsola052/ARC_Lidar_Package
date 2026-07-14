
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown, ExecuteProcess, GroupAction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("test_data",       default_value="",         description="Path to the test data"),
    DeclareLaunchArgument("res_path",        default_value="/tmp",     description="Path to the folder where to store the results"),
    DeclareLaunchArgument("ref_path",        default_value="",         description="Path to the reference data folder for results comparison"),
    DeclareLaunchArgument("wait_init",       default_value="1",        description="Wait for test node initialization to replay data"),
    DeclareLaunchArgument("outdoor",         default_value="true",     description="Decide which set of parameters to use"),
    DeclareLaunchArgument("use_sim_time",    default_value="true",     description="Sim Time, used when replaying rosbag files"),
    DeclareLaunchArgument("velodyne_driver", default_value="false",    description="If true, start Velodyne driver."),
    DeclareLaunchArgument("scan_frame",      default_value="velodyne", description="Frame ID of the LiDAR scan"),
    DeclareLaunchArgument("verbose",         default_value="false",    description="If true, print the difference with reference during the comparison"),
    DeclareLaunchArgument("domain_id",       default_value="0",        description="Set to different value to avoid interference when several computers running ROS2 on the same network."),
    SetEnvironmentVariable(name='ROS_DOMAIN_ID',value=LaunchConfiguration('domain_id')),
    SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),

    # Velodyne arguments
    DeclareLaunchArgument("calibration_file_path", default_value=os.path.join(get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml'), description="calibration file path"),
    DeclareLaunchArgument("model", default_value="VLP16", description="Model of Velodyne Lidar, choices are : VLP16 / 32C / VLS128"),
    DeclareLaunchArgument("device_ip", default_value=""),
    DeclareLaunchArgument("rpm",  default_value="600.0"),
    DeclareLaunchArgument("port", default_value="2368"),
    DeclareLaunchArgument("pcap", default_value=""),
  ])

  ##############
  ## Velodyne ##
  ##############

  #! For now, velodyne packages are not ported on Windows 10
  if os.name != 'nt':
    # Manually override velodyne_driver_node parameters
    params_driver_path = os.path.join(get_package_share_directory('velodyne_driver'), 'config', 'VLP16-velodyne_driver_node-params.yaml')
    with open(params_driver_path, 'r') as f:
      params_velod_driv = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    params_velod_driv['device_ip']    =  LaunchConfiguration('device_ip')
    params_velod_driv["gps_time"]     = False
    params_velod_driv["read_once"]    = True
    params_velod_driv["read_fast"]    = False
    params_velod_driv["repeat_delay"] = 0.0
    params_velod_driv["frame_id"]     = "velodyne"
    params_velod_driv["model"]        = LaunchConfiguration('model')
    params_velod_driv["rpm"]          = LaunchConfiguration('rpm')
    params_velod_driv["port"]         = LaunchConfiguration('port')
    params_velod_driv["pcap"]         = LaunchConfiguration('pcap')
    params_velod_driv["use_sim_time"] = LaunchConfiguration("use_sim_time")

    # Manually override velodyne_convert_node parameters
    velodyne_pointcloud_share_path = get_package_share_directory('velodyne_pointcloud')
    params_velod_pcl_path = os.path.join(velodyne_pointcloud_share_path, 'config', 'VLP16-velodyne_transform_node-params.yaml')
    with open(params_velod_pcl_path, 'r') as f:
        params_velod_pcl = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']

    params_velod_pcl['calibration']    = LaunchConfiguration('calibration_file_path')
    params_velod_pcl["model"]          = LaunchConfiguration('model')
    params_velod_pcl["min_range"]      = 0.4
    params_velod_pcl["max_range"]      = 130.0
    params_velod_pcl["organize_cloud"] = False
    params_velod_pcl["target_frame"]   = ""
    params_velod_pcl["fixed_frame"]    = ""
    params_velod_pcl["use_sim_time"]   = LaunchConfiguration('use_sim_time')

    velodyne_driver_node = Node(
      package='velodyne_pointcloud',
      executable='velodyne_transform_node',
      output='both',
      name='velodyne_transform_node',
      parameters=[params_velod_pcl],
      arguments=['--ros-args', '-p', f'reliability:=reliable', '-p', f'durability:=transient_local', '-p', f'history:=keep_all'],
      condition = IfCondition(LaunchConfiguration("velodyne_driver"))
    )

  ##########
  ## Slam ##
  ##########

  # Velodyne points conversion
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
    params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("use_sim_time")

  velodyne_conversion_node = Node(
    package="lidar_conversions",
    executable="velodyne_conversion_node",
    name="velodyne_conversion",
    output="screen",
    parameters=[params_conversion],
    arguments=['--ros-args', '-p', f'reliability:=reliable', '-p', f'durability:=transient_local', '-p', f'history:=keep_all'],
  )

  # Test : catch outputs and compare with reference
  # If comparison is required, end process when the reference is finished
  # If comparison is not required (simple evaluation), end process when the bag ends

  with open(os.path.join(get_package_share_directory('lidar_slam_test'), "params/eval.yaml"), 'r') as f:
      params_lidar_slam_test = yaml.safe_load(f)['/lidar_slam_test']['ros__parameters']

  params_lidar_slam_test["res_path"]      = LaunchConfiguration("res_path")
  params_lidar_slam_test["ref_path"]      = LaunchConfiguration("ref_path")
  params_lidar_slam_test["use_sim_time"]  = LaunchConfiguration("use_sim_time")
  params_lidar_slam_test["verbose"]       = LaunchConfiguration("verbose")

  lidar_slam_test = Node(
    name="test",
    package="lidar_slam_test",
    executable="lidar_slam_test_node",
    output="screen",
    parameters=[params_lidar_slam_test],
    arguments=['--ros-args', '-p', f'reliability:=reliable', '-p', f'durability:=transient_local', '-p', f'history:=keep_all'],
    on_exit=GroupAction([Shutdown()],
                        condition=LaunchConfigurationNotEquals('ref_path', ''))
  )

  # LiDAR SLAM

  # Outdoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
    params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  # Manualy override lidar_config_outdoor_node parameters from parameter file
  params_slam_out['use_sim_time'] = LaunchConfiguration("use_sim_time")
  params_slam_out['slam.verbosity'] = 0
  params_slam_out['slam.n_threads'] = 1
  params_slam_out['slam.confidence.overlap.sampling_ratio'] = 0.33

  slam_outdoor_node = Node(
    name="lidar_slam",
    package="lidar_slam",
    executable="lidar_slam_node",
    output="screen",
    parameters=[params_slam_out],
    arguments=['--ros-args', '-p', f'reliability:=reliable', '-p', f'durability:=transient_local', '-p', f'history:=keep_all'],
    condition=IfCondition(LaunchConfiguration("outdoor"))
  )

  # Indoor Lidar Slam node
  with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
    params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
  # Manualy override lidar_config_indoor_node parameters from parameter file
  params_slam_in['use_sim_time'] = LaunchConfiguration("use_sim_time")
  params_slam_in['slam.verbosity'] = 0
  params_slam_out['slam.n_threads'] = 1
  params_slam_out['slam.confidence.overlap.sampling_ratio'] = 0.33

  slam_indoor_node = Node(
    name="lidar_slam",
    package="lidar_slam",
    executable="lidar_slam_node",
    output="screen",
    parameters=[params_slam_in],
    arguments=['--ros-args', '-p', f'reliability:=reliable', '-p', f'durability:=transient_local', '-p', f'history:=keep_all'],
    condition= UnlessCondition(LaunchConfiguration("outdoor"))
  )

  # Moving base coordinates systems description
  tf_base_to_lidar = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_lidar",
    #           X    Y    Z    rZ   rY   rX     FROM          TO
    arguments=['0', '0', '0', '0', '0', '0', 'base_link', LaunchConfiguration("scan_frame"), '--ros-args', '-p', f'reliability:=reliable', '-p', f'durability:=transient_local', '-p', f'history:=keep_all'],
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},]
  )

  # Play bag
  # If comparison is required, end process when the reference is finished
  # If comparison is not required (simple evaluation), end process when the bag ends
  rosbag_action = ExecuteProcess(
    name='exec_rosbag',
    cmd=['ros2', 'bag', 'play', LaunchConfiguration("test_data"), '--clock', '--rate', '1.0', '--disable-keyboard-controls',
         '--qos-profile-overrides-path', os.path.join(get_package_share_directory('lidar_slam_test'), "params/qos_settings.yaml")],
    output= 'screen',
    log_cmd= True,
    on_exit=GroupAction([Shutdown()],
                        condition=LaunchConfigurationEquals('ref_path', ''))
  )
  rosbag_timer = TimerAction(
      period=LaunchConfiguration("wait_init"),
      actions=[rosbag_action]
  )

  ld.add_action(velodyne_driver_node)
  ld.add_action(velodyne_conversion_node)
  ld.add_action(lidar_slam_test)
  ld.add_action(slam_outdoor_node)
  ld.add_action(slam_indoor_node)
  ld.add_action(tf_base_to_lidar)
  ld.add_action(rosbag_timer)

  return (ld)
