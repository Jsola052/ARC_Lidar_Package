# Using LiDAR SLAM with ROS2 wrapping

- [lidar\_slam](#using-lidar-slam-with-ros-wrapping)
  - [LiDAR SLAM node](#lidar-slam-node)
    - [Description and basic usage](#description-and-basic-usage)
      - [Slam node](#slam-node)
      - [Pipeline use](#pipeline-use)
        - [With a Velodyne Lidar](#with-a-velodyne-lidar)
        - [With an Ouster Lidar](#with-an-ouster-lidar)
        - [With other LiDAR](#with-other-lidar)
      - [Choosing a domain ID](#choosing-a-domain-id)
    - [More advanced usage](#more-advanced-usage)
      - [Detailed pipeline](#detailed-pipeline)
      - [Multiple LiDAR sensors](#multiple-lidar-sensors)
      - [Online configuration](#online-configuration)
        - [Reset state](#reset-state)
        - [Set initial state](#set-initial-state)
        - [Reset ODOM](#reset-odom)
        - [Map update modes](#map-update-modes)
        - [Reset the trajectory](#reset-the-trajectory)
        - [Save the current trajectory](#save-the-current-trajectory)
        - [Save maps](#save-maps)
        - [Set pose](#set-pose)
        - [Switch ON/OFF the process](#switch-onoff-the-process)
        - [Switch ON/OFF the external sensors](#switch-onoff-the-external-sensors)
        - [Calibrate with external pose sensor](#calibrate-with-external-pose-sensor)
      - [Failure detection](#failure-detection)
      - [Optional loop closure use](#optional-loop-closure-use)
  - [Optional external sensors use](#optional-external-sensors-use)
    - [GPS](#gps)
    - [Tag detector](#tag-detector)
      - [Local constraint](#local-constraint)
      - [Pose graph optimization](#pose-graph-optimization)
    - [Camera](#camera)
    - [Wheel encoder](#wheel-encoder)
    - [External pose](#external-pose)
      - [Local constraint](#local-constraint-1)
      - [Pose graph optimization](#pose-graph-optimization-1)
    - [IMU](#imu)
  - [About the published TF tree](#about-the-published-tf-tree)
- [Points aggregation](#points-aggregation)
  - [Classic aggregation](#classic-aggregation)
  - [Advanced](#advanced)
    - [Vertical slice extraction and analysis](#vertical-slice-extraction-and-analysis)
    - [Horizontal slice extraction/rejection](#horizontal-slice-extractionrejection)
    - [Obstacles detection](#obstacles-detection)

## LiDAR SLAM node

### Description and basic usage

#### Slam node
The SLAM node subscribes to one or several topics of LiDAR pointclouds, and computes current pose of the tracked frame relative to the fixed odometry frame. It can output various data, such as SLAM pose (as Odometry msg or TF), keypoint maps, etc.

SLAM node supports massive configuration from ROS parameter server (even if default values are already provided). Examples of yaml configuration files can be found in [`params/`](params/). All parameters have to be set as private parameters.

To start only raw LiDAR SLAM, just start *lidar_slam_node*:
```bash
ros2 run lidar_slam lidar_slam_node
```
If you want to specify parameters, you should consider using a launchfile. Two launch files are provided in the folder [`launch/`](launch/).

**NOTE** : all launchfile options are described in the launchfile itself

#### Pipeline use

##### With a Velodyne Lidar
To use a Velodyne lidar with the SLAM, install the Velodyne driver and use the velodyne launch file.

- To install the Velodyne driver, run :
```bash
sudo apt install ros-$ROS_DISTRO-velodyne
```

- To start SLAM when replaying a velodyne rosbag file, run :
```bash
ros2 launch lidar_slam slam_velodyne.launch.py   # in 1st shell
ros2 bag play --clock <my_bag_file>  # in 2nd shell
```
- When using it in real live conditions, use :
```bash
ros2 launch lidar_slam slam_velodyne.launch.py use_sim_time:=false
```

##### With an Ouster Lidar
To use an Ouster lidar with the SLAM, install the Ouster driver and use the ouster launch file.

**NOTE** : Use this [commit id](https://github.com/ouster-lidar/ouster-ros/commit/6c9b3f514c7e4c0c8e1240c47c1c2ecbd3c7a365) of the ouster driver.

- To install the Ouster driver, run :
```bash
cd colcon_ws
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros -b ros2 src/ouster-lidar
# Change to the right commit id
cd src/ouster-lidar
git checkout 6c9b3f514c7e4c0c8e1240c47c1c2ecbd3c7a365
# Build the driver
cd ../..
colcon build --base-paths src/ouster-lidar
# Source the package to use it
source ./install/setup.bash
```

- To start SLAM when replaying a ouster rosbag file, run :
```bash
ros2 launch lidar_slam slam_ouster.launch.py replay:=true os_driver:=true # in 1st shell
ros2 bag play --clock <my_bag_file>  # in 2nd shell
```
- When using it in real live conditions :

Change the sensor_hostname parameter to your ip address in the file ros2_wrapping/lidar_slam/params/ouster_driver_parameters.yaml.
Then lauch the slam with the ouster driver on a live usage
```bash
ros2 launch lidar_slam slam_ouster.launch.py os_driver:=true replay:=false
```

You can also use your own ouster driver parameters file with
```bash
ros2 launch lidar_slam slam_ouster.launch.py os_driver:=true replay:=false driver_parameter_file:="path/to/driver_parameters.yaml"
```

**NOTE** : If you don't find the IP address of your lidar, you can remap it using :
```bash
# Find ethernet interface of Ouster Lidar
ip a
# Give an ip address to Ouster Lidar interface
sudo route add ipv4_addr interf_eth
```

These launch files will start :

* the *lidar_slam_node* which implements the SLAM process,
* a pre-configured RViz session,
* the *lidar_conversion_node* which converts the driver point type to expected SLAM use (see next paragraph),
* (optional) The Lidar drivers if required (see velodyne_driver and os_driver parameters),
* (optional) GPS/UTM conversions nodes to publish SLAM pose as a GPS coordinate in WGS84 format (if `gps` arg is enabled). This uses the prior that full GPS pose and GPS/LiDAR calibration are correctly known and set (see [GPS/SLAM calibration](#gpsslam-calibration) section below for more info).

##### With other LiDAR
**WARNING** : We recommend to use the specific launch files explained above if you have the mentionned Lidar.

We've implemented a generic conversion node with a (future) generic launch file to allow the use of many more
types of Lidar. Thanks to it, you can use any Lidar that publishes a PointCloud2 message with the following fields :
- **x**, **y**, **z** (`float`) : point coordinates
- optionnally **intensity** (or **reflectivity**), **laser_id** (or **ring**), **time** (or **t**) of any type (see 8 types [`here`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html)).

**NOTE** : The generic conversion node will be slower than the specific conversion nodes (as it tests the presence of the fields and estimates missing fields (for time and laser_id))

To use the generic conversion node and launch lidar_slam, you can use the following commands :
```bash
ros2 run lidar_conversions generic_conversion_node  # Run generic conversion node
ros2 launch lidar_slam slam_velodyne.launch.py # Launch any slam launch file
ros2 bag play --clock path/to/bag --remap /points_topic_name:=/generic_points # Play ros bag
```

We advise to update the ROS parameters (**nb_lasers**, **possible_frequencies**) to ease the computations during the conversion (see [`conversion_config.yaml`](src/ros2_wrapping/lidar_conversions/params/conversion_config.yaml)).

#### Choosing a domain ID

If you have several computers running ROS2 on a same network, there could be some interference between different groups of computers. You can manage the communication of ROS 2 nodes by setting the domain ID. ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot. The value of domain ID is between 0 and 101, inclusive. The default value is 0 for all ROS 2 nodes. You can set it by:

```bash
ros2 launch lidar_slam slam_launch_file.py domain_id:=1 # in 1st shell
ROS_DOMAIN_ID=1 ros2 bag play --clock <my_bag_file>  # in 2nd shell
```

### More advanced usage

#### Detailed pipeline

The SLAM node subscribes to one or several input pointclouds topics (default single topic is *lidar_points*) as *sensor_msgs/PointCloud2* messages. These pointclouds should have the following fields:
- **x**, **y**, **z** (`float`) : point coordinates
- **time** (`double`) : time offset to add to the pointcloud header timestamp to get approximate point-wise acquisition timestamp
- **intensity** (`float`) : intensity/reflectivity of the point
- **laser_id** (`uint16`) : numeric identifier of the laser ring that shot this point. The lowest/bottom laser ring should be 0, and it should increase upward.
- **label** (`uint16`) : optional input, not yet used.

If your LiDAR driver does not output such data, you can use the `lidar_conversions` nodes.

SLAM outputs can also be configured out to publish :
- current pose as an *nav_msgs/msg/Odometry* message on topic '*slam_odom*' and/or a TF from '*odometry_frame*' to '*tracking_frame*';
- extracted keypoints from current frame as *sensor_msgs/msg/PointCloud2* on topics '*keypoints/{edges,planes,blobs}*';
- keypoints maps as *sensor_msgs/msg/PointCloud2* on topics '*maps/{edges,planes,blobs}*';
- Current target keypoints maps (i.e submaps) as *sensor_msgs/msg/PointCloud2* on topics '*submaps/{edges,planes,blobs}*';
- registered and undistorted point cloud from current frame, in odometry frame, as *sensor_msgs/msg/PointCloud2* on topic '*slam_registered_points*';
- confidence estimations on pose output, as *lidar_slam/msg/Confidence* custom message on topic '*slam_confidence*'. It contains the pose covariance, an overlap estimation, the number of matched keypoints, a binary estimator to check motion limitations and the computation time.

**NOTE** : It is possible to track any *tracking_frame* in *odometry_frame*, using a pointcloud expressed in an *lidar_frame*. However, please ensure that a valid TF tree is beeing published to link *lidar_frame* to *tracking_frame*.

#### Multiple LiDAR sensors

When there are multiple LiDAR devices, frames coming from different devices should be collected. There are two collection modes: _waiting for all lidars_ or _waiting for a time duration_.

A SLAM iteration runs when:
* Only one LiDAR sensor is implied,
* Frames collection mode is "waiting for all LiDAR sensors" and frames from all lidar devices have arrived,
* Frames collection mode is "waiting for a time duration" and interval is greater than waiting time (0.2s).

The **device_id** of LiDAR sensors need to be precised when setting keypoint extraction parameters.

#### Online configuration
Some features are available online. Note that an interface for some of these features is available in the rviz visualization plugin.

**WARNING** : The slam_visualization plugin is not available on Windows

![Slam Control Panel](<doc/Slam_command_list.png>)

##### Reset state
At any time, the SLAM state can be reset, meaning the maps, the trajectory and the external sensor data are cleared, and all metrics are reset as if it were the first frame. This also disables the recovery mode.

```bash
ros2 service call /lidar_slam/reset lidar_slam/srv/Reset
```

##### Set initial state
At any time, one can set an initial pose and/or load an initial pose by calling `/lidar_slam/set_initial_state` service. This will update the ROS parameters _maps.initial_pose_ and _maps.initial_maps_.

**WARNING** : Calling this service will automatically reset the SLAM state. If the aggregation node is enabled, you must also reset it manually using its own service. For more information, see the [aggregation node](#classic-aggregation) section.

Service fields:
- `initial_pose` [geometry_msgs/PoseWithCovarianceStamped]: The initial pose to set for SLAM
- `map_prefix_path` [string]: Path prefix to the initial map files (used to load PCD files)
- `keep_label` [bool, default: false]: If "true", preserves the original point labels in initial maps; if "false" and the map update mode is "NONE" or "ADD_KEPTS_TO_FIXED_MAP", initial maps are loaded as fixed points.
- `override_timestamp` [bool, default: false]: If true, overrides the timestamp of loaded maps with the current time + `time_offset`.
- `time_offset` [float64; default: 0.0]: Time offset in seconds added to the current time when overriding timestamps.


To set both initial pose **and** initial map for slam:

```bash
ros2 service call /lidar_slam/set_initial_state lidar_slam/srv/SetInitialState "{initial_pose: {header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}, map_prefix_path: '/path/to/initial_map_prefix'}"
```
OR to set **only** initial pose or initial map:
```bash
ros2 service call /lidar_slam/set_initial_state lidar_slam/srv/SetInitialState "{initial_pose: {header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'}"
# OR
ros2 service call /lidar_slam/set_initial_state lidar_slam/srv/SetInitialState "{map_prefix_path: '/path/to/initial_map_prefix'}"
```
**NOTE**: The frame_id in initial_pose.header.frame_id must be set correctly (e.g., odom).
If left unset or incorrect, the pose will be ignored.

##### Reset ODOM
At any time, ODOM can be reset so the current pose is null in ODOM. This can reduce numerical instability.
This should be used regularly in outside environment every 200m for example.

```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 13"
```

##### Map update modes

At any time, commands `lidar_slam/msg/SlamCommand/DISABLE_SLAM_MAP_UPDATE`, `lidar_slam/msg/SlamCommand/ENABLE_SLAM_MAP_EXPANSION` and `lidar_slam/msg/SlamCommand/ENABLE_SLAM_MAP_UPDATE` can be published to '*slam_command*' topic as *lidar_slam/msg/SlamCommand* to change SLAM map update mode.
- 8 :`DISABLE_SLAM_MAP_UPDATE` : when an initial map is loaded, it is kept untouched through the SLAM process.
- 9 : `ENABLE_SLAM_MAP_EXPANSION` : when an initial map is loaded, its points are remained untouched but new points can be added if they lay in an unexplored area
- 10 : `ENABLE_SLAM_MAP_UPDATE` : the map is updated at any time

_NOTE_ : if no initial map is loaded, ENABLE_SLAM_MAP_EXPANSION and ENABLE_SLAM_MAP_UPDATE will have the same effect.

Example to disable map update:
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 8"
```

##### Reset the trajectory
You can reset the slam trajectory with a trajectory CSV file, the map will be updated with the new trajectory.
This file should contain fields "time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2" which represents the time and the transformation of a pose.

Example:
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 11, string_arg: path/to/slamTrajectory.csv}"
```

##### Save the current trajectory
At any time, the logged poses can be saved in a trajectory CSV file. This file contains several fields to represent the time and the transformation:
* Index
* Time (in seconds)
* X
* Y
* Z
* rot(0,0)
* rot(1,0)
* ...
* rot(3,3)

**WARNING** Note that the rotation is represented in column major.

The first line of the file contains the frame ID of the pose that is tracked.
The second line is a header line with the field names.

One can save the trajectory of the base frame :
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 14, string_arg: /path/to/traj/traj.csv}"
```
or the trajectory of the LiDAR sensor :
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 15, string_arg: /path/to/traj/traj.csv}"
```

##### Save maps

The current keypoints maps can be saved as `.pcd` files by calling `/lidar_slam/save_pc` service. The request parameters control output behavior:
- `output_prefix_path` [string]: path prefix to save the maps (e.g. /path/to/maps/prefix)
-  `format` [int32]: 0 = ASCII, 1 = binary, 2 = binary compressed
- `filtered` [bool]:  true to remove potential dynamic objects, false to save all points
- `fixed` [bool, default: false]: true to label all points as fixed when saving maps

To save the full keypoints maps:

```bash
ros2 service call /lidar_slam/save_pc lidar_slam/srv/SavePc "{output_prefix_path: /path/to/maps/prefix, format: 2, filtered: false}"
```
OR to save the filtered keypoints maps:
```bash
ros2 service call /lidar_slam/save_pc lidar_slam/srv/SavePc "{output_prefix_path: /path/to/maps/prefix, format: 2, filtered: true}"
```

The `Save maps` button in the SLAM Control Panel performs the same service call to save filtered keypoints maps. If the aggregation node is enabled, it also calls `/aggregation/save_pc` to save the dense aggregated map. For details on saving the dense map, refer to the section [aggregation node](#classic-aggregation).

##### Set pose
At any time, a pose message (`PoseWithCovarianceStamped`) can be sent through the topic `set_pose` to reset the current pose
```bash
ros2 topic pub -1 /set_pose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

##### Switch ON/OFF the process
At any time, the SLAM can be switched ON/OFF using the command message :
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
```
This disables the sensor messages handling of the node.

##### Switch ON/OFF the external sensors
At any time, the data reception can be enabled/disabled for any sensor using the command message :
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 25, string_arg: idSensor}"
```
with idSensor being:
* WHEEL_ENCODER : 0
* GPS : 3
* POSE : 4
* CAMERA : 5

##### Calibrate with external pose sensor
Another command allows to find the calibration transform between the current tracked pose and the pose tracked by an external sensor. The poses can be sent as a PoseStampedWithCovariance message in the topic `ext_poses` or be loaded from a CSV file. This CSV file must have the same format as the one described in [this section](#save-the-current-trajectory).

```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 30, string_arg: /path/to/external/poses.csv}"
```

This command allows to estimate the calibration and to send a static TF transform between the base frame and the frame specified in the CSV file.

If poses have been sent through `ext_poses` topic, the string_arg should remain empty :

```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 30"
```

Then, the TF is published between the tracked frame and the frame ID "ins".

The calibration has 3 parameters :
* *planar_trajectory* : if the trajectory you use to calibrate is planar, this leads to a lack of a degree of liberty and some "invented" values can appear in the calibration matrix. Setting this parameter to true allows to remove the part of the calibration that was overfitted on the noise of the planar trajectory. The trajectory does not need to be on x,y plane.
* *lever_arm* : this is the distance between the center of the frame that is tracked by the SLAM and the center of the frame of the external sensor that supplied the trajectory.
* *window* : this is the trajectory portion on which to compute the relative motion that will be compared to constrain the calibration during optimization. If the window is large, the drift on each trajectory can impact the result but the local noise will not impact so much the result.

#### Failure detection

Some metrics are available to evaluate the SLAM output. They include:
* Overlap estimation of the current frame on the map. A gap of overlap between successive frames can be suspicious
* The compliance of motion limits. The thresholds must be set by the user depending of the acquisition setup
* The number of keypoints matched. A weak number of matches can be suspicious.
* The standard deviation of the position error. This is derived from the covariance of the optimization output. It can be useful to detect a lack of degree of liberty. Warning, it is not robust to failure cases

Another feature has been developped to fuse the confidence estimators to trigger a failure. The failure cases that can be detected include :
- Map doubling, due to an isolated high motion, a temporal big occlusion or to quick scene change (e.g. door crossing)
- Lack of degree of liberty (e.g. corridor case)
- Divergence due to a combination of external factors

These metrics can be visualized on RVIZ using the visualization plugin.

![Slam metric visualizator](<doc/Slam_confidence_estimator.png>)

To enable this feature, you should turn on *failure_detector/enable* in the SLAM parameters.

In case a failure is detected, the filter enters a recovery mode. This mode fixes the map and the trajectory to an older state and automatically updates some of the parameters to allow a bigger motion and a longer computation time. The user should go back to a previous pose to try to be relocalized and get out of this mode to go on with his acquisition without breaking the map. He can also update the parameters for the specific trajectory section that has gone wrong.

**/!\ Warning** : for now, going back to a previous pose includes orientation, so, mind your acquisition direction when looking for recovery.

If this feature is disabled during recovery, the state is reset as before the recovery mode has been triggered. Therefore, if you see the SLAM is relocalized but the recovery mode is still on (the confidence thresholds have not been reached), you can disable the failure detection and reenable it later on to force going out of the recovery mode.

**/!\ Warning** : in recovery mode, some of the parameters are modified. Namely, the *map update mode*, the *ego-motion mode*, the *undistortion mode*, the *maximum number of ICP iterations*, the *maximum distance between nearest neighbors* and the *initial saturation distance* are set to recovery values.
 If you change these parameters during the recovery mode, they will be reset as **before** the recovery mode after relocalization.

#### Optional loop closure use

Loop closure consists in correcting the whole SLAM trajectory when some place is revisited after a period of time. We can use this information to reduce the mapping noise.

To use loop closure constraint for pose graph optimization:
- enable loop closure: `graph.constraint.loop_closure: true`
- set suitable parameters for loop closure: for example, `graph.loop_closure.ICP_max_iter: 100 `
`slam.voxel_grid.decaying_threshold: 50.` (this parameter can avoid a doubled map or oscillated poses when a loop appears)
`logging.timeout: 3000.`

This loop closure is triggered by the user. The user has to specify a revisited frame with which a query frame is closing a loop by loading a file containing loop closure indices. Here is an example of csv file:

```
queryIdx,revisitedIdx
123,12
456,23
343,35
```

To load loop closure indices file, you can click on `Load loop indices` button in rviz or you can use:
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 23, string_arg: path/to/loopClosureIndices.csv}"
```

Then trigger pose graph optimization when needed by clicking on `Optimize graph` button in rviz or using:

```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 20"
```

If loop closure indices are unknown, there are two ways to get them:
* Publish a 3D point ([geometry_msgs::msg::PointStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PointStamped.html)) on *clicked_point* topic, refering to a pose that forms a loop with the current position. This can be done in rviz, selecting `Publish point` and click on the pose.
* Save current trajectory and map. Observe them in a visual tool. Then create a csv file of loop indices. In this way, you can added more than one loop.

**NOTE** : You can save your slam trajectory before launch a pose graph optimization so that you can go back to states before PGO by re-setting slam trajectory.
To save slam trajectory, you can click on `Save trajectory` button in rviz or you can use:
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 14, string_arg: path/to/slamTrajectoryBeforePGO.csv}"
```
Then if you want to reload a slam trajectory, you can click on `Reset trajectory` button in rviz or you can use:
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 11, string_arg: path/to/slamTrajectoryBeforePGO.csv}"
```

## Optional external sensors use

A GPS, a wheel encoder, an IMU, an INS, a camera and/or a tag detector can be used along with the LiDAR SLAM. Full IMU use is available in the slam_lib but its ROS interface has not been implemented for now.

Some external sensors can be used locally (in the local SLAM optimization) or globally as a post process (in a pose graph).
cameras, wheel encoders, IMU are usually used in the first case while GPS, as absolute poses sensor, is usually used in the second case. A tag detector can be used in both cases.

To use them, a time synchronization must be possible. To do so, a parameter *use_header_time* allows to chose whether to use the time coming from the header of the sensors messages or the messages reception times. It is always better to use the header times if the offset between reference times is known and if the sensor times are trustworthy.

Moreover, the data should be stored, at least until the best time fit is found for lidar frame in case of local optimization and to select from when to update the trajectory along with the maps in case of pose graph optimization.

Weights are parameterizable in case of local optimization to calibrate the impact of the sensor relatively to 3D matches built with lidar frames. This weight depends mostly on the sensors accuracy and the confidence that the user must give to it.

Graph constraint and logging must be enabled in case of pose graph optimization.

### GPS

GPS can be used to optimize the pose graph.
To enable the GPS, modify the following parameters :
* `external_sensors.gps.enable: true`
* `external_sensors.max_measures: 1000000` to be determined
* `external_sensors.use_header_time: true` if necessary
* `external_sensors.time_threshold: 0.1` to be determined
* `slam.logging_timeout > t`, t is the duration on which LiDAR measurements can be stored, the oldest measurements are forgotten. Only the remaining poses are used to build and optimize the graph and to rebuild the map.
* `logging/only_keyframes : true`

Then, *LidarSlamNode* subscribes to the GPS odometry on topic '*gps_odom*', and records the most recent GPS positions. To use GPS data, we transform GPS WGS84 fix into cartesian space using UTM projection.

**NOTE** : If GPS odometry expresses the pose of a *gps_frame* different from *tracking_frame*, please ensure a valid calibration static TF is beeing broadcasted.

A possible pipeline is the following :

```bash
# Run the SLAM with gps topic remapping if needed:
ros2 launch lidar_slam slam_velodyne.launch.py # Start SLAM (external pose log must be enabled).
...  # Run 1st real test or bag file
# Stop the acquisition :
# (or pause the system)
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
# Trigger PGO : optimize SLAM trajectory, recompute the map and update last pose
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 20"
# Restart the acquisition :
# (or play the bag again)
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
```

### Tag detector

A tag detector (like a camera with an april tag application) can be used to help the local SLAM process or to optimize a pose graph as post process.

To enable the tag detector, modify the following parameters :
* `external_sensors.landmark_detector.enable: true`
* `external_sensors.max_measures: 1000000` to be determined
* `external_sensors.use_header_time: true` if necessary
* `external_sensors.time_threshold: 0.1` to be determined

#### Local constraint

The landmark detector (e.g. a camera) transform relatively to *tracking_frame* must be supplied as a `TF2 static transform`.

Then, *LidarSlamNode* subscribes to the tag pose on topic '*tag_detections*', and records the most recent tag relative poses as `apriltag_ros` messages.
**NOTE** : You can specify the tag topic directly in the launch command with the argument : `tags_topic:="your_tag_topic"`.

***WARNING***: A virtual package of apriltag_ros exists in this wrapping for compilation issue and must be ignored if one wants to use the actual apriltag_ros package.

Then, a constraint will be built to be included in the local optimization together with the internal geometric constraints.

Various relative parameters are available :
1. `external_sensors.landmark_detector.weight` : floating value to mitigate the tag impact on the optimization relatively to geometric matches.
2. `external_sensors.landmark_detector.position_only=true` : only use the tags' positions and not the orientations provided by the detector. Orientation is usually less accurate.
3. `external_sensors.landmark_detector.saturation_distance` : reject the measurements that seem clearly wrong relatively to the geometric matches.
4. `external_sensors.landmark_detector.publish_tags=true` : publish the tag as TF2 transform for visualization purposes mostly (parameter ).
5. `external_sensors.landmark_detector.landmarks_file_path` : name of a file to load, to initially set absolute poses of the tags. The tag constraints are built relatively to these absolute reference poses and they will never be modified along iterations. If no file is loaded, the reference poses in the constraints are computed using previous detections and are refined along iterations or reset if the tag is not seen in a while. Note that if a file is loaded in mapping mode and some drift appears, it can lead to some jumps and a map distortion. The file format must be csv with one header line : *id,x,y,z,roll,pitch,yaw,cov0,[...],cov35*.

All these parameters are described in the supplied [config files](params).

***WARNING***: Make sure no error occured in the file loading step in the terminal output before supplying data.

#### Pose graph optimization

A pose graph optimization can be performed using the tags to close loops and to create a consistent map. The previous recommendations for landmarks integration in local optimizations hold.

To enable the pose graph, modify the following parameters :
* `graph.constraint.landmark: true`
* `slam.logging_timeout > t`, t is the duration on which LiDAR measurements can be stored, the oldest measurements are forgotten. Only the remaining poses are used to build and optimize the graph and to rebuild the map.
* `logging/only_keyframes : true`

If you want to run SLAM and then to optimize the graph using landmarks, one pipeline can be :

```bash
# Run the SLAM with tag topic remapping :
ros2 launch lidar_slam slam_velodyne.launch.py tags_topic:="your_tag_topic" # Start SLAM (tags use must be enabled).
...  # Run 1st real test or bag file
# Stop the acquisition :
# (or pause the system)
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
# Trigger PGO : optimize SLAM trajectory, recompute the map and update last pose
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 20, string_arg: path/to/landmarksAbsolutePoses.csv}"
# Restart the acquisition :
# (or play the bag again)
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
```

**NOTE** : If the SLAM was running with initially loaded tag poses (`landmarks_file_path` is not empty), and one wants to run the pose graph optimization, the file parameter is not compulsory, one can just launch :
```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 20"
```
Again, running the SLAM in mapping mode with initially loaded tag poses can be dangerous though. If you don't have any absolute tag poses to supply, the last estimated tag poses will be used. It means the map will be updated so it fits the last frames.

### Camera

A camera can be used to add image match based constraints to SLAM local optimization.

To enable the camera, modify the following parameters :
* `external_sensors.camera.enable: true`
* `external_sensors.max_measures: 10` to be determined
* `external_sensors.use_header_time: true` if necessary
* `external_sensors.time_threshold: 0.2` to be determined

Then, *LidarSlamNode* subscribes to RGB images ([sensor_msgs::msg::Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)) on topic '*camera_topic*', and records the most recent images. Note that this feature also needs a subscription to camera_info_topic containing the camera calibration ([sensor_msgs::msg::CameraInfo](https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html)).

**NOTE** : You can specify the camera topics directly in the launch command with the argument : `camera_topic:="your_tag_topic"` and `camera_info_topic:="your_tag_topic"`.

**NOTE** : You can launch a decompression node **republish** contained in *ROS2* default package **image_transport**.

Images can then be used in local optimization if their timestamps are close enough to the Lidar frames' relatively to the threshold : *time_threshold*. To do so, first, the slam points are projected into the corresponding image. Then, an optical flow is computed on the extracted pixels and the image frame corresponding to next lidar frame (see [opencv documentation](https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html) for more details about optical flow computation). Finally, a constraint between pixels of the optical flow is built and added to slam optimization.

A weight `external_sensor.camera.weight`,  is parameterizable for the new camera constraint. It represents the impact of one pixel match relatively to one geometric 3D point match (built from lidar frame) in the optimization. The more we trust the optical flow, the higher this weight should be. This parameter is described in the supplied [config files](params).

If the data are well added and synchronized, a log output should be visible with verbosity to 3 (default).
This output should say :

```bash
Camera constraint added
```

### Wheel encoder

The wheel encoder measurement can be used in the slam front end optimization to solve some degree of liberty (e.g. in corridors).

To enable the wheel encoder, modify the following parameters :
* `external_sensors.wheel_encoder.enable: true`
* `external_sensors.max_measures: 100000` to be determined
* `external_sensors.use_header_time: true` if necessary
* `external_sensors.time_threshold: 0.1` to be determined

Then, the *LidarSlamNode* subscribes to odometer messages (std::msgs::Float64) in the topic called **wheel_odom**.
The calibration (i.e. the transform between the frame **wheel** and the tracking frame which is **base_link** by default) must be sent to the TF tree to be able to receive any data.

***WARNING***: As the message does not contain time, the message reception time is used to synchronize with LiDAR data. Therefore, *use_header_time* must be turned to false.

***WARNING***: Remember to set *max_measures*, *time_threshold* to convenient values to be able to receive the measurements.

Two types of constraint are allowed : *relative*/*from reference*.

* The relative constraint ensures the distance provided by the wheel encoder between two successive frames is the norm of the translation between those frames. In a corridor, if the distances are small, the translation might be applied in one direction or the other and the trajectory might faultly hover. Therefore, in a straight line trajectory case, a motion *direction* (3D) can be set to help the optimization choose a side. This *direction* must be represented in World frame.

* The other constraint allows to notify a distance from a point. This can be useful for a laser measuring a distance from a target or if a wire is fixed to tracking frame with a wheel encoder to measure it. The constraint will ensure that at each new frame the translation norm from the reference point is exactly the distance measurement. A parameter allows to set the reference point relatively to the wheel encoder in the wheel encoder frame. If no reference is set but the relative mode is disabled, the first couple pose/wheel encoder measurement is used to define the reference point.

A weight, `external_sensor.wheel_encoder.weight`, is parameterizable for the new wheel encoder constraint. It represents the impact of the wheel encoder constraint with respect to all the points distance from the SLAM optimization. If 0., the feature is disabled.

If the data are well added and synchronized, a log output should be visible with verbosity to 3 (default).
This output should say :

```bash
Adding wheel encoder residual : W m travelled since position : X,Y,Z...
Wheel odometry constraint added
```

### External pose

External poses can be used in local SLAM optimization or in pose graph. They can come from a INS/GNSS pair or another SLAM source for example.

These poses can be used for different purposes:
1. Undistort the input pointcloud
2. Provide a prior pose
3. Add a constraint in the optimization
4. Be used in a pose graph as postprocess

The external poses can be supplied through a message `geometry_msgs::msg::PoseWithCovarianceStamped` in the topic *ext_poses*. The calibration between the tracked frame and the frame called "ins" must be sent to the TF tree to be able to receive the measurements. Note that the poses can be supplied as a CSV file to optimize the graph (see Pose graph optimization section).

To enable the wheel encoder, modify the following parameters :
* `external_sensors.external_poses.enable: true`
* `external_sensors.max_measures: 100000` to be determined
* `external_sensors.use_header_time: true` if necessary
* `external_sensors.time_threshold: 0.1` to be determined

#### Local constraint

To use the external poses to undistort the pointcloud, the undistortion mode must be set to 3. For high frequency motion, this can be really useful to get a cleaner map.

To use them as prior pose, the egomotion mode must be set to 4 or 5. This can make the SLAM more robust for not smooth motion.

To use them as an optimization constraint, the weight must be greater than 0. This can be useful in unconstrained environment.

If the data are well added and synchronized, a log output should be visible with verbosity to 3 (default).
This output should say :

1.
```bash
Undistortion performed using external poses interpolation
```

2.
```bash
Prior pose computed using external poses supplied
```

3.
```bash
External pose constraint added
```

#### Pose graph optimization

The external poses can be used to optimize a pose graph. Those poses can be received as described in previous sections or they can be loaded at some point from a CSV file using the command LOAD_POSES:

```bash
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "{command: 40, string_arg: path/to/trajectory.csv}"
```

This CSV file must have a header of 2 lines : the first line contains the frame ID and the second line the field names "time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2". xi corresponds to the first element of the ith colum of the rotation matrix.
The calibration between the tracked frame and the frame defined in the header of the message must be sent to the TF tree. If it is not found, the calibration is set to identity.

***WARNING***: Remember to set *max_measures*, and *time_threshold* to convenient values to be able to use the pose measurements.

Modify the following parameters :

* `graph.constraint.ext_poses: true`
* `slam.logging_timeout > t`, t is the duration on which LiDAR measurements can be stored, the oldest measurements are forgotten. Only the remaining poses are used to build and optimize the graph and to rebuild the map.
* `logging/only_keyframes : true`

A possible pipeline is the following :

```bash
# Run the SLAM with external topic remapping :
ros2 launch lidar_slam slam_velodyne.py ext_poses:="your_ext_pose_topic" # Start SLAM (external pose log must be enabled).
...  # Run 1st real test or bag file
# Stop the acquisition :
# (or pause the system)
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
# Trigger PGO : optimize SLAM trajectory, recompute the map and update last pose
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 20"
# Restart the acquisition :
# (or play the bag again)
ros2 topic pub -1 /slam_command lidar_slam/msg/SlamCommand "command: 0"
```

**Tip** : save the maps and the trajectory before optimizing the graph to be sure to be able to come back if something goes wrong with the optimization.

### IMU

If IMU enabled, *LidarSlamNode* subscribes to [IMU messages](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html) in the topic called "imu".

For now, only the linear acceleration is used to apply a gravity constraint between all frames. The gravity constraint can be used in the slam front end optimization to force the orientation of the pose so that the gravity vector corresponds to a reference.

***WARNING***: Remember to set *max_measures*, *use_header_time* and *time_threshold* to convenient values to be able to receive the measurements.

A weight is parameterizable for the gravity constraint. The more we trust the IMU measurement, the higher this weight should be. If 0., the feature is disabled.

If the data are well added and synchronized, a log output should be visible with verbosity to 3 (default).
This output should say :

```bash
Adding gravity residual with gravity reference :  ...
IMU gravity constraint added
```

## About the published TF tree

Here is an example of the complete TF tree that can be maintained by different nodes as well as descriptions of each frame (default frame names) :

```bash
utm
└─ enu
   └─ map
      └─ odom
         └─ base_link
            ├─ lidar
            ├─ landmark_detector
            ├─ ins
            ├─ camera
            ├─ imu
            └─ gps
```

- **utm**: "world" ENU fixed frame, corresponding to the origin of the current considered UTM zone/band in which GPS coordinates are projected into.
- **enu**: local ENU fixed frame attached to 1st received GPS position in UTM coordinates, easier to use than **utm** because UTM coordinates can grow very large, leading to floating points discretization errors. The static TF `utm -> enu` is published by `gps_conversions/gps_to_utm` node.
- **map**: first received full 6D GPS pose. It defines the origin of the local map. If GPS does not provide orientation, pitch and heading can be estimated from motion. The static TF `enu -> map` is published by `gps_conversions/gps_to_utm` node.
- **odom**: origin of the SLAM. The TF `map -> odom` (not static) can be published by a custom node, or it will be published by `lidar_slam/lidar_slam_node` node after PGO.
- **base_link**: current pose computed by SLAM algorithm (here `base_link` is the tracking frame). The TF `odom -> base_link` can be published by `lidar_slam/lidar_slam_node` node.
- **lidar**: pose of the LiDAR sensor on the moving base. The TF `base_link -> lidar` should be published by a `tf2_ros/static_transform_publisher` node.
- **landmark_detector**: pose of the landmark detector on the moving base. The TF `base_link -> landmark_detector` must be published by a `tf2_ros/static_transform_publisher` node.
- **gps**: pose of the GPS sensor in the moving base. The TF `base_link -> gps` should be published by a `tf2_ros/static_transform_publisher` node.
- **ins**: pose of the INS sensor in the moving base. The TF `base_link -> ins` should be published by a `tf2_ros/static_transform_publisher` node.
- **imu**: pose of the IMU sensor in the moving base. The TF `base_link -> imu` should be published by a `tf2_ros/static_transform_publisher` node.
- **wheel**: pose of the wheel encoder in the moving base. The TF `base_link -> wheel` should be published by a `tf2_ros/static_transform_publisher` node.
- **camera**: pose of the camera in the moving base. The TF `base_link -> camera` should be published by a `tf2_ros/static_transform_publisher` node.

# Points aggregation

Another node called **aggregation_node** is included in the **lidar_slam** package and allows to aggregate all points from all frames into a unique pointcloud with a defined resolution : the points are stored in a voxel grid to supply a downsampled output cloud. It also allows to reject moving objects. The output pointcloud is published on the topic *aggregated_cloud*. It requires the output *registered_points* of the node **lidar_slam_node** to be enabled.

## Classic aggregation

**aggregation_node** has 4 parameters :

- *leaf_size* : corresponds to the size of a voxel in meters in which to store one unique point. It is equivalent to the required mean distance between nearest neighbors. The maximum distance between nearest distance to downsample the cloud would be 2 * leaf_size
- *max_size* : corresponds to the maximum size of the voxel grid and so the maximum width of the output cloud to tackle possible memory issues.
- *min_points_per_voxel* : corresponds to the minimum number of frames which should have a point in a voxel to consider this voxel is not a moving object. It has been seen more than *min_points_per_voxel*, so it is considered "enough" to be a static object. All voxels that have been seen less than *min_points_per_voxel* times are not included to the output cloud.
- *min_dist_around_trajectory* : If positive, it is the distance (in meters) from each SLAM pose under which no point will be added to the aggregated map. It allows to remove moving objects passing through trajectory. Hypothesis is : if the robot/person has been there, no point should be added.

This node can answer to a service called `/aggregation/save_pc` to save the pointcloud on disk as a PCD file. The command should be :

```bash
ros2 service call /aggregation/save_pc lidar_slam/srv/SavePc "{output_prefix_path: 'prefix/path/where/to/save/the/cloud', format: 2}"
```
to save the pointcloud as a PCD Binary compressed file at *prefix/path/where/to/save/the/cloud_CurrentTime.pcd*.

All possible formats are :
- **ASCII** : 0
- **Binary** : 1
- **Binary compressed** : 2

Another service called **reset** allows to reset the ongoing map with current input.
```bash
ros2 service call /aggregation/reset lidar_slam/srv/Reset
```

**NOTE** : **aggregation_node** can be directly run from the launch files, adding the argument *aggregate:=true* to the launch command.

**NOTE** : The services are available through the visualization plugin.

## Advanced

### Vertical slice extraction and analysis

One can extract a slice of points perpendicular to the trajectory locally and to create a boundary from it. An area estimation is then provided. This can be useful when exploring closed areas such than undergrounds.

The area is published on the topic **/slice_area** as a [float64 std message](https://docs.ros.org/en/api/std_msgs/html/msg/Float64.html) and
the boundary pointcloud is published on the topic **/slice_points**.

To create a boundary from the slice, the slice points are first projected onto the slice plane and a circular histogram is created centered onto the current pose of the trajectory. Then, an average allows to keep one point per bin. The area is estimated as the sum of the areas of the triangles formed by the boundary points and the current position. The current position  is supposed to be inside of the boundary.

The parameters relative to this feature are the following:
*slice/enable*: Boolean to enable/disable the slice extraction and the area computation.
*slice/traj_length*: Length (in meters) of the local trajectory considered to define the slice plane (perpendicular to it).
*slice/width*: Width (in meters) of the slice of points
*slice/max_dist*: Maximal distance (in meters) of the points from the trajectory position onto the slice plane.
*slice/angle_resolution*: Resolution (in degrees) of the slice pointcloud w.r.t the trajectory position.

### Horizontal slice extraction/rejection

One can extract a slice of points relatively to the z axis of the base_link frame.
**Note** : to extract/reject the ground, one has to adapt the frame base_link to be horizontal with respect to the floor. This can be done in the tf config in the launch file or changing the tracked frame in the parameters file.

The parameters relative to this feature are the following:
* *z_slice/enable*: Boolean to enable/disable the horizontal slice extraction/rejection.
* *z_slice/height_position*: Position on the z axis (in meters) of the base_link frame (most of the time, this will be negative)
* *z_slice/width*: Width (in meters)
* *z_slice/invert*: To reject the slice instead of extracting it

### Obstacles detection

It is possible to use the aggregation node to extract the obstacles relatively to a known map.
The idea is to map an environment with a common SLAM using classic aggregation. At the end of the mapping process, the map can be saved on disk. Then this reference map can be loaded, the objects that are now detected and which don't belong to the reference map will be extracted and outputed in the */aggregated_cloud* topic. The label field of the points will represent their cluster.
An occupancy grid with the clusters is built during the process and sent to the topic *obstacles/occupancy_grid*. Markers are also sent on the topic */obstacles/bboxes*. Both can be displayed in rviz.

The parameters relative to this feature are the following:
* *obstacle.enable*: Boolean to enable/disable the obstacle detection.
* *obstacle.ref_map_path:* Reference map to find new objects
* *obstacle.decay_time*: Maximum lifetime (in seconds) of an obstacle and/or a new point
* *obstacle.publish_occupancy_grid*: Whether or not to publish the occupancy grid as debug info. **Warning**: this can add significant computation time.
* *min_marker_size*: Minimal diagonal size of a marker (in meters) to consider it is an obstacle
