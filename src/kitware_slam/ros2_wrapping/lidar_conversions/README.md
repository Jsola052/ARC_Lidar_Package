# lidar_conversions

## Description

Helper package to convert raw pointclouds output by common LiDAR drivers to the pointcloud format expected by the SLAM algorithm.

The SLAM algorithm expects input pointclouds as *sensor_msgs/PointCloud2* messages. These pointclouds should have the following fields:
- **x**, **y**, **z** (`float`) : point coordinates
- **time** (`double`) : time offset to add to the pointcloud header timestamp to get approximate point-wise acquisition timestamp
- **intensity** (`float`) : intensity/reflectivity of the point
- **laser_id** (`uint16`) : numeric identifier of the laser ring that shot this point. The lowest/bottom laser ring should be 0, and it should increase upward.
- **label** (`uint8`) : optional input, not yet used.

Especially, the point-wise timestamps are necessary if undistortion is enabled in SLAM. The nodes of this package are able to compute approximate timestamps when those are not available in the input pointcloud.

SLAM expects that the lowest/bottom laser ring is 0, and is increasing upward. If this is not your case, you can use the `laser_id_mapping` to correct it in the output cloud.

Currently, this package implements the following nodes :
- **velodyne_conversion_node** : converts pointclouds output by Velodyne spinning sensors using the [ROS Velodyne driver](https://github.com/ros-drivers/velodyne) to SLAM pointcloud format.
- **robosense_conversion_node** : converts pointclouds output by RoboSense spinning sensors using the [ROS RoboSense-LiDAR driver](https://github.com/RoboSense-LiDAR/ros_rslidar) to SLAM pointcloud format. This has been tested only with RS16 sensor, and could need additional changes to support other RS sensors.
- **ouster_conversion_node** : converts pointclouds output by Ouster spinning sensors using the [ROS Ouster driver](https://github.com/ouster-lidar/ouster_example) to SLAM pointcloud format.
- **livox_conversion_node** : converts custom pointclouds output by Livox spinning sensors using the [ROS Livox driver](https://github.com/Livox-SDK/livox_ros_driver) to SLAM pointcloud format.
- **hesai_conversion_node** : converts pointclouds output by Hesai spinning sensors using the [ROS Hesai driver](https://github.com/HesaiTechnology/HesaiLidar_General_ROS/tree/ROS2) to SLAM pointcloud format.
- **generic_conversion_node** : converts any PointCloud2 to SLAM pointcloud format. It can recognize multiple names and types for the fields of PointCloud2, and if some are missing (like laser_id or time), it recomputes them using space coordinates only.
As it looks for multiple types/names, if the user gives a complete PointCloud2, the average conversion time will be doubled compared to using a specific conversion node. We advise to use one the nodes above if the user is certain of his PointCloud2 (types and names of each field included).
If the user is unable to provide all the necessary fields required for SLAM, we advise to use this node. It will automatically compute the missing information.

**WARNING** : Using generic_converison_node will multiply by (at least) 1.5 the time for conversion, as required for the field searching. The autoestimation of missing fields will multiply by 2 the time for conversion (compared to a specific conversion node).

## Usage

### Direct usage :

```bash
ros2 run lidar_conversions velodyne_conversion_node
```

### Direct usage with user invention to compute estimation parameters again (in case laser_id and/or time look unusual) :
To be done in two seperate terminals:

1:
```bash
ros2 run lidar_conversions velodyne_conversion_node
```

2:
- if you want to compute estimation parameters again for Velodyne or Ouster :
```bash
ros2 service call lidar_conversions/estim_sense lidar_conversions/srv/EstimSense
```
- if you want to compute estimation parameters again for any other node (Robosense, Livox, Generic) :
```bash
ros2 service call lidar_conversions/estim_params lidar_conversions/srv/EstimParams
```

### Example of launchfile for a multi-lidar setup:

```xml
<launch>
  <ros2param from="/lidar_conversions" command="load" file="$(find lidar_conversions)/params/conversion_config.yaml"/>

  <node name="velodyne_conversion" pkg="lidar_conversions" type="velodyne_conversion_node" output="screen">
    <remap from="lidar_points" to="velodyne_lidar_points"/>
  </node>

  <node name="robosense_conversion" pkg="lidar_conversions" type="robosense_conversion_node" output="screen">
    <remap from="lidar_points" to="robosense_lidar_points"/>
  </node>
</launch>
```