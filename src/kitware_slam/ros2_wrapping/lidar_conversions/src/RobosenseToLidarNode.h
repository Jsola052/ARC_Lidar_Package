//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2022-09-02
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//==============================================================================

#pragma once

#include <rclcpp/rclcpp.hpp>
// #include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <LidarSlam/LidarPoint.h>
#include <lidar_conversions/srv/estim_params.hpp>
#include "Utilities.h"

namespace lidar_conversions
{

/**
 * @class RobosenseToLidarNode aims at converting pointclouds published by RSLidar
 * ROS driver to the expected SLAM pointcloud format.
 *
 * The ROS RSLidar driver can be found here :
 * https://github.com/RoboSense-LiDAR/ros_rslidar
 */
class RobosenseToLidarNode: public rclcpp::Node
{
public:
  using PointRS = pcl::PointXYZI;
  using CloudRS = pcl::PointCloud<PointRS>;  ///< Pointcloud published by rslidar driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param node_name Name of the node created
   * @param[in] options Options of the node, default no options
   */
  RobosenseToLidarNode(const std::string node_name,
                       const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing RSLidar PointCloud as SLAM LidarPoint.
   * @param msg_received New Lidar Frame, published by rslidar_pointcloud/cloud_node.
   */
  void Callback(const Pcl2_msg& msg_received);

  //----------------------------------------------------------------------------
  /*!
   * @brief Service to re-compute the estimation parameters of the conversion node.
   * @param request Service request
   * @param response Service response
   */
  void EstimParamsService(const std::shared_ptr<lidar_conversions::srv::EstimParams::Request> req,
                          const std::shared_ptr<lidar_conversions::srv::EstimParams::Response> res);

private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber, publisher and service
  // ros::NodeHandle &Nh, &PrivNh;
  rclcpp::Subscription<Pcl2_msg>::SharedPtr Listener;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr Talker;
  rclcpp::Service<lidar_conversions::srv::EstimParams>::SharedPtr EstimService;

  unsigned int NbLasers = 16; ///< Number of lasers of the LiDAR. Optional as it can be taken from header attribute .height of the PointCloud.
  bool RotSenseAndClustersEstimated = false; ///< Flag to initialize the parameters useful for laser_id and time estimations.
  bool RotationIsClockwise;  ///< True if the LiDAR rotates clockwise, false otherwise.

  // Useful variable to estimate rotation duration (itself used to estimate time)
  // NOTE: to be precise, this rotation duration estimation requires that each input
  // scan is an entire scan covering excatly 360°
  double RotationDuration = -1.;
  double RotationDurationPrior = -1.;
  double PrevFrameTime = -1.;
  std::vector<double> PossibleFrequencies = {5., 10., 20.}; ///< Vector of all the possible frequencies for robosense LiDAR

  // Useful variable to estimate laser_id
  std::vector<Utils::Cluster> Clusters;

  // Number of threads to use for the conversion
  int NbThreads = 1;
};

}  // end of namespace lidar_conversions
