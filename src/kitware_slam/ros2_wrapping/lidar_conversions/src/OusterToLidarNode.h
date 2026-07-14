//==============================================================================
// Copyright 2021-2022 Kitware, Inc., Kitware SAS
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
#include <os_point.h>
#include <LidarSlam/LidarPoint.h>
#include <lidar_conversions/srv/estim_sense.hpp>
#include "Utilities.h"

namespace lidar_conversions
{

/**
 * @class OusterToLidarNode aims at converting pointclouds published by ROS
 * Ouster driver to the expected SLAM pointcloud format.
 *
 * The ROS Ouster driver can be found here :
 * https://github.com/ouster-lidar/ouster-ros/tree/ros2
 */
class OusterToLidarNode : public rclcpp::Node
{
public:
  using PointO = ouster_ros::Point;
  using CloudO = pcl::PointCloud<PointO>;  ///< Pointcloud published by ouster driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param node_name name of the node created
   * @param[in] options Options of the node, default no options
   */
  OusterToLidarNode(const std::string node_name,
                    const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback, converting and publishing Ouster PointCloud as SLAM LidarPoint.
   * @param msg_received New Lidar Frame from an Ouster lidar
   */
  void Callback(const Pcl2_msg& msg_received);

  //----------------------------------------------------------------------------
  /*!
   * @brief Service to re-estimate the rotation sense of the LiDAR.
    * @param request Service request
    * @param response Service response
   */
  void EstimSenseService(const std::shared_ptr<lidar_conversions::srv::EstimSense::Request> request,
                         const std::shared_ptr<lidar_conversions::srv::EstimSense::Response> response);


private:

  //----------------------------------------------------------------------------

  // ROS node handles, subscriber, publisher and service
  rclcpp::Subscription<Pcl2_msg>::SharedPtr Listener;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr Talker;
  rclcpp::Service<lidar_conversions::srv::EstimSense>::SharedPtr EstimService;

  unsigned int NbLasers = 64;  ///< Minimal number of lasers in the LiDAR
  bool RotationSenseEstimated = false; ///< Flag to initialize the parameters useful for time estimations.
  bool RotationIsClockwise;  ///< True if the LiDAR rotates clockwise, false otherwise.

  // Useful variable to estimate RotationDuration (itself used to estimate time)
  // NOTE: to be precise, this RotationDuration estimation requires that each input
  // scan is an entire scan covering excatly 360°
  double RotationDuration = -1.;
  double RotationDurationPrior = -1.;
  double PrevFrameTime = -1.;
  std::vector<double> PossibleFrequencies; ///< Vector of all the possible frequencies for Ouster LiDAR

  // Number of threads to use for the conversion
  int NbThreads = 1;
};

}  // end of namespace lidar_conversions
