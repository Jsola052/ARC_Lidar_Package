//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2023-07-10
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
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_point.h>
#include <LidarSlam/LidarPoint.h>
#include <livox_ros_driver/msg/custom_msg.hpp>

namespace lidar_conversions
{

/**
 * @class LivoxToLidarNode aims at converting pointclouds published by ROS
 * livox driver to the expected SLAM pointcloud format.
 *
 * The ROS livox driver can be found here :
 * https://github.com/Livox-SDK/livox_ros_driver
 */
class LivoxToLidarNode : public rclcpp::Node
{
public:
  using PointL = pcl::PointXYZI;
  using CloudL = pcl::PointCloud<PointL>;  ///< Pointcloud published by livox driver
  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2;
  using LivoxCustomMsg = livox_ros_driver::msg::CustomMsg;
  using LivoxCustomPoint = livox_ros_driver::msg::CustomPoint;

  //----------------------------------------------------------------------------
  /*!
   * @brief Constructor.
   * @param node_name Name of the node created
   * @param options Options of the node, default no options
   */
  LivoxToLidarNode(const std::string node_name,
                   const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback with Custom Livox message (used for horizon), converting and publishing Livox PointCloud as SLAM LidarPoint.
   * @param msg_received New Lidar Frame, published by livox driver.
   */
  void PointCloud2Callback(const Pcl2_msg& msg_received);

  //----------------------------------------------------------------------------
  /*!
   * @brief New lidar frame callback with Custom Livox message (used for horizon), converting and publishing Livox PointCloud as SLAM LidarPoint.
   * @param cloud New Lidar Frame, published by livox driver.
   */
  void LivoxCustomMsgCallback(const LivoxCustomMsg& cloudLmsg);

private:

  //----------------------------------------------------------------------------

  rclcpp::Subscription<Pcl2_msg>::SharedPtr Pcl2Listener;
  rclcpp::Subscription<LivoxCustomMsg>::SharedPtr LivoxMsgListener;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr Talker;

  bool IsPcl2 = false;
};

}  // end of namespace lidar_conversions
