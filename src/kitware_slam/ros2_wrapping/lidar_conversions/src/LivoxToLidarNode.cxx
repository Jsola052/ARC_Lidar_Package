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

#include "LivoxToLidarNode.h"
#include "Utilities.h"
#include <pcl_conversions/pcl_conversions.h>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

LivoxToLidarNode::LivoxToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : rclcpp::Node(node_name, options)
{
  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Get LiDAR id
  this->get_parameter("pointcloud2", this->IsPcl2);

  // Init ROS subscriber
  if (this->IsPcl2)
    this->Pcl2Listener = this->create_subscription<Pcl2_msg>("livox/lidar", 2,
                                              std::bind(&LivoxToLidarNode::PointCloud2Callback, this, std::placeholders::_1));
  else
    this->LivoxMsgListener = this->create_subscription<LivoxCustomMsg>("livox/lidar", 1,
                                              std::bind(&LivoxToLidarNode::LivoxCustomMsgCallback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("Livox data converter is ready !"));
}

//------------------------------------------------------------------------------
void LivoxToLidarNode::PointCloud2Callback(const Pcl2_msg& msg_received)
{
  //convertion message to PointCloud
  //doc : https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
  CloudL cloudL;
  pcl::fromROSMsg(msg_received, cloudL);

  // If input cloud is empty, ignore it
  if (cloudL.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Livox pointcloud is empty : frame ignored.");
    return;
  }

  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudL.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudL, cloudS);

  // Build SLAM pointcloud
  double prevTime = -0.1;
  for (const PointL& livoxPoint : cloudL)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(livoxPoint))
      continue;

    PointS slamPoint;
    slamPoint.x = livoxPoint.x;
    slamPoint.y = livoxPoint.y;
    slamPoint.z = livoxPoint.z;
    slamPoint.intensity = livoxPoint.intensity;
    slamPoint.laser_id = 0;

    slamPoint.time = prevTime + 0.1/300000.; // Supposing 10 Hz and 300 000 points
    prevTime = slamPoint.time;

    if (!Utils::HasNanField(slamPoint))
      cloudS.push_back(slamPoint);
  }

  // Publish pointcloud only if it is not empty
  if (cloudS.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Slam pointcloud is empty : frame ignored.");
    return;
  }
  // Convert PointCloud into msg
  Pcl2_msg msg_sent;
  pcl::toROSMsg(cloudS, msg_sent);

  this->Talker->publish(msg_sent);
}

//------------------------------------------------------------------------------
void LivoxToLidarNode::LivoxCustomMsgCallback(const LivoxCustomMsg& cloudLmsg)
{
  // Init SLAM pointcloud
  CloudS cloudS;
  cloudS.reserve(cloudLmsg.point_num);
  cloudS.header.stamp = cloudLmsg.timebase * 1e-3; // microseconds
  cloudS.header.frame_id = cloudLmsg.header.frame_id;

  // Build SLAM pointcloud
  for (int i = 0; i < cloudLmsg.point_num; ++i)
  {
    const LivoxCustomPoint& livoxPoint = cloudLmsg.points[i];

    if (!Utils::IsPointValid(pcl::PointXYZ(livoxPoint.x, livoxPoint.y, livoxPoint.z)))
      continue;

    PointS slamPoint;
    slamPoint.x = livoxPoint.x;
    slamPoint.y = livoxPoint.y;
    slamPoint.z = livoxPoint.z;
    slamPoint.intensity = livoxPoint.reflectivity;
    slamPoint.laser_id = livoxPoint.line;

    slamPoint.time = double(livoxPoint.offset_time) * 1e-9; // seconds
    cloudS.push_back(slamPoint);
  }

  // Convertion PointCloud to msg
  Pcl2_msg msg_sent;
  pcl::toROSMsg(cloudS, msg_sent);

  this->Talker->publish(msg_sent);
}

}  // end of namespace lidar_conversions

//------------------------------------------------------------------------------
/*!
 * @brief Main node entry point.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  std::shared_ptr<lidar_conversions::LivoxToLidarNode> v2s
      = std::make_shared<lidar_conversions::LivoxToLidarNode>("livox_conversion", options);

  rclcpp::spin(v2s);

  return 0;
}
