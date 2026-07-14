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

#include "VelodyneToLidarNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

VelodyneToLidarNode::VelodyneToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : rclcpp::Node(node_name, options)
{
  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

  // Get possible frequencies
  this->get_parameter("possible_frequencies", this->PossibleFrequencies);

  // Get number of threads
  this->get_parameter("nb_threads", this->NbThreads);

  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Init ROS subscriber
  this->Listener = this->create_subscription<Pcl2_msg>("velodyne_points", 2,
                                            std::bind(&VelodyneToLidarNode::Callback, this, std::placeholders::_1));

  // Init ROS service
  this->EstimService = this->create_service<lidar_conversions::srv::EstimSense>(
      "lidar_conversions/estim_sense",
      std::bind(&VelodyneToLidarNode::EstimSenseService, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN( "Velodyne data converter is ready !"));
}

//------------------------------------------------------------------------------
void VelodyneToLidarNode::Callback(const Pcl2_msg& msg_received)
{
  //convertion message to PointCloud
  //doc : https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
  CloudV cloudV = Utils::InitCloudRaw<CloudV>(msg_received);

  // If input cloud is empty, ignore it
  if (cloudV.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Velodyne pointcloud is empty : frame ignored.");
    return;
  }

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudV.header.stamp);
  double diffTimePrevFrame = currFrameTime - this->PrevFrameTime;
  this->PrevFrameTime = currFrameTime;

  // If the rotation duration has not been estimated
  if (this->RotationDuration < 0.)
  {
    // Check if this duration is possible
    if (Utils::CheckRotationDuration(diffTimePrevFrame, this->PossibleFrequencies))
    {
      // Check a confirmation of the frame duration to avoid outliers (frames dropped)
      // For the first frame, RotationDurationPrior is -1, this condition won't be fulfilled
      // For the second frame, RotationDurationPrior is absurd, this condition won't be fulfilled
      // First real intempt occurs at the 3rd frame
      if (std::abs(diffTimePrevFrame - this->RotationDurationPrior) < 5e-3) // 5ms threshold
        this->RotationDuration = (diffTimePrevFrame + this->RotationDurationPrior) / 2.;
      this->RotationDurationPrior = diffTimePrevFrame;
      RCLCPP_INFO_STREAM(this->get_logger(), std::setprecision(12) << "Difference between successive frames is :" << diffTimePrevFrame);
    }
  }

  if (this->RotationDuration < 0.)
    return;

  // Init SLAM pointcloud
  CloudS cloudS = Utils::InitCloudS<CloudV>(cloudV);

  const unsigned int nbLasers = (cloudV.height >= 8 && cloudV.height <= 128) ?
                                 cloudV.height :
                                  (cloudV.width >= 8 && cloudV.width <= 128) ?
                                   cloudV.width :
                                   this->NbLasers;

  // Estimate the rotation sense
  if (!this->RotationSenseEstimated)
  {
    this->RotationIsClockwise = Utils::IsRotationClockwise<PointV>(cloudV, nbLasers);
    this->RotationSenseEstimated = true;
  }

  // Check if time field looks properly set
  auto minmaxTime = std::minmax_element(cloudV.points.begin(), cloudV.points.end(),
                    [](const PointV& lhs, const PointV& rhs) { return lhs.time < rhs.time; });
  double duration = minmaxTime.second->time - minmaxTime.first->time;
  double factor = Utils::GetTimeFactor(duration, this->RotationDuration);
  duration *= factor;

  bool timeIsValid = duration > 1e-8 && duration < 2. * this->RotationDuration;

  if (!timeIsValid)
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid 'time' field, it will be built from azimuth advancement.");

  Eigen::Vector2d firstPoint = Utils::GetFirstValidPoint(cloudV);;

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
  for (const PointV& velodynePoint : cloudV)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(velodynePoint))
      continue;

    PointS slamPoint;
    slamPoint.x = velodynePoint.x;
    slamPoint.y = velodynePoint.y;
    slamPoint.z = velodynePoint.z;
    slamPoint.intensity = velodynePoint.intensity;
    slamPoint.laser_id = velodynePoint.ring;

    // Use time field if available, else estimate it from azimuth advancement
    if (timeIsValid)
      slamPoint.time = factor * velodynePoint.time;
    else
      slamPoint.time = Utils::EstimateTime({slamPoint.x, slamPoint.y}, this->RotationDuration, firstPoint, this->RotationIsClockwise);

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
void VelodyneToLidarNode::EstimSenseService(
  const std::shared_ptr<lidar_conversions::srv::EstimSense::Request> req,
  const std::shared_ptr<lidar_conversions::srv::EstimSense::Response> res)
{
  this->RotationSenseEstimated = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "Rotation sense will be re-estimated with next frames.");
  res->success = true;
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

  std::shared_ptr<lidar_conversions::VelodyneToLidarNode> v2s
      = std::make_shared<lidar_conversions::VelodyneToLidarNode>("velodyne_conversion", options);

  rclcpp::spin(v2s);

  return 0;
}
