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

#include "OusterToLidarNode.h"
#include "rmw/qos_profiles.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

OusterToLidarNode::OusterToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
  : Node(node_name, options)
{
  // Get number of lasers
  this->get_parameter("nb_lasers", this->NbLasers);

  // Get possible frequencies
  this->get_parameter("possible_frequencies", this->PossibleFrequencies);

  // Get number of threads
  this->get_parameter("nb_threads", this->NbThreads);

  // Init ROS publisher
  this->Talker = this->create_publisher<Pcl2_msg>("lidar_points", 1);

  // Create custom QoS
  rclcpp::QoS custom_qos_profile(1); // History mode : only keep last
  // Put reliability to the same mode than Ouster Driver
  custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  // Init ROS subscriber
  this->Listener = this->create_subscription<Pcl2_msg>("/ouster/points", custom_qos_profile,
                                        std::bind(&OusterToLidarNode::Callback, this, std::placeholders::_1));

  // Init ROS service
  this->EstimService = this->create_service<lidar_conversions::srv::EstimSense>(
      "lidar_conversions/estim_sense",
      std::bind(&OusterToLidarNode::EstimSenseService, this, std::placeholders::_1, std::placeholders::_2));


  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("Ouster data converter is ready !"));
}

//------------------------------------------------------------------------------
void OusterToLidarNode::Callback(const Pcl2_msg& msg_received)
{
  CloudO cloudO = Utils::InitCloudRaw<CloudO>(msg_received);
  // If input cloud is empty, ignore it
  if (cloudO.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input Ouster pointcloud is empty : frame ignored.");
    return;
  }

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudO.header.stamp);
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
  CloudS cloudS = Utils::InitCloudS<CloudO>(cloudO);

  const unsigned int nbLasers = (cloudO.height >= 8 && cloudO.height <= 128) ?
                                 cloudO.height :
                                  (cloudO.width >= 8 && cloudO.width <= 128) ?
                                   cloudO.width :
                                   this->NbLasers;

  // Estimate the rotation sense
  if (!this->RotationSenseEstimated)
  {
    this->RotationIsClockwise = Utils::IsRotationClockwise<PointO>(cloudO, nbLasers);
    this->RotationSenseEstimated = true;
  }

  // Check if time field looks properly set
  auto minmaxTime = std::minmax_element(cloudO.points.begin(), cloudO.points.end(),
                    [](const PointO& lhs, const PointO& rhs) { return lhs.t < rhs.t; });
  double duration = minmaxTime.second->t - minmaxTime.first->t;
  double factor = Utils::GetTimeFactor(duration, this->RotationDuration);
  duration *= factor;

  bool timeIsValid = duration > 1e-8 && duration < 2. * this->RotationDuration;

  if (!timeIsValid)
    RCLCPP_WARN_STREAM(this->get_logger(), "Invalid 'time' field, it will be built from azimuth advancement.");

  Eigen::Vector2d firstPoint = Utils::GetFirstValidPoint(cloudO);

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
  for (const PointO& ousterPoint : cloudO)
  {
    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(ousterPoint))
      continue;

    PointS slamPoint;
    slamPoint.x = ousterPoint.x;
    slamPoint.y = ousterPoint.y;
    slamPoint.z = ousterPoint.z;
    slamPoint.intensity = ousterPoint.reflectivity;
    slamPoint.laser_id = ousterPoint.ring;

    // Use time field if available, else estimate it from azimuth advancement
    if (timeIsValid)
      slamPoint.time = factor * ousterPoint.t;
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
  Pcl2_msg toPublish;
  pcl::toROSMsg(cloudS, toPublish);

  this->Talker->publish(toPublish);
}

//------------------------------------------------------------------------------
void OusterToLidarNode::EstimSenseService(
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

  std::shared_ptr<lidar_conversions::OusterToLidarNode> v2s
    = std::make_shared<lidar_conversions::OusterToLidarNode>("ouster_conversion", options);

  rclcpp::spin(v2s);

  return 0;
}
