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

#include "RobosenseToLidarNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

namespace lidar_conversions
{

RobosenseToLidarNode::RobosenseToLidarNode(std::string node_name, const rclcpp::NodeOptions options)
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

  // Init ROS subscriber
  this->Listener = this->create_subscription<Pcl2_msg>("rslidar_points", 2,
                              std::bind(&RobosenseToLidarNode::Callback, this, std::placeholders::_1));

  // Init ROS service
  this->EstimService = this->create_service<lidar_conversions::srv::EstimParams>(
      "lidar_conversions/estim_params",
      std::bind(&RobosenseToLidarNode::EstimParamsService, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("RSLidar data converter is ready !"));
}

//------------------------------------------------------------------------------
void RobosenseToLidarNode::Callback(const Pcl2_msg& msg_received)
{
  //convertion to CloudRS
  CloudRS cloudRS = Utils::InitCloudRaw<CloudRS>(msg_received);

  // If input cloud is empty, ignore it
  if (cloudRS.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input RSLidar pointcloud is empty : frame ignored.");
    return;
  }

  // Rotation duration is estimated to be used in time estimation if needed
  double currFrameTime = Utils::PclStampToSec(cloudRS.header.stamp);
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
  CloudS cloudS = Utils::InitCloudS<CloudRS>(cloudRS);

  const unsigned int nbLasers = (cloudRS.height >= 8 && cloudRS.height <= 128) ?
                                 cloudRS.height :
                                  (cloudRS.width >= 8 && cloudRS.width <= 128) ?
                                   cloudRS.width :
                                   this->NbLasers;

  // Init of parameters useful for laser_id and time estimations
  if (!this->RotSenseAndClustersEstimated)
  {
    Utils::InitEstimationParameters<PointRS>(cloudRS, nbLasers, this->Clusters, this->RotationIsClockwise, this->NbThreads);
    this->RotSenseAndClustersEstimated = true;
  }

  Eigen::Vector2d firstPoint = Utils::GetFirstValidPoint(cloudRS);

  // Build SLAM pointcloud
  #pragma omp parallel for num_threads(this->NbThreads)
  for (unsigned int i = 0; i < cloudRS.size(); ++i)
  {
    const PointRS& rsPoint = cloudRS[i];

    // Remove no return points by checking unvalid values (NaNs or zeros)
    if (!Utils::IsPointValid(rsPoint))
      continue;

    // In case of dual returns mode, check that the second return is not identical to the first
    // CHECK this operation for other sensors than RS16
    if (!cloudS.empty() && std::equal(rsPoint.data, rsPoint.data + 3, cloudS.back().data))
      continue;

    // Copy coordinates and intensity
    PointS slamPoint;
    slamPoint.x = rsPoint.x;
    slamPoint.y = rsPoint.y;
    slamPoint.z = rsPoint.z;
    slamPoint.intensity = rsPoint.intensity;
    slamPoint.laser_id = Utils::ComputeLaserId({slamPoint.x, slamPoint.y, slamPoint.z}, nbLasers, this->Clusters);
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
  Pcl2_msg msg_sended;
  pcl::toROSMsg(cloudS, msg_sended);

  this->Talker->publish(msg_sended);
}

//------------------------------------------------------------------------------
void RobosenseToLidarNode::EstimParamsService(
  const std::shared_ptr<lidar_conversions::srv::EstimParams::Request> req,
  const std::shared_ptr<lidar_conversions::srv::EstimParams::Response> res)
{
  this->RotSenseAndClustersEstimated = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "Estimation parameters will be re-estimated with next frames.");
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

  std::shared_ptr<lidar_conversions::RobosenseToLidarNode> rs2s
    = std::make_shared<lidar_conversions::RobosenseToLidarNode>("rslidar_conversion", options);

  rclcpp::spin(rs2s);

  return 0;
}
