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

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "LidarSlamTestNode.h"

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"
#define BOLD_RED(s) "\033[1;31m" << s << "\033[0m"

namespace lidar_slam_test
{

//------------------------------------------------------------------------------
// TOOLS
//------------------------------------------------------------------------------

namespace Utils
{

//------------------------------------------------------------------------------
Eigen::Matrix3d RPYtoRotationMatrix(double roll, double pitch, double yaw)
{
  return Eigen::Matrix3d(Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
}

//------------------------------------------------------------------------------
Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d& rot)
{
  // `rpy = rot.eulerAngles(2, 1, 0).reverse()`             returns angles in range [-PI:PI]x[-PI:PI]x[0:PI].
  // `rpy = Eigen::EulerAnglesZYXd(rot).angles().reverse()` returns angles in range [-PI:PI]x[-PI:PI]x[-PI:PI].
  // But these are bad. For first range, yaw angle cannot be negative : this
  // leads to un-necessary non trivial RPY decomposition, and to unstable
  // optimization result as we are not optimizing around 0.
  // For second ranges, there exist several RPY decomposition for the same
  // rotation (one of them being non-trivial too). Therefore the optimization
  // may also be unstable by oscillating between them.
  // We prefer to output angles in range [-PI:PI]x[-PI/2:PI/2]x[-PI:PI] : we
  // allow negative values to avoid oscillation artefacts, and minimize the
  // pitch angle to fix representation.
  Eigen::Vector3d rpy;
  rpy.x() = std::atan2(rot(2, 1), rot(2, 2));
  rpy.y() = -std::asin(rot(2, 0));
  rpy.z() = std::atan2(rot(1, 0), rot(0, 0));
  return rpy;
}

//------------------------------------------------------------------------------
Eigen::Vector6d IsometryToXYZRPY(const Eigen::Isometry3d& transform)
{
  Eigen::Vector6d xyzrpy;
  xyzrpy << transform.translation(), RotationMatrixToRPY(transform.linear());
  return xyzrpy;
}

//------------------------------------------------------------------------------
Eigen::Isometry3d PoseMsgToIsometry(const geometry_msgs::msg::Pose& poseMsg)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond rot(poseMsg.orientation.w,
                         poseMsg.orientation.x,
                         poseMsg.orientation.y,
                         poseMsg.orientation.z);
  transform.rotate(rot);
  transform.translation() = Eigen::Vector3d({poseMsg.position.x,
                                             poseMsg.position.y,
                                             poseMsg.position.z});

  return transform;
}

//------------------------------------------------------------------------------
double Normalize(double value)
{
  return abs(value) < 1e-15 ? 0.f : value;
}

}

//------------------------------------------------------------------------------
// CLASS METHODS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
LidarSlamTestNode::LidarSlamTestNode(std::string name_node, const rclcpp::NodeOptions& options):
  Node(name_node, options)
{
  // Set the path to the reference if comparison is required
  this->get_parameter("ref_path", this->RefPath);
  if (!this->RefPath.empty())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading reference at " << this->RefPath);
    this->LoadRef();
  }
  else
    RCLCPP_INFO_STREAM(this->get_logger(), "No reference data supplied : comparison ignored");

  // Set result output path
  // Poses
  this->get_parameter("res_path", this->ResPath);
  if (this->ResPath.empty())
    RCLCPP_WARN_STREAM(this->get_logger(), "No result folder specified : the results will be stored in /tmp if it exists");

  // Init result folder
  std::ofstream resPosesFile(this->ResPath + "/Poses.csv");
  if (resPosesFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("Cannot save poses at " << this->ResPath));
    rclcpp::shutdown();
    return;
  }
  resPosesFile << "time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2\n";
  // Evaluators
  std::ofstream resEvalFile(this->ResPath + "/Evaluators.csv");
  if (resEvalFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("Cannot save evaluators at " << this->ResPath));
    rclcpp::shutdown();
    return;
  }
  resEvalFile << "time,overlap,nb_matches,computation_time\n";

  // Loading parameters
  this->get_parameter("nb_frames_dropped", this->MaxNbFramesDropped);
  this->get_parameter("time_threshold", this->TimeThreshold);
  this->get_parameter("position_threshold", this->PositionThreshold);
  this->get_parameter("angle_threshold", this->AngleThreshold);
  this->get_parameter("verbose", this->Verbose);

  // Init ROS subscriber
  this->PoseListener = this->create_subscription<nav_msgs::msg::Odometry>
    ("slam_odom", 1, std::bind(&LidarSlamTestNode::PoseCallback, this, std::placeholders::_1));
  this->ConfidenceListener = this->create_subscription<lidar_slam::msg::Confidence>
    ("slam_confidence", 1, std::bind(&LidarSlamTestNode::ConfidenceCallback, this, std::placeholders::_1));

  RCLCPP_INFO_STREAM(this->get_logger(), ("Lidar slam evaluator is ready !"));
}

//------------------------------------------------------------------------------
bool LidarSlamTestNode::CanBeCompared()
{
  return !this->RefPath.empty() && !this->RefPoses.empty();
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::LoadRef()
{
  // Check the file
  std::string path = this->RefPath + "/Poses.csv";
  std::ifstream refPosesFile(path);
  if (refPosesFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("The poses csv file '"
                                            << path << "' was not found : shutting down the node"));
    rclcpp::shutdown();
    return;
  }

  // Temporal string to store line data
  std::string line;

  // Check and remove header line
  std::getline(refPosesFile, line);
  if (line.find("x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2") == std::string::npos)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("The poses csv file '"
                                            << path << " is badly formatted : shutting down the node"));
    rclcpp::shutdown();
    return;
  }

  // Fill the reference poses vector
  while (std::getline(refPosesFile, line))
  {
    // Temporal struct to store pose info
    Pose pose;
    int pos; // char position
    // Store timestamp :
    pos = line.find(",");
    pose.Stamp = std::stod(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Get pose
    // Translation
    for (int i = 0; i < 3; ++i)
    {
      pos = line.find(",");
      pose.Data.translation()(i) = std::stod(line.substr(0, pos));
      line.erase(0, pos + 1);
    }
    // Rotation
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        pos = line.find(",");
        pose.Data.linear()(j, i) = std::stod(line.substr(0, pos));
        if (pos != std::string::npos)
          line.erase(0, pos + 1);
      }
    }
    this->RefPoses.push_back(pose);
  }
  refPosesFile.close();

  if (this->RefPoses.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("The poses csv file '"
                        << path << " is empty : shutting down the node"));
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Poses loaded!");

  // Fill the reference confidence vector
  std::ifstream refEvaluatorsFile(this->RefPath + "/Evaluators.csv");
  if (refEvaluatorsFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("The evaluators csv file '"
                                            << path << "' was not found : shutting down the node"));
    rclcpp::shutdown();
    return;
  }

  // Check and remove header line
  std::getline(refEvaluatorsFile, line);
  if (line.find("time,overlap,nb_matches,computation_time") == std::string::npos)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("The evaluators csv file '"
                                            << path << "is badly formatted : shutting down the node"));
    rclcpp::shutdown();
    return;
  }

  while (std::getline(refEvaluatorsFile, line))
  {
    // Temporal struct to store evaluator info
    Evaluator eval;
    int pos; // char position
    // Store timestamp :
    pos = line.find(",");
    eval.Stamp = std::stod(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Store overlap
    pos = line.find(",");
    eval.Overlap = std::stof(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Store the number of matches
    pos = line.find(",");
    eval.NbMatches = std::stoi(line.substr(0, pos));
    line.erase(0, pos + 1);
    // Store the computation time
    eval.Duration = std::stof(line);
    this->RefEvaluators.push_back(eval);
  }
  refEvaluatorsFile.close();

  if (this->RefEvaluators.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("The evaluators csv file '"
                        << path << " is empty : shutting down the node"));
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Evaluators loaded!");
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::PoseCallback(const nav_msgs::msg::Odometry& poseMsg)
{
  std::ofstream resPosesFile(this->ResPath + "/Poses.csv", std::ofstream::app);
  if (resPosesFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("Could not save pose"));
    rclcpp::shutdown();
    return;
  }

  // Save the pose in a file
  double time = poseMsg.header.stamp.sec + poseMsg.header.stamp.nanosec * 1e-9;
  Eigen::Isometry3d transform = Utils::PoseMsgToIsometry(poseMsg.pose.pose);

  resPosesFile << std::fixed << std::setprecision(9) << time << ","
               << transform.translation().x() << "," << transform.translation().y() << "," << transform.translation().z() << ","
               << transform.linear()(0,0)     << "," << transform.linear()(1,0)     << "," << transform.linear()(2,0)     << ","
               << transform.linear()(0,1)     << "," << transform.linear()(1,1)     << "," << transform.linear()(2,1)     << ","
               << transform.linear()(0,2)     << "," << transform.linear()(1,2)     << "," << transform.linear()(2,2)     << "\n";
  resPosesFile.close();

  // Check if comparison is required
  if (!this->CanBeCompared())
    return;

  // Search the pose in reference
  const int prevPoseIdx = this->PoseIdx;
  while (this->PoseIdx < this->RefPoses.size() &&
         this->RefPoses[this->PoseIdx].Stamp < time - 1e-6)
    ++this->PoseIdx;
  // If there is more than 1 frame difference, one has been dropped
  if (this->PoseIdx > prevPoseIdx)
    this->NbFramesDropped += this->PoseIdx - prevPoseIdx - 1;

  // No more reference
  if (this->PoseIdx == this->RefPoses.size() ||
      time > this->RefPoses.back().Stamp - 1.f) // 1 second margin
  {
    this->OutputTestResult(); // will shut down the node
    return;
 }

  // If the current frame has not been seen in reference -> return (wait for next frame)
  if (std::abs(this->RefPoses[this->PoseIdx].Stamp - time) > 1e-6)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Reference does not contain a pose at "
                     << std::fixed << std::setprecision(9) << time
                     << " (may have been dropped)."
                     << " Check the reference was computed on the same data.");

    return;
  }

  // Update previous pose counter if the ref pose has been updated
  if (this->PoseIdx > prevPoseIdx)
    this->PrevPoseIdx = prevPoseIdx;
  // Compare the pose with reference trajectory
  Eigen::Isometry3d refTransform = this->RefPoses[this->PoseIdx].Data;
  Eigen::Isometry3d refPrevTransform;
  if (this->PoseIdx >= 1)
    refPrevTransform = this->RefPoses[this->PrevPoseIdx].Data;
  else
  {
    this->PrevTransform = transform;
    refPrevTransform = refTransform;
  }
  Eigen::Isometry3d diffTransform = (refPrevTransform.inverse() * refTransform).inverse() * (this->PrevTransform.inverse() * transform);
  Eigen::Vector6d diffPose = Utils::IsometryToXYZRPY(diffTransform);
  // Compute angle difference
  float currentDiffAngle = diffPose.tail(3).norm();
  this->DiffAngle.Update(currentDiffAngle);
  // Compute translation difference
  float currentDiffPosition = diffPose.head(3).norm();
  this->DiffPosition.Update(currentDiffPosition);

  // Test fails if any pose is too different from its reference pose
  if (currentDiffPosition > this->PositionThreshold ||
      currentDiffAngle * 180.f / M_PI > this->AngleThreshold)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Pose at " << std::fixed << std::setprecision(9) << time << " is not consistent with reference");
    this->Failure = true;
  }

  if (this->Verbose)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose difference (at " << std::fixed << std::setprecision(9) << time << ") :\n"
                    << "\t" << currentDiffAngle * 180.f / M_PI << " degrees\n"
                    << "\t" << currentDiffPosition << " m");
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose difference average (at " << std::fixed << std::setprecision(9) << time << ") :\n"
                    << "\t" << this->DiffAngle.Get() * 180.f / M_PI << " degrees\n"
                    << "\t" << this->DiffPosition.Get() << " m");
  }

  diffTransform = refTransform.inverse() * transform;
  diffPose = Utils::IsometryToXYZRPY(diffTransform);
  this->LastPositionDiff = diffPose.head(3).norm();
  this->LastAngleDiff = diffPose.tail(3).norm();

  this->PrevTransform = transform;
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::ConfidenceCallback(const lidar_slam::msg::Confidence& confidenceMsg)
{
  // Log the confidence values
  double time = confidenceMsg.header.stamp.sec + confidenceMsg.header.stamp.nanosec * 1e-9;
  float overlap = confidenceMsg.overlap;
  float nbMatches = confidenceMsg.nb_matches;
  float computationTime = confidenceMsg.computation_time;
  std::ofstream EvaluatorsFile(this->ResPath + "/Evaluators.csv", std::ofstream::app);
  if (EvaluatorsFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), BOLD_RED("Could not save confidence estimators"));
    rclcpp::shutdown();
    return;
  }

  EvaluatorsFile << std::fixed << std::setprecision(9) << time << ","
                 << overlap << "," << nbMatches << "," << computationTime << "\n";
  EvaluatorsFile.close();

  // Check if comparison is required
  if (!this->CanBeCompared())
    return;

  while (this->ConfidenceCounter < this->RefEvaluators.size() &&
         this->RefEvaluators[this->ConfidenceCounter].Stamp < time - 1e-6)
    ++this->ConfidenceCounter;

  // No more reference
  if (this->ConfidenceCounter == this->RefEvaluators.size() ||
      time > this->RefPoses.back().Stamp - 1.f) // 1 second margin
  {
    this->OutputTestResult(); // will shut down the node
    return;
  }

  // If the current frame has not been seen in reference -> return (wait for next frame)
  // The last frame cannot be dropped so the node should be ended in any case.
  if (std::abs(this->RefEvaluators[this->ConfidenceCounter].Stamp - time) > 1e-6)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Reference does not contain an evaluator at "
                       << std::fixed << std::setprecision(9) << time
                       << " (may have been dropped)."
                       << " Check the reference was computed on the same data");
    return;
  }

  // Compare with reference evaluators
  float diffOverlap   = overlap         - this->RefEvaluators[this->ConfidenceCounter].Overlap;
  float diffNbMatches = nbMatches       - this->RefEvaluators[this->ConfidenceCounter].NbMatches;
  float diffTime      = computationTime - this->RefEvaluators[this->ConfidenceCounter].Duration;
  this->DiffOverlap.Update(diffOverlap);
  this->DiffNbMatches.Update(diffNbMatches);
  this->DiffTime.Update(diffTime);

  if (this->Verbose)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Confidence difference for pose at "
                    << std::fixed << std::setprecision(9) << time << ": \n"
                    << "\t" << "Overlap difference: "            << 100 * diffOverlap << " %\n"
                    << "\t" << "Number of matches difference : " << diffNbMatches     << " matches\n"
                    << "\t" << "Computation time difference : "  << diffTime          << " s");
  }
}

//------------------------------------------------------------------------------
void LidarSlamTestNode::OutputTestResult()
{
  if (this->NbFramesDropped > this->MaxNbFramesDropped)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), this->NbFramesDropped
                                           << " frames dropped comparing to reference");
    this->Failure = true;
  }

  // Test fails if the mean computation time is too high
  // compared with the reference processing
  if (this->DiffTime.Get() > this->TimeThreshold)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Computation time is too long compared to reference ("
                                            << this->DiffTime.Get() << "s longer)");
    this->Failure = true;
  }

  if (!this->Failure)
    RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("Test successfully passed"));
  else
    RCLCPP_ERROR_STREAM(this->get_logger(), "Test failed");

  RCLCPP_INFO_STREAM(this->get_logger(), "Comparison with reference (averages) : ");
  RCLCPP_INFO_STREAM(this->get_logger(), "Overlap difference : "
                                         << 100.f * this->DiffOverlap.Get()
                                         << " %");
  RCLCPP_INFO_STREAM(this->get_logger(), "Number of matches difference : "
                                         << this->DiffNbMatches.Get()
                                         << " matches");
  RCLCPP_INFO_STREAM(this->get_logger(), "Computation time difference : "
                                         << this->DiffTime.Get()
                                         << " s");
  RCLCPP_INFO_STREAM(this->get_logger(), "Trajectory difference : "
                                         << this->DiffAngle.Get()
                                         << " degrees and "
                                         << this->DiffPosition.Get()
                                         << " m");
  RCLCPP_INFO_STREAM(this->get_logger(), "Final drift from reference : "
                                          << this->LastAngleDiff
                                          << " degrees and "
                                          << this->LastPositionDiff
                                          << " m");

  // Comparison has stopped : Shut the node down
  rclcpp::shutdown();
  return;
}

}  // end of namespace lidar_slam_test

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

  auto test = std::make_shared<lidar_slam_test::LidarSlamTestNode>("lidar_slam_test", options);

  rclcpp::spin(test);

  return 0;
}
