//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2019-12-13
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

#ifndef ROS_TRANSFORM_UTILS_H
#define ROS_TRANSFORM_UTILS_H

#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/time.hpp>
#include <tf2/convert.h>
#include <chrono>
#include <Eigen/Geometry>

namespace Utils
{

//========================== Transform -> ROS msg ==============================

//------------------------------------------------------------------------------
//! Fill a TF msg with an isometry object.
geometry_msgs::msg::Transform IsometryToTfMsg(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Transform tfMsg;
  tfMsg.translation.x = transform.translation().x();
  tfMsg.translation.y = transform.translation().y();
  tfMsg.translation.z = transform.translation().z();
  Eigen::Quaterniond q = Eigen::Quaterniond(transform.linear());
  tfMsg.rotation.x = q.x();
  tfMsg.rotation.y = q.y();
  tfMsg.rotation.z = q.z();
  tfMsg.rotation.w = q.w();
  return tfMsg;
}

//------------------------------------------------------------------------------
//! Fill a Pose msg with an isometry object.
geometry_msgs::msg::Pose IsometryToPoseMsg(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose PoseMsg;
  PoseMsg.position.x = transform.translation().x();
  PoseMsg.position.y = transform.translation().y();
  PoseMsg.position.z = transform.translation().z();
  Eigen::Quaterniond q = Eigen::Quaterniond(transform.linear());
  PoseMsg.orientation.x = q.x();
  PoseMsg.orientation.y = q.y();
  PoseMsg.orientation.z = q.z();
  PoseMsg.orientation.w = q.w();
  return PoseMsg;
}

//------------------------------------------------------------------------------
//! Fill a PoseStamped msg with a Transform object.
geometry_msgs::msg::PoseStamped IsometryToPoseStampedMsg(const Eigen::Isometry3d& transform, double t, std::string frameId)
{
  geometry_msgs::msg::PoseStamped PoseStampedMsg;
  PoseStampedMsg.header.frame_id = frameId;
  PoseStampedMsg.header.stamp = rclcpp::Time(t);
  PoseStampedMsg.pose.position.x = transform.translation().x();
  PoseStampedMsg.pose.position.y = transform.translation().y();
  PoseStampedMsg.pose.position.z = transform.translation().z();
  Eigen::Quaterniond q = Eigen::Quaterniond(transform.linear());
  PoseStampedMsg.pose.orientation.x = q.x();
  PoseStampedMsg.pose.orientation.y = q.y();
  PoseStampedMsg.pose.orientation.z = q.z();
  PoseStampedMsg.pose.orientation.w = q.w();
  return PoseStampedMsg;
}


//========================== ROS msg -> Transform ==============================

//------------------------------------------------------------------------------
//! Build a Transform object from a Pose msg.
Eigen::Isometry3d PoseMsgToIsometry(const geometry_msgs::msg::Pose& poseMsg)
{
  Eigen::Translation3d trans(poseMsg.position.x,
                             poseMsg.position.y,
                             poseMsg.position.z);
  Eigen::Quaterniond rot(poseMsg.orientation.w,
                         poseMsg.orientation.x,
                         poseMsg.orientation.y,
                         poseMsg.orientation.z);
  return trans * rot.normalized();
}

//========================== ROS TF2 -> Eigen Isometry3d =======================

//------------------------------------------------------------------------------
//! Safely get a transform between 2 frames from TF2 server
// new API: https://docs.ros2.org/latest/api/tf2_ros/classtf2__ros_1_1Buffer.html
// using tf2::TimePoint = typedef std::chrono::time_point<std::chrono::system_clock, Duration>
// using tf2::Duration = typedef std::chrono::nanoseconds
// ? Not sure lookupTransform works with a duration of 0
bool Tf2LookupTransform(Eigen::Isometry3d& transform,
                        const tf2_ros::Buffer& tfBuffer,
                        const std::string& targetFrame,
                        const std::string& sourceFrame,
                        const builtin_interfaces::msg::Time msg_time = builtin_interfaces::msg::Time(),
                        const tf2::Duration timeout = tf2::durationFromSec(0.))
{
  geometry_msgs::msg::TransformStamped tfStamped;
  try
  {
    tf2::TimePoint tf2_time = tf2::timeFromSec(msg_time.sec + msg_time.nanosec * 1e-9);
    tfStamped = tfBuffer.lookupTransform(targetFrame, sourceFrame, tf2_time, timeout);
  }
  catch (tf2::TransformException& ex)
  {
    std::cerr << ex.what() << std::endl;
    return false;
  }
  const geometry_msgs::msg::Transform& t = tfStamped.transform;
  transform = Eigen::Translation3d(t.translation.x, t.translation.y, t.translation.z)
              * Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  return true;
}

} // end of Utils namespace

#endif  // ROS_TRANSFORM_UTILS_H