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


#include "LidarSlamNode.h"
#include "ros_transform_utils.h"

#include <LidarSlam/Utilities.h>
#include "pcl_conversions/pcl_conversions.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

// std
#include <filesystem>     // For std::filesystem (C++17)
#include <iomanip>

#ifdef USE_CV_BRIDGE
  #if __has_include(<cv_bridge/cv_bridge.hpp>)
    #include <cv_bridge/cv_bridge.hpp>
  #else
    #include <cv_bridge/cv_bridge.h>
  #endif
#endif
//? #include <sensor_msgs/msg/image_encodings.hpp>

#define BOLD_GREEN(s) "\033[1;32m" << s << "\033[0m"

static const std::vector<std::string> DELIMITERS = {";", ",", " ", "\t"};

// Macro to publish with a cast toward the type of publisher
// because Ros2, the publisher has a template
#define publishWithCast(publisher, type_msg, message) \
    std::static_pointer_cast<rclcpp::Publisher<type_msg>>(publisher)->publish(message);

enum Output
{
  POSE_ODOM,                 // Publish SLAM pose as an Odometry msg on 'slam_odom' topic (default : true).
  POSE_TF,                   // Publish SLAM pose as a TF from 'odometry_frame' to 'tracking_frame' (default : true).
  POSE_PREDICTION_ODOM,      // Publish latency-corrected SLAM pose as an Odometry msg on 'slam_predicted_odom' topic.
  POSE_PREDICTION_TF,        // Publish latency-corrected SLAM pose as a TF from 'odometry_frame' to '<tracking_frame>_prediction'.

  EDGES_MAP,                 // Publish edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/edges'.
  INTENSITY_EDGES_MAP,       // Publish intensity edge keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/intensity_edges'.
  PLANES_MAP,                // Publish plane keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/planes'.
  BLOBS_MAP,                 // Publish blob keypoints map as a LidarPoint PointCloud2 msg to topic 'maps/blobs'.

  EDGES_SUBMAP,              // Publish edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/edges'.
  INTENSITY_EDGES_SUBMAP,    // Publish intensity edge keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/intensity_edges'.
  PLANES_SUBMAP,             // Publish plane keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/planes'.
  BLOBS_SUBMAP,              // Publish blob keypoints submap as a LidarPoint PointCloud2 msg to topic 'submaps/blobs'.

  EDGE_KEYPOINTS,            // Publish extracted edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/edges'.
  INTENSITY_EDGE_KEYPOINTS,  // Publish extracted intensity edge keypoints from current frame as a PointCloud2 msg to topic 'keypoints/intensity_edges'.
  PLANE_KEYPOINTS,           // Publish extracted plane keypoints from current frame as a PointCloud2 msg to topic 'keypoints/planes'.
  BLOB_KEYPOINTS,            // Publish extracted blob keypoints from current frame as a PointCloud2 msg to topic 'keypoints/blobs'.

  SLAM_REGISTERED_POINTS,    // Publish SLAM pointcloud as LidarPoint PointCloud2 msg to topic 'slam_registered_points'.

  CONFIDENCE,                // Publish confidence estimators on output pose to topic 'slam_confidence'.

  PGO_PATH,                  // Publish optimized SLAM trajectory as Path msg to 'pgo_slam_path' latched topic.
};

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
LidarSlamNode::LidarSlamNode(std::string name_node, const rclcpp::NodeOptions& options):
  Node(name_node, options)
{
  // Init Tf2 variables
  // doc: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
  this->TfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->TfListener = std::make_shared<tf2_ros::TransformListener>(*this->TfBuffer);
  this->TfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  this->StaticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // ***************************************************************************
  // Get SLAM params
  // Init SLAM state
  this->SetSlamParameters();
  this->SetSlamInitialState();

  // ***************************************************************************
  // Init ROS publishers

  #define initPublisher(publisher, topic, type, rosParam, publishDefault, queue, latch)   \
    this->get_parameter_or<bool>(rosParam, this->Publish[publisher], publishDefault);     \
    if (this->Publish[publisher])                                                         \
      this->Publishers[publisher] = this->create_publisher<type>(topic, queue);

  this->get_parameter_or<bool>("output.pose.tf",           this->Publish[POSE_TF],            true);
  this->get_parameter_or<bool>("output.pose.predicted_tf", this->Publish[POSE_PREDICTION_TF], false);

  initPublisher(POSE_ODOM,            "slam_odom",           nav_msgs::msg::Odometry, "output.pose.odom",           true,  1, false);
  initPublisher(POSE_PREDICTION_ODOM, "slam_predicted_odom", nav_msgs::msg::Odometry, "output.pose.predicted_odom", false, 1, false);

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
  {
    initPublisher(EDGES_MAP,  "maps/edges",  Pcl2_msg, "output.maps.edges",  true, 1, false);
    initPublisher(EDGES_SUBMAP,  "submaps/edges",  Pcl2_msg, "output.submaps.edges",  true, 1, false);
    initPublisher(EDGE_KEYPOINTS,  "keypoints/edges",  Pcl2_msg, "output.keypoints.edges",  true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::INTENSITY_EDGE))
  {
    initPublisher(INTENSITY_EDGES_MAP,  "maps/intensity_edges",  Pcl2_msg, "output.maps.intensity_edges",  true, 1, false);
    initPublisher(INTENSITY_EDGES_SUBMAP,  "submaps/intensity_edges",  Pcl2_msg, "output.submaps.intensity_edges",  true, 1, false);
    initPublisher(INTENSITY_EDGE_KEYPOINTS,  "keypoints/intensity_edges",  Pcl2_msg, "output.keypoints.intensity_edges",  true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
  {
    initPublisher(PLANES_MAP, "maps/planes", Pcl2_msg, "output.maps.planes", true, 1, false);
    initPublisher(PLANES_SUBMAP, "submaps/planes", Pcl2_msg, "output.submaps.planes", true, 1, false);
    initPublisher(PLANE_KEYPOINTS, "keypoints/planes", Pcl2_msg, "output.keypoints.planes", true, 1, false);
  }

  if(this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
  {
    initPublisher(BLOBS_MAP,  "maps/blobs",  Pcl2_msg, "output.maps.blobs",  true, 1, false);
    initPublisher(BLOBS_SUBMAP,  "submaps/blobs",  Pcl2_msg, "output.submaps.blobs",  true, 1, false);
    initPublisher(BLOB_KEYPOINTS,  "keypoints/blobs",  Pcl2_msg, "output.keypoints.blobs",  true, 1, false);
  }

  initPublisher(SLAM_REGISTERED_POINTS, "slam_registered_points", Pcl2_msg, "output.registered_points", true, 1, false);

  initPublisher(CONFIDENCE, "slam_confidence", lidar_slam::msg::Confidence, "output.confidence", true, 1, false);

  if (this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS) ||
      this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LANDMARK) ||
      this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE) ||
      this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE))
    initPublisher(PGO_PATH, "pgo_slam_path", nav_msgs::msg::Path, "graph.publish_path", false, 1, true);

  // Set frequency of output pose (all poses are published at the end of the frames process)
  this->get_parameter_or<double>("output.pose.frequency", this->TrajFrequency, -1.);

  // ***************************************************************************
  // Init ROS subscribers

  // LiDAR inputs
  this->get_parameter_or<int>("lidars_nb", this->MultiLidarsNb, 1);
  int frameMode;
  this->get_parameter_or<int>("frames_mode", frameMode, 1);
  this->WaitFramesDef = static_cast<LidarSlamNode::FramesCollectionMode>(frameMode);
  this->get_parameter_or<double>("frames_waiting_time", this->WaitFramesTime, 0.2);

  // Check if the parameter type is a string_array
  // If the parameter doesn't exist or is a string, push back a value
  std::vector<std::string> lidarTopics;
  if (this->has_parameter("input") && this->get_parameter_types({"input"})[0] == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    this->get_parameter<std::vector<std::string>>("input", lidarTopics);
  else
  {
    std::string input;
    this->get_parameter_or<std::string>("input", input, "lidar_points");
    lidarTopics.push_back(input);
  }

  for (unsigned int lidarTopicId = 0; lidarTopicId < lidarTopics.size(); lidarTopicId++)
  {
    this->CloudSubs.push_back(this->create_subscription<Pcl2_msg>(lidarTopics[lidarTopicId], 1,
                                                                  std::bind(&LidarSlamNode::ScanCallback, this, std::placeholders::_1)));
    RCLCPP_INFO_STREAM(this->get_logger(), "Using LiDAR frames on topic '" << lidarTopics[lidarTopicId] << "'");
  }

  // SLAM commands
  this->SlamCommandSub = this->create_subscription<lidar_slam::msg::SlamCommand>("slam_command", 1,
                                                                                 std::bind(&LidarSlamNode::SlamCommandCallback, this, std::placeholders::_1));

  // Reset pose with input message
  this->SetPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("set_pose", 1,
                                                                                              std::bind(&LidarSlamNode::SetPoseCallback, this, std::placeholders::_1));
  // Init logging of GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  // Perfect synchronization is not required as GPS data are not used in SLAM local process
  if (this->UseExtSensor[LidarSlam::ExternalSensor::GPS])
    this->GpsOdomSub = this->create_subscription<nav_msgs::msg::Odometry>("gps_odom", 1,
                                                                          std::bind(&LidarSlamNode::GpsCallback, this, std::placeholders::_1));

  if (this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE))
  {
    this->ClickedPtSub = this->create_subscription<geometry_msgs::msg::PointStamped>("clicked_point", 1,
                                                                                     std::bind(&LidarSlamNode::ClickedPointCallback, this, std::placeholders::_1));
    // Init loop detection service
    this->DetectLoopService = this->create_service<lidar_slam::srv::DetectLoop>(
      "lidar_slam/detect_loop",
      std::bind(&LidarSlamNode::DetectLoopClosureService, this, std::placeholders::_1, std::placeholders::_2));
  }

  // Init logging of landmark data, Camera data, external pose data, wheel encoder data and/or IMU data
  if (this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR] ||
      this->UseExtSensor[LidarSlam::ExternalSensor::CAMERA] ||
      this->UseExtSensor[LidarSlam::ExternalSensor::POSE] ||
      this->UseExtSensor[LidarSlam::ExternalSensor::WHEEL_ODOM] ||
      this->UseExtSensor[LidarSlam::ExternalSensor::IMU])
  {
    // Create an external independent spinner to get the landmarks and/or camera info in a parallel way
    this->ExternalSensorGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions ops;
    ops.callback_group = this->ExternalSensorGroup;

    if (this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR])
    {
      this->LandmarkSub = this->create_subscription<apriltag_ros::msg::AprilTagDetectionArray>("tag_detections", 200,
                                                                                               std::bind(&LidarSlamNode::TagCallback, this, std::placeholders::_1), ops);
    }

    if (this->UseExtSensor[LidarSlam::ExternalSensor::CAMERA])
    {
      this->CameraSub = this->create_subscription<sensor_msgs::msg::Image>("camera", 10,
                                                                           std::bind(&LidarSlamNode::ImageCallback, this, std::placeholders::_1), ops);
      this->CameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", 10,
                                                                                    std::bind(&LidarSlamNode::CameraInfoCallback, this, std::placeholders::_1), ops);
    }

    if (this->UseExtSensor[LidarSlam::ExternalSensor::POSE])
    {
      this->ExtPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("ext_poses",
                                        10, std::bind(&LidarSlamNode::ExtPoseCallback, this, std::placeholders::_1), ops);
    }

    if (this->UseExtSensor[LidarSlam::ExternalSensor::WHEEL_ODOM])
    {
      this->WheelOdomSub = this->create_subscription<std_msgs::msg::Float64>("wheel_odom", 10,
                                                                             std::bind(&LidarSlamNode::WheelOdomCallback, this, std::placeholders::_1), ops);
    }

    if (this->UseExtSensor[LidarSlam::ExternalSensor::IMU])
    {
      this->ImuSub = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(),
                                                                      std::bind(&LidarSlamNode::ImuCallback, this, std::placeholders::_1), ops);
    }
  }

  // Init save keypoints maps service
  this->SaveService = this->create_service<lidar_slam::srv::SavePc>(
      "lidar_slam/save_pc",
      std::bind(&LidarSlamNode::SavePointcloudService, this, std::placeholders::_1, std::placeholders::_2));

  // Init reset service
  this->RstService = this->create_service<lidar_slam::srv::Reset>(
      "lidar_slam/reset",
      std::bind(&LidarSlamNode::ResetService, this, std::placeholders::_1, std::placeholders::_2));

  // Init set initial slam state service
  this->SetInitStateService = this->create_service<lidar_slam::srv::SetInitialState>(
      "lidar_slam/set_initial_state",
      std::bind(&LidarSlamNode::SetInitialStateService, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(this->get_logger(), BOLD_GREEN("LiDAR SLAM is ready !"));
}

//------------------------------------------------------------------------------
LidarSlamNode::~LidarSlamNode()
{
}

//------------------------------------------------------------------------------
void LidarSlamNode::ScanCallback(const Pcl2_msg& pcl_msg)
{
  this->StartTimeChrono = std::chrono::high_resolution_clock::now();
  this->StartTime = this->now().seconds();

  if (!this->SlamEnabled)
    return;

  CloudS::Ptr cloudS_ptr = std::make_shared<CloudS>();

  // Convert message to cloudS pointer
  pcl::fromROSMsg(pcl_msg, *cloudS_ptr);

  if(cloudS_ptr->empty())
  {
    RCLCPP_WARN(this->get_logger(), "Input point cloud sent by Lidar sensor driver is empty -> ignoring message");
    return;
  }

  // Update TF from BASE to LiDAR
  if (!this->UpdateBaseToLidarOffset(cloudS_ptr->header.frame_id))
    return;

  double timeInterval = this->Frames.empty() ? 0 : (LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp) - LidarSlam::Utils::PclStampToSec(this->Frames[0]->header.stamp));

  // Add other frames when there is more than one lidar
  if (!this->Frames.empty())
  {
    this->Frames.push_back(cloudS_ptr);
    auto check_device = this->MultiLidarsCounter.find(cloudS_ptr->header.frame_id);
    if (check_device == this->MultiLidarsCounter.end())
      this->MultiLidarsCounter.insert(std::make_pair(cloudS_ptr->header.frame_id, 1));
    else
      check_device->second++;

    // Multilidar mode: drop old frame when waiting for long time
    if (this->WaitFramesDef == LidarSlamNode::FramesCollectionMode::BY_NBLIDARS && timeInterval > this->WaitFramesTime)
    {
      auto check = this->MultiLidarsCounter.find(this->Frames[0]->header.frame_id);
      check->second--;
      if (check->second == 0)
        this->MultiLidarsCounter.erase(check);
      this->Frames.erase(this->Frames.begin());
    }
  }
  // Add first frame
  else
  {
    // Fill Frames with pointcloud
    this->Frames = {cloudS_ptr};
    this->MultiLidarsCounter.insert(std::make_pair(cloudS_ptr->header.frame_id, 1));

    this->MainLidarId = cloudS_ptr->header.frame_id;

    if (!this->UseHeaderTime)
    {
      // Compute time offset
      // Get ROS frame reception time
      double timeFrameReception = this->StartTime;
      // Get last acquired point timestamp
      double timeFrameHeader = LidarSlam::Utils::PclStampToSec(cloudS_ptr->header.stamp);
      // Compute offset
      double potentialOffset = timeFrameHeader - timeFrameReception + this->SensorTimeOffset;
      // Get current offset
      double currentOffset = std::abs(this->LidarSlam.GetSensorTimeOffset());
      // If the current computed offset is more accurate, replace it
      if (currentOffset < 1e-6 || std::abs(potentialOffset) < currentOffset)
        this->LidarSlam.SetSensorTimeOffset(potentialOffset);
    }
  }

  // Run SLAM when:
  // 1. frames are arrived from all lidar devices when frames collection mode is defined by waiting all lidar,
  // 2. time interval is greater than waiting time (0.2s) when frames collection mode is defined by waiting time
  // 3. the frame is collected when there is only one lidar
  if ((this->WaitFramesDef == LidarSlamNode::FramesCollectionMode::BY_NBLIDARS && this->MultiLidarsCounter.size() == this->MultiLidarsNb) ||
      (this->WaitFramesDef == LidarSlamNode::FramesCollectionMode::BY_TIME && this->MultiLidarsNb > 1 && timeInterval > 0.2) ||
      this->MultiLidarsNb == 1)
  {
    // Run SLAM : register new frames and update localization and map.
    this->LidarSlam.AddFrames(this->Frames);

    // Check if SLAM has failed
    if (this->LidarSlam.IsRecovery())
    {
      // TMP : in the future, the user should have a look
      // at the result to validate recovery
      // Check if the SLAM can go on and pose has to be displayed
      if (this->LidarSlam.GetOverlapEstimation() > 0.2f &&
          this->LidarSlam.GetPositionErrorStd()  < 0.1f)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Getting out of recovery mode");
        // Frame is relocalized, reset params
        this->LidarSlam.EndRecovery();
      }
      else
        RCLCPP_WARN_STREAM(this->get_logger(), "Still waiting for recovery");
    }
    else if (this->LidarSlam.HasFailed())
    {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "SLAM has failed : entering recovery mode :\n"
                         << "\t -Maps will not be updated\n"
                         << "\t -Egomotion and undistortion are disabled\n"
                         << "\t -The number of ICP iterations is increased\n"
                         << "\t -The maximum distance between a frame point and a map target point is increased");
      // Enable recovery mode :
      // Last frames are removed
      // Maps are not updated
      // Param are tuned to handle bigger motions
      // Warning : real time is not ensured
      this->LidarSlam.StartRecovery(this->RecoveryTime);
    }

    // Publish SLAM output as requested by user
    this->PublishOutput();

    // Reset frames vector and multi lidar counter
    this->Frames.clear();
    this->MultiLidarsCounter.clear();
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::ImageCallback(const sensor_msgs::msg::Image& imageMsg)
{
  double rcpTime = this->now().seconds();

  if (!this->SlamEnabled)
    return;

  #ifdef USE_CV_BRIDGE
  if (!this->UseExtSensor[LidarSlam::ExternalSensor::CAMERA])
    return;

  // Transform to apply to points represented in GPS frame to express them in base frame
  Eigen::Isometry3d baseToCamera;
  if (Utils::Tf2LookupTransform(baseToCamera, *this->TfBuffer, this->TrackingFrameId, imageMsg.header.frame_id, imageMsg.header.stamp))
  {
    if (!this->LidarSlam.CameraCanBeUsedLocally())
      this->LidarSlam.SetCameraCalibration(baseToCamera);

    cv_bridge::CvImagePtr cvPtr;
    try
    {
      cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Camera info cannot be used -> cv_bridge exception: %s", e.what());
      return;
    }

    // Add camera measurement to measurements list
    LidarSlam::ExternalSensors::Image image;
    image.Time = this->UseHeaderTime ?
                 imageMsg.header.stamp.sec + imageMsg.header.stamp.nanosec * 1e-9 :
                 rcpTime;
    image.Data = cvPtr->image;
    this->LidarSlam.AddCameraImage(image);

    if (this->LidarSlam.GetVerbosity() >= 3)
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Camera image added with time "
                         << std::fixed << std::setprecision(9)
                         << image.Time);
  }
  #else
  static_cast<void>(imageMsg);
  RCLCPP_WARN_STREAM(this->get_logger(), "cv_bridge was not found so images cannot be processed, camera will not be used");
  #endif
}

//------------------------------------------------------------------------------
void LidarSlamNode::CameraInfoCallback(const sensor_msgs::msg::CameraInfo& calibMsg)
{
  if (!this->SlamEnabled)
    return;

  #ifdef USE_CV_BRIDGE
  // The intrinsic calibration must not changed so we can only use
  // the camera info until the camera is ready to be used
  if (this->LidarSlam.CameraCanBeUsedLocally())
    return;

  Eigen::Matrix3f k;
  for (int i = 0; i < 9; ++i)
    k(int(i / 3), i % 3) = calibMsg.k[i];

  this->LidarSlam.SetCameraIntrinsicCalibration(k);
  #else
  static_cast<void>(calibMsg);
  #endif
}

//------------------------------------------------------------------------------
void LidarSlamNode::ExtPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& poseMsg)
{
  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::ExternalSensor::POSE])
    return;

  // Set calibration
  Eigen::Isometry3d baseToPose;
  if(Utils::Tf2LookupTransform(baseToPose, *this->TfBuffer, this->TrackingFrameId, this->ExtPoseFrameId, poseMsg.header.stamp))
    this->LidarSlam.SetPoseCalibration(baseToPose);

  this->MapFrameId = poseMsg.header.frame_id;

  // Get external pose
  LidarSlam::ExternalSensors::PoseMeasurement poseMeas;
  poseMeas.Pose = Utils::PoseMsgToIsometry(poseMsg.pose.pose);
  // Get external pose timestamp
  poseMeas.Time = poseMsg.header.stamp.sec + poseMsg.header.stamp.nanosec * 1e-9;

  // Get Pose covariance
  // ROS covariance message is row major
  // Eigen matrix is col major by default
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
      poseMeas.Covariance(i, j) = poseMsg.pose.covariance[i * 6 + j];
  }
  // Correct covariance if needed
  if (!LidarSlam::Utils::isCovarianceValid(poseMeas.Covariance))
    poseMeas.Covariance = LidarSlam::Utils::CreateDefaultCovariance(2e-2, 5. * M_PI/180.); // 2cm, 5°

  // Add pose measurement to measurements list
  this->LidarSlam.AddPoseMeasurement(poseMeas);

  if (this->LidarSlam.GetVerbosity() >= 3)
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "External pose added with time "
                       << std::fixed << std::setprecision(9)
                       << poseMeas.Time);
}

//------------------------------------------------------------------------------
void LidarSlamNode::GpsCallback(const nav_msgs::msg::Odometry& gpsMsg)
{
  double rcpTime = this->now().seconds();

  if (!this->SlamEnabled)
    return;

    if (!this->UseExtSensor[LidarSlam::ExternalSensor::GPS])
      return;

    // Calibration
    Eigen::Isometry3d baseToGps;
    if (Utils::Tf2LookupTransform(baseToGps, *this->TfBuffer, this->TrackingFrameId, this->GpsFrameId, gpsMsg.header.stamp))
    {
      // Get gps pose
      this->LastGpsMeas.Position = Utils::PoseMsgToIsometry(gpsMsg.pose.pose).translation();
      // Get gps timestamp
      this->LastGpsMeas.Time = this->UseHeaderTime ?
                               gpsMsg.header.stamp.sec + gpsMsg.header.stamp.nanosec * 1e-9 :
                               rcpTime;

      // Get GPS covariance
      // ROS covariance message is row major
      // Eigen matrix is col major by default
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
          this->LastGpsMeas.Covariance(i, j) = gpsMsg.pose.covariance[i * 6 + j];
      }
      // Correct GPS covariance if needed
      if (!LidarSlam::Utils::isCovarianceValid(this->LastGpsMeas.Covariance))
        this->LastGpsMeas.Covariance = Eigen::Matrix3d::Identity() * 4e-4; // 2cm

      if (!this->LidarSlam.GpsHasData())
        this->LidarSlam.SetGpsCalibration(baseToGps);

      // Add gps measurement to measurements list
      this->LidarSlam.AddGpsMeasurement(this->LastGpsMeas);
      this->MapFrameId = gpsMsg.header.frame_id;

      if (this->LidarSlam.GetVerbosity() >= 3)
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "GPS position added with time "
                           << std::fixed << std::setprecision(9)
                           << this->LastGpsMeas.Time);
    }
    else
      RCLCPP_WARN(this->get_logger(), "The transform between the GPS and the tracking frame was not found -> GPS info ignored");
}

//------------------------------------------------------------------------------
void LidarSlamNode::WheelOdomCallback(const std_msgs::msg::Float64& odomMsg)
{
  double rcpTime = this->now().seconds();

  /// TODO refact with a type "sensor message" with header frame and time
  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::ExternalSensor::WHEEL_ODOM])
    return;

  // Transform between base link and the wheel encoder
  Eigen::Isometry3d baseToWheel;
  if (Utils::Tf2LookupTransform(baseToWheel, *this->TfBuffer, this->TrackingFrameId, this->WheelFrameId))
  {
    if (!this->LidarSlam.WheelOdomCanBeUsedLocally())
      this->LidarSlam.SetWheelOdomCalibration(baseToWheel);

    // Add wheel odometer measurement to measurements list
    LidarSlam::ExternalSensors::WheelOdomMeasurement measure;
    measure.Time = rcpTime;
    measure.Distance = odomMsg.data;
    this->LidarSlam.AddWheelOdomMeasurement(measure);

    if (this->LidarSlam.GetVerbosity() >= 3)
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Wheel encoder measure added with time "
                         << std::fixed << std::setprecision(9)
                         << measure.Time);
  }
}

// ------------------------------------------------------------------------------
int LidarSlamNode::BuildId(const std::vector<int>& ids)
{
  int id = ids[0];
  if (ids.size() > 1)
  {
    id = 0;
    int power = 0;
    for (unsigned int i = 0; i < ids.size(); ++i)
    {
      power += int(std::log10(ids[i]));
      id += std::pow(10, power) * ids[i];
    }
  }
  return id;
}

//------------------------------------------------------------------------------
void LidarSlamNode::TagCallback(const apriltag_ros::msg::AprilTagDetectionArray& tagsInfo)
{
  double rcpTime = this->now().seconds();

  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR])
    return;

  for (auto& tagInfo : tagsInfo.detections)
  {
    // Transform to apply to points represented in detector frame to express them in base frame
    Eigen::Isometry3d baseToLmDetector;
    if (Utils::Tf2LookupTransform(baseToLmDetector, *this->TfBuffer, this->TrackingFrameId, tagInfo.pose.header.frame_id, tagInfo.pose.header.stamp))
    {
      LidarSlam::ExternalSensors::LandmarkMeasurement lm;
      // Get tag pose
      lm.TransfoRelative = Utils::PoseMsgToIsometry(tagInfo.pose.pose.pose);
      // Get tag timestamp
      lm.Time = this->UseHeaderTime ?
                tagInfo.pose.header.stamp.sec + tagInfo.pose.header.stamp.nanosec * 1e-9 :
                rcpTime;

      // Get tag covariance
      // ROS covariance message is row major
      // Eigen matrix is col major by default
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
          lm.Covariance(i, j) = tagInfo.pose.pose.covariance[i * 6 + j];
      }
      // Correct tag covariance if needed
      if (!LidarSlam::Utils::isCovarianceValid(lm.Covariance))
        lm.Covariance = LidarSlam::Utils::CreateDefaultCovariance(1e-2, LidarSlam::Utils::Deg2Rad(1)); // 1cm, 1°

      // Compute tag ID
      int id = this->BuildId(tagInfo.id);

      // Add detector calibration for the first tag detection
      if (!this->LidarSlam.LmHasData())
        this->LidarSlam.SetLmDetectorCalibration(baseToLmDetector);

      // Add tag detection to measurements
      this->LidarSlam.AddLandmarkMeasurement(lm, id);

      if (this->PublishTags)
      {
        // Publish tf
        double TagTimeSec = lm.Time;
        this->PublishTransformTF(TagTimeSec, this->TrackingFrameId, "tag_" + std::to_string(id), lm.TransfoRelative);
      }

      if (this->LidarSlam.GetVerbosity() >= 3)
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Tag pose added with time "
                           << std::fixed << std::setprecision(9)
                           << lm.Time);
    }
    else
      RCLCPP_WARN(this->get_logger(), "The transform between the landmark detector and the tracking frame was not found -> landmarks info ignored");
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::ImuCallback(const sensor_msgs::msg::Imu& imuMsg)
{
  double rcpTime = this->now().seconds();

  if (!this->SlamEnabled)
    return;

  if (!this->UseExtSensor[LidarSlam::ExternalSensor::IMU])
    return;

  // Calibration
  Eigen::Isometry3d baseToImu;
  if (Utils::Tf2LookupTransform(baseToImu, *this->TfBuffer, this->TrackingFrameId, imuMsg.header.frame_id, imuMsg.header.stamp))
  {
    // Set IMU calibration
    if (!this->LidarSlam.GravityHasData())
      this->LidarSlam.SetGravityCalibration(baseToImu);
    // Get IMU data for gravity integration
    // For now, only accelerations are used to add gravity constraint
    LidarSlam::ExternalSensors::GravityMeasurement gravityMeasurement;
    gravityMeasurement.Time = this->UseHeaderTime ?
                              imuMsg.header.stamp.sec + imuMsg.header.stamp.nanosec * 1e-9 :
                              rcpTime;
    gravityMeasurement.Acceleration.x() = imuMsg.linear_acceleration.x;
    gravityMeasurement.Acceleration.y() = imuMsg.linear_acceleration.y;
    gravityMeasurement.Acceleration.z() = imuMsg.linear_acceleration.z;
    this->LidarSlam.AddGravityMeasurement(gravityMeasurement);

    if (this->LidarSlam.GetVerbosity() >= 3)
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Imu acceleration added with time "
                         << std::fixed << std::setprecision(9)
                         << gravityMeasurement.Time);
  }
  else
    RCLCPP_WARN_STREAM(this->get_logger(), "The transform between the IMU and the tracking frame was not found -> IMU info ignored");
}

//------------------------------------------------------------------------------
void LidarSlamNode::ClickedPointCallback(const geometry_msgs::msg::PointStamped& pointMsg)
{
  if (!this->SlamEnabled)
    return;

  if (!this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE))
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Loop closure constraint is disabled: clicked point is not considered");
    return;
  }

  // Get the clicked point in rviz
  Eigen::Vector3d clickedPoint = Eigen::Vector3d(pointMsg.point.x, pointMsg.point.y, pointMsg.point.z);
  double time = pointMsg.header.stamp.sec + pointMsg.header.stamp.nanosec * 1e-9;
  auto itState = this->LidarSlam.GetClosestState(clickedPoint);
  if ((itState->Isometry.translation() - clickedPoint).squaredNorm() > 1.0)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "The clicked point is too far from the trajectory: Please pick another point.");
    return;
  }
  // Add clicked point as revisited frame of the loop which is formed with current frame
  LidarSlam::LoopClosure::LoopInfo loop(this->LidarSlam.GetLastState().Index, itState->Index, time);
  this->LidarSlam.AddLoopClosureIndices(loop);
  if (this->LidarSlam.GetVerbosity() >= 3)
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loop closure point added with time "
                       << std::fixed << std::setprecision(14)
                       << time);
}

//------------------------------------------------------------------------------
std::vector<std::string> LidarSlamNode::ParseSentence(std::string& currentLine,
                                                      const std::string& delimiter)
{
  if (currentLine.empty())
    return {};

  // Remove the last character if the last character is not an alphabet or a number
  if (!std::isalnum(currentLine.back()))
    currentLine.pop_back();

  // Remove potential extra spaces before and after the delimiter
  // Change comma decimal separator to point.
  auto extractWord = [&delimiter](const std::string& currentLine, unsigned int pos)
  {
    unsigned int beginIdx = 0;
    unsigned int endIdx = pos;
    while (beginIdx < endIdx && currentLine[beginIdx] == ' ')
      ++beginIdx;
    while (endIdx - 1 > beginIdx && currentLine[endIdx - 1] == ' ')
      --endIdx;
    std::string word = currentLine.substr(beginIdx, endIdx - beginIdx);
    size_t commaPos = 0;
    if (delimiter != "," && (commaPos = word.find(",")) != std::string::npos)
      word[commaPos] = '.';
    return word;
  };

  size_t pos = 0;
  std::vector<std::string> sentence;
  while ((pos = currentLine.find(delimiter)) != std::string::npos)
  {
    sentence.emplace_back(extractWord(currentLine, pos));
    currentLine.erase(0, pos + delimiter.length());
  }
  sentence.emplace_back(extractWord(currentLine, currentLine.length()));
  return sentence;
}

//------------------------------------------------------------------------------
std::vector<std::vector<std::string>> LidarSlamNode::ParseCSV(const std::string& path,
                                                              const unsigned int startLineIdx,
                                                              const std::string& delimiter)
{
  // The check of csv file path should be done out of this function
  // Get the first data line
  std::ifstream csvFile(path);
  std::string currentLine;
  for (int i = 0; i <= startLineIdx; ++i)
    std::getline(csvFile, currentLine);

  int lineIdx = startLineIdx;
  std::vector<std::vector<std::string>> lines;
  do
  {
    std::vector<std::string> sentence = this->ParseSentence(currentLine, delimiter);

    // Check numerical values in the studied line
    // If the studied line contains non numerical value, skip it
    bool numericalIssue = false;
    for (std::string field : sentence)
    {
      try
      {
        std::stof(field);
      }
      catch(std::invalid_argument& e)
      {
        numericalIssue = true;
        break;
      }
    }
    if (numericalIssue)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Data on line " << lineIdx << " contains a not numerical value -> Skip");
      ++lineIdx;
      continue;
    }

    lines.emplace_back(sentence);
    ++lineIdx;
  }
  while (std::getline(csvFile, currentLine));

  return lines;
}

//------------------------------------------------------------------------------
std::vector<std::vector<std::string>> LidarSlamNode::ReadCSV(const std::string& path,
                                                             const unsigned int nbHeaderLines,
                                                             const unsigned int nbFields)
{
  // Check the file
  if (path.substr(path.find_last_of(".") + 1) != "csv")
  {
    RCLCPP_ERROR(this->get_logger(), "The file is not CSV! Cancel loading");
    return {};
  }

  std::ifstream csvFile(path);
  if (csvFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "The CSV file " << path << " was not found!");
    return {};
  }

  // Get the first data line (after header)
  std::string firstLine;
  for (int i = 0; i <= nbHeaderLines; ++i)
    std::getline(csvFile, firstLine);
  csvFile.close();

  // Check which delimiter is used
  std::string delimiter;
  std::vector<std::string> fields;
  for (const auto& del : DELIMITERS)
  {
    fields.clear();
    size_t pos = 0;
    pos = firstLine.find(del);
    while (pos != std::string::npos)
    {
      fields.emplace_back(firstLine.substr(0, pos));
      firstLine.erase(0, pos + del.length());
      pos = firstLine.find(del);
    }
    // If there is some element after the last delimiter, add it
    if (!firstLine.substr(0, pos).empty())
      fields.emplace_back(firstLine.substr(0, pos));
    // Check that the number of fields is correct
    if (fields.size() == nbFields)
    {
      delimiter = del;
      break;
    }
  }
  if (fields.size() != nbFields)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "The CSV file is ill formed : " << fields.size() << " fields were found ("
                                            << nbFields << " expected), the loading is cancelled");
    return {};
  }

  // Get vector of numerical sentences from the csv file
  auto lines = this->ParseCSV(path, nbHeaderLines, delimiter);

  // Check field number for each sentence
  auto itLine = lines.begin();
  while (itLine != lines.end())
  {
    if (itLine->size() != nbFields)
      lines.erase(itLine);
    else
      ++itLine;
  }

  return lines;
}
//------------------------------------------------------------------------------
std::vector<std::vector<std::string>> LidarSlamNode::ReadCSV(const std::string& path,
                                                             const unsigned int nbHeaderLines,
                                                             const std::vector<std::string>& fieldsToCheck)
{
  if (fieldsToCheck.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "There is no field to check! Cancel loading csv file!");
    return {};
  }

  // Check the file
  if (path.substr(path.find_last_of(".") + 1) != "csv")
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "The file is not CSV! Cancel loading");
    return {};
  }
  std::ifstream csvFile(path);
  if (csvFile.fail())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "The CSV file " << path << " was not found!");
    return {};
  }

  // Find the delimiter
  // In the header line, find the position of a field of vector fieldsToCheck
  // Examine the left and the right of a field in the fieldToCheck to find the delimiter
  std::string headerLine;
  for (int i = 0; i < nbHeaderLines; ++i)
    std::getline(csvFile, headerLine);
  csvFile.close();
  // Find the real position of a field where the field is not included in a word
  const std::string& field = fieldsToCheck[0];
  size_t pos = headerLine.find(field);
  while (pos != std::string::npos &&
         ((pos > 0 && std::isalnum(headerLine[pos - 1])) ||
          (pos < headerLine.length() - field.length() && std::isalnum(headerLine[pos + field.length()]))))
  {
    pos = headerLine.find(field, pos + field.length());
  }
  if (pos == std::string::npos)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "The CSV file is ill formed : " << fieldsToCheck[0] << " field is not found,"
                                           << " the loading is cancelled");
    return {};
  }

  // Helper to check delimiter to the left or to the right
  auto checkSide = [](const std::string &header, size_t newPos, bool isLeftSide) -> std::string
  {
    std::string sideStr = header.substr(newPos, 1);
    if (sideStr == " ")
    {
      // If the character is a space, check for consecutive spaces
      size_t consecutiveSpaces = isLeftSide ?  header.find_last_not_of(" ", newPos)
                                 : header.find_first_not_of(" ", newPos);
      if (consecutiveSpaces == std::string::npos || std::isalnum(header[consecutiveSpaces]))
        return sideStr;
      else if (consecutiveSpaces != std::string::npos &&
                std::find(DELIMITERS.begin(), DELIMITERS.end(), header.substr(consecutiveSpaces, 1)) != DELIMITERS.end())
        return header.substr(consecutiveSpaces, 1);
    }
    else if (std::find(DELIMITERS.begin(), DELIMITERS.end(), sideStr) != DELIMITERS.end())
    {
      return sideStr;
    }
    return "";
  };
  // Check the character to the left
  std::string delimiter = "";
  if (pos > 0)
  {
    std::string leftDelimiter = checkSide(headerLine, pos - 1, true);
    if (!leftDelimiter.empty())
      delimiter = leftDelimiter;
  }
  // Check the character to the right if the delimiter is not found at the left
  if (delimiter.empty() && (pos + field.length() < headerLine.length() - field.length()))
  {
    std::string rightDelimiter = checkSide(headerLine, pos + field.length(), false);
    if (!rightDelimiter.empty())
      delimiter = rightDelimiter;
  }
  if (delimiter.empty())
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "The CSV file is ill formed :  the delimiter is not found,"
                                           << " the loading is cancelled");
    return {};
  }

  // Check fields and store their position indices
  std::vector<std::string> headerFields = this->ParseSentence(headerLine, delimiter);
  std::vector<int> fieldsIdx;
  fieldsIdx.reserve(fieldsToCheck.size());
  for (const auto& field : fieldsToCheck)
  {
    auto itField = std::find(headerFields.begin(), headerFields.end(), field);
    if (itField == headerFields.end())
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "The CSV file is ill formed : " << field << " field is not found,"
                                             << " the loading is cancelled");
      return {};
    }
    fieldsIdx.push_back(itField - headerFields.begin());
  }

  // Get vector of sentences
  auto lines = this->ParseCSV(path, nbHeaderLines, delimiter);
  // Reorder lines
  for (int col = 0; col < fieldsIdx.size(); col++)
  {
    int swapId = std::find(fieldsIdx.begin(), fieldsIdx.end(), col) - fieldsIdx.begin();
    for (int row = 0; row < lines.size(); row++)
    {
      std::swap(lines[row][col], lines[row][fieldsIdx.at(col)]);
    }
    std::swap(fieldsIdx[col], fieldsIdx[swapId]);
  }

  return lines;
}

//------------------------------------------------------------------------------
std::string LidarSlamNode::ReadPoses(const std::string& path, bool resetTraj)
{
  std::vector<std::string> fieldsToCheck{"Time","X","Y","Z","Rx(Roll)","Ry(Pitch)","Rz(Yaw)"};
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 2, fieldsToCheck);
  if (lines.empty())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot read file :" << path << ", poses are not loaded");
    return "";
  }

  // Get frame ID
  std::ifstream poseFile(path);
  std::string frameID;
  std::getline(poseFile, frameID);
  for (const auto& del : DELIMITERS)
  {
    if (frameID.find(del) != std::string::npos)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Frame ID is not specified in the first line of the CSV file, stop loading");
      return "";
    }
  }
  // Remove the new line character in frameID string
  if (!std::isalnum(frameID.back()))
    frameID.pop_back();

  // Reset time offset if used by another sensor
  double timeOffsetTmp = this->LidarSlam.GetSensorTimeOffset();
  this->LidarSlam.SetSensorTimeOffset(0.);

  LidarSlam::ExternalSensors::PoseManager trajectoryManager("new trajectory");
  trajectoryManager.SetVerbose(true);
  trajectoryManager.SetDistanceThreshold(std::max(2., 2 * this->LidarSlam.GetKfDistanceThreshold()));
  for (auto& l : lines)
  {
    // Build pose measurement
    LidarSlam::ExternalSensors::PoseMeasurement poseMeas;
    // Time
    poseMeas.Time = std::stod(l[0]);
    // Pose
    poseMeas.Pose = LidarSlam::Utils::XYZRPYtoIsometry(std::stof(l[1]), std::stof(l[2]), std::stof(l[3]), std::stof(l[4]), std::stof(l[5]), std::stof(l[6]));
    if (resetTraj)
      trajectoryManager.AddMeasurement(poseMeas);
    else
      this->LidarSlam.AddPoseMeasurement(poseMeas);
  }

  if (resetTraj)
  {
    this->LidarSlam.ResetStatePoses(trajectoryManager);
    RCLCPP_INFO_STREAM(this->get_logger(), "Trajectory successfully loaded!");
  }

  // Reset time offset as before
  this->LidarSlam.SetSensorTimeOffset(timeOffsetTmp);

  return frameID;
}

//------------------------------------------------------------------------------
void LidarSlamNode::ReadTags(const std::string& path)
{
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 1, 43);
  for (auto& l : lines)
  {
    // Build measurement
    // Set landmark id
    int id = std::stoi(l[0]);
    // Fill pose
    Eigen::Vector6d absolutePose;
    absolutePose << std::stof(l[1]), std::stof(l[2]), std::stof(l[3]), std::stof(l[4]), std::stof(l[5]), std::stof(l[6]);
    // Fill covariance
    Eigen::Matrix6d absolutePoseCovariance;
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        absolutePoseCovariance(i, j) = std::stof(l[7 + 6 * i + j]);
    }
    // Add a new landmark manager for absolute constraint computing
    this->LidarSlam.AddLandmarkManager(id, absolutePose, absolutePoseCovariance);
    RCLCPP_INFO_STREAM(this->get_logger(), "Tag #" << id << " initialized to \n" << absolutePose.transpose());
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::ReadLoopIndices(const std::string& path)
{
  std::vector<std::vector<std::string>> lines = this->ReadCSV(path, 1, 2);
  if (lines.empty())
  {
    const std::list<LidarSlam::LidarState>& lidarStates = this->LidarSlam.GetLogStates();
    if (lidarStates.size()<2)
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Cannot read loop closure indices file and no enough states logged, no loop closure indices will be loaded");
      return;
    }
    // Use the first and the last frame indices as loop closure indices
    LidarSlam::LoopClosure::LoopInfo loop(lidarStates.back().Index, lidarStates.front().Index, -1);
    this->LidarSlam.AddLoopClosureIndices(loop);
    RCLCPP_INFO_STREAM(this->get_logger(), "Loop closure file is empty:\n"
                        <<" the last and the first states are used");
    return;
  }

  for (auto& l : lines)
  {
    LidarSlam::LoopClosure::LoopInfo loop(std::stoi(l[0]), std::stoi(l[1]), -1);
    // Add a new loop closure indices into LoopDetections
    this->LidarSlam.AddLoopClosureIndices(loop);
    RCLCPP_INFO_STREAM(this->get_logger(), "Query id #" << loop.QueryIdx << " Revisited id #" << loop.RevisitedIdx<< ".");
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Loop closure indices have been loaded successfully from: " << path);
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  // Get offset between Lidar SLAM ref frame (odom)
  // and pose message reference frame (msg.header.frame_id)
  Eigen::Isometry3d offset;
  if (Utils::Tf2LookupTransform(offset, *this->TfBuffer, this->OdometryFrameId, msg.header.frame_id, msg.header.stamp))
  {
    // Compute pose in odometry frame and set SLAM pose
    Eigen::Isometry3d poseInOdom = offset * Utils::PoseMsgToIsometry(msg.pose.pose);
    this->LidarSlam.SetCurrentPose(poseInOdom);
    RCLCPP_WARN_STREAM(this->get_logger(), "SLAM pose set to :\n" << poseInOdom.matrix());
    // TODO: properly deal with covariance: rotate it, pass it to SLAM, notify trajectory jump?
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SavePointcloudService(
    const std::shared_ptr<lidar_slam::srv::SavePc::Request> req,
    const std::shared_ptr<lidar_slam::srv::SavePc::Response> res)
{
  std::filesystem::path outputPrefixPath;
  if (!req->output_prefix_path.empty())
    outputPrefixPath = std::filesystem::path(req->output_prefix_path);
  else
  {
    const char* home = nullptr;
    #ifdef _WIN32
      home = std::getenv("USERPROFILE");
    #else
      home = std::getenv("HOME");
    #endif
    outputPrefixPath = home ? std::filesystem::path(home) : std::filesystem::current_path();
    outputPrefixPath /= "keypoint_maps";
  }

  if (!std::filesystem::exists(outputPrefixPath.parent_path()))
  {
    std::filesystem::create_directories(outputPrefixPath.parent_path());
    RCLCPP_WARN_STREAM(this->get_logger(), "Output folder does not exist, creating new folder :" << outputPrefixPath.parent_path());
  }

  LidarSlam::PCDFormat pcdFormat = static_cast<LidarSlam::PCDFormat>(req->format);
  if (pcdFormat != LidarSlam::PCDFormat::ASCII &&
      pcdFormat != LidarSlam::PCDFormat::BINARY &&
      pcdFormat != LidarSlam::PCDFormat::BINARY_COMPRESSED)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Incorrect PCD format value (" << pcdFormat << "). Setting it to 'BINARY_COMPRESSED'.");
    pcdFormat = LidarSlam::PCDFormat::BINARY_COMPRESSED;
  }

  bool filtered = req->filtered;
  bool fixed = req->fixed;
  this->LidarSlam.SaveMapsToPCD(outputPrefixPath.string(), pcdFormat, filtered, fixed);
  RCLCPP_INFO_STREAM(this->get_logger(), "Pointcloud saved to " << outputPrefixPath.string());

  if (this->LidarSlam.GetMapUpdate() == LidarSlam::MappingMode::NONE)
    RCLCPP_WARN_STREAM(this->get_logger(), "The initially loaded maps were not modified but are saved anyway.");

  res->success = true;
}

//------------------------------------------------------------------------------
void LidarSlamNode::ResetService(const std::shared_ptr<lidar_slam::srv::Reset::Request> req,
                                 const std::shared_ptr<lidar_slam::srv::Reset::Response> res)
{
  RCLCPP_WARN(this->get_logger(), "Resetting the SLAM internal state.");
  this->LidarSlam.Reset(true);
  this->SetSlamInitialState();
  res->success = true;
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetInitialStateService(const std::shared_ptr<lidar_slam::srv::SetInitialState::Request> req,
                                           const std::shared_ptr<lidar_slam::srv::SetInitialState::Response> res)
{
  // Reset SLAM
  this->LidarSlam.Reset(true);

  // Set initial SLAM pose
  if (!req->initial_pose.header.frame_id.empty())
  {
    Eigen::Isometry3d initialTransform = Utils::PoseMsgToIsometry(req->initial_pose.pose.pose);
    // Get initial pose represented in Lidar SLAM ref frame (odom)
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    if (Utils::Tf2LookupTransform(offset, *this->TfBuffer, this->OdometryFrameId, req->initial_pose.header.frame_id, req->initial_pose.header.stamp))
      initialTransform = offset * initialTransform;

    // Update initial_pose parameter
    Eigen::Vector6d initialPose6D = LidarSlam::Utils::IsometryToXYZRPY(initialTransform);
    std::vector<double> pose(initialPose6D.data(), initialPose6D.data() + initialPose6D.size());
    rclcpp::Parameter updated_param("maps.initial_pose", pose);
    this->set_parameter(updated_param);
  }

  // Load initial SLAM maps if requested
  std::string& mapsPathPrefix = req->map_prefix_path;
  if (!mapsPathPrefix.empty())
  {
    // Update initial_maps path
    rclcpp::Parameter updated_param("maps.initial_maps", mapsPathPrefix);
    this->set_parameter(updated_param);
  }
  bool keepLabel = req->keep_label;
  rclcpp::Parameter updated_param("maps.keep_label", keepLabel);
  this->set_parameter(updated_param);

  // Set slam initial states
  this->SetSlamInitialState();

  // Send response
  res->success = true;
}

//------------------------------------------------------------------------------
void LidarSlamNode::DetectLoopClosureService(const std::shared_ptr<lidar_slam::srv::DetectLoop::Request> req,
                                             const std::shared_ptr<lidar_slam::srv::DetectLoop::Response> res)
{
  const std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
  if (states.size() < 2)
  {
    RCLCPP_WARN(this->get_logger(), "Not enough logged states to detect loop closure");
    res->success = false;
    return;
  }
  this->LidarSlam.AutoDetectLoopClosure();
  res->success = true;
}

//------------------------------------------------------------------------------
void LidarSlamNode::SlamCommandCallback(const lidar_slam::msg::SlamCommand& msg)
{
  // Parse command
  switch(msg.command)
  {
    // Disable SLAM maps update
    case lidar_slam::msg::SlamCommand::DISABLE_SLAM_MAP_UPDATE:
    {
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::NONE);
      RCLCPP_WARN(this->get_logger(), "Disabling SLAM maps update.");
      break;
    }

    // Enable the agregation of keypoints to a fixed initial map
    case lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_EXPANSION:
    {
      if (this->LidarSlam.IsRecovery())
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot unable map expansion in recovery mode!");
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP);
      RCLCPP_WARN(this->get_logger(), "Enabling SLAM maps expansion with new keypoints.");
      break;
    }

    // Enable the update of the map with new keypoints
    case lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_UPDATE:
    {
      if (this->LidarSlam.IsRecovery())
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot unable map update in recovery mode!");
      this->LidarSlam.SetMapUpdate(LidarSlam::MappingMode::UPDATE);
      RCLCPP_WARN(this->get_logger(), "Enabling SLAM maps update with new keypoints.");
      break;
    }

    // Reset slam trajectory and update map
    case lidar_slam::msg::SlamCommand::RESET_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No path is specified, the trajectory cannot be reset");
        break;
      }
      this->ReadPoses(msg.string_arg, true);

      this->LidarSlam.ClearLoopDetections();
      RCLCPP_INFO_STREAM(this->get_logger(), "Loop indices are cleared");

      break;
    }

    // Reset slam trajectory and update map
    case lidar_slam::msg::SlamCommand::RESET_ODOM:
    {
      Eigen::Isometry3d mapToOdom;
      if (!Utils::Tf2LookupTransform(mapToOdom, *this->TfBuffer, this->MapFrameId, this->OdometryFrameId))
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No transform from " << this->MapFrameId << " to " << this->OdometryFrameId << " : cannot reset ODOM.");
        break;
      }

      LidarSlam::LidarState state = this->LidarSlam.GetLastState();
      this->PublishTransformTF(state.Time, this->MapFrameId, this->OdometryFrameId, mapToOdom * state.Isometry);
      this->LidarSlam.SetCurrentPose(Eigen::Isometry3d::Identity());
      RCLCPP_INFO_STREAM(this->get_logger(), "ODOM has been moved, SLAM pose is now identity");
      break;
    }

    // Enable/Disable the SLAM process
    case lidar_slam::msg::SlamCommand::SWITCH_ON_OFF:
    {
      if (this->SlamEnabled)
        RCLCPP_WARN_STREAM(this->get_logger(), "Disabling the SLAM process");
      else
        RCLCPP_WARN_STREAM(this->get_logger(), "Enabling again the SLAM process");

      this->SlamEnabled = !this->SlamEnabled;
      break;
    }

    // Save current trajectory tracking base frame
    case lidar_slam::msg::SlamCommand::SAVE_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No path is specified, the trajectory cannot be saved");
        return;
      }
      std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
      if (states.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No current trajectory logged, nothing will be saved");
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving current trajectory of base frame as " << msg.string_arg);
      std::ofstream fout(msg.string_arg);
      fout << this->TrackingFrameId << "\n";
      fout << "index,Time,X,Y,Z,Rx(Roll),Ry(Pitch),Rz(Yaw)\n";
      for (auto& s : states)
      {
        auto pose6D = LidarSlam::Utils::IsometryToXYZRPY(s.Isometry);
        fout << s.Index << "," << std::fixed << std::setprecision(9) << s.Time << std::scientific << ","
             << pose6D(0) << "," << pose6D(1) << "," << pose6D(2) << ","
             << pose6D(3) << "," << pose6D(4) << "," << pose6D(5) << "\n";
      }
      fout.close();
      break;
    }

    // Save current trajectory tracking Lidar
    case lidar_slam::msg::SlamCommand::SAVE_LIDAR_TRAJECTORY:
    {
      if (msg.string_arg.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No path is specified, the trajectory cannot be saved");
        return;
      }
      Eigen::Isometry3d baseToLidar;
      if (!Utils::Tf2LookupTransform(baseToLidar, *this->TfBuffer, this->TrackingFrameId, this->MainLidarId))
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No transform from base to Lidar : cannot save Lidar trajectory.");
        return;
      }
      std::list<LidarSlam::LidarState> states = this->LidarSlam.GetLogStates();
      if (states.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No current trajectory logged, nothing will be saved");
        return;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Saving current trajectory of the Lidar sensor as " << msg.string_arg);
      std::ofstream fout(msg.string_arg);
      fout << this->MainLidarId << "\n";
      fout << "Time,X,Y,Z,Rx(Roll),Ry(Pitch),Rz(Yaw)\n";
      for (auto& s : states)
      {
        s.Isometry = s.Isometry * baseToLidar;
        auto pose6D = LidarSlam::Utils::IsometryToXYZRPY(s.Isometry);
        fout << s.Index << "," << std::fixed << std::setprecision(9) << s.Time << std::scientific << ","
             << pose6D(0) << "," << pose6D(1) << "," << pose6D(2) << ","
             << pose6D(3) << "," << pose6D(4) << "," << pose6D(5) << "\n";
        fout << s;
      }
      fout.close();
      break;
    }

    case lidar_slam::msg::SlamCommand::OPTIMIZE_GRAPH:
    {

      if ((!this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS) &&
           !this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LANDMARK) &&
           !this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE) &&
           !this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE)) ||
           this->LidarSlam.GetSensorMaxMeasures() < 2 || this->LidarSlam.GetLoggingTimeout() < 0.2)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Cannot optimize pose graph: "
                            "Please make sure that one external sensor or "
                            "loop closure has been enabled and 'logging/timeout' "
                            "parameters are set to convenient values.");
        break;
      }

      if (this->LidarSlam.LmHasData())
      {
        if (!msg.string_arg.empty())
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "Loading the absolute landmark poses");
          this->ReadTags(msg.string_arg);
        }
        else if (this->LidarSlam.GetLandmarkConstraintLocal())
          RCLCPP_WARN(this->get_logger(), "No absolute landmark poses are supplied : the last estimated poses will be used");
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "Optimizing the pose graph");
      if (!this->LidarSlam.OptimizeGraph())
        break;

      if (this->LidarSlam.GpsHasData() &&
          this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS))
      {
        // Broadcast new calibration offset (GPS ref to odom)
        // which has been computed/refined with pose graph optimization
        // Note : the offset in the library is defined as odom to GPS ref
        // for computation simplications
        Eigen::Isometry3d offset = this->LidarSlam.GetGpsOffset().inverse();
        PublishTransformTF(this->LastGpsMeas.Time, this->MapFrameId, this->OdometryFrameId, offset);
      }

      if (this->LidarSlam.PoseHasData() &&
          this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE))
      {
        // Broadcast new calibration offset (Pose ref to odom)
        // which has been computed/refined with pose graph optimization
        // Note : the offset in the library is defined as odom to external sensor ref
        // for computation simplications
        Eigen::Isometry3d offset = this->LidarSlam.GetPoseOffset().inverse();
        PublishTransformTF(this->LastGpsMeas.Time, this->MapFrameId, this->OdometryFrameId, offset);
      }

      // Publish new trajectory
      if (this->Publish[PGO_PATH])
      {
        nav_msgs::msg::Path optimSlamTraj;
        optimSlamTraj.header.frame_id = this->OdometryFrameId;
        std::list<LidarSlam::LidarState> optimizedSlamStates = this->LidarSlam.GetLogStates();
        optimSlamTraj.header.stamp = rclcpp::Time(optimizedSlamStates.back().Time * 1e9);
        for (const LidarSlam::LidarState& s: optimizedSlamStates)
          optimSlamTraj.poses.emplace_back(Utils::IsometryToPoseStampedMsg(s.Isometry, s.Time, this->OdometryFrameId));
        publishWithCast(this->Publishers[PGO_PATH], nav_msgs::msg::Path, optimSlamTraj);
      }
      break;
    }

    case lidar_slam::msg::SlamCommand::LOAD_LOOP_INDICES:
    {
      if (!this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE))
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Loop closure constraint is disabled: loading loop indices is cancelled");
        break;
      }
      // Clear current LoopDetections vector
      this->LidarSlam.ClearLoopDetections();
      if (!msg.string_arg.empty())
      {
        this->LidarSlam.SetLoopDetector(LidarSlam::LoopClosureDetector::EXTERNAL);
        // Fill external pose manager with poses from a CSV file
        this->ReadLoopIndices(msg.string_arg);
      }
      else
      {
        const std::list<LidarSlam::LidarState>& lidarStates = this->LidarSlam.GetLogStates();
        if (lidarStates.size()<2)
        {
          RCLCPP_WARN_STREAM(this->get_logger(), "Not enough states logged and no external file provided, no loop closure indices will be loaded");
          break;
        }
        // Use the first and the last frame indices as loop closure indices
        LidarSlam::LoopClosure::LoopInfo loop(lidarStates.back().Index, lidarStates.front().Index, -1);
        this->LidarSlam.AddLoopClosureIndices(loop);
        RCLCPP_INFO_STREAM(this->get_logger(), "No external file is provided for loop closure indices : "
                           <<"the last and the first states are used");
      }
      break;
    }

    case lidar_slam::msg::SlamCommand::SWITCH_SENSOR:
    {
      // Get sensor to enable/disable
      LidarSlam::ExternalSensor sensor;
      try
      {
        sensor = static_cast<LidarSlam::ExternalSensor>(std::stoi(msg.string_arg));
      }
      catch(...)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "External sensor #" << msg.string_arg << " does not exist.");
        break;
      }

      std::string onOff;
      onOff = this->UseExtSensor[sensor]? "Disabling " : "Enabling ";
      RCLCPP_INFO_STREAM(this->get_logger(), onOff << LidarSlam::ExternalSensorNames.at(sensor));
      this->UseExtSensor[sensor] = !this->UseExtSensor[sensor];
      break;
    }

    case lidar_slam::msg::SlamCommand::CALIBRATE_WITH_POSES:
    {
      // If an input file is provided, load the poses
      if (!msg.string_arg.empty())
      {
        // Clear current pose manager
        this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);
        // Fill external pose manager with poses from a CSV file
        this->ExtPoseFrameId = this->ReadPoses(msg.string_arg);
        if (this->ExtPoseFrameId.empty())
          return;
      }
      else if (!this->LidarSlam.PoseHasData())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No file provided and no poses registered, "
                            << "the calibration cannot be performed");
        break;
      }
      // Calibrate the external poses with current SLAM trajectory
      this->LidarSlam.CalibrateWithExtPoses(this->CalibrationWindow, this->LeverArm, false, this->PlanarTrajectory);      // Get the calibration
      Eigen::Isometry3d calibration = this->LidarSlam.GetPoseCalibration();

      //Publish new static TF
      geometry_msgs::msg::TransformStamped tfStamped;
      tfStamped.header.stamp = this->now();
      tfStamped.header.frame_id = this->TrackingFrameId;
      tfStamped.child_frame_id = this->ExtPoseFrameId;
      tfStamped.transform = Utils::IsometryToTfMsg(calibration);
      this->StaticTfBroadcaster->sendTransform(tfStamped);

      RCLCPP_INFO_STREAM(this->get_logger(), "Calibration estimated to :\n" << calibration.matrix());
      // Clean the pose manager in the SLAM
      this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);
      break;
    }

    case lidar_slam::msg::SlamCommand::LOAD_POSES:
    {
      // Clear current pose manager
      this->LidarSlam.ResetSensor(true, LidarSlam::ExternalSensor::POSE);

      // If an input file is provided, load the poses
      if (msg.string_arg.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "No file provided, cannot load poses");
        break;
      }

      // Fill external pose manager with poses from a CSV file
      this->ExtPoseFrameId = this->ReadPoses(msg.string_arg);
      if (this->ExtPoseFrameId.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "External file does not contain a "
                                                << "frame ID in its header, external "
                                                << "poses are not loaded.");
        break;
      }

      Eigen::Isometry3d baseToExtPoseSensor = Eigen::Isometry3d::Identity();
      if (!Utils::Tf2LookupTransform(baseToExtPoseSensor, *this->TfBuffer, this->TrackingFrameId, this->ExtPoseFrameId))
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No calibration found in TF between " << this->TrackingFrameId << " and " << this->ExtPoseFrameId
                                               << "\n\tCalibration set to identity.");
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Calibration for ext poses set to :\n" << baseToExtPoseSensor.matrix());
      this->LidarSlam.SetPoseCalibration(baseToExtPoseSensor);

      RCLCPP_INFO_STREAM(this->get_logger(), "External poses loaded!");

      break;
    }

    // Unknown command
    default:
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown SLAM command : " << (unsigned int) msg.command);
      break;
  }
}

//==============================================================================
//   Utilities
//==============================================================================

//------------------------------------------------------------------------------
bool LidarSlamNode::UpdateBaseToLidarOffset(const std::string& lidarFrameId)
{
  // If tracking frame is different from input frame, get TF from LiDAR to BASE
  if (lidarFrameId != this->TrackingFrameId)
  {
    // We expect a static transform between BASE and LIDAR, so we don't care
    // about timestamp and get only the latest transform
    Eigen::Isometry3d baseToLidar;
    if (Utils::Tf2LookupTransform(baseToLidar, *this->TfBuffer, this->TrackingFrameId, lidarFrameId))
      this->LidarSlam.SetBaseToLidarOffset(baseToLidar, lidarFrameId);
    else
      return false;
  }
  return true;
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishOutput()
{
  // Get current SLAM poses in WORLD coordinates at the specified frequency
  std::vector<LidarSlam::LidarState> lastStates = this->LidarSlam.GetLastStates(this->TrajFrequency);
  std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now()
                                                                                                     - this->StartTimeChrono);
  double computationTime = duration.count();

  // Publish SLAM pose
  if (this->Publish[POSE_ODOM] || this->Publish[POSE_TF])
  {
    for (const auto& state : lastStates)
    {
      // Publish as odometry msg
      if (this->Publish[POSE_ODOM])
      {
        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = rclcpp::Time(state.Time * 1e9);
        odomMsg.header.frame_id = this->OdometryFrameId;
        odomMsg.child_frame_id = this->TrackingFrameId;
        odomMsg.pose.pose = Utils::IsometryToPoseMsg(state.Isometry);
        // Set covariance
        // ROS covariance message is row major
        // Eigen matrix is col major by default
        for (int i = 0; i < 6; ++i)
        {
          for (int j = 0; j < 6; ++j)
            odomMsg.pose.covariance[i * 6 + j] = state.Covariance(i, j);
        }
        publishWithCast(this->Publishers[POSE_ODOM], nav_msgs::msg::Odometry, odomMsg);
      }

      // Publish as TF from OdometryFrameId to TrackingFrameId
      if (this->Publish[POSE_TF])
        this->PublishTransformTF(state.Time, this->OdometryFrameId, this->TrackingFrameId, state.Isometry);

      // Enable subscribers to receive those messages
      // Warning : this may alter cout working in this code area
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // Publish latency compensated SLAM pose
  if (this->Publish[POSE_PREDICTION_ODOM] || this->Publish[POSE_PREDICTION_TF])
  {
    double predTime = lastStates.back().Time + computationTime;
    Eigen::Isometry3d predIsometry = this->LidarSlam.GetTworld(predTime);

    // Publish as odometry msg
    if (this->Publish[POSE_PREDICTION_ODOM])
    {
      nav_msgs::msg::Odometry odomMsg;
      odomMsg.header.stamp = rclcpp::Time(predTime * 1e9);
      odomMsg.header.frame_id = this->OdometryFrameId;
      odomMsg.child_frame_id = this->TrackingFrameId + "_prediction";
      odomMsg.pose.pose = Utils::IsometryToPoseMsg(predIsometry);
      // Set covariance
      // ROS covariance message is row major
      // Eigen matrix is col major by default
      for (int i = 0; i < 6; ++i)
      {
        for (int j = 0; j < 6; ++j)
          odomMsg.pose.covariance[i * 6 + j] = lastStates.back().Covariance(i, j);
      }
      publishWithCast(this->Publishers[POSE_PREDICTION_ODOM], nav_msgs::msg::Odometry, odomMsg);
    }

    // Publish as TF from OdometryFrameId to <TrackingFrameId>_prediction
    if (this->Publish[POSE_PREDICTION_TF])
      this->PublishTransformTF(predTime, this->OdometryFrameId, this->TrackingFrameId + "_prediction", predIsometry);
  }


  // Publish a pointcloud only if required and if someone is listening to it to spare bandwidth.
  // Change to publish pcl2 msgs
  // Url : https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
  #define publishPointCloud(publisher, pc)                                                  \
  if (this->Publish[publisher] && this->Publishers[publisher]->get_subscription_count())    \
    {                                                                                       \
      Pcl2_msg pcl_msg;                                                                     \
      pcl::toROSMsg(*pc, pcl_msg);                                                          \
      publishWithCast(this->Publishers[publisher], Pcl2_msg, pcl_msg)                       \
    }

  // Keypoints maps
  publishPointCloud(EDGES_MAP,           this->LidarSlam.GetMap(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGES_MAP, this->LidarSlam.GetMap(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANES_MAP,          this->LidarSlam.GetMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_MAP,           this->LidarSlam.GetMap(LidarSlam::BLOB));

  // Keypoints submaps
  publishPointCloud(EDGES_SUBMAP,           this->LidarSlam.GetTargetSubMap(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGES_SUBMAP, this->LidarSlam.GetTargetSubMap(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANES_SUBMAP,          this->LidarSlam.GetTargetSubMap(LidarSlam::PLANE));
  publishPointCloud(BLOBS_SUBMAP,           this->LidarSlam.GetTargetSubMap(LidarSlam::BLOB));

  // Current keypoints
  publishPointCloud(EDGE_KEYPOINTS,           this->LidarSlam.GetKeypoints(LidarSlam::EDGE));
  publishPointCloud(INTENSITY_EDGE_KEYPOINTS, this->LidarSlam.GetKeypoints(LidarSlam::INTENSITY_EDGE));
  publishPointCloud(PLANE_KEYPOINTS,          this->LidarSlam.GetKeypoints(LidarSlam::PLANE));
  publishPointCloud(BLOB_KEYPOINTS,           this->LidarSlam.GetKeypoints(LidarSlam::BLOB));

  // Registered aggregated (and optionally undistorted) input scans points
  publishPointCloud(SLAM_REGISTERED_POINTS, this->LidarSlam.GetRegisteredFrame());

  // Overlap estimation
  if (this->Publish[CONFIDENCE])
  {
    // Get SLAM pose
    lidar_slam::msg::Confidence confidenceMsg;
    confidenceMsg.header.stamp = rclcpp::Time(lastStates.back().Time * 1e9);
    confidenceMsg.header.frame_id = this->OdometryFrameId;
    confidenceMsg.overlap = this->LidarSlam.GetOverlapEstimation();
    confidenceMsg.computation_time = computationTime;
    // Set confidence covariance
    // ROS covariance message is row major
    // Eigen matrix is col major by default
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
        confidenceMsg.covariance[i * 6 + j] = lastStates.back().Covariance(i, j);
    }

    confidenceMsg.nb_matches = this->LidarSlam.GetTotalMatchedKeypoints();
    confidenceMsg.comply_motion_limits = this->LidarSlam.GetComplyMotionLimits();
    confidenceMsg.std_position_error = this->LidarSlam.GetPositionErrorStd();
    confidenceMsg.failure = this->LidarSlam.HasFailed() || this->LidarSlam.IsRecovery();
    publishWithCast(this->Publishers[CONFIDENCE], lidar_slam::msg::Confidence, confidenceMsg);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamParameters()
{
  #define SetSlamParam(type, rosParam, slamParam) { type val; if (this->get_parameter<type>(rosParam, val)) this->LidarSlam.Set##slamParam(val); }
  // General
  SetSlamParam(bool,   "slam.2d_mode", TwoDMode)
  SetSlamParam(int,    "slam.verbosity", Verbosity)
  SetSlamParam(int,    "slam.n_threads", NbThreads)
  SetSlamParam(double, "logging.timeout", LoggingTimeout)
  SetSlamParam(bool,   "logging.only_keyframes", LogOnlyKeyframes)

  int egoMotionMode;
  if (this->get_parameter<int>("slam.ego_motion", egoMotionMode))
  {
    LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(egoMotionMode);
    if (egoMotion != LidarSlam::EgoMotionMode::NONE &&
        egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
        egoMotion != LidarSlam::EgoMotionMode::REGISTRATION &&
        egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION &&
        egoMotion != LidarSlam::EgoMotionMode::EXTERNAL &&
        egoMotion != LidarSlam::EgoMotionMode::EXTERNAL_OR_MOTION_EXTRAPOLATION)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid ego-motion mode (" << egoMotionMode << "). Setting it to 'MOTION_EXTRAPOLATION'.");
      egoMotion = LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION;
    }
    this->LidarSlam.SetEgoMotion(egoMotion);
  }

  int undistortionMode;
  if (this->get_parameter<int>("slam.undistortion", undistortionMode))
  {
    LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(undistortionMode);
    if (undistortion != LidarSlam::UndistortionMode::NONE &&
        undistortion != LidarSlam::UndistortionMode::ONCE &&
        undistortion != LidarSlam::UndistortionMode::REFINED &&
        undistortion != LidarSlam::UndistortionMode::EXTERNAL)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid undistortion mode (" << undistortion << "). Setting it to 'REFINED'.");
      undistortion = LidarSlam::UndistortionMode::REFINED;
    }
    LidarSlam.SetUndistortion(undistortion);
  }
  int i_interpolationModel;
  if (this->get_parameter<int>("slam.interpolation_model", i_interpolationModel))
  {
    auto interpoModel = static_cast<LidarSlam::Interpolation::Model>(i_interpolationModel);
    if (interpoModel != LidarSlam::Interpolation::Model::LINEAR &&
        interpoModel != LidarSlam::Interpolation::Model::QUADRATIC &&
        interpoModel != LidarSlam::Interpolation::Model::CUBIC)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid interpolation model (" << interpoModel << "). Setting it to 'LINEAR'.");
      interpoModel = LidarSlam::Interpolation::Model::LINEAR;
    }
    LidarSlam.SetInterpolation(interpoModel);
  }

  int pointCloudStorage;
  if (this->get_parameter<int>("logging.storage_type", pointCloudStorage))
  {
    LidarSlam::PointCloudStorageType storage = static_cast<LidarSlam::PointCloudStorageType>(pointCloudStorage);
    if (storage != LidarSlam::PointCloudStorageType::PCL_CLOUD &&
        storage != LidarSlam::PointCloudStorageType::OCTREE_COMPRESSED &&
        storage != LidarSlam::PointCloudStorageType::PCD_ASCII &&
        storage != LidarSlam::PointCloudStorageType::PCD_BINARY &&
        storage != LidarSlam::PointCloudStorageType::PCD_BINARY_COMPRESSED)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Incorrect pointcloud logging type value (" << storage << "). Setting it to 'PCL'.");
      storage = LidarSlam::PointCloudStorageType::PCL_CLOUD;
    }
    LidarSlam.SetLoggingStorage(storage);
  }

  // Frame Ids
  this->get_parameter_or<std::string>("odometry_frame", this->OdometryFrameId, "odom");
  this->LidarSlam.SetWorldFrameId(this->OdometryFrameId);
  this->get_parameter_or<std::string>("tracking_frame", this->TrackingFrameId, "base_link");
  this->LidarSlam.SetBaseFrameId(this->TrackingFrameId);
  this->get_parameter_or<std::string>("wheel_frame", this->WheelFrameId, "wheel");
  this->get_parameter_or<std::string>("ins_frame", this->ExtPoseFrameId, "ins");
  this->get_parameter_or<std::string>("gps_frame", this->GpsFrameId, "gps");

  // Keypoint extractors
  auto setExtractorMode = [this](LidarSlam::KeypointExtractorMode& mode,
                                 const std::string& prefix,
                                 const std::string& message = "")
  {
    int extractorMode;
    if (this->get_parameter<int>(prefix + "extractor_mode", extractorMode))
    {
      mode = static_cast<LidarSlam::KeypointExtractorMode>(extractorMode);
      if (mode != LidarSlam::KeypointExtractorMode::SPARSE &&
          mode != LidarSlam::KeypointExtractorMode::DENSE)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Incorrect keypoint extractor mode value (" << extractorMode << ")"
                                                << message << ". Setting it to '0', corresponding to sparse extractor.");
        mode = LidarSlam::KeypointExtractorMode::SPARSE;
      }
    }
  };

  auto setSamplingMode = [this](auto& dsske,
                                const std::string& prefix,
                                const std::string& message = "")
  {
    int samplingMode;
    if (this->get_parameter(prefix + "downsampling.sampling_mode", samplingMode))
    {
      LidarSlam::SamplingModeDSSKE sampling = static_cast<LidarSlam::SamplingModeDSSKE>(samplingMode);
      if (sampling != LidarSlam::SamplingModeDSSKE::PATCH &&
          sampling != LidarSlam::SamplingModeDSSKE::VOXEL)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid DSSKE sampling mode (" << samplingMode << ")"
                                                << message << ". Setting it to 'PATCH'.");
        sampling = LidarSlam::SamplingModeDSSKE::PATCH;
      }
      dsske->SetSamplingDSSKE(sampling);
    }
  };

  auto initKeypointsExtractor = [this](auto& ke, const std::string& prefix)
  {
    #define SetKeypointsExtractorParam(type, rosParam, keParam) {type val; if (this->get_parameter<type>(rosParam, val)) ke->Set##keParam(val);}
    SetKeypointsExtractorParam(int,   "slam.n_threads", NbThreads)
    SetKeypointsExtractorParam(int,   prefix + "neighbors_side_nb", MinNeighNb)
    SetKeypointsExtractorParam(float, prefix + "neighbors_radius", MinNeighRadius)
    SetKeypointsExtractorParam(float, prefix + "min_distance_to_sensor", MinDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "max_distance_to_sensor", MaxDistanceToSensor)
    SetKeypointsExtractorParam(float, prefix + "min_beam_surface_angle", MinBeamSurfaceAngle)
    SetKeypointsExtractorParam(float, prefix + "min_azimuth", AzimuthMin)
    SetKeypointsExtractorParam(float, prefix + "max_azimuth", AzimuthMax)
    SetKeypointsExtractorParam(float, prefix + "edge_angle_threshold", EdgeAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "plane_angle_threshold", PlaneAngleThreshold)
    SetKeypointsExtractorParam(float, prefix + "edge_depth_gap_threshold", EdgeDepthGapThreshold)
    SetKeypointsExtractorParam(int, prefix + "edge_nb_gap_points", EdgeNbGapPoints)
    SetKeypointsExtractorParam(float, prefix + "edge_intensity_gap_threshold", EdgeIntensityGapThreshold)
    SetKeypointsExtractorParam(int,   prefix + "downsampling.max_points", MaxPoints)
    SetKeypointsExtractorParam(float, prefix + "downsampling.voxel_grid_resolution", VoxelResolution)
    SetKeypointsExtractorParam(float, prefix + "input_sampling_ratio", InputSamplingRatio)

    #define EnableKeypoint(kType) \
    { \
      bool enabled = false; \
      std::string name = LidarSlam::KeypointTypeNames.at(kType); \
      if (this->get_parameter<bool>(prefix + "enable."+ name, enabled)) \
        this->LidarSlam.EnableKeypointType(kType, enabled); \
      if (enabled) \
        {RCLCPP_INFO_STREAM(this->get_logger(), "Keypoint of type " + name + " enabled");} \
      else \
        RCLCPP_INFO_STREAM(this->get_logger(), "Keypoint of type " + name + " disabled"); \
    }
    EnableKeypoint(LidarSlam::Keypoint::EDGE);
    EnableKeypoint(LidarSlam::Keypoint::INTENSITY_EDGE);
    EnableKeypoint(LidarSlam::Keypoint::PLANE);
    EnableKeypoint(LidarSlam::Keypoint::BLOB);
  };

  auto addExtractor = [this](auto& ke,
                             const std::string& name,
                             const std::string& message = "",
                             const std::string& deviceId = "")
  {
    deviceId != "" ? this->LidarSlam.SetKeyPointsExtractor(ke, deviceId)
                   : this->LidarSlam.SetKeyPointsExtractor(ke);
    RCLCPP_INFO_STREAM(this->get_logger(), "Adding " << name << " keypoints extractor " << message);
  };

  // Init deviceIds vector
  std::vector<std::string> deviceIds;
  if (this->get_parameter("slam.ke.device_ids", deviceIds))
    RCLCPP_INFO_STREAM(this->get_logger(), "Multi-LiDAR devices setup");
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Single LiDAR device setup");
    deviceIds.push_back("");
  }

  for (auto deviceId: deviceIds)
  {
    std::string prefix = "slam.ke.";
    if (deviceIds.size() > 1)
      prefix += "device_" + deviceId + ".";

    // Set keypoints extractor mode. If multilidar, will add an information in error message.
    LidarSlam::KeypointExtractorMode mode;
    deviceIds.size() == 1 ? setExtractorMode(mode, prefix)
                          : setExtractorMode(mode, prefix, " for LiDAR device " + deviceId);

    // Set keypoints extractor
    std::shared_ptr<LidarSlam::KeypointExtractor> ke;
    if (mode == LidarSlam::KeypointExtractorMode::SPARSE)
      ke = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();
    else
    {
      auto dsske = std::make_shared<LidarSlam::DenseSpinningSensorKeypointExtractor>();
      // Set specific parameters for DSSKE
      int patchSize;
      if (this->get_parameter(prefix + "downsampling.patch_size", patchSize))
        dsske->SetPatchSize(patchSize);
      deviceIds.size() == 1 ? setSamplingMode(dsske, prefix)
                            : setSamplingMode(dsske, prefix, " for LiDAR device " + deviceId);
      ke = dsske;
    }

    // Set keypoints extractor parameters and enable keypoints
    initKeypointsExtractor(ke, prefix);
    // Add extractor to LidarSlam
    deviceIds.size() == 1 ? addExtractor(ke, LidarSlam::KeypointExtractorModeNames.at(mode))
                          : addExtractor(ke, LidarSlam::KeypointExtractorModeNames.at(mode), "for LiDAR device " + deviceId, deviceId);
  }

  // Ego motion
  SetSlamParam(int,    "slam.ego_motion_registration.ICP_max_iter", EgoMotionICPMaxIter)
  SetSlamParam(int,    "slam.ego_motion_registration.LM_max_iter", EgoMotionLMMaxIter)
  SetSlamParam(double, "slam.ego_motion_registration.max_neighbors_distance", EgoMotionMaxNeighborsDistance)
  SetSlamParam(int,    "slam.ego_motion_registration.edge_nb_neighbors", EgoMotionEdgeNbNeighbors)
  SetSlamParam(int,    "slam.ego_motion_registration.edge_min_nb_neighbors", EgoMotionEdgeMinNbNeighbors)
  SetSlamParam(double, "slam.ego_motion_registration.edge_max_model_error", EgoMotionEdgeMaxModelError)
  SetSlamParam(int,    "slam.ego_motion_registration.plane_nb_neighbors", EgoMotionPlaneNbNeighbors)
  SetSlamParam(double, "slam.ego_motion_registration.planarity_threshold", EgoMotionPlanarityThreshold)
  SetSlamParam(double, "slam.ego_motion_registration.plane_max_model_error", EgoMotionPlaneMaxModelError)
  SetSlamParam(double, "slam.ego_motion_registration.init_saturation_distance", EgoMotionInitSaturationDistance)
  SetSlamParam(double, "slam.ego_motion_registration.final_saturation_distance", EgoMotionFinalSaturationDistance)

  // Localization
  SetSlamParam(int,    "slam.localization.ICP_max_iter", LocalizationICPMaxIter)
  SetSlamParam(int,    "slam.localization.LM_max_iter", LocalizationLMMaxIter)
  SetSlamParam(double, "slam.localization.max_neighbors_distance", LocalizationMaxNeighborsDistance)
  SetSlamParam(int,    "slam.localization.edge_nb_neighbors", LocalizationEdgeNbNeighbors)
  SetSlamParam(int,    "slam.localization.edge_min_nb_neighbors", LocalizationEdgeMinNbNeighbors)
  SetSlamParam(double, "slam.localization.edge_max_model_error", LocalizationEdgeMaxModelError)
  SetSlamParam(int,    "slam.localization.plane_nb_neighbors", LocalizationPlaneNbNeighbors)
  SetSlamParam(double, "slam.localization.planarity_threshold", LocalizationPlanarityThreshold)
  SetSlamParam(double, "slam.localization.plane_max_model_error", LocalizationPlaneMaxModelError)
  SetSlamParam(int,    "slam.localization.blob_nb_neighbors", LocalizationBlobNbNeighbors)
  SetSlamParam(double, "slam.localization.init_saturation_distance", LocalizationInitSaturationDistance)
  SetSlamParam(double, "slam.localization.final_saturation_distance", LocalizationFinalSaturationDistance)

  // External sensors
  SetSlamParam(int,    "external_sensors.max_measures", SensorMaxMeasures)
  SetSlamParam(float,  "external_sensors.time_threshold", SensorTimeThreshold)
  this->get_parameter_or<bool>("external_sensors.use_header_time", this->UseHeaderTime, true);
  SetSlamParam(float,   "external_sensors.time_offset", SensorTimeOffset)
  this->SensorTimeOffset = this->LidarSlam.GetSensorTimeOffset();
  this->get_parameter<bool>("external_sensors.calibration.planar_trajectory", this->PlanarTrajectory);
  this->get_parameter<double>("external_sensors.calibration.lever_arm", this->LeverArm);
  this->get_parameter<int>("external_sensors.calibration.window", this->CalibrationWindow);

  // Use GPS data for GPS/SLAM calibration or Pose Graph Optimization.
  this->get_parameter_or<bool>("external_sensors.gps.enable", this->UseExtSensor[LidarSlam::ExternalSensor::GPS], false);

  // Use tags data for local optimization.
  this->get_parameter_or<bool>("external_sensors.landmark_detector.enable", this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR], false);
  SetSlamParam(float,  "external_sensors.landmark_detector.weight", LandmarkWeight)
  SetSlamParam(float,  "external_sensors.landmark_detector.saturation_distance", LandmarkSaturationDistance)
  SetSlamParam(bool,   "external_sensors.landmark_detector.position_only", LandmarkPositionOnly)
  this->get_parameter_or<bool>("external_sensors.landmark_detector.publish_tags", this->PublishTags, false);

  // Use camera rgb images in local optimization.
  this->get_parameter_or<bool>("external_sensors.camera.enable", this->UseExtSensor[LidarSlam::ExternalSensor::CAMERA], false);
  SetSlamParam(float,  "external_sensors.camera.weight", CameraWeight)
  SetSlamParam(float,  "external_sensors.camera.saturation_distance", CameraSaturationDistance)

  // Use external poses in local optimization or in graph optimization
  this->get_parameter_or<bool>("external_sensors.external_poses.enable", this->UseExtSensor[LidarSlam::ExternalSensor::POSE], false);
  SetSlamParam(double, "external_sensors.external_poses.weight", PoseWeight)
  SetSlamParam(float,  "external_sensors.external_poses.saturation_distance", PoseSaturationDistance)

  // Use wheel encoder in local optimization
  this->get_parameter_or<bool>("external_sensors.wheel_encoder.enable", this->UseExtSensor[LidarSlam::ExternalSensor::WHEEL_ODOM], false);
  SetSlamParam(double, "external_sensors.wheel_encoder.weight", WheelOdomWeight)
  SetSlamParam(bool,   "external_sensors.wheel_encoder.relative", WheelOdomRelative)
  SetSlamParam(double, "external_sensors.wheel_encoder.saturation_distance", WheelOdomSaturationDistance)

  std::vector<double> wheelOdomRef;
  if (this->get_parameter("external_sensors.wheel_encoder.reference", wheelOdomRef) &&
      wheelOdomRef.size() == 3  &&
      !this->LidarSlam.GetWheelOdomRelative())
  {
    Eigen::Vector3d ref = Eigen::Vector3d(wheelOdomRef[0], wheelOdomRef[1], wheelOdomRef[2]);
    if (ref.norm() > 1e-6)
      this->LidarSlam.SetWheelOdomReference(ref);
  }

  std::vector<double> wheelOdomDir;
  if (this->get_parameter("external_sensors.wheel_encoder.direction", wheelOdomDir) &&
      wheelOdomDir.size() == 3  &&
      !this->LidarSlam.GetWheelOdomRelative())
  {
    Eigen::Vector3d dir = Eigen::Vector3d(wheelOdomDir[0], wheelOdomDir[1], wheelOdomDir[2]);
    if (dir.norm() > 1e-6)
      this->LidarSlam.SetWheelOdomDirection(dir);
  }

  // Use IMU gravity in local optimization
  this->get_parameter_or<bool>("external_sensors.imu.enable", this->UseExtSensor[LidarSlam::ExternalSensor::IMU], false);
  SetSlamParam(double, "external_sensors.imu.gravity_weight", GravityWeight)

  // Graph parameters
  SetSlamParam(std::string, "graph.g2o_file_name", G2oFileName)
  SetSlamParam(float,       "graph.covariance_scale", CovarianceScale)
  SetSlamParam(int,         "graph.iterations_nb", NbGraphIterations)
  SetSlamParam(bool,        "graph.fix_first", FixFirst)
  SetSlamParam(bool,        "graph.fix_last", FixLast)

  // Set PGO Constraint
  auto enablePGOConstraint = [this](const LidarSlam::PGOConstraint& pgoType, const LidarSlam::ExternalSensor& checkExtSensorType = LidarSlam::ExternalSensor::nbExternalSensors)
  {
    bool enabled = false;
    std::string name = LidarSlam::PGOConstraintNames.at(pgoType);
    if (this->get_parameter<bool>("graph.constraint." + name, enabled))
      this->LidarSlam.EnablePGOConstraint(pgoType, enabled);

    // Check whether the relevant sensor is enabled when a pgo is enabled (except loop closure)
    if (pgoType == LidarSlam::PGOConstraint::LOOP_CLOSURE)
      return;
    if (!this->UseExtSensor[checkExtSensorType] && enabled)
      RCLCPP_WARN_STREAM(this->get_logger(), name << " constraint is ON for PGO but the sensor is disabled");
  };
  enablePGOConstraint(LidarSlam::PGOConstraint::LOOP_CLOSURE);
  enablePGOConstraint(LidarSlam::PGOConstraint::LANDMARK, LidarSlam::ExternalSensor::LANDMARK_DETECTOR);
  enablePGOConstraint(LidarSlam::PGOConstraint::GPS,      LidarSlam::ExternalSensor::GPS);
  enablePGOConstraint(LidarSlam::PGOConstraint::EXT_POSE, LidarSlam::ExternalSensor::POSE);

  // Loop closure parameters
  SetSlamParam(double, "graph.loop_closure.query_map_start_range",     LoopQueryMapStartRange)
  SetSlamParam(double, "graph.loop_closure.query_map_end_range",       LoopQueryMapEndRange)
  SetSlamParam(double, "graph.loop_closure.revisited_map_start_range", LoopRevisitedMapStartRange)
  SetSlamParam(double, "graph.loop_closure.revisited_map_end_range",   LoopRevisitedMapEndRange)
  SetSlamParam(double, "graph.loop_closure.gap_length",                LoopGapLength)
  SetSlamParam(double, "graph.loop_closure.sample_step",               LoopSampleStep)
  SetSlamParam(bool,   "graph.loop_closure.enable_offset",             LoopEnableOffset)
  SetSlamParam(bool,   "graph.loop_closure.use_submap",                LoopICPWithSubmap)
  SetSlamParam(bool,   "graph.loop_closure.enable_overlap_check",      LoopEnableOverlapCheck)
  SetSlamParam(double, "graph.loop_closure.overlap_threshold",         LoopEvaluationOverlapThreshold)
  SetSlamParam(double, "graph.loop_closure.distance_threshold",        LoopEvaluationDistanceThreshold)
  // Loop closure advanced optimization parameters
  SetSlamParam(int,    "graph.loop_closure.ICP_max_iter",              LoopICPMaxIter)
  SetSlamParam(int,    "graph.loop_closure.LM_max_iter",               LoopLMMaxIter)
  SetSlamParam(double, "graph.loop_closure.max_neighbors_distance",    LoopMaxNeighborsDistance)
  SetSlamParam(int,    "graph.loop_closure.edge_nb_neighbors",         LoopEdgeNbNeighbors)
  SetSlamParam(int,    "graph.loop_closure.edge_min_nb_neighbors",     LoopEdgeMinNbNeighbors)
  SetSlamParam(double, "graph.loop_closure.edge_max_model_error",      LoopEdgeMaxModelError)
  SetSlamParam(int,    "graph.loop_closure.plane_nb_neighbors",        LoopPlaneNbNeighbors)
  SetSlamParam(double, "graph.loop_closure.planarity_threshold",       LoopPlanarityThreshold)
  SetSlamParam(double, "graph.loop_closure.plane_max_model_error",     LoopPlaneMaxModelError)
  SetSlamParam(int,    "graph.loop_closure.blob_nb_neighbors",         LoopBlobNbNeighbors)
  SetSlamParam(double, "graph.loop_closure.init_saturation_distance",  LoopInitSaturationDistance)
  SetSlamParam(double, "graph.loop_closure.final_saturation_distance", LoopFinalSaturationDistance)

  // Confidence estimators
  // Overlap
  SetSlamParam(float,  "slam.confidence.overlap.sampling_ratio", OverlapSamplingRatio)

  // Motion limitations (hard constraints to detect failure)
  std::vector<double> acc;
  if (this->get_parameter("slam.confidence.motion_limits.acceleration", acc) && acc.size() == 2)
  {
    // Convertion needed because infinity is a double in ROS
    Eigen::Array2f acc_array((float)acc[0], (float)acc[1]);
    this->LidarSlam.SetAccelerationLimits(acc_array);
  }
  std::vector<double> vel;
  if (this->get_parameter("slam.confidence.motion_limits.velocity", vel) && vel.size() == 2)
  {
    // Convertion needed because infinity is a double in ROS
    Eigen::Array2f vel_array((float)vel[0], (float)vel[1]);
    this->LidarSlam.SetVelocityLimits(vel_array);
  }
  std::vector<double> pos;
  if (this->get_parameter("slam.confidence.motion_limits.pose", pos) && pos.size() == 2)
  {
    // Convertion needed because infinity is a double in ROS
    Eigen::Array2f pos_array((float)pos[0], (float)pos[1]);
    this->LidarSlam.SetPoseLimits(pos_array);
  }

  SetSlamParam(int,   "slam.confidence.window", ConfidenceWindow)
  SetSlamParam(float, "slam.confidence.overlap.gap_threshold", OverlapDerivativeThreshold)
  SetSlamParam(float, "slam.confidence.position_error.threshold", PositionErrorThreshold)
  this->get_parameter_or<float>("slam.confidence.failure_detector.recovery_time", this->RecoveryTime, 1.);
  SetSlamParam(bool,  "slam.confidence.failure_detector.enable", FailureDetectionEnabled)

  // Keyframes
  SetSlamParam(double, "slam.keyframes.distance_threshold", KfDistanceThreshold)
  SetSlamParam(double, "slam.keyframes.angle_threshold", KfAngleThreshold)

  // Maps
  int mapUpdateMode;
  if (this->get_parameter<int>("slam.voxel_grid.update_maps", mapUpdateMode))
  {
    LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mapUpdateMode);
    if (mapUpdate != LidarSlam::MappingMode::NONE &&
        mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
        mapUpdate != LidarSlam::MappingMode::UPDATE)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid map update mode (" << mapUpdateMode << "). Setting it to 'UPDATE'.");
      mapUpdate = LidarSlam::MappingMode::UPDATE;
    }
    this->LidarSlam.SetMapUpdate(mapUpdate);
  }
  int submapMode;
  if (this->get_parameter("slam.voxel_grid.submap_extraction_mode", submapMode))
  {
    LidarSlam::PreSearchMode submap = static_cast<LidarSlam::PreSearchMode>(submapMode);
    if (submap != LidarSlam::PreSearchMode::BOUNDING_BOX &&
        submap != LidarSlam::PreSearchMode::PROFILE)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid submap mode (" << submapMode << "). Setting it to 'BOUNDING_BOX'.");
      submap = LidarSlam::PreSearchMode::BOUNDING_BOX;
    }
    this->LidarSlam.SetSubmapMode(submap);
  }
  double size;
  if (this->get_parameter_or<double>("slam.voxel_grid.leaf_size.edges", size, 0.2) &&
      this->LidarSlam.KeypointTypeEnabled(LidarSlam::EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::EDGE, size);
  if (this->get_parameter_or<double>("slam.voxel_grid.leaf_size.intensity_edges", size, 0.2) &&
      this->LidarSlam.KeypointTypeEnabled(LidarSlam::INTENSITY_EDGE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::INTENSITY_EDGE, size);
  if (this->get_parameter_or<double>("slam.voxel_grid.leaf_size.planes", size, 0.3) &&
      this->LidarSlam.KeypointTypeEnabled(LidarSlam::PLANE))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::PLANE, size);
  if (this->get_parameter_or<double>("slam.voxel_grid.leaf_size.blobs", size, 0.3) &&
      this->LidarSlam.KeypointTypeEnabled(LidarSlam::BLOB))
    this->LidarSlam.SetVoxelGridLeafSize(LidarSlam::BLOB, size);
  SetSlamParam(double, "slam.voxel_grid.resolution", VoxelGridResolution)
  SetSlamParam(int,    "slam.voxel_grid.size", VoxelGridSize)
  SetSlamParam(int,    "slam.voxel_grid.decaying_threshold", VoxelGridDecayingThreshold)
  SetSlamParam(int,    "slam.voxel_grid.min_frames_per_voxel", VoxelGridMinFramesPerVoxel)
  for (auto k : LidarSlam::KeypointTypes)
  {
    if (!this->LidarSlam.KeypointTypeEnabled(k))
      continue;
    int samplingMode;
    if (this->get_parameter("slam.voxel_grid.sampling_mode." + LidarSlam::KeypointTypeNames.at(k), samplingMode))
    {
      LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(samplingMode);
      if (sampling != LidarSlam::SamplingMode::FIRST &&
          sampling != LidarSlam::SamplingMode::LAST &&
          sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
          sampling != LidarSlam::SamplingMode::CENTER_POINT &&
          sampling != LidarSlam::SamplingMode::CENTROID)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid sampling mode (" << samplingMode << ") for "
                                                   << LidarSlam::Utils::Plural(LidarSlam::KeypointTypeNames.at(k))
                                                   << ". Setting it to 'MAX_INTENSITY'.");
        sampling = LidarSlam::SamplingMode::MAX_INTENSITY;
      }
      this->LidarSlam.SetVoxelGridSamplingMode(k, sampling);
    }
  }

  RCLCPP_WARN_STREAM(this->get_logger(), "Please review the following parameters setting:");

  // Display external sensor information
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "Sensor            |"
                     << std::setw(13) << " is enabled |"
                     << std::setw(22) << " can be used locally |"
                     << std::setw(22) << " can be used for PGO |");
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "Camera            |"
                     << std::setw(13) << (this->UseExtSensor[LidarSlam::ExternalSensor::CAMERA] ? " ON |" : " OFF |")
                     << std::setw(22) << (this->UseExtSensor[LidarSlam::ExternalSensor::CAMERA] &&
                     this->LidarSlam.GetCameraWeight() > 1e-6 ? " YES |" : " NO |")
                     << std::setw(22) << " NO |");
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "Landmark detector |"
                     << std::setw(13) << (this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR] ? " ON |" : " OFF |")
                     << std::setw(22) << (this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR] &&
                     this->LidarSlam.GetLandmarkWeight() > 1e-6 ? " YES |" : "  NO |")
                     << std::setw(22) << ((this->UseExtSensor[LidarSlam::ExternalSensor::LANDMARK_DETECTOR] &&
                     this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LANDMARK)) ? " YES |" : " NO |"));
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "Pose sensor       |"
                     << std::setw(13) << (this->UseExtSensor[LidarSlam::ExternalSensor::POSE] ? " ON |" : " OFF |")
                     << std::setw(22) << (this->UseExtSensor[LidarSlam::ExternalSensor::POSE] &&
                     this->LidarSlam.GetPoseWeight() > 1e-6 ? " YES |" : "  NO |")
                     << std::setw(22) << ((this->UseExtSensor[LidarSlam::ExternalSensor::POSE] &&
                     this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE)) ? " YES |" : " NO |"));
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "GPS               |"
                     << std::setw(13) << (this->UseExtSensor[LidarSlam::ExternalSensor::GPS] ? " ON |" : " OFF |")
                     << std::setw(22) << " NO |"
                     << std::setw(22) << ((this->UseExtSensor[LidarSlam::ExternalSensor::GPS] &&
                     this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS)) ? " YES |" : " NO |"));
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "Wheel encoder     |"
                     << std::setw(13) << (this->UseExtSensor[LidarSlam::ExternalSensor::WHEEL_ODOM] ? " ON |" : " OFF |")
                     << std::setw(22) << (this->UseExtSensor[LidarSlam::ExternalSensor::WHEEL_ODOM] &&
                     this->LidarSlam.GetWheelOdomWeight() > 1e-6 ? " YES |" : " NO |")
                     << std::setw(22) << " NO |");
  RCLCPP_INFO_STREAM(this->get_logger(), std::setw(19) << "IMU               |"
                     << std::setw(13) << (this->UseExtSensor[LidarSlam::ExternalSensor::IMU] ? " ON |" : " OFF |")
                     << std::setw(22) << (this->UseExtSensor[LidarSlam::ExternalSensor::IMU] &&
                     this->LidarSlam.GetGravityWeight() > 1e-6 ? " YES |" : " NO |")
                     << std::setw(22) << " NO |");

  // Check if or not one pgo constraint is enabled at least
  if (!this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE) &&
      !this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LANDMARK) &&
      !this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE) &&
      !this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS))
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Pose graph optimization is not enabled.");
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Loop closure constraint is "
                     << (this->LidarSlam.IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE) ? "enabled " : "disabled ")
                     <<"for pose graph");
  if (this->LidarSlam.GetLoggingTimeout() > 0.2)
    RCLCPP_INFO_STREAM(this->get_logger(), "Logging timeout is set to " << this->LidarSlam.GetLoggingTimeout() << "s. "
                                           "Pose graph optimization can be applied on this time window.");
  else
    RCLCPP_INFO_STREAM(this->get_logger(), "Logging timeout is set to " << this->LidarSlam.GetLoggingTimeout() << "s. "
                                           "Please set a convenient value to use pose graph optimization!");
}

//------------------------------------------------------------------------------
void LidarSlamNode::SetSlamInitialState()
{
  // Load initial Landmarks poses if requested
  std::string lmpath;
  this->get_parameter("external_sensors.landmark_detector.landmarks_file_path", lmpath);
  if (!lmpath.empty())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading initial landmarks info from CSV.");
    this->ReadTags(lmpath);
    this->LidarSlam.SetLandmarkConstraintLocal(false);
  }
  else
    this->LidarSlam.SetLandmarkConstraintLocal(true);

  // Set initial SLAM pose if requested
  // Setting initial SLAM pose is equivalent to move odom frame
  // so the first pose corresponds to the input in this new frame
  // T_base = offset * T_base_new
  // offset = T_base * T_base_new^-1
  // T_base is identity at initialization
  std::vector<double> initialPose;
  if (this->get_parameter<std::vector<double>>("maps.initial_pose", initialPose) && initialPose.size() == 6)
  {
    Eigen::Isometry3d initialTransform = LidarSlam::Utils::XYZRPYtoIsometry(initialPose.data());
    this->LidarSlam.SetInitialPose(initialTransform);
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting initial SLAM pose to:\n" << initialTransform.matrix());
  }

  // Load initial SLAM maps if requested
  std::string mapsPathPrefix;
  this->get_parameter<std::string>("maps.initial_maps", mapsPathPrefix);
  bool keepLabel = false;
  this->get_parameter<bool>("maps.keep_label", keepLabel);
  bool overrideTimestamp = false;
  this->get_parameter<bool>("maps.override_timestamp", overrideTimestamp);
  double timeOffset = 0.;
  this->get_parameter<double>("maps.timestamp_offset", timeOffset);
  double mapTime = -1;
  if (overrideTimestamp)
  {
    if (this->now().seconds() < 1e-6)
      RCLCPP_WARN_STREAM(this->get_logger(), "Current time not available: cannot override timestamps when loading maps");
    else
      mapTime = this->now().seconds() - timeOffset;
  }

  if (!mapsPathPrefix.empty())
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading initial keypoints maps from PCD.");
    this->LidarSlam.LoadMapsFromPCD(mapsPathPrefix, true, keepLabel, mapTime);
  }
}

//------------------------------------------------------------------------------
void LidarSlamNode::PublishTransformTF(double timeSec, std::string frameId, std::string childFrameId, const Eigen::Isometry3d& transfo)
{
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = rclcpp::Time(timeSec * 1e9);
  tfMsg.header.frame_id = frameId;
  tfMsg.child_frame_id = childFrameId;
  tfMsg.transform = Utils::IsometryToTfMsg(transfo);
  this->TfBroadcaster->sendTransform(tfMsg);
}
