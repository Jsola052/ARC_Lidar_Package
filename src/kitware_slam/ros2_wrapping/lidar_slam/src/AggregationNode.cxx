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

#include "AggregationNode.h"

// LidarSlam
#include <LidarSlam/Utilities.h>
#include <LidarSlam/PointCloudStorage.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>

// std
#include <filesystem>     // For std::filesystem (C++17)

// PCL
#include <pcl/common/transforms.h>

//==============================================================================
//   Basic SLAM use
//==============================================================================

//------------------------------------------------------------------------------
AggregationNode::AggregationNode(std::string name_node, const rclcpp::NodeOptions& options)
  : rclcpp::Node(name_node, options)
{
  // ***************************************************************************
  // Init ROS publisher
  // aggregated points with specified density
  this->PointsPublisher = this->create_publisher<Pcl2_msg>("aggregated_cloud", 10);
  // Optional publisher
  this->get_parameter_or<bool>("slice.enable", this->DoExtractSlice, false);
  if (this->DoExtractSlice)
  {
    this->SlicePublisher = this->create_publisher<Pcl2_msg>("slice_cloud", 10);
    this->SliceAreaPublisher = this->create_publisher<std_msgs::msg::Float64>("slice_area", 10);
  }

  // Init ROS subscribers
  // Lidar frame undistorted
  this->FrameSubscriber = this->create_subscription<Pcl2_msg>(
            "slam_registered_points", 1, std::bind(&AggregationNode::Callback, this, std::placeholders::_1));
  // Optional lidar SLAM pose
  this->PoseSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "slam_odom", 1, std::bind(&AggregationNode::PoseCallback, this, std::placeholders::_1));

  // Init save pointcloud service
  this->SaveService = this->create_service<lidar_slam::srv::SavePc>(
      "aggregation/save_pc",
      std::bind(&AggregationNode::SavePointcloudService, this, std::placeholders::_1, std::placeholders::_2));

  // Init reset service
  this->RstService = this->create_service<lidar_slam::srv::Reset>(
      "aggregation/reset",
      std::bind(&AggregationNode::ResetService, this, std::placeholders::_1, std::placeholders::_2));

  // Performances parameter
  this->get_parameter_or<unsigned int>("nbthreads", this->NbThreads, 1);

  // Init rolling grid with parameters
  this->DenseMap = std::make_shared<LidarSlam::RollingGrid>();
  this->RefMap = std::make_shared<LidarSlam::RollingGrid>();

  // Slice parameters (needed to define the main voxel grid)
  this->get_parameter_or<double>("slice.traj_length", this->TrajectoryMaxLength, 1.);
  // Compute the width of the slice to extract
  this->get_parameter_or<double>("slice.width", this->SliceWidth, 0.2);
  // Set the max distance from the pose to compute a slice
  this->get_parameter_or<double>("slice.max_dist", this->SliceMaxDist, 5.);
  double angleStepDeg;
  this->get_parameter_or<double>("slice.angle_resolution", angleStepDeg, 3.);
  this->AngleStep = (M_PI / 180.) * angleStepDeg;

  // Horizontal slice extraction parameters
  this->get_parameter_or<bool>("z_slice.enable", this->DoExtractZslice, false);
  this->get_parameter_or<double>("z_slice.width", this->ZsliceWidth, 0.1);
  this->get_parameter_or<double>("z_slice.height_position", this->ZsliceHeightPosition, -0.5);
  this->get_parameter_or<bool>("z_slice.invert", this->InvertZsliceExtraction, false);

  // Obstacle extraction parameters
  this->get_parameter_or<bool>("obstacle.enable", this->DoExtractObstacle, false);
  std::string refMapPath;
  this->get_parameter<std::string>("obstacle.ref_map_path", refMapPath);
  this->get_parameter_or<bool>("obstacle.enable", this->DoExtractObstacle, false);
  double decayTime;
  this->get_parameter_or<double>("obstacle.decay_time", decayTime, 1000.);
  this->get_parameter_or<float>("obstacle.fov_dist", this->FOVDist, 50.);
  this->DenseMap->SetDecayingThreshold(decayTime);
  bool publishGrid;
  this->get_parameter_or<bool>("obstacle.publish_occupancy_grid", publishGrid, false);
  this->get_parameter_or<float>("obstacle.merge_dist", this->MergeDist, 0.5);
  this->get_parameter_or<double>("obstacle.min_marker_size", this->MinObstacleMarkerSize, 0.3);

  // Get max size in meters
  double maxSize;
  this->get_parameter_or<double>("max_size", maxSize, 200.);
  // Set max size in voxels : the second dimension of
  // the rolling grid is used for the slice
  this->DenseMap->SetGridSize(maxSize / this->SliceMaxDist);
  this->RefMap->SetGridSize(maxSize / this->SliceMaxDist);

  // Set the voxels' sizes
  // Set the inner voxel size
  float leafSize;
  this->get_parameter_or<float>("leaf_size", leafSize, 0.1);
  this->DenseMap->SetLeafSize(leafSize);
  this->RefMap->SetLeafSize(leafSize);
  // Set the outer voxel size,
  // it should be greater than the inner voxel
  this->DenseMap->SetVoxelResolution(std::max(3. * leafSize, this->SliceMaxDist));
  this->RefMap->SetVoxelResolution(std::max(3. * leafSize, this->SliceMaxDist));
  // Min number of frames seeing a voxel to extract it
  int minNbPointsPerVoxel;
  this->get_parameter_or<int>("min_points_per_voxel", minNbPointsPerVoxel, 2);
  this->DenseMap->SetMinFramesPerVoxel(minNbPointsPerVoxel);
  // Set minimal distance of the points to the trajectory
  // Allows to remove traces from the map
  this->get_parameter_or<double>("min_dist_around_trajectory", this->MinDistAroundTrajectory, 1.);
  // Set maximal distance for input points
  this->get_parameter_or<double>("max_dist_around_trajectory", this->MaxDistAroundTrajectory, -1.);

  // Load ref map
  if (!refMapPath.empty())
  {
    this->LoadRefMap(refMapPath);
    this->Pointcloud = this->RefMap->Get();
    this->Pointcloud->header.frame_id = "odom";
    Pcl2_msg aggregatedCloudMsg;
    pcl::toROSMsg(*this->Pointcloud, aggregatedCloudMsg);
    this->PointsPublisher->publish(aggregatedCloudMsg);
  }

  if (this->DoExtractObstacle)
  {
    if (publishGrid)
      this->OccupancyPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacles/occupancy_grid", 10);
    this->MarkerPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacles/bboxes", 10);
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Aggregation node is ready !");
}

//------------------------------------------------------------------------------
void AggregationNode::Callback(const Pcl2_msg& registeredCloudMsg)
{
  //Convert msg to PointCloud
  CloudS::Ptr registeredCloud = std::make_shared<CloudS>();
  pcl::fromROSMsg(registeredCloudMsg, *registeredCloud);
  double currentTime = LidarSlam::Utils::PclStampToSec(registeredCloud->header.stamp);

  // Clear old points
  this->DenseMap->ClearPoints(currentTime);

  // Remove the farthest points if required
  CloudS::Ptr closest;
  if (this->MaxDistAroundTrajectory > 0)
  {
    closest.reset(new CloudS);
    closest->reserve(registeredCloud->size());
    closest->header = registeredCloud->header;
    for (const auto& pt : *registeredCloud)
    {
      if ((pt.getVector3fMap() - this->CurrentPose.translation().cast<float>()).norm() < this->MaxDistAroundTrajectory)
        closest->emplace_back(pt);
    }
  }
  else
    closest = registeredCloud;

  // Extract/reject a slice in z axis if required
  CloudS::Ptr preprocessedCloud;
  if (this->DoExtractZslice)
  {
    preprocessedCloud.reset(new CloudS);
    preprocessedCloud->reserve(closest->size());
    preprocessedCloud->header = closest->header;
    // Extract the z slice points
    this->ExtractZslice(*closest, *preprocessedCloud);
  }
  else
    preprocessedCloud = closest;

  // Extract obstacle and clusters of obstacles
  if (this->DoExtractObstacle)
  {
    // Label points w.r.t reference map
    // Old points are labeled as FIXED
    // Others are labeled as DEFAULT
    this->RefMap->LabelNewPoints(preprocessedCloud, true);
    // Extract new points
    CloudS::Ptr newPoints = std::make_shared<CloudS>();
    newPoints->reserve(preprocessedCloud->size());
    newPoints->header = preprocessedCloud->header;
    for (auto& pt : *preprocessedCloud)
    {
      if (pt.label != static_cast<uint16_t>(LidarSlam::LidarPointLabel::FIXED))
        newPoints->emplace_back(pt);
    }

    // Label obstacles with cluster indices
    this->LabelObstacle(*newPoints, currentTime);
    preprocessedCloud = newPoints;

    // publish occupancyGrid
    if (this->OccupancyPublisher)
    {
      auto oGMsg = nav_msgs::msg::OccupancyGrid();
      oGMsg.header.stamp = rclcpp::Clock().now();
      oGMsg.header.frame_id = registeredCloudMsg.header.frame_id;
      oGMsg.info.map_load_time = rclcpp::Clock().now();
      oGMsg.info.resolution = this->DenseMap->GetLeafSize();
      oGMsg.info.width = this->ObstaclesGrid.Width;
      oGMsg.info.height = this->ObstaclesGrid.Height;
      oGMsg.info.origin.position.x = this->ObstaclesGrid.Origin.x();
      oGMsg.info.origin.position.y = this->ObstaclesGrid.Origin.y();
      oGMsg.info.origin.position.z = this->ZsliceHeightPosition;
      oGMsg.info.origin.orientation.x = 0.0;
      oGMsg.info.origin.orientation.y = 0.0;
      oGMsg.info.origin.orientation.z = 0.0;
      oGMsg.info.origin.orientation.w = 1.0;
      oGMsg.data = this->ObstaclesGrid.Flatten();
      this->OccupancyPublisher->publish(oGMsg);
    }
  }

  // Add the points to the map
  // Map is not rolled because if some points are wrong or a frame is bad,
  // the whole map will be rolled and parts will be forgotten.
  // The size of the map is relative to the first frame reveived
  this->DenseMap->Add(preprocessedCloud, false, false);

  // Publish the map
  this->Pointcloud = this->DenseMap->Get(true); // "true" to remove moving objects
  this->Pointcloud->header = registeredCloud->header;

  // Publish bounding boxes of clusters
  if (this->MarkerPublisher)
  {
    // Group cluster
    std::unordered_map<int, CloudS> clusters;
    clusters.reserve(this->NewClusterIdx);
    for (auto& pt : *this->Pointcloud)
    {
      // Compute 2D coordinates
      int i = (pt.y - this->ObstaclesGrid.Origin.y()) / this->DenseMap->GetLeafSize();
      if (i < 0 || i >= this->ObstaclesGrid.Height)
        continue;
      int j = (pt.x - this->ObstaclesGrid.Origin.x()) / this->DenseMap->GetLeafSize();
      if (j < 0 || j >= this->ObstaclesGrid.Width)
        continue;

      pt.label = this->ObstaclesGrid(i,j).ClusterIdx >= 0 ?
                  static_cast<uint16_t>(this->ObstaclesGrid(i,j).ClusterIdx) : 0;
      // Add point to grid
      clusters[this->ObstaclesGrid(i,j).ClusterIdx].emplace_back(pt);
    }

    visualization_msgs::msg::MarkerArray marker_array;
    for (auto& cluster : clusters)
    {
      if (cluster.first < 2)
        continue;

      if (cluster.second.size() < 5)
        continue;

      // Compute PCA
      std::vector<int> knnIndices(cluster.second.size());
      std::iota(knnIndices.begin(), knnIndices.end(), 0);
      Eigen::Vector3d centroid;
      Eigen::Vector3d eigVals;
      Eigen::Matrix3d eigVecs;
      LidarSlam::Utils::ComputeMeanAndPCA(cluster.second, knnIndices, centroid, eigVecs, eigVals);

      // Deduce transform
      Eigen::Isometry3d pose;
      pose.linear() = eigVecs;
      pose.translation() = centroid;

      Eigen::Quaterniond quat(eigVecs);

      // Compute box bounds
      // Transform in object frame
      CloudS transformedCluster;
      pcl::transformPointCloud(cluster.second, transformedCluster, pose.inverse().matrix());

      // Compute bounds
      Eigen::Vector4f minPoint, maxPoint;
      pcl::getMinMax3D(transformedCluster, minPoint, maxPoint);
      Eigen::Vector3d boxDiag = (maxPoint - minPoint).head(3).cast<double>();
      if (std::isnan(boxDiag.norm()) || boxDiag.norm() < this->MinObstacleMarkerSize)
        continue;
      // Compute box centroid
      Eigen::Vector3d center = (minPoint.head(3).cast<double>() + boxDiag / 2.);
      center = pose * center;
      this->ObstaclesBBox[cluster.first].Center = center;
      this->ObstaclesBBox[cluster.first].Diagonal = pose * boxDiag;

      // Create marker
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = registeredCloudMsg.header.frame_id;
      marker.header.stamp = registeredCloudMsg.header.stamp;
      marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(200)); // 0.2 second of validity
      marker.ns = "bounding_boxes";
      marker.id = cluster.first;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = center.x();
      marker.pose.position.y = center.y();
      marker.pose.position.z = center.z();
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();
      marker.scale.x = boxDiag.x();
      marker.scale.y = boxDiag.y();
      marker.scale.z = boxDiag.z();
      marker.color.a = 0.5;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      marker_array.markers.push_back(marker);
    }
    this->MarkerPublisher->publish(marker_array);
  }

  Pcl2_msg aggregatedCloudMsg;
  pcl::toROSMsg(*this->Pointcloud, aggregatedCloudMsg);
  this->PointsPublisher->publish(aggregatedCloudMsg);
}

//------------------------------------------------------------------------------
void AggregationNode::SavePointcloudService(
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

  std::string outputFilePath = outputPrefixPath.string() + "_" + std::to_string(int(this->now().seconds())) + ".pcd";
  if (req->fixed)
  {
    auto pc = *this->Pointcloud;
    for (auto& pt : pc)
      pt.label = static_cast<uint16_t>(LidarSlam::LidarPointLabel::FIXED);
    LidarSlam::savePointCloudToPCD<PointS>(outputFilePath, pc, pcdFormat);
  }
  else
    LidarSlam::savePointCloudToPCD<PointS>(outputFilePath, *this->Pointcloud, pcdFormat);
  RCLCPP_INFO_STREAM(this->get_logger(), "Pointcloud saved to " << outputFilePath);
  res->success = true;
}

//------------------------------------------------------------------------------
void AggregationNode::ResetService(const std::shared_ptr<lidar_slam::srv::Reset::Request> req,
                                   const std::shared_ptr<lidar_slam::srv::Reset::Response> res)
{
  this->DenseMap->Reset();
  RCLCPP_INFO_STREAM(this->get_logger(), "Resetting aggregation");
  res->success = true;
}

//------------------------------------------------------------------------------
void AggregationNode::PoseCallback(const nav_msgs::msg::Odometry& poseMsg)
{
  this->CurrentPose = Utils::PoseMsgToIsometry(poseMsg.pose.pose);

  if (this->MinDistAroundTrajectory > 0.)
    this->DenseMap->EmptyAroundPoint(this->MinDistAroundTrajectory, this->CurrentPose.translation().cast<float>().array());

  if (!this->DoExtractSlice)
    return;

  // Store pose
  this->Positions.push_back(this->CurrentPose.translation());
  while ((this->CurrentPose.translation() - this->Positions.front()).norm() > this->TrajectoryMaxLength)
    this->Positions.pop_front();

  if (this->Positions.size() < 2)
    return;

  // Extract the slice boundary and compute its area
  // using the last positions and the map
  CloudS boundary;
  double area = this->ExtractSlice(this->SliceWidth, this->SliceMaxDist, this->AngleStep, boundary);

  // Publish the area value
  std_msgs::msg::Float64 areaMsg;
  areaMsg.data = area;
  this->SliceAreaPublisher->publish(areaMsg);

  // Publish the slice points
  Pcl2_msg sliceMsg;
  pcl::toROSMsg(boundary, sliceMsg);
  this->SlicePublisher->publish(sliceMsg);
}

//------------------------------------------------------------------------------
double AggregationNode::ExtractSlice(double sliceWidth, double sliceMaxDist, double angleStep, CloudS& boundary)
{
  // Shortcut to last position
  Eigen::Vector3d& currPosition = this->Positions.back();
  // Deduce the motion direction
  Eigen::Vector3d motionDirection = (currPosition - this->Positions.front()).normalized();
  // Extract the outer voxel in which lays the new pose
  // To reduce the projection tests
  CloudS tmpCloud;
  tmpCloud.resize(1);
  tmpCloud[0].getVector3fMap() = currPosition.cast<float>();
  this->DenseMap->BuildSubMap(tmpCloud, this->MinSlicePtsWithoutMovObjects);
  CloudS::Ptr submap = this->DenseMap->GetSubMap();

  // Initialize the slice pointcloud
  CloudS slice;
  slice.header = this->Pointcloud->header;
  slice.reserve(submap->size());

  // Project them onto the slice plane
  for (const auto& pt : *submap)
  {
    // Check if point in sphere around the trajectory pose
    if ((pt.getVector3fMap().cast<double>() - currPosition).norm() > sliceMaxDist)
      continue;

    double ptDistance = (pt.getVector3fMap().cast<double>() - currPosition).dot(motionDirection);
    // Check if the point is not too far from the trajectory point
    if (std::abs(ptDistance) > sliceWidth)
      continue;

    slice.emplace_back(pt);
    slice.back().getVector3fMap() -= (ptDistance * motionDirection).cast<float>();
  }

  // Create the boundary using a moving average
  // 1. Create a circular histogram
  std::vector<std::vector<Eigen::Vector3d>> histogram(std::ceil((2. * M_PI + 1e-6) / angleStep));
  Eigen::Vector3d refDir = (slice.front().getVector3fMap().cast<double>() - currPosition).normalized();
  for (const auto& pt : slice)
  {
    Eigen::Vector3d currVec = (pt.getVector3fMap().cast<double>() - currPosition).normalized();
    double dotProduct = std::min(std::max(currVec.dot(refDir), -1.), 1.);
    double angle = std::acos(dotProduct);
    if (currVec.cross(refDir).dot(motionDirection) < 0.)
      angle = 2. * M_PI - angle;
    histogram[int(angle / angleStep)].push_back(pt.getVector3fMap().cast<double>());
  }

  // 2. Average bins and fill the boundary cloud
  boundary.header = this->Pointcloud->header;
  boundary.reserve(histogram.size());
  for (const auto& bin : histogram)
  {
    if (bin.empty())
      continue;
    Eigen::Vector3d ptMean = Eigen::Vector3d::Zero();
    for (const auto& pt : bin)
      ptMean += pt;
    ptMean /= bin.size();
    PointS ptBoundary;
    ptBoundary.getVector3fMap() = ptMean.cast<float>();
    boundary.emplace_back(ptBoundary);
  }

  // Browse over the bins to build the area
  double area = 0.;
  for (unsigned int i = 1; i < boundary.size(); ++i)
  {
    Eigen::Vector3d vec1 = boundary[i].getVector3fMap().cast<double>() - currPosition;
    Eigen::Vector3d vec2 = boundary[i - 1].getVector3fMap().cast<double>() - currPosition;
    area += 0.5 * (vec1).cross(vec2).norm();
  }

  return area;
}

//------------------------------------------------------------------------------
void AggregationNode::ExtractZslice(const CloudS& inputCloud, CloudS& zSliceCloud)
{
  // Get plane inliers
  zSliceCloud.header = inputCloud.header;
  CloudS inputCloudInBase;
  pcl::transformPointCloud(inputCloud, inputCloudInBase, this->CurrentPose.inverse().matrix());
  for (int idxPt = 0; idxPt < inputCloud.size(); ++idxPt)
  {
    float distToPlane = std::abs((inputCloudInBase[idxPt].z - this->ZsliceHeightPosition));
    if (!this->InvertZsliceExtraction && distToPlane <= this->ZsliceWidth / 2. ||
        this->InvertZsliceExtraction && distToPlane > this->ZsliceWidth / 2.)
      zSliceCloud.emplace_back(inputCloud[idxPt]);
  }
}

//------------------------------------------------------------------------------
void AggregationNode::LabelObstacle(CloudS& inputCloud, double currentTime)
{
  int unknownLabel = static_cast<int>(LidarSlam::LidarPointLabel::DEFAULT); // shortcut to unknown label

  // Project input potential obstacle points onto grid and
  // fill the current point indices of each pixel
  #pragma omp parallel for num_threads(this->NbThreads)
  for (int idxPt = 0; idxPt < inputCloud.size(); ++idxPt)
  {
    // Check point is not in a fixed voxel
    if (inputCloud[idxPt].label == static_cast<std::uint16_t>(LidarSlam::LidarPointLabel::FIXED))
      continue;

    // Compute 2D coordinates
    int i = (inputCloud[idxPt].y - this->ObstaclesGrid.Origin.y()) / this->DenseMap->GetLeafSize();
    if (i < 0 || i >= this->ObstaclesGrid.Height)
      continue;
    int j = (inputCloud[idxPt].x - this->ObstaclesGrid.Origin.x()) / this->DenseMap->GetLeafSize();
    if (j < 0 || j >= this->ObstaclesGrid.Width)
      continue;

    // Add point to grid
    this->ObstaclesGrid(i,j).CurrentPtIndices.emplace_back(idxPt);
  }

  // Track number of frames that have seen the pixels
  #pragma omp parallel for num_threads(this->NbThreads)
  for (int idxGrid = 0; idxGrid < this->ObstaclesGrid.Data.size(); ++idxGrid)
  {
    auto it = this->ObstaclesGrid.Data.begin();
    std::advance(it, idxGrid);
    Pixel& pix = it->second; // shortcut
    // Notify it has been seen
    if (pix.CurrentPtIndices.size() > 0)
    {
      ++pix.SeenTimes;
      pix.Time = currentTime;
    }
  }

  // Label pixels to clusterize
  #pragma omp parallel for num_threads(this->NbThreads)
  for (int idxGrid = 0; idxGrid < this->ObstaclesGrid.Data.size(); ++idxGrid)
  {
    auto it = this->ObstaclesGrid.Data.begin();
    std::advance(it, idxGrid);
    Pixel& pix = it->second; // shortcut
    // Update the cluster label of the pixel
    // if it is not registered and
    // if the pixel has been seen at least once
    if (pix.SeenTimes > 0 &&
        pix.ClusterIdx < 0)
      pix.ClusterIdx = unknownLabel;
  }

  // Link cluster idx to grid indices to speed up search
  std::unordered_map<int, std::vector<int>> clus2gridIndices;
  for (auto& el : this->ObstaclesGrid.Data)
  {
    int pixIdx = el.first; // shortcut
    Pixel& pix = el.second; // shortcut
    if (pix.ClusterIdx < 2)
      continue;
    clus2gridIndices[pix.ClusterIdx].push_back(pixIdx);
  }

  const std::vector<Eigen::Array2i> radii = {{-1, -1}, {0, -1}, {1, -1},
                                             {-1,  0},          {1,  0},
                                             {-1,  1}, {0,  1}, {1,  1}};

  // Grow existing regions
  for (auto& el : clus2gridIndices)
  {
    auto& clusIdx = el.first; // shortcut
    auto& indices = el.second; // shortcut

    std::vector<Eigen::Array2i> clusterPixels;
    clusterPixels.reserve(indices.size());
    // Fill region with existing pixels
    for (int idx : indices)
      clusterPixels.push_back({idx / this->ObstaclesGrid.Width, idx % this->ObstaclesGrid.Width});

    // Grow region
    int idxPix = 0;
    // To avoid additional computation
    std::unordered_map<int, bool> hasCheckedNeighbor;
    hasCheckedNeighbor.reserve(clus2gridIndices.size());
    while (idxPix < clusterPixels.size())
    {
      int i = clusterPixels[idxPix](0);
      int j = clusterPixels[idxPix](1);

      // Label corresponding points
      for (int ptIdx : this->ObstaclesGrid(i, j).CurrentPtIndices)
        inputCloud[ptIdx].label = static_cast<std::uint16_t>(clusIdx);

      // Add neighbors
      #pragma omp parallel for num_threads(this->NbThreads)
      for (int idxRad = 0; idxRad < radii.size(); ++idxRad)
      {
        const Eigen::Array2i& r = radii[idxRad];
        Eigen::Array2i neigh = clusterPixels[idxPix] + r;
        // Check neighbor validity
        if (!this->ObstaclesGrid.Check(neigh.x(), neigh.y()))
          continue;

        // If neighbor is occupied, add it to current cluster
        int neighClusIdx = this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx;

        if (neighClusIdx == unknownLabel)
        {
          // Add neighbor to cluster
          clusterPixels.push_back({neigh.x(), neigh.y()});
          // Update label
          this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx = clusIdx;
          continue;
        }

        // If the neighbor belongs to another existing cluster
        // Check the sizes and the distance between two clusters
        if (neighClusIdx != clusIdx && !hasCheckedNeighbor.count(neighClusIdx))
        {
          hasCheckedNeighbor[neighClusIdx] = true;
          if (!this->ObstaclesBBox.count(neighClusIdx) || !this->ObstaclesBBox.count(clusIdx))
            continue;

          // If clusters are big, do not merge them
          if (this->ObstaclesBBox[neighClusIdx].Diagonal.norm() > this->MergeDist &&
              this->ObstaclesBBox[clusIdx].Diagonal.norm() > this->MergeDist)
            continue;

          // Otherwise, if clusters are far from each other, do not merge them
          if ((this->ObstaclesBBox[neighClusIdx].Center -
               this->ObstaclesBBox[clusIdx].Center).norm() > this->MergeDist)
            continue;

          // Merge the new cluster to the current one
          for (int idx : clus2gridIndices[neighClusIdx])
          {
            // Add to cluster
            clusterPixels.push_back({idx / this->ObstaclesGrid.Width, idx % this->ObstaclesGrid.Width});
            // Update label
            this->ObstaclesGrid.Data[idx].ClusterIdx = clusIdx;
          }

          // Remove neighbor cluster from clus2gridIndices
          // to not process it afterwards
          clus2gridIndices.erase(neighClusIdx);
        }
      }
      ++idxPix;
    }
  }

  // Fill the potential seeds with remaining unknown pixels
  std::vector<Eigen::Array2i> potentialSeeds;
  #pragma omp parallel for num_threads(this->NbThreads)
  for (int idxGrid = 0; idxGrid < this->ObstaclesGrid.Data.size(); ++idxGrid)
  {
    auto it = this->ObstaclesGrid.Data.begin();
    std::advance(it, idxGrid);
    int idx = it->first; // shortcut
    Pixel& pix = it->second; // shortcut
    // Select unknown pixels which are trustworthy enough (they have been seen enough)
    if (pix.ClusterIdx == unknownLabel &&
        pix.SeenTimes >= this->DenseMap->GetMinFramesPerVoxel())
      potentialSeeds.push_back({idx / this->ObstaclesGrid.Width, idx % this->ObstaclesGrid.Width});
  }

  // Grow new regions
  for (const auto& seed : potentialSeeds)
  {
    // Check seed is still unknown
    if (this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx != unknownLabel)
      continue;

    this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx = this->NewClusterIdx;

    std::vector<Eigen::Array2i> clusterPixels;
    clusterPixels.reserve(this->ObstaclesGrid.Size());
    clusterPixels.emplace_back(seed);
    int idxPix = 0;
    while (idxPix < clusterPixels.size())
    {
      int i = clusterPixels[idxPix](0);
      int j = clusterPixels[idxPix](1);

      // Label corresponding 3D points
      for (int ptIdx : this->ObstaclesGrid(i, j).CurrentPtIndices)
        inputCloud[ptIdx].label = static_cast<std::uint16_t>(this->NewClusterIdx);

      // Check neighbors
      #pragma omp parallel for num_threads(this->NbThreads)
      for (int idxRad = 0; idxRad < radii.size(); ++idxRad)
      {
        const Eigen::Array2i& r = radii[idxRad];
        Eigen::Array2i neigh = clusterPixels[idxPix] + r;
        // If neighbor is occupied, add it to current cluster
        if (this->ObstaclesGrid.Check(neigh.x(), neigh.y()) &&
            this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx == unknownLabel)
        {
          // Add neighbor to cluster
          clusterPixels.push_back({neigh.x(), neigh.y()});
          // Update cluster label of pixel
          this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx = this->NewClusterIdx;
        }
      }
      ++idxPix;
    }
    ++this->NewClusterIdx;
  }

  // Clear occupancy grid
  this->ClearObstaclesGrid(inputCloud, currentTime);
}

//-----------------------------------------------------------------------------
void AggregationNode::ClearObstaclesGrid(CloudS& inputCloud, double currentTime)
{
  int unknownLabel = static_cast<int>(LidarSlam::LidarPointLabel::DEFAULT); // shortcut to unknown label

  // 1. Remove old isolated pixels
  for (auto it = this->ObstaclesGrid.Data.begin(); it != this->ObstaclesGrid.Data.end();)
  {
    Pixel& pix = it->second; // shortcut
    if (pix.ClusterIdx == unknownLabel &&
        currentTime - pix.Time > this->DenseMap->GetDecayingThreshold())
    {
      // Erase and return the iterator to the next element
      it = this->ObstaclesGrid.Data.erase(it);
      continue;
    }
    // Only increment if not erasing
    ++it;
  }

  // 2. Fill clus2DecayTime to remove old clusters and compute standard deviation of time in 3.
  std::unordered_map<int, std::vector<double>> clus2DecayTime;
  std::unordered_map<int, std::vector<int>> clus2PixelIndices;
  for (auto& el : this->ObstaclesGrid.Data)
  {
    int idx = el.first; // shortcut
    Pixel& pix = el.second; // shortcut

    if (pix.ClusterIdx <= 0)
      continue;

    // Fill decay time array for each cluster
    clus2DecayTime[pix.ClusterIdx].emplace_back(currentTime - pix.Time);
    // Fill pixel indices for each cluster
    clus2PixelIndices[pix.ClusterIdx].emplace_back(idx);
  }
  // Do not check clusters which are out of the current fov
  for (auto it = clus2DecayTime.begin(); it != clus2DecayTime.end();)
  {
    auto& clusIdx = it->first;
    if (!this->ObstaclesBBox.count(clusIdx))
    {
      ++it;
      continue;
    }
    Eigen::Vector3d minPoint = this->ObstaclesBBox[clusIdx].Center - 0.5 * this->ObstaclesBBox[clusIdx].Diagonal;
    Eigen::Vector3d maxPoint = this->ObstaclesBBox[clusIdx].Center + 0.5 * this->ObstaclesBBox[clusIdx].Diagonal;
    if ((minPoint - this->CurrentPose.translation().head(3)).norm() > this->FOVDist &&
        (maxPoint - this->CurrentPose.translation().head(3)).norm() > this->FOVDist)
      it = clus2DecayTime.erase(it);
    else
      ++it;
  }

  // 3. Remove too old clusters and separate static and moving objects in a cluster
  for (auto& c2t : clus2DecayTime)
  {
    int clusIdx = c2t.first; // shortcut
    double time = *std::min_element(c2t.second.begin(), c2t.second.end());
    // Check time
    if (time > this->DenseMap->GetDecayingThreshold())
    {
      // Remove old cluster
      for (auto it = this->ObstaclesGrid.Data.begin(); it != this->ObstaclesGrid.Data.end();)
      {
        Pixel& pix = it->second; // shortcut
        if (pix.ClusterIdx == clusIdx)
          // Erase and return the iterator to the next element
          it = this->ObstaclesGrid.Data.erase(it);
        else
          // Only increment if not erasing
          ++it;
      }
      this->ObstaclesBBox.erase(clusIdx);
      continue;
    }

    // Check whether or not a big cluster need to be separated
    if (!this->ObstaclesBBox.count(clusIdx) || this->ObstaclesBBox[clusIdx].Diagonal.norm() < 2 * this->MergeDist )
      continue;

    // Check standard deviation of decay time
    double mean = std::accumulate(c2t.second.begin(), c2t.second.end(), 0.0) / c2t.second.size();
    double stdDev = 0;
    for (const auto& t : c2t.second)
      stdDev += std::pow(t - mean, 2.0);
    stdDev /= c2t.second.size();
    stdDev = std::sqrt(stdDev);
    if (stdDev < 2. ) // 2 seconds
      continue;

    // Divide pixels into recent part and old part by decay time
    std::vector<Eigen::Array2i> recentPixels;
    std::vector<Eigen::Array2i> oldPixels;
    recentPixels.reserve(clus2PixelIndices[clusIdx].size());
    oldPixels.reserve(clus2PixelIndices[clusIdx].size());
    for (int i = 0; i < clus2PixelIndices[clusIdx].size(); i++)
    {
      auto& id = clus2PixelIndices[clusIdx][i];
      if (c2t.second[i] < mean)
        recentPixels.push_back({id / this->ObstaclesGrid.Width, id % this->ObstaclesGrid.Width});
      else
        oldPixels.push_back({id / this->ObstaclesGrid.Width, id % this->ObstaclesGrid.Width});
    }
    if (recentPixels.empty() || oldPixels.empty())
      continue;

    // Check whether or not recentPixels can be separated into sub clusters
    std::vector<std::vector<Eigen::Array2i>> subClusters;
    std::vector<int> subClusId;
    // Label old pixels as unknownLabel to re-grow region
    for (const auto& pix : oldPixels)
      this->ObstaclesGrid(pix(0), pix(1)).ClusterIdx = unknownLabel;
    const std::vector<Eigen::Array2i> radii = {{-1, -1}, {0, -1}, {1, -1},
                                               {-1,  0},          {1,  0},
                                               {-1,  1}, {0,  1}, {1,  1}};
    for (const auto& seed : recentPixels)
    {
      // Check cluster id has changed
      if (this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx != clusIdx)
        continue;

      this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx = this->NewClusterIdx;

      std::vector<Eigen::Array2i> clusterPixels;
      clusterPixels.reserve(clus2PixelIndices[clusIdx].size());
      clusterPixels.emplace_back(seed);
      int idxPix = 0;
      while (idxPix < clusterPixels.size())
      {
        int i = clusterPixels[idxPix](0);
        int j = clusterPixels[idxPix](1);

        // Check neighbors
        #pragma omp parallel for num_threads(this->NbThreads)
        for (int idxRad = 0; idxRad < radii.size(); ++idxRad)
        {
          const Eigen::Array2i& r = radii[idxRad];
          Eigen::Array2i neigh = clusterPixels[idxPix] + r;
          if (this->ObstaclesGrid.Check(neigh.x(), neigh.y()) &&
              this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx == clusIdx)
          {
            // Add neighbor to cluster
            clusterPixels.push_back({neigh.x(), neigh.y()});
            // Update cluster label of pixel
            this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx = this->NewClusterIdx;
          }
        }
        ++idxPix;
      }
      subClusters.emplace_back(clusterPixels);
      subClusId.emplace_back(this->NewClusterIdx);
      ++this->NewClusterIdx;
    }
    // Recover the cluster index if recentPixels cannot be separated into sub clusters
    if (subClusters.size() == 1)
    {
      for (const auto& pix : recentPixels)
        this->ObstaclesGrid(pix(0), pix(1)).ClusterIdx = clusIdx;
      for (const auto& pix : oldPixels)
        this->ObstaclesGrid(pix(0), pix(1)).ClusterIdx = clusIdx;
      --this->NewClusterIdx;
      continue;
    }
    // Otherwise, re-label points in recent pixels with new cluster indices
    for (int i = 0; i< subClusters.size(); i++)
    {
      int& newClusId = subClusId[i];
      for (auto& pix : subClusters[i])
      {
        for (int ptIdx : this->ObstaclesGrid(pix(0), pix(1)).CurrentPtIndices)
          inputCloud[ptIdx].label = static_cast<std::uint16_t>(newClusId);
      }
    }

    // Grow region for old pixels in the cluster
    // Remove old pixels which split the clusters and
    // relabel the old pixels which connect with a recent sub cluster
    std::vector<std::vector<Eigen::Array2i>> oldSubClusters;
    std::vector<std::set<int>> oldSubClusNeighId;
    for (const auto& seed : oldPixels)
    {
      // Check cluster id has changed
      if (this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx != unknownLabel)
        continue;

      this->ObstaclesGrid(seed(0), seed(1)).ClusterIdx = this->NewClusterIdx;

      std::vector<Eigen::Array2i> clusterPixels;
      std::set<int> neighborClusId;
      clusterPixels.reserve(clus2PixelIndices[clusIdx].size());
      clusterPixels.emplace_back(seed);
      int idxPix = 0;
      while (idxPix < clusterPixels.size())
      {
        int i = clusterPixels[idxPix](0);
        int j = clusterPixels[idxPix](1);

        // Check neighbors
        for (const Eigen::Array2i& r : radii)
        {
          Eigen::Array2i neigh = clusterPixels[idxPix] + r;
          // Check neighbor validity
          if (!this->ObstaclesGrid.Check(neigh.x(), neigh.y()))
            continue;

          int neighClusIdx = this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx;
          if (neighClusIdx == unknownLabel)
          {
            // Add neighbor to cluster
            clusterPixels.push_back({neigh.x(), neigh.y()});
            // Update cluster label of pixel
            this->ObstaclesGrid(neigh.x(), neigh.y()).ClusterIdx = this->NewClusterIdx;
          }
          else if (std::find(subClusId.begin(), subClusId.end(), neighClusIdx) != subClusId.end())
          {
            neighborClusId.insert(neighClusIdx);
          }
        }
        ++idxPix;
      }
      oldSubClusters.emplace_back(clusterPixels);
      oldSubClusNeighId.emplace_back(neighborClusId);
      ++this->NewClusterIdx;
    }
    // Remove or re-label old pixels
    for (int i = 0; i < oldSubClusters.size(); i++)
    {
      // Remove old pixels if they are connected with more than one part of recent pixels
      if (oldSubClusNeighId[i].size() > 1)
      {
        for (auto& pix : oldSubClusters[i])
          this->ObstaclesGrid.Data.erase(pix(0) * this->ObstaclesGrid.Width + pix(1));
      }
      // Otherwise, re-label old pixels with its neighbor cluster id
      else
      {
        for (auto& pix : oldSubClusters[i])
        {
          int neighClusId = oldSubClusNeighId[i].size() > 0 ? *oldSubClusNeighId[i].begin() : clusIdx;
          this->ObstaclesGrid(pix(0), pix(1)).ClusterIdx = neighClusId;
          for (int ptIdx : this->ObstaclesGrid(pix(0), pix(1)).CurrentPtIndices)
            inputCloud[ptIdx].label = static_cast<std::uint16_t>(neighClusId);
        }
      }
    }
    // Recover the cluster id counter
    this->NewClusterIdx -= oldSubClusters.size();
  }

  // 4. Clear pts indices in remaining pixels and reset unknown labels to -1
  #pragma omp parallel for num_threads(this->NbThreads)
  for (int idxGrid = 0; idxGrid < this->ObstaclesGrid.Data.size(); ++idxGrid)
  {
    auto it = this->ObstaclesGrid.Data.begin();
    std::advance(it, idxGrid);
    Pixel& pix = it->second; // shortcut

    // Clear current points of each pixel for next fill
    pix.CurrentPtIndices.clear();

    // Reset to -1 the remaining unknown pixels (for visu purposes)
    if (pix.ClusterIdx == unknownLabel)
      pix.ClusterIdx = -1;
  }
}

//-----------------------------------------------------------------------------
void AggregationNode::LoadRefMap(const std::string& path)
{
  if (path.empty())
    return;
  CloudS::Ptr referencePoints(new CloudS);
  if (pcl::io::loadPCDFile(path, *referencePoints) == 0)
  {
    this->RefMap->Add(referencePoints, true, false);
    RCLCPP_INFO_STREAM(this->get_logger(), "Reference map successfully loaded : " << path);

    // Initialize occupancy grid.
    // The grid size and the origin depends on the reference map
    // The resolution is the same as the dense map
    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*referencePoints, minPoint, maxPoint);
    this->ObstaclesGrid.Origin = minPoint.head(2);
    this->ObstaclesGrid.Resolution = this->DenseMap->GetLeafSize();
    Eigen::Vector2f diff = maxPoint.head(2) - minPoint.head(2);
    this->ObstaclesGrid.Height = diff.y() / this->DenseMap->GetLeafSize();
    this->ObstaclesGrid.Width = diff.x() / this->DenseMap->GetLeafSize();
  }
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create options for the node to use undeclared parameters
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);

  // Create lidar slam node, which subscribes to pointclouds coming from conversion node
  // and to external sensor messages in parallel.
  std::shared_ptr<AggregationNode> aggregationNodePtr =
            std::make_shared<AggregationNode>("aggregation", options);

  // Handle callbacks until shut down
  rclcpp::spin(aggregationNodePtr);

  return 0;
}
