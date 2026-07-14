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

#ifndef AGGREGATION_NODE_H
#define AGGREGATION_NODE_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// LidarSlam
#include <LidarSlam/LidarPoint.h>
#include <LidarSlam/RollingGrid.h>

// Local
#include "ros_transform_utils.h"
#include <lidar_slam/srv/save_pc.hpp>
#include <lidar_slam/srv/reset.hpp>

class AggregationNode : public rclcpp::Node
{
public:

  using PointS = LidarSlam::LidarPoint;
  using CloudS = pcl::PointCloud<PointS>;  ///< Pointcloud needed by SLAM
  using Pcl2_msg = sensor_msgs::msg::PointCloud2; //messages for Pointcloud

  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] name_node Name of the node
   * @param[in] options Options of the node
   */
  AggregationNode(std::string name_node = "aggregation",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  //----------------------------------------------------------------------------
  /*!
   * @brief     New main frame callback, aggregating frames
   * @param[in] registeredCloudMsg New registered frame message published by the LidarSlamNode
   *
   * Input pointcloud must have following fields :
   *  - x, y, z (float): point coordinates
   *  - time (double): time offset to add to the pointcloud header timestamp to
   *    get approximate point-wise acquisition timestamp
   *  - intensity (float): intensity/reflectivity of the point
   *  - laser_id (uint16): numeric identifier of the laser ring that shot this point.
   *    The lowest/bottom laser ring should be 0, and it should increase upward.
   *  - label (uint8): optional input, not yet used.
   */
  void Callback(const Pcl2_msg& registeredCloudMsg);

  void SavePointcloudService(
    const std::shared_ptr<lidar_slam::srv::SavePc::Request> req,
    const std::shared_ptr<lidar_slam::srv::SavePc::Response> res);

  void ResetService(
    const std::shared_ptr<lidar_slam::srv::Reset::Request> req,
    const std::shared_ptr<lidar_slam::srv::Reset::Response> res);

  void PoseCallback(const nav_msgs::msg::Odometry& poseMsg);

private:

  struct Pixel
  {
    // Cluster id
    int ClusterIdx = -1;

    // Point indices for current frame
    std::vector<int> CurrentPtIndices;

    int SeenTimes = 0;
    double Time = -1.;
  };

  // 2D grid to save if a grid contains obstacles points
  struct ClusteringGrid
  {
    // Grid of pixels
    std::unordered_map<int, Pixel> Data; // map because few should be occupied
    // Grid size and origin depends on reference map
    Eigen::Vector2f Origin = {0., 0.};
    float Resolution = 0.1;

    std::vector<signed char> Flatten() const
    {
      std::vector<signed char> flattenedClusterIdx;
      flattenedClusterIdx.reserve(this->Data.size());
      for (int i = 0; i < Height; ++i)
      {
        for (int j = 0; j < Width; ++j)
        {
          int idx = i * Width + j;
          if (this->Data.count(idx))
            flattenedClusterIdx.emplace_back(static_cast<signed char>(this->Data.at(idx).ClusterIdx));
          else
            flattenedClusterIdx.emplace_back(static_cast<signed char>(-1));
        }
      }
      return flattenedClusterIdx;
    }

    bool Check(int i, int j)
    {
      int idx = i * Width + j;
      return this->Data.count(idx);
    }

    Pixel& operator()(int i, int j)
    {
      int idx = i * Width + j;
      return this->Data[idx];
    }

    int Height = 0;
    int Width = 0;
    int Size() const {return this->Data.size();}
  };

  struct BBox
  {
    Eigen::Vector3d Center = {0., 0., 0.};
    Eigen::Vector3d Diagonal = {0., 0., 0.};
  };

  // Tool functions
  // Extract the slice of points perpendicular to the local trajectory
  // Compute its boundary and return its area
  double ExtractSlice(double sliceWidth, double sliceMaxDist, double angleStep, CloudS& boundary);

  // Extract the points belonging to a z slice defined by user
  void ExtractZslice(const CloudS& inputCloud, CloudS& outputCloud);

  // Extract the obstacle points
  void LabelObstacle(CloudS& inputCloud, double currentTime);

  // Load reference map
  void LoadRefMap(const std::string& path);

  // Clear obstacles grid after growing region
  void ClearObstaclesGrid(CloudS& inputCloud, double currentTime);

  // ROS subscribers, publishers and services
  rclcpp::Subscription<Pcl2_msg>::SharedPtr FrameSubscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  PoseSubscriber;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr PointsPublisher;
  rclcpp::Publisher<Pcl2_msg>::SharedPtr SlicePublisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr SliceAreaPublisher;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr OccupancyPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr MarkerPublisher;
  rclcpp::Service<lidar_slam::srv::SavePc>::SharedPtr SaveService;
  rclcpp::Service<lidar_slam::srv::Reset>::SharedPtr RstService;

  // Dense map containing aggregated points from all frames
  std::shared_ptr<LidarSlam::RollingGrid> DenseMap;
  std::shared_ptr<LidarSlam::RollingGrid> RefMap;
  CloudS::Ptr Pointcloud;

  // Number of threads to use in parallel processing
  unsigned int NbThreads = 1;

  // Slice extraction parameters
  bool DoExtractSlice = false;
  // Optional positions logged to compute
  // the direction to create a slice
  std::list<Eigen::Vector3d> Positions;
  Eigen::Isometry3d CurrentPose = Eigen::Isometry3d::Identity();
  double TrajectoryMaxLength = 0.5; // 50 cm
  double SliceWidth = 0.2; // 20 cm
  double SliceMaxDist = 5.; // 5 m
  unsigned int MinSlicePtsWithoutMovObjects = 50;
  // Bin range for the circular histogram
  double AngleStep = 3. * M_PI / 180.; // 3°

  // Z slice extraction parameters
  bool DoExtractZslice = false;
  bool InvertZsliceExtraction = false;
  // Horizontal slice parameters
  double ZsliceWidth = 0.2;
  double ZsliceHeightPosition = -0.5;
  double MinObstacleMarkerSize = 0.3;

  // Obstacle extraction parameters
  bool DoExtractObstacle = false;
  // 2D occupancy grid to extract clusters of obstacle
  ClusteringGrid ObstaclesGrid;
  // Oriented bounding box of obstacles
  std::unordered_map<int, BBox> ObstaclesBBox;
  // Maximum distance between two clusters to merge them
  float MergeDist = 0.5;
  // Distance of the field of view
  float FOVDist = 20.;
  // Storage for new cluster idx
  int NewClusterIdx = 2; // 0 is default, 1 is for fixed pixel

  // Minimal distance around trajectory to remove points from the map
  double MinDistAroundTrajectory = 1.;
  // Maximal distance around trajectory to remove far points from the last pose
  double MaxDistAroundTrajectory = -1.;
};

#endif // AGGREGATION_NODE_H
