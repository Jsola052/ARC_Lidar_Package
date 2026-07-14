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

#pragma once

#include <pcl/point_cloud.h>
#include <cmath>
#include <map>

#include <LidarSlam/LidarPoint.h>

namespace lidar_conversions
{
namespace Utils
{

struct Cluster
{
  double Mean = 0.;
  double Std = 0.;
  bool Empty = true;
  std::vector<double> Inliers;
};

//------------------------------------------------------------------------------
/*!
 * @brief Copy pointcloud metadata to an other cloud
 * @param[in] from The pointcloud to copy info from
 * @param[out] to The pointcloud to copy info to
 */
template<typename PointI, typename PointO>
inline void CopyPointCloudMetadata(const pcl::PointCloud<PointI>& from, pcl::PointCloud<PointO>& to)
{
  to.header = from.header;
  to.is_dense = from.is_dense;
  to.sensor_orientation_ = from.sensor_orientation_;
  to.sensor_origin_ = from.sensor_origin_;
}

//----------------------------------------------------------------------------
/*!
  * @brief Initialize the PointCloud published by lidar driver
  * @param msg_received New Lidar Frame, published by lidar_pointcloud/cloud_node.
  */
template<typename CloudRawType>
CloudRawType InitCloudRaw(const sensor_msgs::msg::PointCloud2& msg_received)
{
  CloudRawType cloudRaw;
  pcl::fromROSMsg(msg_received, cloudRaw);
  return cloudRaw;
}

//------------------------------------------------------------------------------
/*!
  * @brief Initialize the PointCloud needed by SLAM
  * @param cloudRaw PointCloud published by lidar driver
  */
template<typename CloudRawType>
pcl::PointCloud<LidarSlam::LidarPoint> InitCloudS(CloudRawType cloudRaw)
{
  // Init SLAM pointcloud
  pcl::PointCloud<LidarSlam::LidarPoint> cloudS;
  cloudS.reserve(cloudRaw.size());

  // Copy pointcloud metadata
  Utils::CopyPointCloudMetadata(cloudRaw, cloudS);
  cloudS.is_dense = true;

  return cloudS;
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if a PCL point is valid
 * @param point A PCL point, with possible unvalid coordinates.
 * @return true if point coordinates are finite and
 *         the point distance is in range [1e-6, 1000], false otherwise.
 */
template<typename PointT>
inline bool IsPointValid(const PointT& point)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
    return false;

  float distance = point.getVector3fMap().norm();
  return distance > 1e-6 && distance < 1000.;
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if a lidarPoint is valid
 * @param point A lidarpoint, with possible NaN as field values.
 * @return true if a field value is NaN.
 */
inline bool HasNanField(const LidarSlam::LidarPoint& point)
{
  return !std::isfinite(point.x)         ||
         !std::isfinite(point.y)         ||
         !std::isfinite(point.z)         ||
         !std::isfinite(point.time)      ||
         !std::isfinite(point.intensity) ||
         !std::isfinite(point.laser_id)  ||
         !std::isfinite(point.label);
}

//------------------------------------------------------------------------------
/*!
 * @brief Check if the rotation duration has a likely value
 * @return boolean to know if the rotation duration is correct
 * @param rotationDuration RotationDuration of the lidar
 */
inline bool CheckRotationDuration(double rotationDuration, std::vector<double> possibleFrequencies)
{
  // We assume that if the user hasn't specified a list of possible frequencies,
  // it's because he doesn't want to check the rotation duration or doesn't care about outliers
  if (possibleFrequencies.empty())
    return true;

  for (double frequency : possibleFrequencies)
  {
    double currentFreq = 1. / rotationDuration;
    double epsilon = 0.05 * frequency; // We accept 5% error
    if (frequency - epsilon < currentFreq && currentFreq < frequency + epsilon)
      return true;
  }
  return false;
}

//------------------------------------------------------------------------------
/*!
 * @brief Convert PCL timestamp (in microseconds) to seconds
 * @param pclStampUs PCL timestamp, in microseconds
 * @return Timestamp in seconds
 */
inline constexpr double PclStampToSec(uint64_t pclStampUs)
{
  return pclStampUs * 1e-6;
}

//------------------------------------------------------------------------------
/*!
 * @brief Get the factor between points time and frame time
 * @return factor to scale point time to frame time : frame.Time = point.Time * factor
 * @param duration difference between first point and last point of frame
 * @param rotationDuration difference between one frame and the following one
 */
inline double GetTimeFactor(double duration, double rotationDuration)
{
  int power = std::round(std::log10(rotationDuration)) - std::round(std::log10(duration));
  return std::pow(10, power);
}

//------------------------------------------------------------------------------
/*!
 * @brief Organize angles in clusters (in order to deduce laser_id)
 * @return Clusters of vertical angles
 * @param cloudRaw PointCloud published by lidar driver
 * @param nbLasers Number of lasers of the lidar
 * @param clusters Clusters of vertical angles to initialize
 * @param nbThreads Number of threads to use for the clustering
 */
template <typename PointType>
inline void ClusterizeVerticalAngles(const pcl::PointCloud<PointType>& cloudRaw, unsigned int nbLasers, std::vector<Cluster>& clusters, int nbThreads)
{
  std::vector<double> verticalAngles;
  verticalAngles.reserve(cloudRaw.size());
  // Initialisation of the means of each cluster
  for (const auto& pt : cloudRaw)
  {
    if (!Utils::IsPointValid(pt))
      continue;
    double ratio = pt.z / pt.getVector3fMap().norm();
    ratio = std::max(std::min(ratio, 1.-1e-6), -1.+1e-6);
    verticalAngles.emplace_back((180./M_PI) * std::acos(ratio));
  }

  double maxAngle = *std::max_element(verticalAngles.begin(), verticalAngles.end());
  double minAngle = *std::min_element(verticalAngles.begin(), verticalAngles.end());
  double clusterWidth = (maxAngle - minAngle) / double(nbLasers);

  clusters.resize(nbLasers);
  #pragma omp parallel for num_threads(nbThreads)
  for (size_t idxCluster = 0; idxCluster < nbLasers; ++idxCluster)
    clusters[idxCluster].Mean = minAngle + idxCluster * clusterWidth;

  std::vector<int> prevNbInliers(clusters.size());
  bool hasConverged = false;
  // Adjust these cluster means
  while (!hasConverged)
  {
    // Fill the inliers vector of a Cluster (~ the angles are sorted in clusters)
    // Necessary hypothesis : clusters vector is sorted by increasing mean (handled)
    for (const double& angle : verticalAngles)
    {
      auto insertionIt = std::upper_bound(clusters.begin(), clusters.end(), angle,
                                          [](double angle, const Cluster &cluster)
                                          {return angle < cluster.Mean;});
      if (insertionIt == clusters.begin())
      {
        clusters.front().Inliers.push_back(angle);
        clusters.front().Empty = false;
      }
      else if (insertionIt == clusters.end())
      {
        clusters.back().Inliers.push_back(angle);
        clusters.back().Empty = false;
      }
      else
      {
        auto prev = std::prev(insertionIt);
        auto& minElemIt = angle - prev->Mean < insertionIt->Mean - angle ? prev : insertionIt;
        minElemIt->Inliers.push_back(angle);
        minElemIt->Empty = false;
      }
    }

    // Compute standard dev (std) of each vector of angles (inliers) from a Cluster
    for (Cluster& cluster : clusters)
    {
      if (cluster.Empty)
      {
        cluster.Std = 0.;
        continue;
      }
      double newMean = std::accumulate(cluster.Inliers.begin(), cluster.Inliers.end(), 0.) /
                       double(cluster.Inliers.size());
      cluster.Mean = newMean;
      double sumSquaredSteps = 0.;
      for (const double& angle : cluster.Inliers)
        sumSquaredSteps += std::pow(angle - newMean, 2.0);
      cluster.Std = std::sqrt(sumSquaredSteps / cluster.Inliers.size());
    }

    hasConverged = true;

    // Compare old sizes of each cluster to their new one, to see if the model tends to stabilize
    for (unsigned int idxCluster = 0; idxCluster < clusters.size(); ++idxCluster)
    {
      if (prevNbInliers[idxCluster] - int(clusters[idxCluster].Inliers.size()) != 0)
        hasConverged = false;
      // Prepare the next iteration of the while loop
      prevNbInliers[idxCluster] = clusters[idxCluster].Inliers.size();
    }

    // Identify the empty clusters and reset them to the higher standard dev cluster
    for (Cluster& cluster : clusters)
    {
      if (cluster.Empty)
      {
        Cluster clusterMaxStd = *std::max_element(clusters.begin(), clusters.end(),
                                                   [](const Cluster &cluster1,
                                                      const Cluster &cluster2)
                                                    {return cluster1.Std < cluster2.Std;});
        clusterMaxStd.Std /= 2.;
        cluster.Mean = clusterMaxStd.Mean + clusterMaxStd.Std;
        hasConverged = false;
      }
      std::vector<double>().swap(cluster.Inliers); // Empty the inliers vector to prepare the next iteration
      cluster.Empty = true;
    }

    // Before iterating again, we sort the clusters by mean since the mean of the empty ones has been changed
    std::sort(clusters.begin(), clusters.end(),
              [](const Cluster& cluster1, const Cluster& cluster2)
              {return cluster1.Mean < cluster2.Mean;});
  }
}

//----------------------------------------------------------------------------
/*!
* @brief Compute laser_id of a point missing this field
* @return ID of the laser the current point comes from
* @param currentPoint Point(x,y,z) extracted from current slamPoint
* @param nbLasers Number of lasers of the LiDAR sensor
* @param clusters Clusters of vertical angles
*/
inline int ComputeLaserId(const Eigen::Vector3d& currentPoint, unsigned int nbLasers, const std::vector<Cluster>& clusters)
{
  // Estimate laser ID thanks to a clustering of vertical angles
  double vertAngle = (180./M_PI) * std::acos(double(currentPoint.z()) / currentPoint.norm());
  auto insertionIt = std::upper_bound(clusters.begin(), clusters.end(),
                                      vertAngle,
                                      [](double vertAngle, const Cluster &cluster)
                                      {return vertAngle < cluster.Mean;});
  if (insertionIt == clusters.begin())
    return 0;

  if (insertionIt == clusters.end())
    return nbLasers - 1;

  auto prev = std::prev(insertionIt);
  return std::distance(clusters.begin(), (vertAngle - prev->Mean < insertionIt->Mean - vertAngle) ? prev : insertionIt);
}

//----------------------------------------------------------------------------
/*!
 * @brief Get first valid point in the pointcloud
 * @return First valid point in the pointcloud.
 * @param cloudRaw PointCloud published by lidar driver
 */
template <typename PointType>
inline Eigen::Vector2d GetFirstValidPoint(const pcl::PointCloud<PointType>& cloudRaw)
{
  for (auto id = 0; id < cloudRaw.size(); id++)
  {
    if (Utils::IsPointValid(cloudRaw[id]))
    {
      return {cloudRaw[id].x, cloudRaw[id].y};
    }
  }
  return Eigen::Vector2d::Zero();
}

//----------------------------------------------------------------------------
/*!
 * @brief Deduce the rotation sense of the lidar
 * @return true if the LiDAR rotates clockwise, false otherwise.
 * @param cloudRaw PointCloud published by lidar driver
 * @param nbLasers Number of lasers of the lidar
 */
template <typename PointType>
inline bool IsRotationClockwise(const pcl::PointCloud<PointType>& cloudRaw, unsigned int nbLasers)
{
  for (auto id = 0; id < cloudRaw.size(); id++)
  {
    if (Utils::IsPointValid(cloudRaw[id]) && Utils::IsPointValid(cloudRaw[id + nbLasers]))
    {
      return cloudRaw[id].x * cloudRaw[id + nbLasers].y - cloudRaw[id].y * cloudRaw[id + nbLasers].x > 0;
    }
  }
  return false;
}

//----------------------------------------------------------------------------
/*!
 * @brief Estimate time of a point missing this field
 * @return time of the current point
 * @param currentPoint Point(x,y) extracted from current slamPoint
 * @param rotationDuration Time for a full rotation of the lidar
 * @param firstPoint First point of the frame
 * @param rotationIsClockwise True if the LiDAR rotates clockwise, false otherwise.
 */
inline double EstimateTime(const Eigen::Vector2d& currentPoint, double rotationDuration, const Eigen::Vector2d& firstPoint, bool rotationIsClockwise = true)
{
  double angle_h = std::acos(firstPoint.normalized().dot(currentPoint.normalized()));
  double crossZ = firstPoint.x() * currentPoint.y() - firstPoint.y() * currentPoint.x();
  if ((crossZ <= 0 && rotationIsClockwise) || (crossZ > 0 && !rotationIsClockwise))
    angle_h = 2. * M_PI - angle_h;
  return ((angle_h / (2.*M_PI)) - 1) * rotationDuration;
}

//------------------------------------------------------------------------------
/*!
 * @brief Initialize estimation parameter : clusters
 * @param cloudRaw PointCloud published by lidar driver
 * @param nbLasers Number of lasers of the lidar
 * @param clusters Clusters of vertical angles to initialize
 * @param rotationIsClockwise True if the LiDAR rotates clockwise, false otherwise.
 * @param nbThreads Number of threads to use for the conversion
 */
template<typename PointT>
inline void InitEstimationParameters(const pcl::PointCloud<PointT>& cloudRaw,
                                     unsigned int nbLasers,
                                     std::vector<Cluster>& clusters,
                                     bool& rotationIsClockwise,
                                     int nbThreads = 1)
{
  Utils::ClusterizeVerticalAngles<PointT>(cloudRaw, nbLasers, clusters, nbThreads);
  rotationIsClockwise = Utils::IsRotationClockwise<PointT>(cloudRaw, nbLasers);
}

//------------------------------------------------------------------------------
/*!
 * @struct Helper to estimate point-wise within frame advancement for a spinning
 * lidar sensor using azimuth angle.
 */
struct SpinningFrameAdvancementEstimator
{
  /*!
   * @brief Reset the estimator. This should be called when a new frame is received.
   */
  void Reset()
  {
    this->PreviousAdvancementPerRing.clear();
  }

  /*!
   * @brief Estimate point advancement within current frame using azimuth angle.
   * @param point the point to estimate its relative advancement. It MUST have a 'laser_id' field.
   * @return relative advancement in range [0;~1].
   *
   * This computation is based on azimuth angle of each measured point.
   * The first point will return a 0 advancement.
   * Advancement will increase clock-wise, reaching 1 when azimuth angle reaches
   * initial azimuth value. It may be greater than 1 if the frame spins more
   * than 360 degrees.
   *
   * NOTE: this estimation uses 2 priors:
   * - real azimuth is always strictly inceasing within each laser ring,
   * - real azimuth angle of any first point of a laser ring should be greater than initial azimuth.
   */
  template<typename PointT>
  double operator()(const PointT& point)
  {
    // Compute normalized azimuth angle (in range [0-1])
    double pointAdvancement = (M_PI - std::atan2(point.y, point.x)) / (2 * M_PI);

    // If this is the first point of the frame
    if (this->PreviousAdvancementPerRing.empty())
      this->InitAdvancement = pointAdvancement;

    // Get normalized angle (in [0-1]), with angle 0 being first point direction
    auto wrapMax = [](double x, double max) { return std::fmod(max + std::fmod(x, max), max); };
    double frameAdvancement = wrapMax(pointAdvancement - this->InitAdvancement, 1.);

    // If we detect overflow, correct it
    // If current laser_id is not in map, the following line will insert it,
    // associating it to value 0.0.
    if (frameAdvancement < this->PreviousAdvancementPerRing[point.laser_id])
      frameAdvancement += 1.;
    this->PreviousAdvancementPerRing[point.laser_id] = frameAdvancement;

    return frameAdvancement;
  }

private:
  double InitAdvancement;
  std::map<int, double> PreviousAdvancementPerRing;
};

}  // end of namespace Utils
}  // end of namespace lidar_conversions
