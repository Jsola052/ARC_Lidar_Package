//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Jeanne Faure (Kitware SAS)
// Creation date: 2023-12-01
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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/KeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace LidarSlam
{

//-----------------------------------------------------------------------------
bool LineFitting::FitLineAndCheckConsistency(const KeypointExtractor::PointCloud& cloud,
                                             const std::vector<int>& indices)
{
  // Check consistency of the line
  if (indices.size() < 2)
    return false;

  float sqDistMax = 0.f;
  float sqDistMin = std::numeric_limits<float>::max();
  float sqDistCurr;
  for (int i = 1; i < indices.size(); ++i)
  {
    sqDistCurr = (cloud[indices[i]].getVector3fMap() - cloud[indices[i-1]].getVector3fMap()).squaredNorm();
    sqDistMax = std::max(sqDistMax, sqDistCurr);
    sqDistMin = std::min(sqDistMin, sqDistCurr);
  }
  if (sqDistMax / sqDistMin > this->SquaredRatio)
    return false;

  Eigen::Vector3f diffVec = cloud[indices.back()].getVector3fMap() - cloud[indices.front()].getVector3fMap();
  this->Direction = diffVec.normalized();
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(cloud, indices, centroid);
  this->Position = centroid.head(3);

  // Check line width
  float lineLength = diffVec.norm();
  float widthThreshold = lineLength / this->LengthWidthRatio;

  for (auto idx : indices)
  {
    float error = (cloud[idx].getVector3fMap() - this->Position).cross(this->Direction).norm();
    if (error > widthThreshold)
      return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
void KeypointExtractor::Enable(const std::vector<Keypoint>& kptTypes)
{
  for (auto& en : this->Enabled)
    en.second = false;
  for (auto& k : kptTypes)
    this->Enabled[k] = true;
}

//-----------------------------------------------------------------------------
void KeypointExtractor::EstimateAzimuthalResolution()
{
  // Compute horizontal angle values between successive points
  std::vector<float> angles;
  angles.reserve(this->Scan->size());
  // Container storing the last point encountered in each line
  // to compute the angle difference with the new one in that line
  std::map<int, LidarPoint> prevPtByLine;

  for (const LidarPoint& point : *this->Scan)
  {
    if (prevPtByLine.count(point.laser_id))
    {
      Eigen::Map<const Eigen::Vector2f> pt1(point.data);
      Eigen::Map<const Eigen::Vector2f> pt2(prevPtByLine[point.laser_id].data);
      float cosAngle = pt1.dot(pt2) / (pt1.norm() * pt2.norm());
      cosAngle = std::clamp(cosAngle, -1.f, 1.f);
      float angle = std::abs(std::acos(cosAngle));
      angles.push_back(angle);
    }
    prevPtByLine[point.laser_id] = point;
  }

  // A minimum number of angles is needed to get a trustable estimator
  if (angles.size() < 100)
  {
    PRINT_WARNING("Not enough points to estimate azimuthal resolution");
    return;
  }

  // Estimate azimuthal resolution from these angles
  std::sort(angles.begin(), angles.end());
  unsigned int maxInliersIdx = angles.size();
  float maxAngle = Utils::Deg2Rad(5.);
  float medianAngle = 0.;
  // Iterate until only angles between direct LiDAR beam neighbors remain.
  // The max resolution angle is decreased at each iteration.
  while (maxAngle > 1.8 * medianAngle)
  {
    maxInliersIdx = std::upper_bound(angles.begin(), angles.begin() + maxInliersIdx, maxAngle) - angles.begin();
    medianAngle = angles[maxInliersIdx / 2];
    maxAngle = std::min(medianAngle * 2., maxAngle / 1.8);
  }
  this->AzimuthalResolution = medianAngle;

  if (this->Verbosity >= 3)
  {
    std::cout << "LiDAR (id: " << this->Scan->header.frame_id <<")'s azimuthal resolution estimated to " << Utils::Rad2Deg(this->AzimuthalResolution) << "°" << std::endl;
  }
}

//-----------------------------------------------------------------------------
void KeypointExtractor::ComputeBlobs()
{
  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  for (const auto& point : *this->Scan)
  {
    // Random sampling to decrease keypoints extraction
    // computation time
    if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
      continue;

    this->AddKeypoint(Keypoint::BLOB, point);
  }
}

//-----------------------------------------------------------------------------
bool KeypointExtractor::CheckAzimuthAngle(const Eigen::Vector3f& centralPoint)
{
  if (std::abs(std::abs(this->AzimuthMax - this->AzimuthMin) - 2.f * M_PI) < 1e-6)
    return true;

  if (std::abs(this->AzimuthMax - this->AzimuthMin) > 2.f * M_PI + 1e-6 ||
      this->AzimuthMin == this->AzimuthMax)
    return false;

  const float cosAzimuth = centralPoint.x() / std::sqrt(std::pow(centralPoint.x(), 2) + std::pow(centralPoint.y(), 2));
  const float azimuth = centralPoint.y() > 0 ? std::acos(cosAzimuth) : 2.f * M_PI - std::acos(cosAzimuth);

  if (this->AzimuthMin < this->AzimuthMax &&
      (azimuth < this->AzimuthMin || azimuth > this->AzimuthMax))
    return false;

  if (this->AzimuthMin > this->AzimuthMax &&
      (azimuth < this->AzimuthMin && azimuth > this->AzimuthMax))
    return false;

  return true;
}

//-----------------------------------------------------------------------------
bool KeypointExtractor::IsBeamAngleValid(const Eigen::Vector3f& centralPt,
                                         float centralDepth,
                                         const LineFitting& line)
{
  const float beamLineAngle = std::abs(line.Direction.dot(centralPt) / centralDepth);
  return beamLineAngle <= this->MinBeamSurfaceAngle;
}
} // End of LidarSlam namespace
