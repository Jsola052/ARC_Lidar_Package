//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
//         Laurenson Nick (Kitware SAS)
//         Cadart Nicolas (Kitware SAS)
// Creation date: 2018-03-27
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

#include "LidarSlam/LidarPoint.h"
#include "LidarSlam/Enums.h"
#include "LidarSlam/VoxelGrid.h"
#include "KeypointExtractor.h"

#include <pcl/point_cloud.h>

#include <unordered_map>
#include <map>
#include <bitset>

namespace LidarSlam
{
class SpinningSensorKeypointExtractor : public KeypointExtractor
{
public:
  GetMacro(VoxelResolution, float)
  SetMacro(VoxelResolution, float)

  // Set EdgeSinAngleThreshold and PlaneSinAngleThreshold from angle in degrees
  void SetEdgeAngleThreshold(float angle) override;
  void SetPlaneAngleThreshold(float angle) override;

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  void ComputeKeyPoints(const PointCloud::Ptr& pc) override;

  PointCloud::Ptr GetKeypoints(Keypoint k) override;

  // Function to enable to have some inside on why a given point was detected as a keypoint
  std::unordered_map<std::string, std::vector<float>> GetDebugArray() const override;

  // Function to clone the extractor;
  std::shared_ptr<KeypointExtractor> Clone() const override;

private:

  // Split the whole pointcloud into separate laser ring clouds,
  // sorted by their vertical angles.
  // This expects that the lowest/bottom laser ring is 0, and is increasing upward.
  void ConvertAndSortScanLines();

  // Reset all the features vectors and keypoints clouds
  void PrepareDataForNextFrame();

  // Compute the curvature and other features within each the scan line.
  // The curvature is not the one of the surface that intersects the lines but
  // the 1D curvature within each isolated scan line.
  void ComputeCurvature() override;

  // Labelize points (unvalid, edge, plane)
  // and extract them in correspondant pointcloud
  // note : blobs are extracted identically in SSKE and DSSKE so ComputeBlobs() is defined in parent abstract class
  void ComputePlanes() override;
  void ComputeEdges() override;
  void ComputeIntensityEdges() override;

  // Add point to the voxel grid
  void AddKeypoint(const Keypoint& k, const LidarPoint &pt) override;

  // Add all keypoints of the type k that comply with the threshold criteria for these values
  // The threshold can be a minimum or maximum value (threshIsMax)
  // The weight basis allow to weight the keypoint depending on its certainty
  void AddKptsUsingCriterion (Keypoint k,
                              const std::vector<std::vector<float>>& values,
                              float threshold,
                              bool threshIsMax = true,
                              double weightBasis = 1.);

  // Helper to get the pointcloud of the scan line of index i
  // i being a continuous index from 0 to nbScanlines (not equivalent to laser_id)
  PointCloud::Ptr GetScanlineCloud(unsigned int i);

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  // Size of a voxel used to downsample the keypoints
  // It corresponds approx to the mean distance between closest neighbors in the output keypoints cloud.
  float VoxelResolution = 0.1; // [m]

  // Sharpness threshold to select a planar keypoint
  float PlaneSinAngleThreshold = 0.5;  // sin(30°) (selected if sin angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  float EdgeSinAngleThreshold = 0.86; // ~sin(60°) (selected, if sin angle is more than threshold)

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  //! Label of a point as a keypoint
  //! We use binary flags as each point can have different keypoint labels.
  using KeypointFlags = std::bitset<Keypoint::nKeypointTypes>;

  // Curvature and other differential operations (scan by scan, point by point)
  std::vector<std::vector<float>> Angles;
  std::vector<std::vector<float>> DepthGap;
  std::vector<std::vector<float>> SpaceGap;
  std::vector<std::vector<float>> IntensityGap;
  std::vector<std::vector<KeypointFlags>> Label;

  // Extracted keypoints of current frame
  std::map<Keypoint, VoxelGrid> Keypoints;

  // Map of the scan lines, sorted by their laser_id.
  std::unordered_map<int, PointCloud::Ptr> ScanLines;
};
} // end of LidarSlam namespace