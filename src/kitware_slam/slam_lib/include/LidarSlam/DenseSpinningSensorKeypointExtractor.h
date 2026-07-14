//==============================================================================
// Copyright 2022 Kitware, Inc., Kitware SAS
// Author: Jeanne Faure (Kitware SAS)
// Creation date: 2023-10-12
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

#include "Utilities.h"
#include "LidarPoint.h"
#include "Enums.h"
#include "KeypointExtractor.h"
#include <unordered_map>

namespace LidarSlam
{

struct PtFeat
{
  int Index;
  float Depth;
  float SpaceGap;
  float DepthGap;
  float IntensityGap;
  float Angle;
  std::bitset<Keypoint::nKeypointTypes> KptTypes;

  PtFeat() : Index(0), Depth(0.0f), SpaceGap(-1.0f), DepthGap(0.0f), IntensityGap(-1.0f), Angle(1.0f), KptTypes({}) {}
};

struct IdxVM
{
  int Row;
  int Col;
};

class DenseSpinningSensorKeypointExtractor : public KeypointExtractor
{
public:

  void SetEdgeAngleThreshold(float angle) override;
  void SetPlaneAngleThreshold(float angle) override;

  GetMacro(PatchSize, int)
  SetMacro(PatchSize, int)

  GetMacro(SamplingDSSKE, LidarSlam::SamplingModeDSSKE)
  SetMacro(SamplingDSSKE, LidarSlam::SamplingModeDSSKE)

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

  // Find the pointer to PtFeat of a point in the scan (designated by its index in the scan)
  // (PtFeat containing the features of the point : space gap, depth...)
  std::shared_ptr<PtFeat> GetPtFeat(int idxInScan) const;

  // Count the number of non null ptr in a scanline
  int GetScanLineSize(const std::vector<std::shared_ptr<PtFeat>>& scanLine);

  // Initialize LaserIdMap, NbLaserRings, AzimuthalResolution and Pc2VmIndices
  void InitInternalParameters();

  // Create vertex map from input pointcloud using indices stored in Pc2VmIndices
  void CreateVertexMap();

  // Clear Keypoints Poinclouds, reserve new size
  void ClearKeypoints();

  // Output separate point features contained in Vertex Map in pgm format to visualize as 2D image
  void OutputFeatures(std::string path);

  // Output Keypoints in csv format to use as 3D pointcloud
  // and in pgm format to visualize as 2D image
  bool OutputKeypoints(std::string path);

  // Compute the curvature features within each scan line : depth
  // space gap, intensity gap and line angle
  void ComputeCurvature() override;

  // Labelize points (unvalid, edge, plane)
  // and extract them in correspondant pointcloud
  // note : blobs are extracted identically in SSKE and DSSKE so ComputeBlobs() is defined in parent abstract class
  void ComputePlanes() override;
  void ComputeEdges() override;
  void ComputeIntensityEdges() override;

  // Add point to keypoint structure
  void AddKeypoint(const Keypoint& k, const LidarPoint &pt) override;

  // Create square division of the image using 2 dimensions
  void Create2DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid);

  // Create cubic division of the pointcloud using 3 dimensions
  void Create3DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid);

  // Clear grid and reset the number of points in the grid
  // To be called at each keypoint computation (in ComputeEdges, ComputePlanes, etc.))
  void ClearGrid();

  // Add keypoints of type k to a keypoint pointcloud
  // Using patches to have a uniform distribution of keypoints
  void AddKptsUsingGrid(Keypoint k,
                        std::function<bool(const std::shared_ptr<PtFeat>&,
                                           const std::shared_ptr<PtFeat>&)> comparePtFeats);

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  // Sharpness threshold to select a planar keypoint
  // Also used, with its opposite value, to filter too sharp edges
  float PlaneCosAngleThreshold = -0.86;  // ~cos(150°) (selected if cos angle is less than threshold)

  // Sharpness threshold to select an edge keypoint
  float EdgeCosAngleThreshold = -0.5; // ~cos(120°) (selected, if cos angle is more than threshold)

  // Downsampling mode
  SamplingModeDSSKE SamplingDSSKE = SamplingModeDSSKE::PATCH;

  // Size of a patch (nb of points in one dimension, a patch is a square)
  // Patches are used for 2D grid construction to downsample the keypoints
  // A patch with size 32 means that the patch will contain at most 32x32 points
  int PatchSize = 32; // [nb]

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  // Dimensions of the Vertex Map
  int HeightVM;
  int WidthVM;

  // Map of laser_id to fit random laser_ids into {0, ..., NbLaserRings-1}
  std::unordered_map<int, int> LaserIdMap;

  // Vector linking the index of a point in the pointcloud to its index in the Vertex Map
  std::vector<IdxVM> Pc2VmIndices;

  // Vertex Map of points' indices in the pointcloud
  std::vector<std::vector<std::shared_ptr<PtFeat>>> VertexMap;

  // Grid (voxel or patch grid) used to downsample the keypoints
  std::unordered_map<int, std::vector<std::shared_ptr<PtFeat>>> Grid;

  // Struct to store the number of points in each voxel/patch of the used grid;
  int NbPointsInGrid;

  // Extracted keypoints of current frame
  std::map<Keypoint, std::vector<LidarPoint>> Keypoints;
};

} // namespace LidarSlam