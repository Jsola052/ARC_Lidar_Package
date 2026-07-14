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

#pragma once

#include "Utilities.h"
#include "LidarPoint.h"
#include "Enums.h"
#include <unordered_map>

#define SetMacro(name,type) void Set##name (type _arg) { name = _arg; }
#define GetMacro(name,type) type Get##name () const { return name; }

namespace LidarSlam
{

struct LineFitting
{
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

  //! Fitting using very local line and check if this local
  //! line is consistent in a more global neighborhood.
  //! Warning : this implies factorial calculations relatively to the points number
  bool FitLineAndCheckConsistency(const PointCloud& cloud,
                                  const std::vector<int>& indices);

  //! Compute the squared distance of a point to the fitted line
  inline float DistanceToPoint(const Eigen::Vector3f& point) const
  {
    return ((point - this->Position).cross(this->Direction)).norm();
  };

  // Direction and position
  Eigen::Vector3f Direction = Eigen::Vector3f::Zero();
  Eigen::Vector3f Position = Eigen::Vector3f::Zero();

  //! Ratio (squared) for squared distances of direct neighbors to consider a neighborhood as valid to fit line on
  float SquaredRatio = 100.; // [.]

  // Ratio between length and width to be trustworthy
  float LengthWidthRatio = 10.; // [.]
};

class KeypointExtractor
{
public:
  using Point = LidarPoint;
  using PointCloud = pcl::PointCloud<Point>;

  GetMacro(NbThreads, int)
  SetMacro(NbThreads, int)

  GetMacro(Verbosity, int)
  SetMacro(Verbosity, int)

  GetMacro(MaxPoints, int)
  SetMacro(MaxPoints, int)

  GetMacro(InputSamplingRatio, float)
  SetMacro(InputSamplingRatio, float)

  GetMacro(MinNeighNb, int)
  SetMacro(MinNeighNb, int)

  GetMacro(MinNeighRadius, float)
  SetMacro(MinNeighRadius, float)

  GetMacro(MinDistanceToSensor, float)
  SetMacro(MinDistanceToSensor, float)

  GetMacro(MaxDistanceToSensor, float)
  SetMacro(MaxDistanceToSensor, float)

  GetMacro(EdgeDepthGapThreshold, float)
  SetMacro(EdgeDepthGapThreshold, float)

  GetMacro(EdgeIntensityGapThreshold, float)
  SetMacro(EdgeIntensityGapThreshold, float)

  GetMacro(AzimuthalResolution, float)
  SetMacro(AzimuthalResolution, float)

  GetMacro(EdgeNbGapPoints, int)
  SetMacro(EdgeNbGapPoints, int)

  GetMacro(VoxelResolution, float)
  SetMacro(VoxelResolution, float)

  GetMacro(EdgeAngleThreshold, float)
  virtual void SetEdgeAngleThreshold (float thresh) { EdgeAngleThreshold = thresh; }

  GetMacro(PlaneAngleThreshold, float)
  virtual void SetPlaneAngleThreshold (float thresh) { PlaneAngleThreshold = thresh; }

  GetMacro(NbLaserRings, unsigned int)

  void SetAzimuthMin(float angle)
  {
    float azimMin = Utils::Deg2Rad(angle);
    while (azimMin < 0)
      azimMin += 2 * M_PI;
    this->AzimuthMin = azimMin;
  };
  float GetAzimuthMin()
  {
    return Utils::Rad2Deg(this->AzimuthMin);
  };
  void SetAzimuthMax(float angle)
  {
    float azimMax = Utils::Deg2Rad(angle);
    while (azimMax < 0)
      azimMax += 2 * M_PI;
    this->AzimuthMax = azimMax;
  };
  float GetAzimuthMax()
  {
    return Utils::Rad2Deg(this->AzimuthMax);
  };

  void SetMinBeamSurfaceAngle(float angle)
  {
    this->MinBeamSurfaceAngle = std::cos(Utils::Deg2Rad(angle));
  };
  GetMacro(MinBeamSurfaceAngle, float)

  // Select the keypoint types to extract
  // This function resets the member map "Enabled"
  void Enable(const std::vector<Keypoint>& kptTypes);

  // Extract keypoints from the pointcloud. The key points
  // will be separated in two classes : Edges keypoints which
  // correspond to area with high curvature scan lines and
  // planar keypoints which have small curvature.
  // NOTE: This expects that the lowest/bottom laser_id is 0, and is increasing upward.
  virtual void ComputeKeyPoints(const PointCloud::Ptr& pc) = 0;

  virtual PointCloud::Ptr GetKeypoints(Keypoint k) = 0;

  // Function to enable to have some inside on why a given point was detected as a keypoint
  virtual std::unordered_map<std::string, std::vector<float>> GetDebugArray() const = 0;

  // Function to clone the extractor;
  virtual std::shared_ptr<KeypointExtractor> Clone() const = 0;

protected:

  // Check if scanLine is almost empty
  inline bool IsScanLineAlmostEmpty(int nScanLinePts) const { return nScanLinePts < 2 * this->MinNeighNb + 1; };

  // Auto estimate azimuth angle resolution based on current ScanLines
  // WARNING: to be correct, the points need to be in the LIDAR sensor
  // coordinates system, where the sensor is spinning around Z axis.
  void EstimateAzimuthalResolution();

  // Compute the curvature and other features within each the scan line.
  // The curvature is not the one of the surface that intersects the lines but
  // the 1D curvature within each isolated scan line.
  virtual void ComputeCurvature() = 0;

  // Labelize points (unvalid, edge, plane, blob)
  // and extract them in correspondant pointcloud
  virtual void ComputePlanes() = 0;
  virtual void ComputeEdges() = 0;
  virtual void ComputeIntensityEdges() = 0;
  void ComputeBlobs();

  // Add point to the keypoint pointcloud
  virtual void AddKeypoint(const Keypoint& k, const LidarPoint &pt) = 0;

  // ---------------------------------------------------------------------------
  //   Helpers
  // ---------------------------------------------------------------------------

  // Helpers for ComputeCurvature:
  // Check if a point is inside the range of distance to sensor
  inline bool CheckDistanceToSensor(float centralDepth) {return centralDepth >= this->MinDistanceToSensor && centralDepth < this->MaxDistanceToSensor;};
  // Check if the azimuth of a point is inside the range of azimuth angles
  bool CheckAzimuthAngle(const Eigen::Vector3f& centralPoint);
  // Compute the cos of the azimuth between the central point and the point
  inline float ComputeCosAngle(const Eigen::Vector3f& point, const Eigen::Vector3f& centralPt, float depth, float centralDepth)
  {
    return std::abs(point.dot(centralPt) / (depth * centralDepth));
  };
  // Compare the cos of the azimuth between the central point and the point to the max = azimuthal resolution
  inline bool IsAngleValid(float cosAngle) {return (cosAngle >= std::cos(1.5 * this->AzimuthalResolution));};
  // Compare the cos of the angle between the line and the central point to the threshold
  bool IsBeamAngleValid(const Eigen::Vector3f& centralPt, float centralDepth, const LineFitting& line);

  // ---------------------------------------------------------------------------
  //   Parameters
  // ---------------------------------------------------------------------------

  // Keypoints activated
  std::map<Keypoint, bool> Enabled = {{EDGE, true}, {INTENSITY_EDGE, true}, {PLANE, true}, {BLOB, false}};

  // Max number of threads to use to process points in parallel
  int NbThreads = 1;

  // Maximum number of keypoints to extract
  int MaxPoints = INT_MAX;

  // Sampling ratio to perform for real time issues
  float InputSamplingRatio = 1.;

  // Minimum number of points used on each side of the studied point to compute its curvature
  int MinNeighNb = 5;

  // Minimum radius to define the neighborhood to compute curvature of a studied point
  float MinNeighRadius = 0.10f;

  // Minimal point/sensor sensor to consider a point as valid
  float MinDistanceToSensor = 1.5;  // [m]

  // Maximal point/sensor sensor to consider a point as valid
  float MaxDistanceToSensor = 200.;  // [m]

  // Minimum angle between laser beam and surface to consider a point as valid
  float MinBeamSurfaceAngle = 10; // [°]

  float AzimuthMin = 0.f;        // [rad]
  float AzimuthMax = 2.f * M_PI; // [rad]

  // Threshold upon depth gap in neighborhood to select an edge keypoint
  float EdgeDepthGapThreshold = 0.5;  // [m]

  // Threshold upon intensity gap to select an edge keypoint
  float EdgeIntensityGapThreshold = 50.;

  // Nb of points missed to define a space gap
  int EdgeNbGapPoints = 3; // [nb]

  // Size of a voxel used to downsample the keypoints
  // It corresponds approx to the mean distance between closest neighbors in the output keypoints cloud.
  float VoxelResolution = 0.1f; // [m]

  // Angle thresholds to consider a point is a plane or an edge
  float EdgeAngleThreshold = 120.f;  // [°]
  float PlaneAngleThreshold = 150.f; // [°]

  // Log verbosity
  int Verbosity = 0;

  // ---------------------------------------------------------------------------
  //   Internal variables
  // ---------------------------------------------------------------------------

  // Azimuthal (= horizontal angle) resolution of the spinning lidar sensor
  // If it is less or equal to 0, it will be auto-estimated from next frame.
  // This angular resolution is used to compute an expected distance between two
  // consecutives firings.
  float AzimuthalResolution;

  // Number of lasers scan lines composing the pointcloud
  unsigned int NbLaserRings;

  // Current point cloud
  PointCloud::Ptr Scan;

};
} // end of namespace LidarSlam
