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

#include "LidarSlam/Utilities.h"
#include "LidarSlam/SpinningSensorKeypointExtractor.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <cmath>
#include <random>

namespace LidarSlam
{

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetEdgeAngleThreshold(float angle)
{
  this->EdgeAngleThreshold = angle;
  this->EdgeSinAngleThreshold = std::abs(std::sin(Utils::Deg2Rad(angle)));
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::SetPlaneAngleThreshold(float angle)
{
  this->PlaneAngleThreshold = angle;
  this->PlaneSinAngleThreshold = std::abs(std::sin(Utils::Deg2Rad(angle)));
}

//-----------------------------------------------------------------------------
SpinningSensorKeypointExtractor::PointCloud::Ptr SpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  if (!this->Enabled.count(k) || !this->Enabled[k])
  {
    PRINT_ERROR("Unable to get keypoints of type " << KeypointTypeNames.at(k));
    return PointCloud::Ptr();
  }

  PointCloud::Ptr keypoints = this->Keypoints.at(k).GetCloud(this->MaxPoints);
  Utils::CopyPointCloudMetadata(*this->Scan, *keypoints);
  return keypoints;
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::AddKeypoint(const Keypoint& k, const Point& pt)
{
  this->Keypoints[k].AddPoint(pt);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ConvertAndSortScanLines()
{
  // Clear previous scan lines
  for (auto& scanLineCloud: this->ScanLines)
  {
    // No worry as ScanLines is never shared with outer scope.
    scanLineCloud.second.reset(new PointCloud);
  }

  // Separate pointcloud into different scan lines
  for (const Point& point: *this->Scan)
  {
    auto it = this->ScanLines.find(point.laser_id);
    // Add laser line if it doesn't exist
    if (it == this->ScanLines.end())
      it = this->ScanLines.emplace(point.laser_id, std::make_unique<PointCloud>()).first;
    // Add the current point to its corresponding laser scan
    it->second->push_back(point);
  }

  // Save the number of lasers
  this->NbLaserRings = this->ScanLines.size();

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution || std::isnan(this->AzimuthalResolution))
    this->EstimateAzimuthalResolution();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  // Split whole pointcloud into separate laser ring clouds
  this->ConvertAndSortScanLines();

  // Initialize the features vectors and keypoints
  this->PrepareDataForNextFrame();

  // Compute keypoints scores
  this->ComputeCurvature();

  // Labelize and extract keypoints
  // Warning : order matters
  if (this->Enabled[Keypoint::BLOB])
    this->ComputeBlobs();
  if (this->Enabled[Keypoint::PLANE])
    this->ComputePlanes();
  if (this->Enabled[Keypoint::EDGE])
    this->ComputeEdges();
  if (this->Enabled[Keypoint::INTENSITY_EDGE])
    this->ComputeIntensityEdges();
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::PrepareDataForNextFrame()
{
  // Initialize the features vectors with the correct length
  this->Angles.resize(this->NbLaserRings);
  this->DepthGap.resize(this->NbLaserRings);
  this->SpaceGap.resize(this->NbLaserRings);
  this->IntensityGap.resize(this->NbLaserRings);
  this->Label.resize(this->NbLaserRings);

  // Initialize the scan lines features vectors with the correct length
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLineIdx = 0; scanLineIdx < static_cast<int>(this->NbLaserRings); ++scanLineIdx)
  {
    size_t nbPoint = this->GetScanlineCloud(scanLineIdx)->size();
    this->Label[scanLineIdx].assign(nbPoint, KeypointFlags().reset());  // set all flags to 0
    this->Angles[scanLineIdx].assign(nbPoint, -1.);
    this->DepthGap[scanLineIdx].assign(nbPoint, -1.);
    this->SpaceGap[scanLineIdx].assign(nbPoint, -1.);
    this->IntensityGap[scanLineIdx].assign(nbPoint, -1.);
  }

  // Reset voxel grids
  LidarPoint minPt, maxPt;
  pcl::getMinMax3D(*this->Scan, minPt, maxPt);

  // Clear keypoints
  this->Keypoints.clear();
  // Initialize keypoints
  for (auto k : KeypointTypes)
  {
    if (this->Enabled[k])
      this->Keypoints[k].Init(minPt.getVector3fMap(), maxPt.getVector3fMap(), this->VoxelResolution, this->Scan->size());
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeCurvature()
{
  // Compute useful const values to lighten the loop
  const float cosMinBeamSurfaceAngle = std::cos(Utils::Deg2Rad(this->MinBeamSurfaceAngle));
  const float cosMaxAzimuth = std::cos(1.5 * this->AzimuthalResolution);
  const float cosSpaceGapAngle = std::cos(this->EdgeNbGapPoints * this->AzimuthalResolution);

  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  // loop over scans lines
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int scanLine = 0; scanLine < static_cast<int>(this->NbLaserRings); ++scanLine)
  {
    // Useful shortcuts
    const PointCloud& scanLineCloud = *this->GetScanlineCloud(scanLine);
    const int nPts = scanLineCloud.size();

    // if the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(nPts))
      continue;

    // Loop over points in the current scan line
    for (int index = 0; index < nPts; ++index)
    {
      // Random sampling to decrease keypoints extraction
      // computation time
      if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
        continue;
      // Central point
      const Eigen::Vector3f& centralPoint = scanLineCloud[index].getVector3fMap();
      float centralDepth = centralPoint.norm();

      if (!this->CheckDistanceToSensor(centralDepth))
        continue;

      if (!this->CheckAzimuthAngle(centralPoint))
        continue;

      // Fill left and right neighbors
      // Those points must be more numerous than MinNeighNb and occupy more space than MinNeighRadius
      auto getNeighbors = [&](bool right, std::vector<int>& neighbors)
      {
        neighbors.reserve(nPts);
        // Check number criterion
        int step = right ? 1 : -1;
        int idxNeigh = index;
        while (int(neighbors.size()) < this->MinNeighNb &&
              neighbors.size() < nPts) // ensure enough points in the line
        {
          neighbors.emplace_back(idxNeigh);
          idxNeigh = (idxNeigh + step + nPts) % nPts;
        }
        // Check radius criterion
        float lineLength = ((scanLineCloud[neighbors.back()]).getVector3fMap() - (scanLineCloud[neighbors.front()]).getVector3fMap()).norm();
        while (lineLength < this->MinNeighRadius &&
              neighbors.size() < nPts) // ensure enough points in the line
        {
          idxNeigh = (idxNeigh + step + nPts) % nPts;
          neighbors.emplace_back(idxNeigh);
          lineLength = ((scanLineCloud[neighbors.back()]).getVector3fMap() - (scanLineCloud[neighbors.front()]).getVector3fMap()).norm();
        }
        neighbors.shrink_to_fit();
      };

      std::vector<int> leftNeighbors, rightNeighbors;
      getNeighbors(false, leftNeighbors);
      getNeighbors(true, rightNeighbors);

      const auto& rightPt = scanLineCloud[rightNeighbors[1]].getVector3fMap();
      const auto& leftPt = scanLineCloud[leftNeighbors[1]].getVector3fMap();

      const float rightDepth = rightPt.norm();
      const float leftDepth = leftPt.norm();

      const float cosAngleRight = this->ComputeCosAngle(rightPt, centralPoint, rightDepth, centralDepth);
      const float cosAngleLeft = this->ComputeCosAngle(leftPt, centralPoint, leftDepth, centralDepth);

      const Eigen::Vector3f& diffVecRight = rightPt - centralPoint;
      const Eigen::Vector3f& diffVecLeft = leftPt - centralPoint;

      const float diffRightNorm = diffVecRight.norm();
      const float diffLeftNorm = diffVecLeft.norm();

      float cosBeamLineAngleLeft = std::abs(diffVecLeft.dot(centralPoint) / (diffLeftNorm * centralDepth) );
      float cosBeamLineAngleRight = std::abs(diffVecRight.dot(centralPoint) / (diffRightNorm * centralDepth));

      if (this->Enabled[EDGE])
      {
        // Compute space gap

        // Init variables
        float distRight = -1.f;
        float distLeft = -1.f;

        // Compute space gap (if some neighbors were missed)
        if (cosBeamLineAngleRight < cosMinBeamSurfaceAngle && cosAngleRight < cosSpaceGapAngle)
          distRight = diffRightNorm;

        if (cosBeamLineAngleLeft < cosMinBeamSurfaceAngle && cosAngleLeft < cosSpaceGapAngle)
          distLeft = diffLeftNorm;

        this->SpaceGap[scanLine][index] = std::max(distLeft, distRight);
      }

      // Stop search for first and last points of the scan line
      // because the discontinuity may alter the other criteria detection
      if (index < int(leftNeighbors.size()) || index >= nPts - int(rightNeighbors.size()))
        continue;

      if (this->Enabled[EDGE])
      {
        // Compute depth gap

        // Init variables
        float distRight = -1.f;
        float distLeft = -1.f;

        if (cosAngleRight > cosMaxAzimuth)
          distRight = centralPoint.dot(diffVecRight) / centralDepth;
        if (cosAngleLeft > cosMaxAzimuth)
          distLeft = leftPt.dot(diffVecLeft) / leftDepth;

        // Check right points are consecutive + not on a bended wall
        // If the points lay on a bended wall, previous and next points should be in the same direction
        if (distRight > this->EdgeDepthGapThreshold)
        {
          auto nextdiffVecRight = (scanLineCloud[rightNeighbors[2]].getVector3fMap() - rightPt).normalized();
          if ((nextdiffVecRight.dot(diffVecRight) / diffRightNorm) > cosMinBeamSurfaceAngle ||
              (-diffVecLeft.dot(diffVecRight) / (diffRightNorm * diffLeftNorm)) > cosMinBeamSurfaceAngle)
            distRight = -1.f;
        }

        // Check left points are consecutive + not on a bended wall
        // If the points lay on a bended wall, previous and next points should be in the same direction
        if (distLeft > this->EdgeDepthGapThreshold)
        {
          auto prevdiffVecLeft = (scanLineCloud[leftNeighbors[2]].getVector3fMap() - leftPt).normalized();
          if ((prevdiffVecLeft.dot(diffVecLeft) / diffLeftNorm > cosMinBeamSurfaceAngle) ||
              (-diffVecRight.dot(diffVecLeft) / (diffRightNorm * diffLeftNorm)) > cosMinBeamSurfaceAngle)
            distLeft = -1.f;
        }

        this->DepthGap[scanLine][index] = std::max(distLeft, distRight);
      }

      if (!this->IsAngleValid(cosAngleRight) ||
          !this->IsAngleValid(cosAngleLeft))
        continue;

      // Fit line on the left and right neighborhoods and
      // skip point if they are not usable
      LineFitting leftLine, rightLine;
      if (!leftLine.FitLineAndCheckConsistency(scanLineCloud, leftNeighbors) ||
          !rightLine.FitLineAndCheckConsistency(scanLineCloud, rightNeighbors))
        continue;

      if (!this->IsBeamAngleValid(centralPoint, centralDepth, rightLine) ||
          !this->IsBeamAngleValid(centralPoint, centralDepth, leftLine))
        continue;

      if (this->Enabled[INTENSITY_EDGE])
      {
        // Compute intensity gap
        if (std::abs(scanLineCloud[rightNeighbors[1]].intensity - scanLineCloud[leftNeighbors[1]].intensity) > this->EdgeIntensityGapThreshold)
        {
          // Compute mean intensity on the left
          float meanIntensityLeft = 0.f;
          // We sample neighborhoods for computation time concerns
          int step = leftNeighbors.size() > this->MinNeighNb ? leftNeighbors.size() / this->MinNeighNb : 1;
          int cptMean = 0;
          // The first element of the neighborhood is the central point itself so we skip it
          for (int i = 1; i < leftNeighbors.size(); i += step)
          {
            meanIntensityLeft += scanLineCloud[leftNeighbors[i]].intensity;
            cptMean++;
          }
          meanIntensityLeft /= cptMean;
          // Compute mean intensity on the right
          float meanIntensityRight = 0.f;
          cptMean = 0;
          step = rightNeighbors.size() > this->MinNeighNb ? rightNeighbors.size() / this->MinNeighNb : 1;
          for (int i = 1; i < rightNeighbors.size(); i += step)
          {
            meanIntensityRight += scanLineCloud[rightNeighbors[i]].intensity;
            cptMean++;
          }
          meanIntensityRight /= cptMean;
          this->IntensityGap[scanLine][index] = std::abs(meanIntensityLeft - meanIntensityRight);

          // Remove neighbor points to get the best intensity discontinuity locally
          if (this->IntensityGap[scanLine][index-1] < this->IntensityGap[scanLine][index])
            this->IntensityGap[scanLine][index-1] = -1;
          else
            this->IntensityGap[scanLine][index] = -1;
        }
      }

      if (this->Enabled[PLANE] || this->Enabled[EDGE])
      {
        // Compute angles
        this->Angles[scanLine][index] = (leftLine.Direction.cross(rightLine.Direction)).norm();
        // Remove previous point from angle inspection if the angle is not maximal locally
        if (this->Enabled[EDGE] && this->Angles[scanLine][index] > this->EdgeSinAngleThreshold)
        {
          // Check previously computed angle to keep only the maximum angle keypoint locally
          // We avoid first point as it's the central point itself
          for (int i = 1; i < leftNeighbors.size(); i++)
          {
            if (this->Angles[scanLine][leftNeighbors[i]] <= this->Angles[scanLine][index])
              this->Angles[scanLine][leftNeighbors[i]] = -1;
            else
            {
              this->Angles[scanLine][index] = -1;
              break;
            }
          }
        }
      }
    } // Loop on points
  } // Loop on scanlines
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::AddKptsUsingCriterion (Keypoint k,
                                                             const std::vector<std::vector<float>>& values,
                                                             float threshold,
                                                             bool threshIsMax,
                                                             double weightBasis)
{
  // Loop over the scan lines
  for (unsigned int scanlineIdx = 0; scanlineIdx < this->NbLaserRings; ++scanlineIdx)
  {
    const PointCloud& scanlineCloud = *this->GetScanlineCloud(scanlineIdx);
    const int nPts = scanlineCloud.size();

    // If the line is almost empty, skip it
    if (this->IsScanLineAlmostEmpty(nPts))
      continue;

    // Initialize original index locations
    // Remove indices corresponding to negative values
    // and indices corresponding to values beside threshold
    std::vector<size_t> sortedValuesIndices;
    sortedValuesIndices.reserve(values[scanlineIdx].size());
    for (int idx = 0; idx < values[scanlineIdx].size(); ++idx)
    {
      if (values[scanlineIdx][idx] > 0 &&
          (threshIsMax  && values[scanlineIdx][idx] < threshold ||
           !threshIsMax && values[scanlineIdx][idx] > threshold))
        sortedValuesIndices.emplace_back(idx);
    }

    // If threshIsMax : ascending order (lowest first)
    // If threshIsMin : descending order (greatest first)
    Utils::SortIdx(values[scanlineIdx], sortedValuesIndices, threshIsMax, this->MaxPoints);

    for (const auto& index: sortedValuesIndices)
    {
      // If the point was already picked, continue
      if (this->Label[scanlineIdx][index][k])
        continue;

      // Check criterion threshold
      bool valueAboveThresh = values[scanlineIdx][index] > threshold;
      // If criterion is not respected, break loop as indices are sorted.
      if ((threshIsMax && valueAboveThresh) ||
          (!threshIsMax && !valueAboveThresh))
        break;

      // The points with the lowest weight have priority for extraction
      float weight = threshIsMax? weightBasis + values[scanlineIdx][index] / threshold
                                : weightBasis - values[scanlineIdx][index] / values[scanlineIdx][0];

      // Indicate the type of the keypoint to debug and to exclude double edges
      this->Label[scanlineIdx][index].set(k);
      // Add keypoint
      this->Keypoints[k].AddPoint(scanlineCloud.at(index), weight);
    }
  }
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputePlanes()
{
  this->AddKptsUsingCriterion(Keypoint::PLANE, this->Angles, this->PlaneSinAngleThreshold);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeEdges()
{
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->DepthGap, this->EdgeDepthGapThreshold, false, 1);
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->Angles, this->EdgeSinAngleThreshold, false, 2);
  this->AddKptsUsingCriterion(Keypoint::EDGE, this->SpaceGap, this->EdgeDepthGapThreshold, false, 3);
}

//-----------------------------------------------------------------------------
void SpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  this->AddKptsUsingCriterion(Keypoint::INTENSITY_EDGE, this->IntensityGap, this->EdgeIntensityGapThreshold, false);
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> SpinningSensorKeypointExtractor::GetDebugArray() const
{
  auto get1DVector = [this](auto const& vector2d)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      int scanlineIdx = std::distance(this->ScanLines.begin(), this->ScanLines.find(laserId));
      v[i] = vector2d[scanlineIdx][indexByScanLine[laserId]];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  auto get1DVectorFromFlag = [this](auto const& vector2d, int flag)
  {
    std::vector<float> v(this->Scan->size());
    std::vector<int> indexByScanLine(this->NbLaserRings, 0);
    for (unsigned int i = 0; i < this->Scan->size(); i++)
    {
      const auto& laserId = this->Scan->points[i].laser_id;
      int scanlineIdx = std::distance(this->ScanLines.begin(), this->ScanLines.find(laserId));
      v[i] = vector2d[scanlineIdx][indexByScanLine[laserId]][flag];
      indexByScanLine[laserId]++;
    }
    return v;
  }; // end of lambda expression

  std::unordered_map<std::string, std::vector<float>> map;
  map["sin_angle"]      = get1DVector(this->Angles);
  map["depth_gap"]      = get1DVector(this->DepthGap);
  map["space_gap"]      = get1DVector(this->SpaceGap);
  map["intensity_gap"]  = get1DVector(this->IntensityGap);
  map["edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::EDGE);
  map["intensity_edge_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::INTENSITY_EDGE);
  map["plane_keypoint"] = get1DVectorFromFlag(this->Label, Keypoint::PLANE);
  map["blob_keypoint"]  = get1DVectorFromFlag(this->Label, Keypoint::BLOB);
  return map;
}

//-----------------------------------------------------------------------------
std::shared_ptr<KeypointExtractor> SpinningSensorKeypointExtractor::Clone() const
{
  return std::make_shared<SpinningSensorKeypointExtractor>(*this);
}

//-----------------------------------------------------------------------------
pcl::PointCloud<LidarPoint>::Ptr SpinningSensorKeypointExtractor::GetScanlineCloud(unsigned int i)
{
  if (i >= this->ScanLines.size())
    return nullptr;
  return std::next(this->ScanLines.begin(), i)->second;
}

} // end of LidarSlam namespace