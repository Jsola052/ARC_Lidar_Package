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

#include "LidarSlam/DenseSpinningSensorKeypointExtractor.h"
#include "LidarSlam/Utilities.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>

#include <cmath>
#include <fstream>
#include <random>

namespace LidarSlam
{

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::SetEdgeAngleThreshold(float angle)
{
  this->EdgeAngleThreshold = angle;
  this->EdgeCosAngleThreshold = std::cos(Utils::Deg2Rad(angle));
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::SetPlaneAngleThreshold(float angle)
{
  this->PlaneAngleThreshold = angle;
  this->PlaneCosAngleThreshold = std::cos(Utils::Deg2Rad(angle));
}

//-----------------------------------------------------------------------------
DenseSpinningSensorKeypointExtractor::PointCloud::Ptr DenseSpinningSensorKeypointExtractor::GetKeypoints(Keypoint k)
{
  if (!this->Enabled.count(k) || !this->Enabled[k])
  {
    PRINT_ERROR("Unable to get keypoints of type " << KeypointTypeNames.at(k));
    return PointCloud::Ptr();
  }

  PointCloud::Ptr keypointsCloud(new PointCloud);
  Utils::CopyPointCloudMetadata(*this->Scan, *keypointsCloud);

  std::vector<LidarPoint> points = this->Keypoints.at(k);
  keypointsCloud->reserve(points.size());
  for (const auto& pt : points)
    keypointsCloud->push_back(pt);

  return keypointsCloud;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKeypoint(const Keypoint& k, const Point& pt)
{
  this->Keypoints.at(k).push_back(pt);
}

//-----------------------------------------------------------------------------
std::shared_ptr<PtFeat> DenseSpinningSensorKeypointExtractor::GetPtFeat(int idxInScan) const
{
  auto& indices = this->Pc2VmIndices[idxInScan];
  return this->VertexMap[indices.Row][indices.Col];
}

//-----------------------------------------------------------------------------
int DenseSpinningSensorKeypointExtractor::GetScanLineSize(const std::vector<std::shared_ptr<PtFeat>>& scanLine)
{
  int nbPoints;
  for (auto& point : scanLine)
  {
    if (point != nullptr)
      nbPoints++;
  }
  return nbPoints;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::InitInternalParameters()
{
  // Map laser_ids
  for (const auto& point : *this->Scan)
  {
    if (this->LaserIdMap.count(point.laser_id) == 0)
      this->LaserIdMap[point.laser_id] = this->LaserIdMap.size();
  }

  // Save the number of lasers
  this->NbLaserRings = this->LaserIdMap.size();

  bool rotationIsClockwise = Utils::IsRotationClockwise<Point>(*this->Scan, this->NbLaserRings);

  // Estimate azimuthal resolution if not already done
  // or if the previous value found is not plausible
  // (because last scan was badly formed, e.g. lack of points)
  if (this->AzimuthalResolution < 1e-6 || M_PI/4. < this->AzimuthalResolution || std::isnan(this->AzimuthalResolution))
    this->EstimateAzimuthalResolution();

  // Compute the indices of scan points in the future vertex map
  this->Pc2VmIndices.clear(); // Clear previous indices
  this->Pc2VmIndices.reserve(this->Scan->size());
  Eigen::Vector3f firstPt(this->Scan->at(0).data);
  for (const auto& point : *this->Scan)
  {
    double cross = firstPt.x() * point.y - firstPt.y() * point.x;
    double angle = std::atan2(cross, firstPt.head(2).dot(point.getVector3fMap().head(2)));
    float azimuth = rotationIsClockwise ? M_PI + angle : M_PI - angle;
    this->Pc2VmIndices.emplace_back(IdxVM{this->LaserIdMap.find(point.laser_id)->second,
                                          static_cast<int>(azimuth / this->AzimuthalResolution)});
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::CreateVertexMap()
{
  // Clear previous Vertex Map
  for (auto& row : this->VertexMap)
  {
    for (auto& ptr : row)
      ptr.reset(); // ptr = nullptr
  }

  this->WidthVM = 2.f * M_PI / this->AzimuthalResolution + 1;
  this->HeightVM = this->NbLaserRings;

  this->VertexMap.resize(this->HeightVM, std::vector<std::shared_ptr<PtFeat>>(this->WidthVM));
  // Fill Vertex Map with index and depth values
  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int i = 0; i < this->Scan->size(); ++i)
  {
    const Point& point = this->Scan->at(i);
    auto& indices = this->Pc2VmIndices[i];

    this->VertexMap[indices.Row][indices.Col] = std::make_shared<PtFeat>();
    this->VertexMap[indices.Row][indices.Col]->Index = i;
    this->VertexMap[indices.Row][indices.Col]->Depth = point.getVector3fMap().norm();
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ClearKeypoints()
{
  // Clear keypoints
  this->Keypoints.clear();
  // Initialize keypoints
  for (const auto& k : KeypointTypes)
  {
    if (this->Enabled[k])
      this->Keypoints[k].reserve(this->Scan->size());
  }
}

//-----------------------------------------------------------------------------
#define OUTPUT_FEATURE(filename, featName, maxValue, specialValue, minValue)                                      \
{                                                                                                                 \
  int maxPix = 255;                                                                                               \
  int specialPix = 30;                                                                                            \
  std::ofstream file(filename);                                                                                   \
  file << "P2\n";                                                                                                 \
  file << this->WidthVM << " " << this->HeightVM << "\n";                                                         \
  file << maxPix << "\n";                                                                                         \
  for (int i = 0; i < this->HeightVM; ++i)                                                                        \
  {                                                                                                               \
    for (int j = 0; j < this->WidthVM; ++j)                                                                       \
    {                                                                                                             \
      if (this->VertexMap[i][j] == nullptr)                                                                       \
        file << 0 << " ";                                                                                         \
      else                                                                                                        \
      {                                                                                                           \
        float value = this->VertexMap[i][j]->featName;                                                            \
        if (std::abs(value - specialValue) < 1e-6)                                                                \
          file << specialPix << " ";                                                                              \
        else                                                                                                      \
        {                                                                                                         \
          value = std::min(static_cast<int>(value), static_cast<int>(maxValue));                                  \
          value = std::max(static_cast<int>(value), static_cast<int>(minValue));                                  \
          file << int((value - maxValue) * (maxPix - (specialPix + 1)) / (maxValue - minValue)) + maxPix << " ";  \
        }                                                                                                         \
      }                                                                                                           \
    }                                                                                                             \
    file << "\n";                                                                                                 \
  }                                                                                                               \
  file.close();                                                                                                   \
}                                                                                                                 \

void DenseSpinningSensorKeypointExtractor::OutputFeatures(std::string path)
{
  int totalSize = this->WidthVM * this->HeightVM;
  OUTPUT_FEATURE(path + "Index.pgm", Index, totalSize, 0., 0.);
  OUTPUT_FEATURE(path + "Depth.pgm", Depth, 20., 0., 0.);
  OUTPUT_FEATURE(path + "SpaceGap.pgm", SpaceGap, 1., -1., 0.);
  OUTPUT_FEATURE(path + "DepthGap.pgm", DepthGap, 10., 0., -15.);
  OUTPUT_FEATURE(path + "IntensityGap.pgm", IntensityGap, 35., -1., 0.);
  OUTPUT_FEATURE(path + "Angles.pgm", Angle, 1., 1., -1.);
}

//-----------------------------------------------------------------------------
bool DenseSpinningSensorKeypointExtractor::OutputKeypoints(std::string path)
{
  for (const auto& type : this->Keypoints)
  {
    const Keypoint& k = type.first;
    // In case a keypoint type has been disabled during the process but not changed in this->Keypoints
    if (!this->Enabled[k])
      continue;

    std::ofstream fileCsv(path + "keypoints_" + KeypointTypeNames.at(k) + ".csv");
    fileCsv << "x,y,z,kpt\n";

    std::ofstream filePgm(path + "keypoints_" + KeypointTypeNames.at(k) + ".pgm");
    filePgm << "P2\n";
    filePgm << this->WidthVM << " " << this->HeightVM << "\n";
    filePgm << 1 << "\n";
    for (int i = 0; i < this->HeightVM; ++i)
    {
      for (int j = 0; j < this->WidthVM; ++j)
      {
        const auto& ptFeat = this->VertexMap[i][j];
        if (ptFeat == nullptr)
          continue;
        const Point& point = this->Scan->at(ptFeat->Index);
        fileCsv << point.x << "," << point.y << "," << point.z << "," << ptFeat->KptTypes[static_cast<int>(k)] << "\n";
        filePgm << ptFeat->KptTypes[static_cast<int>(k)] << " ";
      }
      filePgm << "\n";
    }
    fileCsv.close();
    filePgm.close();
  }

  return true;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeKeyPoints(const PointCloud::Ptr& pc)
{
  this->Scan = pc;

  this->InitInternalParameters();

  this->ClearKeypoints();

  this->CreateVertexMap();

  this->ComputeCurvature();

  // Label and extract keypoints
  //! Warning : order matters for computation time concerns
  if (this->Enabled[Keypoint::PLANE])
    this->ComputePlanes();
  if (this->Enabled[Keypoint::EDGE])
    this->ComputeEdges();
  if (this->Enabled[Keypoint::INTENSITY_EDGE])
    this->ComputeIntensityEdges();
  if (this->Enabled[Keypoint::BLOB])
    this->ComputeBlobs();

  this->ClearGrid();
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeCurvature()
{
  // Init random distribution
  std::mt19937 gen(2023); // Fix seed for deterministic processes
  std::uniform_real_distribution<> dis(0.0, 1.0);

  #pragma omp parallel for num_threads(this->NbThreads) schedule(guided)
  for (int i = 0; i < this->HeightVM; ++i)
  {
    // If the line is almost empty, skip it
    int nPoints = this->GetScanLineSize(this->VertexMap[i]);
    if (this->IsScanLineAlmostEmpty(nPoints))
      continue;

    // Useful index to skip first and last points of the scan line
    int idxInLine = 0;

    for (unsigned int j = 0; j < this->WidthVM; ++j)
    {
      // PtFeat struct associated with current point
      const auto& currentFeat = this->VertexMap[i][j];

      // Ignore empty pixels
      if (!currentFeat)
        continue;

      // Count every valid point in the scan line, to be compared with nPoints later
      idxInLine++;

      // Random sampling to decrease keypoints extraction
      // computation time
      if (this->InputSamplingRatio < 1.f && dis(gen) > this->InputSamplingRatio)
        continue;

      // Central point
      const Point& currentPoint = this->Scan->at(currentFeat->Index);
      const Eigen::Vector3f& centralPoint = currentPoint.getVector3fMap();

      if (!this->CheckDistanceToSensor(currentFeat->Depth))
        continue;

      if (!this->CheckAzimuthAngle(centralPoint))
        continue;

      // Fill neighbors (vectors of indices) for each side (left and right)
      // Those points must be more numerous than MinNeighNb and occupy more space than MinNeighRadius
      auto getNeighbors = [&](bool right, std::vector<int>& neighbors)
      {
        neighbors.reserve(nPoints);
        neighbors.emplace_back(currentFeat->Index);
        int rightOrLeft = right ? 1 : -1;
        int idxNeigh = 1;
        float lineLength = 0.f;
        while ((lineLength < this->MinNeighRadius ||
               int(neighbors.size()) < this->MinNeighNb) &&
               int(neighbors.size()) < nPoints)
        {
          const auto& ptrFeat = this->VertexMap[i][(j + rightOrLeft * idxNeigh + this->WidthVM) % this->WidthVM];
          if (ptrFeat != nullptr)
          {
            neighbors.emplace_back(ptrFeat->Index);
            if (lineLength < MinNeighRadius)
              lineLength = (this->Scan->at(neighbors.back()).getVector3fMap() - this->Scan->at(neighbors.front()).getVector3fMap()).norm();
          }
          ++idxNeigh;
        }
        neighbors.shrink_to_fit();
      };

      std::vector<int> leftNeighbors, rightNeighbors;
      getNeighbors(false, leftNeighbors);
      getNeighbors(true, rightNeighbors);

      if (this->Enabled[EDGE])
      {
        // ---- Compute horizontal space gap ----

        // Find the first empty neighbor on the right and on the left
        auto& idxRightNeigh = this->Pc2VmIndices[rightNeighbors[1]];
        int nbEmptyRightNeigh = idxRightNeigh.Col - j;

        auto& idxLeftNeigh = this->Pc2VmIndices[leftNeighbors[1]];
        int nbEmptyLeftNeigh = j - idxLeftNeigh.Col;

        const float cosMinBeamSurfaceAngle = std::cos(Utils::Deg2Rad(this->MinBeamSurfaceAngle));

        float distRight = -1.f;
        float distLeft = -1.f;

        if (nbEmptyRightNeigh >= this->EdgeNbGapPoints)
        {
          const auto& rightPt = this->Scan->at(rightNeighbors[1]).getVector3fMap();
          float diffRightNorm = (rightPt - centralPoint).norm();
          float cosBeamLineAngleRight = std::abs((rightPt - centralPoint).dot(centralPoint) / (diffRightNorm * currentFeat->Depth));
          if (cosBeamLineAngleRight < cosMinBeamSurfaceAngle)
            distRight = diffRightNorm;
        }
        if (nbEmptyLeftNeigh >= this->EdgeNbGapPoints)
        {
          const auto& leftPt = this->Scan->at(leftNeighbors[1]).getVector3fMap();
          float diffLeftNorm = (leftPt - centralPoint).norm();
          float cosBeamLineAngleLeft = std::abs((leftPt - centralPoint).dot(centralPoint) / (diffLeftNorm * currentFeat->Depth));
          if (cosBeamLineAngleLeft < cosMinBeamSurfaceAngle)
            distLeft = diffLeftNorm;
        }
        currentFeat->SpaceGap = std::max(distLeft, distRight);
      }

      // Stop search for first and last points of the scan line
      // because the discontinuity may alter the other criteria detection
      if (idxInLine < int(leftNeighbors.size()) || idxInLine >= nPoints - int(rightNeighbors.size()))
        continue;

      if (this->Enabled[EDGE])
      {
        // ---- Compute horizontal depth gap ----

        int idxNext = this->VertexMap[i][j + 1] ? j + 1 : this->VertexMap[i][j + 2] ? j + 2 : -1;
        int idxPrev = this->VertexMap[i][j - 1] ? j - 1 : this->VertexMap[i][j - 2] ? j - 2 : -1;
        if (idxNext > 0 && idxPrev > 0)
        {
          auto& directRightNeigh = this->VertexMap[i][idxNext];
          float distRight = directRightNeigh->Depth - currentFeat->Depth;
          auto& directLeftNeigh = this->VertexMap[i][idxPrev];
          float distLeft = directLeftNeigh->Depth - currentFeat->Depth;

          auto& postRightNeigh = this->VertexMap[i][idxNext + 1];
          auto& preLeftNeigh = this->VertexMap[i][idxPrev - 1];
          if (postRightNeigh != nullptr)
          {
            float distPostRight = postRightNeigh->Depth - directRightNeigh->Depth;
            if (distRight < distPostRight)
              distRight = 0.0f;
          }
          if (preLeftNeigh != nullptr)
          {
            float distPreLeft = preLeftNeigh->Depth - directLeftNeigh->Depth;
            if (distLeft < distPreLeft)
              distLeft = 0.0f;
          }
          currentFeat->DepthGap = std::abs(distLeft) > std::abs(distRight) ? distLeft : distRight;
        }
      }

      if (currentFeat->SpaceGap > this->EdgeDepthGapThreshold || currentFeat->DepthGap > this->EdgeDepthGapThreshold)
        continue;

      // ---- Compute intensity gap ----

      LineFitting leftLine, rightLine;
      // Fit line on the left and right neighborhoods and
      // skip point if they are not usable
      if (!leftLine.FitLineAndCheckConsistency(*this->Scan, leftNeighbors) ||
          !rightLine.FitLineAndCheckConsistency(*this->Scan, rightNeighbors))
        continue;

      if (!this->IsBeamAngleValid(centralPoint, currentFeat->Depth, rightLine) ||
          !this->IsBeamAngleValid(centralPoint, currentFeat->Depth, leftLine))
        continue;

      if (this->Enabled[INTENSITY_EDGE])
      {
        if (std::abs(this->Scan->at(rightNeighbors[1]).intensity - this->Scan->at(leftNeighbors[1]).intensity) > this->EdgeIntensityGapThreshold)
        {
          // Compute mean intensity on the left
          // We sample neighborhoods for computation time concerns
          float meanIntensityLeft = 0.f;
          int step = leftNeighbors.size() > this->MinNeighNb ? leftNeighbors.size() / this->MinNeighNb : 1;
          int cptMean = 0;
          // The first element of the neighborhood is the central point itself so we skip it
          for (int i = 1; i < leftNeighbors.size(); i += step)
          {
            meanIntensityLeft += this->Scan->at(leftNeighbors[i]).intensity;
            cptMean++;
          }
          meanIntensityLeft /= cptMean;
          // Compute mean intensity on the right
          float meanIntensityRight = 0.f;
          step = rightNeighbors.size() > this->MinNeighNb ? rightNeighbors.size() / this->MinNeighNb : 1;
          cptMean = 0;
          for (int i = 1; i < rightNeighbors.size(); i += step)
          {
            meanIntensityRight += this->Scan->at(rightNeighbors[i]).intensity;
            cptMean++;
          }
          meanIntensityRight /= cptMean;
          currentFeat->IntensityGap = std::abs(meanIntensityLeft - meanIntensityRight);

          // Remove neighbor points to get the best intensity discontinuity locally
          auto neighPtr = this->GetPtFeat(leftNeighbors[1]);
          if (neighPtr->IntensityGap < currentFeat->IntensityGap)
            neighPtr->IntensityGap = -1;
          else
            currentFeat->IntensityGap = -1;
        }
      }

      // ---- Compute angle ----

      if (this->Enabled[PLANE] || this->Enabled[EDGE])
      {
        // Compute angles
        currentFeat->Angle = leftLine.Direction.dot(rightLine.Direction);
        // Remove angles too small to be edges
        if (currentFeat->Angle > -this->PlaneCosAngleThreshold)
        {
          currentFeat->Angle = 1.f;
          continue;
        }

        // Remove previous point from angle inspection if the angle is not maximal locally
        if (this->Enabled[EDGE] && currentFeat->Angle > this->EdgeCosAngleThreshold)
        {
          // Check previously computed angle to keep only the maximal angle keypoint locally
          for (int idx = 1; idx < leftNeighbors.size(); idx++)
          {
            const std::shared_ptr<PtFeat>& neighPtr = this->GetPtFeat(leftNeighbors[idx]);
            if (neighPtr->Angle > this->EdgeCosAngleThreshold &&
                neighPtr->Angle < -this->PlaneCosAngleThreshold)
            {
              if (neighPtr->Angle > currentFeat->Angle)
                currentFeat->Angle = 1.f;
              else
                neighPtr->Angle = 1.f;
              break;
            }
          }
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::Create2DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid)
{
  this->ClearGrid();
  int nbPatchesX = std::ceil(this->WidthVM / this->PatchSize);
  int nbPatchesY = std::ceil(this->HeightVM / this->PatchSize);
  this->Grid.reserve(nbPatchesX * nbPatchesY);

  // Fill grid with smart pointers to PtFeat
  for (unsigned int i = 0; i < this->HeightVM; ++i)
  {
    for (unsigned int j = 0; j < this->WidthVM; ++j)
    {
      const auto& ptFeat = this->VertexMap[i][j];
      if (!isPtFeatValid(ptFeat))
        continue;
      int idxPatchY = ((i + this->HeightVM) % this->HeightVM) / this->PatchSize;
      int idxPatchX = ((j + this->WidthVM)  % this->WidthVM)  / this->PatchSize;
      int indexPatch = idxPatchY * nbPatchesX + idxPatchX;
      this->Grid[indexPatch].emplace_back(ptFeat);
      this->NbPointsInGrid++;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::Create3DGrid(std::function<bool(const std::shared_ptr<PtFeat>&)> isPtFeatValid)
{
  this->ClearGrid();
  // Find width and height of the scan (in meters)
  Eigen::Vector4f ptMin, ptMax;
  pcl::getMinMax3D(*this->Scan, ptMin, ptMax);
  int xSize = std::abs(ptMax.x() - ptMin.x());
  int ySize = std::abs(ptMax.y() - ptMin.y());
  int zSize = std::abs(ptMax.z() - ptMin.z());

  // Compute the number of voxels in each direction
  int nbVoxelsX = std::ceil(xSize / this->VoxelResolution);
  int nbVoxelsY = std::ceil(ySize / this->VoxelResolution);
  int nbVoxelsZ = std::ceil(zSize / this->VoxelResolution);
  // Reserve memory for the grid
  this->Grid.reserve(std::min(nbVoxelsX * nbVoxelsY * nbVoxelsZ, static_cast<int>(this->Scan->size())));

  // Fill grid with smart pointers to PtFeat
  for (unsigned int i = 0; i < this->Scan->size(); i++)
  {
    const LidarPoint& point = this->Scan->at(i);
    auto ptFeat = this->GetPtFeat(i);
    if (!isPtFeatValid(ptFeat))
      continue;
    Eigen::Vector3i voxel = ((point.getVector3fMap() - ptMin.head(3)) / this->VoxelResolution).cast<int>();
    int index1D = voxel.z() * nbVoxelsX * nbVoxelsY + voxel.y() * nbVoxelsX + voxel.x();
    this->Grid[index1D].push_back(ptFeat);
    this->NbPointsInGrid++;
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ClearGrid()
{
  this->Grid.clear();
  this->NbPointsInGrid = 0;
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::AddKptsUsingGrid(Keypoint k,
                                                            std::function<bool(const std::shared_ptr<PtFeat>&,
                                                                               const std::shared_ptr<PtFeat>&)> comparePtFeat)
{
  // If we have less candidates than the max keypoints number
  if (this->NbPointsInGrid < this->MaxPoints)
  {
    for (auto& cell : this->Grid)
    {
      auto& vec = cell.second;
      for (auto& pt : vec)
      {
        this->AddKeypoint(k, this->Scan->at(pt->Index));
        pt->KptTypes.set(static_cast<int>(k));
      }
    }
    return;
  }

  // While nbkeypointextracted < nbpointmax && remains keypoints candidates
  int ptIdx = 0;
  while (ptIdx < this->MaxPoints)
  {
    bool remainKptCandidate = false;
    for (auto& cell : this->Grid)
    {
      auto& vec = cell.second;

      // Check if cell has point that could be keypoints.
      // Reminder : cells only contain valid points, so here we only check for points that have not been extracted as keypoints for this type
      bool hasKptCandidate = std::any_of(vec.begin(), vec.end(), [&](const std::shared_ptr<PtFeat>& pt) {return !pt->KptTypes[static_cast<int>(k)];});
      if (!hasKptCandidate)
        continue;
      remainKptCandidate = true;

      // Search for max element in patch using comparePtFeat to compare all feats simultaneously
      auto maxPtIt = std::max_element(vec.begin(), vec.end(), comparePtFeat);
      const auto& maxPt = *maxPtIt;

      // Add to keypoints
      this->AddKeypoint(k, this->Scan->at(maxPt->Index));
      maxPt->KptTypes.set(static_cast<int>(k));
      ++ptIdx;

      if (ptIdx >= this->MaxPoints)
        break;
    }
    if (!remainKptCandidate)
      break;
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputePlanes()
{
  auto isPtValid = [this](const std::shared_ptr<PtFeat>& pt)
  {
    return pt != nullptr &&
           (!pt->KptTypes[static_cast<int>(PLANE)] && !pt->KptTypes[static_cast<int>(EDGE)]) &&
           pt->Angle < this->PlaneCosAngleThreshold;
  };
  switch (this->SamplingDSSKE)
  {
    // If Patch mode activated : use a 2D grid built on vertex map
    // to extract keypoints
    case SamplingModeDSSKE::PATCH:
    {
      this->Create2DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::PLANE,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               return (a->Angle > b->Angle);
                             });
      break;
    }
    // If Voxel mode activated : use a 3D grid built on scan cloud
    // to extract keypoints
    case SamplingModeDSSKE::VOXEL:
    {
      this->Create3DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::PLANE,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               return (a->Angle > b->Angle);
                             });
      break;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeEdges()
{
  auto isPtValid = [this](const std::shared_ptr<PtFeat>& pt)
  {
    return pt != nullptr &&
           (!pt->KptTypes[static_cast<int>(PLANE)] && !pt->KptTypes[static_cast<int>(EDGE)])&&
           ((pt->DepthGap - (-1.0f) > 1e-6 && pt->DepthGap > this->EdgeDepthGapThreshold) ||
            (pt->Angle < -this->PlaneCosAngleThreshold && pt->Angle > this->EdgeCosAngleThreshold) ||
            (pt->SpaceGap - (-1.0f) > 1e-6 && pt->SpaceGap > this->EdgeDepthGapThreshold));
  };
  switch (this->SamplingDSSKE)
  {
    // If Patch mode activated : use a 2D grid built on vertex map
    // to extract keypoints
    case SamplingModeDSSKE::PATCH:
    {
      this->Create2DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::EDGE,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->DepthGap < b->DepthGap && b->DepthGap > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->DepthGap < a->DepthGap && a->DepthGap > this->EdgeDepthGapThreshold)
                                 return false;

                               if (a->Angle < b->Angle && b->Angle < -this->PlaneCosAngleThreshold && b->Angle > this->EdgeCosAngleThreshold)
                                 return true;
                               if (b->Angle < a->Angle && a->Angle < -this->PlaneCosAngleThreshold && a->Angle > this->EdgeCosAngleThreshold)
                                 return false;

                               if (a->SpaceGap < b->SpaceGap && b->SpaceGap > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->SpaceGap < a->SpaceGap && a->SpaceGap > this->EdgeDepthGapThreshold)
                                 return false;

                               return true;
                              });
      break;
    }
    // If Voxel mode activated : use a 3D grid built on scan cloud
    // to extract keypoints
    case SamplingModeDSSKE::VOXEL:
    {
      this->Create3DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::EDGE,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->DepthGap < b->DepthGap && b->DepthGap > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->DepthGap < a->DepthGap && a->DepthGap > this->EdgeDepthGapThreshold)
                                 return false;

                               if (a->Angle < b->Angle && b->Angle < -this->PlaneCosAngleThreshold && b->Angle > this->EdgeCosAngleThreshold)
                                  return true;
                                if (b->Angle < a->Angle && a->Angle < -this->PlaneCosAngleThreshold && a->Angle > this->EdgeCosAngleThreshold)
                                  return false;

                               if (a->SpaceGap < b->SpaceGap && b->SpaceGap > this->EdgeDepthGapThreshold)
                                 return true;
                               if (b->SpaceGap < a->SpaceGap && a->SpaceGap > this->EdgeDepthGapThreshold)
                                 return false;

                               return true;
                             });
      break;
    }
  }
}

//-----------------------------------------------------------------------------
void DenseSpinningSensorKeypointExtractor::ComputeIntensityEdges()
{
  auto isPtValid = [this](const std::shared_ptr<PtFeat>& pt)
  {
    return pt != nullptr &&
           (!pt->KptTypes[static_cast<int>(EDGE)] && !pt->KptTypes[static_cast<int>(INTENSITY_EDGE)]) &&
           (pt->IntensityGap - (-1.0f) > 1e-6 && pt->IntensityGap > this->EdgeIntensityGapThreshold);
  };
  switch (this->SamplingDSSKE)
  {
    // If Patch mode activated : use a 2D grid built on vertex map
    // to extract keypoints
    case SamplingModeDSSKE::PATCH:
    {
      this->Create2DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::INTENSITY_EDGE,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->IntensityGap < b->IntensityGap && b->IntensityGap > this->EdgeIntensityGapThreshold)
                                 return true;
                               if (b->IntensityGap < a->IntensityGap && a->IntensityGap > this->EdgeIntensityGapThreshold)
                                 return false;

                               return true;
                             });
      break;
    }
    // If Voxel mode activated : use a 3D grid built on scan cloud
    // to extract keypoints
    case SamplingModeDSSKE::VOXEL:
    {
      this->Create3DGrid(isPtValid);
      this->AddKptsUsingGrid(Keypoint::INTENSITY_EDGE,
                             [&](const std::shared_ptr<PtFeat>& a, const std::shared_ptr<PtFeat>& b)
                             {
                               if (!isPtValid(a) && isPtValid(b))
                                 return true;  // b is considered greater when a is nullptr
                               else if (isPtValid(a) && !isPtValid(b))
                                 return false;   // a is considered greater when b is nullptr
                               else if (!isPtValid(a) && !isPtValid(b))
                                 return true;  // Both are nullptr, no preference

                               if (a->IntensityGap < b->IntensityGap && b->IntensityGap > this->EdgeIntensityGapThreshold)
                                 return true;
                               if (b->IntensityGap < a->IntensityGap && a->IntensityGap > this->EdgeIntensityGapThreshold)
                                 return false;

                               return true;
                             });
      break;
    }
  }
}

//-----------------------------------------------------------------------------
std::unordered_map<std::string, std::vector<float>> DenseSpinningSensorKeypointExtractor::GetDebugArray() const
{
  std::unordered_map<std::string, std::vector<float>> debugArray;
  debugArray["space_gap"]               = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["depth_gap"]               = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["intensity_gap"]           = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["cos_angle"]               = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["edge_keypoint"]           = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["plane_keypoint"]          = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["intensity_edge_keypoint"] = std::vector<float>(this->Scan->size(), 0.f);
  debugArray["blob_keypoint"]           = std::vector<float>(this->Scan->size(), 0.f);

  for (int i = 0; i < this->Scan->size(); i++)
  {
    const std::shared_ptr<PtFeat>& ptrFeat = this->GetPtFeat(i);
    debugArray["space_gap"][i]               = ptrFeat->SpaceGap;
    debugArray["depth_gap"][i]               = ptrFeat->DepthGap;
    debugArray["intensity_gap"][i]           = ptrFeat->IntensityGap;
    debugArray["cos_angle"][i]               = ptrFeat->Angle;
    debugArray["edge_keypoint"][i]           = ptrFeat->KptTypes[static_cast<int>(Keypoint::EDGE)];
    debugArray["plane_keypoint"][i]          = ptrFeat->KptTypes[static_cast<int>(Keypoint::PLANE)];
    debugArray["intensity_edge_keypoint"][i] = ptrFeat->KptTypes[static_cast<int>(Keypoint::INTENSITY_EDGE)];
    debugArray["blob_keypoint"][i]           = ptrFeat->KptTypes[static_cast<int>(Keypoint::BLOB)];
  }

  return debugArray;
}

//-----------------------------------------------------------------------------
std::shared_ptr<KeypointExtractor> DenseSpinningSensorKeypointExtractor::Clone() const
{
  return std::make_shared<DenseSpinningSensorKeypointExtractor>(*this);
}

}// end of LidarSlam namespace