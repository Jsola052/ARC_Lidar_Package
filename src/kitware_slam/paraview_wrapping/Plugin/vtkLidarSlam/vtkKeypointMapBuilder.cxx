// SPDX-FileCopyrightText: Copyright (c) Kitware Inc.
// SPDX-License-Identifier: Apache-2.0

#include "vtkKeypointMapBuilder.h"

#include <vtkAbstractArray.h>
#include <vtkDoubleArray.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>

#include "LidarSlam/Utilities.h"

#include <pcl/common/transforms.h>

namespace
{
constexpr unsigned int LIDAR_FRAME_PORT = 0;
constexpr unsigned int TRAJECTORY_PORT = 1;
}

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkKeypointMapBuilder);

//-----------------------------------------------------------------------------
vtkKeypointMapBuilder::vtkKeypointMapBuilder()
{
  this->SetNumberOfInputPorts(2);

  this->SetInputArrayToProcess(
    0, ::LIDAR_FRAME_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "timestamp");
  this->SetInputArrayToProcess(
    1, ::LIDAR_FRAME_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "intensity");
  this->SetInputArrayToProcess(
    2, ::LIDAR_FRAME_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "laser_id");

  // Default keypoints types
  this->EnableKeypointType(LidarSlam::Keypoint::EDGE, true);
  this->EnableKeypointType(LidarSlam::Keypoint::INTENSITY_EDGE, true);
  this->EnableKeypointType(LidarSlam::Keypoint::PLANE, true);
  this->EnableKeypointType(LidarSlam::Keypoint::BLOB, false);
}

//------------------------------------------------------------------------------
vtkKeypointMapBuilder::~vtkKeypointMapBuilder() = default;

//-----------------------------------------------------------------------------
int vtkKeypointMapBuilder::FillInputPortInformation(int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  return 1;
}

//-----------------------------------------------------------------------------
int vtkKeypointMapBuilder::FillOutputPortInformation(int port, vtkInformation* info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMultiBlockDataSet");
  return 1;
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::Reset()
{
  for (auto& [_, map] : this->KeypointMaps)
  {
    map->Clear();
  }
  this->Modified();
}

//-----------------------------------------------------------------------------
int vtkKeypointMapBuilder::RequestInformation(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkPolyData* trajectory =
    vtkPolyData::GetData(inputVector[::TRAJECTORY_PORT]->GetInformationObject(0));
  if (!trajectory)
  {
    vtkErrorMacro("No input trajectory");
    return 0;
  }

  vtkDoubleArray* trajTimeArray =
    vtkArrayDownCast<vtkDoubleArray>(trajectory->GetPointData()->GetAbstractArray("Time"));
  vtkDoubleArray* trajOrientationArray = vtkArrayDownCast<vtkDoubleArray>(
    trajectory->GetPointData()->GetAbstractArray("Orientation(AxisAngle)"));
  if (!trajTimeArray || !trajOrientationArray)
  {
    vtkErrorMacro("Trajectory input doesn't not have corresponding orientation or time array!");
    return 0;
  }
  if (trajOrientationArray->GetNumberOfComponents() != 4)
  {
    vtkErrorMacro("Wrong number of components of the orientation. Must be 4.");
    return 0;
  }

  this->CheckAndRemoveOffsetFromTrajectory(trajectory);

  this->PoseManager.Reset();
  float minX = std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max();
  float maxX = std::numeric_limits<float>::lowest();
  float maxY = std::numeric_limits<float>::lowest();
  for (unsigned int idx = 0; idx < trajectory->GetNumberOfPoints(); idx++)
  {
    LidarSlam::ExternalSensors::PoseMeasurement mesurement;
    mesurement.Time = trajTimeArray->GetValue(idx);
    double point[3];
    trajectory->GetPoint(idx, point);
    point[0] -= this->OffsetOrigin[0];
    point[1] -= this->OffsetOrigin[1];
    point[2] -= this->OffsetOrigin[2];
    double axisX = trajOrientationArray->GetComponent(idx, 0);
    double axisY = trajOrientationArray->GetComponent(idx, 1);
    double axisZ = trajOrientationArray->GetComponent(idx, 2);
    double angle = trajOrientationArray->GetComponent(idx, 3);
    mesurement.Pose = LidarSlam::Utils::XYZAngleAxistoIsometry(
      point[0], point[1], point[2], angle, axisX, axisY, axisZ);
    this->PoseManager.AddMeasurement(mesurement);
    minX = std::min(static_cast<float>(point[0]), minX);
    maxX = std::max(static_cast<float>(point[0]), maxX);
    minY = std::min(static_cast<float>(point[1]), minY);
    maxY = std::max(static_cast<float>(point[1]), maxY);
  }

  const Eigen::Vector3f gridCenter((maxX - minX) / 2, (maxY - minY) / 2, 0);
  double gridSize = std::max(maxX - minX, maxY - minY) / 2.;
  gridSize += 10; // Add a 10m offset in the rolling grid size.
  this->KeypointMaps.clear();
  for (auto& type : this->UsedKeypoints)
  {
    // Init a new rolling grid
    this->KeypointMaps.insert_or_assign(type, std::make_shared<LidarSlam::RollingGrid>(gridCenter));
    this->KeypointMaps[type]->SetGridSize(gridSize);
    this->KeypointMaps[type]->SetVoxelResolution(this->VoxelResolution);
    double leafSize = this->DefaultLeafSize;
    if (this->LeafSizeMap.find(type) != this->LeafSizeMap.end())
    {
      leafSize = this->LeafSizeMap[type];
    }
    this->KeypointMaps[type]->SetLeafSize(leafSize);
  }
  return 1;
}

//-----------------------------------------------------------------------------
int vtkKeypointMapBuilder::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  vtkInformation* inInfo = inputVector[::LIDAR_FRAME_PORT]->GetInformationObject(0);
  vtkPolyData* pointcloud = vtkPolyData::GetData(inInfo);

  if (!inInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
  {
    vtkErrorMacro("Point cloud pipeline time is needed to retrieve trajectory position");
    return 0;
  }

  for (int i = 0; i < 3; i++)
  {
    if (!this->GetInputArrayToProcess(i, pointcloud))
    {
      vtkErrorMacro(<< "Failed to get input array to process.");
      return 0;
    }
  }

  vtkDataArray* timeArray = this->GetInputArrayToProcess(0, pointcloud);
  double currentFrameTime = timeArray->GetRange()[1] * this->TimeToSecondsFactor;

  LidarSlam::Slam::PointCloud::Ptr pclCloud(new LidarSlam::Slam::PointCloud);
  this->PolyDataToPointCloud(pointcloud, currentFrameTime, pclCloud);

  LidarSlam::ExternalSensors::PoseMeasurement currentTransform;

  this->PoseManager.SetTimeThreshold(0.2);
  if (!this->PoseManager.ComputeSynchronizedMeasure(currentFrameTime, currentTransform))
  {
    this->SetVTKOutput(outputVector);
    return 1;
  }

  auto extractor = this->KeyPointsExtractor->GetExtractor();
  std::vector enableType(this->UsedKeypoints.begin(), this->UsedKeypoints.end());
  extractor->Enable(enableType);
  extractor->ComputeKeyPoints(pclCloud);

  for (auto& type : this->UsedKeypoints)
  {
    auto keypoints = extractor->GetKeypoints(type);
    if (keypoints->empty())
    {
      continue;
    }
    this->PoseManager.UndistortPoints(keypoints, currentFrameTime);
    LidarSlam::Slam::PointCloud::Ptr transformedKeypoints(new LidarSlam::Slam::PointCloud);
    pcl::transformPointCloud(
      *keypoints, *transformedKeypoints, currentTransform.Pose.matrix().cast<float>());

    this->KeypointMaps[type]->Add(transformedKeypoints, false);
  }
  this->SetVTKOutput(outputVector);
  return 1;
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::SetVTKOutput(vtkInformationVector* outputVector) const
{
  unsigned int idx = 0;
  vtkMultiBlockDataSet* mapOutput = vtkMultiBlockDataSet::GetData(outputVector, 0);
  for (auto& type : this->UsedKeypoints)
  {
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    this->PointCloudToPolyData(this->KeypointMaps.at(type)->Get(), polydata);
    mapOutput->SetBlock(idx++, polydata);
  }
}

//----------------------------------------------------------------------------
void vtkKeypointMapBuilder::CheckAndRemoveOffsetFromTrajectory(vtkPolyData* localTrajectory)
{
  this->IsOffsetRemoved = false;

  vtkPoints* trajPositions = localTrajectory->GetPoints();
  if (!trajPositions || trajPositions->GetNumberOfPoints() == 0)
  {
    vtkWarningMacro("Trajectory is missing X, Y or Z array or is empty.");
    return;
  }

  // Get the first position as offset
  trajPositions->GetPoint(0, this->OffsetOrigin);
  double& x0 = this->OffsetOrigin[0];
  double& y0 = this->OffsetOrigin[1];
  double& z0 = this->OffsetOrigin[2];

  double distToZero = std::sqrt(x0 * x0 + y0 * y0 + z0 * z0);
  if (distToZero < 10000.0)
  {
    return;
  }
  this->IsOffsetRemoved = true;
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::PolyDataToPointCloud(
  vtkPolyData* polydata, double frameTime, LidarSlam::Slam::PointCloud::Ptr pclCloud)
{
  vtkPointData* pd = polydata->GetPointData();

  // Get pointers to arrays
  vtkDataArray* timeArray = this->GetInputArrayToProcess(0, polydata);
  vtkDataArray* intensityArray = this->GetInputArrayToProcess(1, polydata);
  vtkDataArray* laserIdArray = this->GetInputArrayToProcess(2, polydata);

  const vtkIdType nbPoints = polydata->GetNumberOfPoints();

  // Loop over points data
  pclCloud->reserve(nbPoints);
  pclCloud->header.stamp = frameTime * 1e6; // max time in microseconds
  pclCloud->header.frame_id = "mainLidar";
  for (vtkIdType i = 0; i < nbPoints; i++)
  {
    // Get point coordinates
    Eigen::Vector3d pos;
    polydata->GetPoint(i, pos.data());
    // Check that points coordinates are not null before adding point
    if (!LidarSlam::Utils::IsPointValid(pos))
    {
      continue;
    }
    LidarSlam::Slam::Point p;
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    p.time = timeArray->GetTuple1(i) * this->TimeToSecondsFactor - frameTime; // time in seconds
    p.laser_id = laserIdArray->GetTuple1(i);
    p.intensity = intensityArray->GetTuple1(i);
    if (LidarSlam::Utils::HasNanField(p))
    {
      continue;
    }
    pclCloud->push_back(p);
  }
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::PointCloudToPolyData(
  LidarSlam::Slam::PointCloud::Ptr pc, vtkPolyData* poly) const
{
  const vtkIdType nbPoints = pc->size();

  vtkNew<vtkPoints> pts;
  pts->SetDataTypeToDouble();
  pts->SetNumberOfPoints(nbPoints);
  poly->SetPoints(pts);

  vtkSmartPointer<vtkDoubleArray> intensityArray = vtkSmartPointer<vtkDoubleArray>::New();
  intensityArray->SetNumberOfComponents(1);
  intensityArray->SetNumberOfTuples(nbPoints);
  intensityArray->SetName("intensity");
  poly->GetPointData()->AddArray(intensityArray);

  vtkNew<vtkIdTypeArray> connectivity;
  connectivity->SetNumberOfValues(nbPoints);
  vtkNew<vtkCellArray> cellArray;
  cellArray->SetData(1, connectivity);
  poly->SetVerts(cellArray);

  for (vtkIdType i = 0; i < nbPoints; ++i)
  {
    // Set point
    const auto& p = pc->points[i];

    if (this->IsOffsetRemoved)
    {
      pts->SetPoint(
        i, p.x + this->OffsetOrigin[0], p.y + this->OffsetOrigin[1], p.z + this->OffsetOrigin[2]);
    }
    else
    {
      pts->SetPoint(i, p.x, p.y, p.z);
    }
    intensityArray->SetTuple1(i, p.intensity);
    connectivity->SetValue(i, i);
  }
}

//-----------------------------------------------------------------------------
vtkKeypointMapBuilder::PointCloudMapsT vtkKeypointMapBuilder::GetMaps(
  Eigen::Isometry3d transform) const
{
  PointCloudMapsT result;
  result.reserve(this->KeypointMaps.size());
  Eigen::Isometry3d offsetTransform = Eigen::Isometry3d::Identity();
  if (this->IsOffsetRemoved)
  {
    offsetTransform.translation() << this->OffsetOrigin[0], this->OffsetOrigin[1],
      this->OffsetOrigin[2];
  }
  auto trans = transform * offsetTransform;
  for (const auto& [key, map] : this->KeypointMaps)
  {
    LidarSlam::Slam::PointCloud::Ptr transformPointCloud(new LidarSlam::Slam::PointCloud);
    pcl::transformPointCloud(*map->Get(), *transformPointCloud, trans.matrix().cast<float>());
    result.emplace(key, transformPointCloud);
  }
  return result;
}

//-----------------------------------------------------------------------------
vtkKeypointMapBuilder::RollingGridMapsT vtkKeypointMapBuilder::GetRollingMaps() const
{
  return this->KeypointMaps;
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor* _arg)
{
  vtkSetObjectBodyMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor, _arg);
  this->KeyPointsExtractor->GetExtractor()->SetNbThreads(this->NumberOfThreads);
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::SetKeypointLeafSize(LidarSlam::Keypoint k, double s)
{
  this->LeafSizeMap[k] = s;
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::EnableEdges(bool enabled)
{
  this->EnableKeypointType(LidarSlam::Keypoint::EDGE, enabled);
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::EnableIntensityEdges(bool enabled)
{
  this->EnableKeypointType(LidarSlam::Keypoint::INTENSITY_EDGE, enabled);
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::EnablePlanes(bool enabled)
{
  this->EnableKeypointType(LidarSlam::Keypoint::PLANE, enabled);
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::EnableBlobs(bool enabled)
{
  this->EnableKeypointType(LidarSlam::Keypoint::BLOB, enabled);
}

//-----------------------------------------------------------------------------
void vtkKeypointMapBuilder::EnableKeypointType(LidarSlam::Keypoint type, bool enable)
{
  if (enable)
  {
    this->UsedKeypoints.insert(type);
  }
  else
  {
    this->UsedKeypoints.erase(type);
  }
  this->Modified();
}
