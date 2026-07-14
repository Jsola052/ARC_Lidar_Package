// SPDX-FileCopyrightText: Copyright (c) Kitware Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef vtkKeypointMapBuilder_h
#define vtkKeypointMapBuilder_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkSetGet.h>

#include "vtkSpinningSensorKeypointExtractor.h"

#include "LidarSlam/ExternalSensorManagers.h"
#include "LidarSlam/RollingGrid.h"
#include "LidarSlam/Slam.h"

#include "vtkLidarSlamModule.h"

#include <set>
#include <unordered_map>

/**
 * This filter allow building a keypoint map outside of the SLAM.
 * As it more lightweight it can be used for example to build map only on a
 * small part of the SLAM trajectory or to debug the SLAM.
 *
 * It needs a point cloud and the associated SLAM trajectory.
 */
class VTKLIDARSLAM_EXPORT vtkKeypointMapBuilder : public vtkPolyDataAlgorithm
{
public:
  static vtkKeypointMapBuilder* New();
  vtkTypeMacro(vtkKeypointMapBuilder, vtkPolyDataAlgorithm);

  /**
   * Clean & reset all maps.
   */
  void Reset();

  ///@{
  /**
   * Number of thread to use.
   */
  vtkSetMacro(NumberOfThreads, int);
  vtkGetMacro(NumberOfThreads, int);
  ///@}

  ///@{
  /**
   * Resolution of the rolling grid outer voxel.
   */
  vtkSetMacro(VoxelResolution, double);
  vtkGetMacro(VoxelResolution, double);
  ///@}

  ///@{
  /**
   * Size of the leaf used to downsample the pointcloud with
   * a VoxelGrid filter within each voxel.
   */
  vtkSetMacro(DefaultLeafSize, double);
  vtkGetMacro(DefaultLeafSize, double);
  void SetKeypointLeafSize(LidarSlam::Keypoint k, double s);
  ///@}

  ///@{
  /**
   * Coef to apply on TimeArray values to express time in seconds.
   */
  vtkSetMacro(TimeToSecondsFactor, double);
  vtkGetMacro(TimeToSecondsFactor, double);
  ///@}

  ///@{
  /**
   * The different type of keypoint that can be enabled.
   * By default only blobs are disabled.
   */
  void EnableEdges(bool enable);
  void EnableIntensityEdges(bool enable);
  void EnablePlanes(bool enable);
  void EnableBlobs(bool enable);
  ///@}

  /**
   * Get built maps.
   */
  using PointCloudMapsT = std::unordered_map<LidarSlam::Keypoint, LidarSlam::Slam::PointCloud::Ptr>;
  using RollingGridMapsT =
    std::unordered_map<LidarSlam::Keypoint, std::shared_ptr<LidarSlam::RollingGrid>>;
  PointCloudMapsT GetMaps(Eigen::Isometry3d transform = {}) const;
  RollingGridMapsT GetRollingMaps() const;

  ///@{
  /**
   * The extractor used to generate keypoint maps.
   */
  vtkGetObjectMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor);
  virtual void SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor*);
  ///@}

protected:
  vtkKeypointMapBuilder();
  ~vtkKeypointMapBuilder() override;

  int RequestInformation(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  int FillInputPortInformation(int port, vtkInformation* info) override;
  int FillOutputPortInformation(int port, vtkInformation* info) override;

private:
  vtkKeypointMapBuilder(const vtkKeypointMapBuilder&) = delete;
  void operator=(const vtkKeypointMapBuilder&) = delete;

  void CheckAndRemoveOffsetFromTrajectory(vtkPolyData* localTrajectory);
  void PolyDataToPointCloud(
    vtkPolyData* polydata, double frameTime, LidarSlam::Slam::PointCloud::Ptr pclCloud);
  void PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr pc, vtkPolyData* poly) const;
  void SetVTKOutput(vtkInformationVector* outputVector) const;

  void EnableKeypointType(LidarSlam::Keypoint type, bool enable);

  int NumberOfThreads = 8;
  double TimeToSecondsFactor = 1.;

  double VoxelResolution = 10.;
  double DefaultLeafSize = 0.2;

  bool IsOffsetRemoved = false;
  double OffsetOrigin[3] = { 0., 0., 0. };

  std::set<LidarSlam::Keypoint> UsedKeypoints;
  std::unordered_map<LidarSlam::Keypoint, double> LeafSizeMap;
  RollingGridMapsT KeypointMaps;
  LidarSlam::ExternalSensors::PoseManager PoseManager;

  vtkSpinningSensorKeypointExtractor* KeyPointsExtractor = nullptr;
};

#endif
