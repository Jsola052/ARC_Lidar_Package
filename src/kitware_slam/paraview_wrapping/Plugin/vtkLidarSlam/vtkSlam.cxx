//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Guilbert Pierre (Kitware SAS)
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

// LOCAL
#include "vtkSlam.h"
#include "vtkSpinningSensorKeypointExtractor.h"

// VTK
#include <vtkCellArray.h>
#include <vtkDataArray.h>
#include <vtkDelimitedTextReader.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkInformation.h>
#include <vtkAlgorithm.h>
#include <vtkInformationVector.h>
#include <vtkLine.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkDataSetAttributes.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkFieldData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTable.h>
#include <vtkPartitionedDataSet.h>
#include <vtkTransformPolyDataFilter.h>

// PCL
#include <pcl/common/transforms.h>

//Boost
// TODO : replace by std when passing to C++17 minimum
#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <map>

// vtkSlam filter input ports (vtkPolyData and vtkTable)
#define LIDAR_FRAME_INPUT_PORT 0       ///< Current LiDAR frame
#define EXTERNAL_SENSOR_INPUT_PORT 1   ///< Optional external sensors data (FieldData on any DataObject)
#define INPUT_PORT_COUNT 2

// vtkSlam filter output ports (vtkPolyData)
#define SLAM_FRAME_OUTPUT_PORT 0       ///< Current transformed SLAM frame enriched with debug arrays
#define SLAM_TRAJECTORY_OUTPUT_PORT 1  ///< Trajectory (with position, orientation, covariance and time)
#define EDGE_MAP_OUTPUT_PORT 2         ///< Edge keypoints map
#define INTENSITY_EDGE_MAP_OUTPUT_PORT 3         ///< intensity edge keypoints map
#define PLANE_MAP_OUTPUT_PORT 4        ///< Plane keypoints map
#define EDGE_KEYPOINTS_OUTPUT_PORT 5   ///< Extracted edge keypoints from current frame
#define INTENSITY_EDGE_KEYPOINTS_OUTPUT_PORT 6         ///< intensity edge keypoints map
#define PLANE_KEYPOINTS_OUTPUT_PORT 7  ///< Extracted plane keypoints from current frame
#define OUTPUT_PORT_COUNT 8

#define IF_VERBOSE(minVerbosityLevel, command) if (this->SlamAlgo->GetVerbosity() >= (minVerbosityLevel)) { command; }

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlam)

namespace Utils
{
namespace
{
// Import helper functions from LidarSlam
using namespace LidarSlam::Utils;

//-----------------------------------------------------------------------------
template<typename T>
vtkSmartPointer<T> CreateArray(const std::string& Name, int NumberOfComponents = 1, int NumberOfTuples = 0)
{
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetNumberOfComponents(NumberOfComponents);
  array->SetNumberOfTuples(NumberOfTuples);
  array->SetName(Name.c_str());
  return array;
}

//-----------------------------------------------------------------------------
bool CheckTableFields(vtkTable* csvTable, std::vector<std::string> fields)
{
  bool allFieldsHere = true;
  for (const std::string& f : fields)
    allFieldsHere = allFieldsHere && csvTable->GetRowData()->HasArray(f.c_str());

  return allFieldsHere;
}

//-----------------------------------------------------------------------------
bool CheckTableFields(vtkDataSetAttributes* data, std::vector<std::string> fields)
{
  for (const std::string& f : fields)
  {
    if (!data->HasArray(f.c_str()))
    {
      return false;
    }
  }

  return true;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkDelimitedTextReader> CreateCSVLoader(const std::string& fileName, const std::string& delimiter)
{
  if (fileName.empty())
    return nullptr;

  vtkSmartPointer<vtkDelimitedTextReader> reader = vtkSmartPointer<vtkDelimitedTextReader>::New();
  reader->SetFileName(fileName.c_str());
  reader->DetectNumericColumnsOn();
  reader->SetHaveHeaders(true);
  reader->SetFieldDelimiterCharacters(delimiter.c_str());
  reader->Update();

  return reader;
}
} // end of anonymous namespace
} // end of Utils namespace

//-----------------------------------------------------------------------------
// Try to load a 4x4 calibration matrix from FieldData using a single array name.
static bool LoadCalibrationFromFD(vtkFieldData* fd,
                                  const std::string& name,
                                  Eigen::Isometry3d& calibration)
{
  if (!fd)
    return false;

  calibration = Eigen::Isometry3d::Identity();

  vtkDataArray* calib = fd->GetArray(name.c_str());
  if (!calib)
    return false;

  if (calib->GetNumberOfTuples() == 4 && calib->GetNumberOfComponents() == 4)
  {
    double row[4];
    for (int r = 0; r < 4; ++r)
    {
      calib->GetTuple(r, row);
      for (int c = 0; c < 4; ++c)
        calibration.matrix()(r, c) = row[c];
    }
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------
vtkSlam::vtkSlam()
: SlamAlgo(new LidarSlam::Slam)
{
  this->InitPose << 0.,0.,0.,0.,0.,0.;
  this->SetNumberOfInputPorts(INPUT_PORT_COUNT);
  this->SetNumberOfOutputPorts(OUTPUT_PORT_COUNT);
  // If auto-detect mode is disabled, user needs to specify input arrays to use
  this->SetInputArrayToProcess(0, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "timestamp");
  this->SetInputArrayToProcess(1, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "intensity");
  this->SetInputArrayToProcess(2, LIDAR_FRAME_INPUT_PORT, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "laser_id");

  // Init slam internal state
  this->SlamAlgo->Reset(true);

  // Enable overlap computation only if required
  this->SlamAlgo->SetOverlapSamplingRatio(this->AdvancedReturnMode ||
                                          this->SlamAlgo->GetFailureDetectionEnabled() ?
                                          this->OverlapSamplingRatio :
                                          0.);
  // Enable motion metrics and averages/derivatives computation if required
  this->SlamAlgo->SetConfidenceWindow(this->AdvancedReturnMode ||
                                      this->SlamAlgo->GetFailureDetectionEnabled() ?
                                      this->ConfidenceWindow :
                                      0.);

  // As the user has supervision on the loop closure detection in PV,
  // the threshold validation value is set to a minimal low value (0.1)
  this->SlamAlgo->SetLoopEvaluationOverlapThreshold(0.1);

  // Init PV trajectory for SLAM trajectory output
  this->ResetTrajectory();

  this->TimeArrayName.resize(1);
  this->IntensityArrayName.resize(1);
  this->LaserIdArrayName.resize(1);
  this->ArePointsValid.resize(1);
}

//-----------------------------------------------------------------------------
void vtkSlam::Reset()
{
  // Reset SLAM time to remove doubled frames
  this->FrameTime = -1;
  this->LastFrameTime = -1;
  this->MultiLidarState.clear();

  if (this->SlamAlgo->IsRecovery())
    vtkWarningMacro(<< "Getting out of recovery mode");

  // Reset slam internal state
  this->SlamAlgo->Reset(true);

  // Init PV trajectory for SLAM trajectory output
  // /!\ Must be done before initializing the SLAM pose/maps
  this->ResetTrajectory();

  // Init the SLAM state (map + pose)
  this->SetInitialSlam();

  // Reset sensor data
  this->ResetSensors();

  // Refill sensor managers
  this->SetSensorData(this->ExtSensorFileName);

  // Reset the modified time so that the external sensor input is
  // reloaded on next execution
  this->ExternalSensorInputMTime = 0;

  // Reset the modified time of keypoints extractors
  this->LastKeUpdateMTime = 0;

  // Reset output cache
  this->OutputCacheShallow.resize(this->GetNumberOfOutputPorts());
  for (int i = 0; i < this->GetNumberOfOutputPorts(); ++i)
  {
    if (this->OutputCacheShallow[i])
    {
      this->OutputCacheShallow[i].TakeReference(this->OutputCacheShallow[i]->NewInstance());
    }
  }

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::ResetSensors()
{
  this->SlamAlgo->ResetSensors(true);
}

//-----------------------------------------------------------------------------
void vtkSlam::ResetOdom()
{
  // Set current slam pose to Identity
  this->SlamAlgo->SetCurrentPose(Eigen::Isometry3d::Identity());
  PRINT_INFO("The slam ODOM is reset to the current pose");

  // Update PV trajectory poses that have been modified by the SLAM
  this->UpdatePVTrajectory();

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::MoveOdomToExtPosesRefFrame()
{
  if (this->SlamAlgo->MoveOdomToExtPosesRefFrame())
    vtkWarningMacro(<< "An offset has been added onto slam odom "
                    << "-> Reset odom is recommended to reduce the numerical instability");

  // Update PV trajectory poses that have been modified by the SLAM
  this->UpdatePVTrajectory();

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
Eigen::Isometry3d vtkSlam::GetOdomToExtTransform()
{
  Eigen::Isometry3d odomToExt = Eigen::Isometry3d::Identity();
  this->SlamAlgo->ComputeOdomToExtRefTransform(odomToExt);
  return odomToExt;
}

//-----------------------------------------------------------------------------
void vtkSlam::MovePVTrajectoryToExtPoseRef()
{
  if (!this->Trajectory || this->Trajectory->GetNumberOfPoints() == 0)
    return;
  Eigen::Isometry3d toExtRef = this->GetOdomToExtTransform();

  vtkPoints* points = this->Trajectory->GetPoints();
  vtkDataArray* quatArray = this->Trajectory->GetPointData()->GetArray("Orientation(Quaternion)");
  vtkDataArray* axisAngleArray = this->Trajectory->GetPointData()->GetArray("Orientation(AxisAngle)");

  if (!points || !quatArray)
    return;

  vtkIdType nPoints = points->GetNumberOfPoints();

  for (vtkIdType i = 0; i < nPoints; ++i)
  {
    // Get slam pose
    double p[3];
    points->GetPoint(i, p);
    Eigen::Vector3d transation(p[0], p[1], p[2]);
    double q[4];
    quatArray->GetTuple(i, q);
    Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = quat.toRotationMatrix();
    pose.translation() = transation;

    // Transform pose
    Eigen::Isometry3d transformedPose = toExtRef * pose;

    // Rewrite pose into pv trajectory
    Eigen::Vector3d newT = transformedPose.translation();
    points->SetPoint(i, newT.x(), newT.y(), newT.z());
    Eigen::Quaterniond newQuat(transformedPose.linear());
    double newQ[4] = {newQuat.w(), newQuat.x(), newQuat.y(), newQuat.z()};
    quatArray->SetTuple(i, newQ);

    if (axisAngleArray)
    {
      Eigen::AngleAxisd aa(transformedPose.linear());
      Eigen::Vector3d axis = aa.axis();
      double xyza[4] = {axis.x(), axis.y(), axis.z(), aa.angle()};
      axisAngleArray->SetTuple(i, xyza);
    }
  }

  points->Modified();
  quatArray->Modified();
  if (axisAngleArray)
    axisAngleArray->Modified();

  this->Trajectory->Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::OptimizeGraphWithIMU()
{
  const std::list<LidarSlam::LidarState>& initLidarStates = this->SlamAlgo->GetLogStates();
  if (initLidarStates.size() < 2)
    return;
  this->SlamAlgo->UpdateTrajectoryAndMapsWithIMU();
  // Update PV trajectory poses that have been optimized by the SLAM
  this->UpdatePVTrajectory(false);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::DetectLoop()
{
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (lidarStates.size() < 2)
    return;

  if (!this->SlamAlgo->DetectLoopClosureIndices(this->LastLoopInfo))
  {
    vtkWarningMacro(<< "Loop closure could not be detected automatically!");
    return;
  }

  this->SetLoopDetected(true);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::AddLoopDetection()
{
  if (this->LoopDetected)
    this->SlamAlgo->AddLoopClosureIndices(this->LastLoopInfo);

  this->SetLoopDetected(false);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearLoopDetections()
{
  this->SlamAlgo->ClearLoopDetections();
}

//-----------------------------------------------------------------------------
void vtkSlam::OptimizeGraph()
{
  // Resolve pending id-based manual position anchors to time-based ones
  this->AddManualAnchorsToSlam();

  if (!this->SlamAlgo->OptimizeGraph())
    return;

  // Update PV trajectory poses that have been optimized by the SLAM
  this->UpdatePVTrajectory(false);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::AddManualAnchorsToSlam()
{
  this->SlamAlgo->ClearManualAnchorsPosition();
  this->SlamAlgo->ClearManualAnchorsPose();

  vtkDataArray* timeArr = this->Trajectory->GetPointData()->GetArray("Time");
  if (!timeArr)
  {
    vtkErrorMacro("Could not find Time array in trajectory!");
    return;
  }
  for (const auto& row : this->PendingManualPosAnchorsById)
  {
    const vtkIdType pid = static_cast<vtkIdType>(std::llround(row[0]));
    if (pid > 0 && pid < timeArr->GetNumberOfTuples())
    {
      const double t = timeArr->GetTuple1(pid);
      this->AddManualPositionAnchor(t, row[1], row[2], row[3]);
    }
  }
  for (double id : this->PendingFixedAnchorsById)
  {
    const vtkIdType pid = static_cast<vtkIdType>(std::llround(id));
    if (pid > 0 && pid < timeArr->GetNumberOfTuples())
    {
      const double t = timeArr->GetTuple1(pid);
      this->AddManualPoseAnchor(t);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::EnablePGOConstraintLoopClosure(bool enabled)
{
  vtkDebugMacro(<< "Enabling loop closure constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::LOOP_CLOSURE, enabled);
}

void vtkSlam::EnablePGOConstraintLandmark(bool enabled)
{
  vtkDebugMacro(<< "Enabling landmark constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::LANDMARK, enabled);
}

void vtkSlam::EnablePGOConstraintGPS(bool enabled)
{
  vtkDebugMacro(<< "Enabling GPS constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::GPS, enabled);
}

void vtkSlam::EnablePGOConstraintExtPose(bool enabled)
{
  vtkDebugMacro(<< "Enabling ext pose constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::EXT_POSE, enabled);
}

void vtkSlam::EnablePGOConstraintManualPosition(bool enabled)
{
  vtkDebugMacro(<< "Enabling manual position constraint for pose graph optimization");
  this->SlamAlgo->EnablePGOConstraint(LidarSlam::PGOConstraint::MANUAL_POSITION, enabled);
}

//-----------------------------------------------------------------------------
void vtkSlam::AddManualPositionAnchor(double time, double x, double y, double z)
{
  if (time < 1e-6)
  {
    return;
  }
  vtkDebugMacro(<< "Add manual position anchor at time = " << time
                << " xyz = (" << x << "," << y << "," << z << ")");

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);

  // Use a default, reasonably tight position covariance (in m^2).
  // Orientation is ignored by SLAM for position-only anchors (top-left 3x3 is used).
  Eigen::Matrix6d cov6 = LidarSlam::Utils::CreateDefaultCovariance(1e-1, 1e-1);

  // Add a manual position measurement
  this->SlamAlgo->AddManualPoseMeasurement(time, pose, cov6, false);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::AddManualPoseAnchor(double time)
{
  if (time < 1e-6)
  {
    return;
  }
  vtkDebugMacro(<< "Add manual pose anchor at time " << time);
  // Provide covariance only; target pose will be read from closest LogState at optimization time
  Eigen::Matrix6d cov6 = LidarSlam::Utils::CreateDefaultCovariance(1e-1, 1e-1);

  // Pass identity pose as placeholder; with useFixedPose=true, SLAM ignores it and uses
  // the pose of the LogState closest in time during optimization
  this->SlamAlgo->AddManualPoseMeasurement(time, Eigen::Isometry3d::Identity(), cov6, true);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetManualAnchorHandle(double x, double y, double z)
{
  this->SeedPosition[0] = x;
  this->SeedPosition[1] = y;
  this->SeedPosition[2] = z;
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
double* vtkSlam::GetSeedPosition()
{
  return this->SeedPosition;
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearManualAnchorsPosition()
{
  this->SlamAlgo->ClearManualAnchorsPosition();
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearManualAnchorsPose()
{
  this->SlamAlgo->ClearManualAnchorsPose();
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::AddFixedAnchorById(double id)
{
  this->PendingFixedAnchorsById.emplace_back(id);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearFixedAnchors()
{
  this->PendingFixedAnchorsById.clear();
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::AddManualPositionAnchorById(double id, double x, double y, double z)
{
  this->PendingManualPosAnchorsById.push_back({id, x, y, z});
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearManualAnchorsPositionId()
{
  this->PendingManualPosAnchorsById.clear();
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
bool vtkSlam::GetPGOConstraintLoopClosure()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LOOP_CLOSURE);
  if (enabled)
    vtkDebugMacro(<< "Loop closure constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "Loop closure constraint for PGO is disabled");
  return enabled;
}

bool vtkSlam::GetPGOConstraintLandmark()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::LANDMARK);
  if (enabled)
    vtkDebugMacro(<< "Landmark constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "Landmark constraint for PGO is disabled");
  return enabled;
}

bool vtkSlam::GetPGOConstraintGPS()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::GPS);
  if (enabled)
    vtkDebugMacro(<< "GPS constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "GPS constraint for PGO is disabled");
  return enabled;
}

bool vtkSlam::GetPGOConstraintExtPose()
{
  bool enabled = this->SlamAlgo->IsPGOConstraintEnabled(LidarSlam::PGOConstraint::EXT_POSE);
  if (enabled)
    vtkDebugMacro(<< "Ext pose constraint for PGO is enabled");
  else
    vtkDebugMacro(<< "Ext pose constraint for PGO is disabled");
  return enabled;
}

//-----------------------------------------------------------------------------
std::vector<bool> vtkSlam::GetArePointsValid(unsigned int deviceIndex)
{
  if (deviceIndex >= this->ArePointsValid.size())
    return std::vector<bool>(0);
  return this->ArePointsValid[deviceIndex];
}

//-----------------------------------------------------------------------------
void vtkSlam::ClearMapsAndLog()
{
  vtkDebugMacro(<< "Clearing the maps and the log");
  this->SlamAlgo->ClearLocalMaps();
  this->SlamAlgo->ClearLog();
  this->FrameTime = -1;
  this->LastFrameTime = -1;
  this->MultiLidarState.clear();
  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialSlam()
{
  // Check number of log states
  const std::list<LidarSlam::LidarState>& initLidarStates = this->SlamAlgo->GetLogStates();
  double motion = (initLidarStates.front().Isometry.translation() - initLidarStates.back().Isometry.translation()).norm();
  auto initPose = LidarSlam::Utils::XYZRPYtoIsometry(this->InitPose);
  // Only set initial pose if it is not identity
  if (!initPose.matrix().isIdentity(1e-6))
  {
    // Before setting initial pose, check whether or not a slam trajectory exist
    if ((initLidarStates.size() <= 2 || motion < this->SlamAlgo->GetKfDistanceThreshold()))
    {
      // Reset slam
      this->SlamAlgo->Reset(true);
      // Init the output SLAM trajectory
      this->ResetTrajectory();
      // There is no log states, jump to initial pose and the pose is added to log states
      this->SlamAlgo->JumpPose(initPose);
      // Set TworldInit
      this->SlamAlgo->SetTworldInit(initPose);
      // Update PV trajectory
      this->AddLastPosesToTrajectory();
    }
    else
    {
      vtkWarningMacro(<< "Could not initialize the SLAM because a trajectory already exists : "
                      << "please reset manually if you wish to proceed with initialization.");
    }
  }
  // Set initial maps for slam if they are provided
  if (!this->InitMapPrefix.empty())
  {
    if (this->InitMapPrefix.substr(this->InitMapPrefix.find('.') + 1, this->InitMapPrefix.size()) == "pcd")
      vtkErrorMacro(<< "Could not load the initial map : only the prefix path must be supplied (not the complete path)");
    else
      this->SlamAlgo->LoadMapsFromPCD(this->InitMapPrefix);
  }

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialMaps(const std::unordered_map<LidarSlam::Keypoint, LidarSlam::Slam::PointCloud::Ptr>& keypointMaps)
{
  this->SlamAlgo->LoadMapsFromPointcloud(keypointMaps);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPoseTranslation(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting InitialPoseTranslation to " << x << " " << y << " " << z);
  this->InitPose.x() = x;
  this->InitPose.y() = y;
  this->InitPose.z() = z;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPoseRotation(double roll, double pitch, double yaw)
{
  vtkDebugMacro(<< "Setting InitialPoseRotation to " << roll << " " << pitch << " " << yaw);
  this->InitPose(3) = roll;
  this->InitPose(4) = pitch;
  this->InitPose(5) = yaw;
}

//-----------------------------------------------------------------------------
void vtkSlam::UpdateMultiLidarState(vtkSmartPointer<vtkCompositeDataSet>& cds, std::vector<vtkSmartPointer<vtkPolyData>>& vecInputPolydata)
{
  unsigned int inputId = 0;
  // Init frame reception check
  for (auto& frame : this->MultiLidarState)
  {
    frame.second.HasFrame = false;
  }
  // Get input iterator
  vtkSmartPointer<vtkCompositeDataIterator> it = vtk::TakeSmartPointer(cds->NewIterator());
  it->InitTraversal();
  it->GoToFirstItem();
  while (!it->IsDoneWithTraversal())
  {
    // Get input polydata
    vtkPolyData* originalInput = vtkPolyData::SafeDownCast(it->GetCurrentDataObject());
    vtkSmartPointer<vtkPolyData> input = vtkSmartPointer<vtkPolyData>::New();
    if (!originalInput)
    {
      // Keep vecInputPolydata aligned with CompositeDataSet block indices.
      // An empty vtkPolyData is inserted for invalid blocks so that
      // inputId can still be used as an index later.
      vecInputPolydata.push_back(input);

      ++inputId;
      it->GoToNextItem();
      continue;
    }
    input->DeepCopy(originalInput);

    // Get frame id from block name
    vtkInformation* info = it->GetCurrentMetaData();
    std::string deviceId = (info && info->Has(vtkCompositeDataSet::NAME())) ? info->Get(vtkCompositeDataSet::NAME())
                                                                              : std::string("Lidar") + std::to_string(inputId);

    // Get base to LiDAR tranform
    vtkFieldData* fieldData = input->GetFieldData();
    Eigen::Isometry3d base2Lidar = Eigen::Isometry3d::Identity();
    if (LoadCalibrationFromFD(fieldData, "BaseToLiDAR", base2Lidar))
    {
      this->SlamAlgo->SetBaseToLidarOffset(base2Lidar, deviceId);
    }
    else
    {
      Eigen::Isometry3d lidar2Base = Eigen::Isometry3d::Identity();
      // If LiDARToBase exists, it means LidarView has already transformed
      // the input cloud from the LiDAR frame into the base frame.
      // Re-apply LiDARToBase to convert the cloud back into the LiDAR frame,
      // since SLAM keypoint extraction expects data expressed in the LiDAR frame.
      if (LoadCalibrationFromFD(fieldData, "LiDARToBase", lidar2Base))
      {
        base2Lidar = lidar2Base.inverse();
        this->SlamAlgo->SetBaseToLidarOffset(base2Lidar, deviceId);
        // Transform inputdata into lidar frame
        vtkNew<vtkMatrix4x4> matrix;

        for (int r = 0; r < 4; ++r)
          for (int c = 0; c < 4; ++c)
            matrix->SetElement(r, c, lidar2Base.matrix()(r, c));

        vtkNew<vtkTransform> transform;
        transform->SetMatrix(matrix);

        vtkNew<vtkTransformPolyDataFilter> tf;
        tf->SetInputData(input);
        tf->SetTransform(transform);
        tf->Update();

        input = tf->GetOutput();
      }
    }
    vecInputPolydata.push_back(input);
    if (input->GetNumberOfPoints() == 0)
    {
      ++inputId;
      it->GoToNextItem();
      continue;
    }
    vtkPointData* pd = input->GetPointData();
    if (!pd)
    {
      ++inputId;
      it->GoToNextItem();
      continue;
    }

    // Get frame time and update frame status
    vtkDataArray* array = pd->GetArray(this->TimeArrayName[inputId].c_str());
    if (!array || array->GetNumberOfTuples() == 0)
    {
      ++inputId;
      it->GoToNextItem();
      continue;
    }
    double currentFrameTime = array->GetRange()[1] * this->TimeToSecondsFactor;
    // Timestamps of points from mcap are often saved as a relative time
    if (this->PointTimeRelativeToFrame)
    {
      currentFrameTime = this->FrameReceptionPOSIXTime;
      // Use timestamps from field data if it is available
      // /!\ Doesn't work with a multilidar setup when the point time of input data is relative to frame
      // and no Timestamps value available in field data
      if (fieldData)
      {
        vtkDataArray* timestampArray = fieldData->GetArray("Timestamp");
        if (timestampArray && timestampArray->GetNumberOfTuples() > 0)
        {
          currentFrameTime = timestampArray->GetTuple1(0);
        }
      }
    }

    // Update multi-lidar state only if newer frames are recevied
    if (this->MultiLidarState.find(deviceId) == this->MultiLidarState.end()
        || currentFrameTime > this->MultiLidarState.at(deviceId).Time)
    {
      this->MultiLidarState[deviceId] = LidarFrameState(inputId, currentFrameTime, true, true);
    }
    else
    {
      this->MultiLidarState[deviceId].InputId = inputId;
      this->MultiLidarState[deviceId].HasFrame = true;
    }
    ++inputId;
    it->GoToNextItem();
  }
}

//-----------------------------------------------------------------------------
bool vtkSlam::AreAllFramesUpdated()
{
  // Check whether or not all lidar frames are updated
  for (const auto& lidarStatus : this->MultiLidarState)
  {
    if (!lidarStatus.second.Updated && lidarStatus.second.HasFrame)
      return false;
  }
  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::CheckMultiLidarFramesTimeDifference()
{
  if (this->MultiLidarState.size() <= 1)
    return true;

  // Check internal frames consistency in case of multi-lidar set up
  for (auto it1 = this->MultiLidarState.begin(); it1 != this->MultiLidarState.end(); ++it1)
  {
    if (!it1->second.HasFrame)
      continue;
    auto it2 = it1;
    ++it2;

    for (; it2 != this->MultiLidarState.end(); ++it2)
    {
      if (!it2->second.HasFrame)
        continue;
      double timeDifference = it2->second.Time - it1->second.Time;
      // Between 1Hz and 100Hz: 2 * (1 / 1Hz - 1 / 100Hz)
      if (std::fabs(timeDifference) > 2.)
      {
        return false;
      }
    }
  }

  return true;
}

//-----------------------------------------------------------------------------
std::vector<std::string> vtkSlam::GetDeviceIdsSortedByTime()
{
  std::vector<std::pair<std::string, double>> sortedFrames;
  for (const auto& lidarState : this->MultiLidarState)
  {
    sortedFrames.emplace_back(std::make_pair(lidarState.first, lidarState.second.Time));
  }
  // Sort frames by time, the newest first
  std::sort(sortedFrames.begin(), sortedFrames.end(),
    [](const auto& lidar1, const auto& lidar2){ return lidar1.second > lidar2.second; });

  std::vector<std::string> sortedDeviceId;
  sortedDeviceId.reserve(sortedFrames.size());
  for (const auto& frame : sortedFrames)
  {
    sortedDeviceId.emplace_back(frame.first);
  }
  return sortedDeviceId;
}

//-----------------------------------------------------------------------------
void vtkSlam::UpdateKeypointsExtractors()
{
  if (!this->KeyPointsExtractor || !this->KeyPointsExtractor->GetExtractor())
    return;

  // Rebuild extractor if extractor parameters have been modified
  auto templateKe = this->KeyPointsExtractor->GetExtractor();
  auto updateTime = this->KeyPointsExtractor->GetMTime();
  bool rebuildExtractor = updateTime > this->LastKeUpdateMTime;
  if (rebuildExtractor)
  {
    this->LastKeUpdateMTime = updateTime;
  }

  // Only one LiDAR device
  if (this->MultiLidarState.size() == 1)
  {
    if (rebuildExtractor || !this->SlamAlgo->GetKeyPointsExtractor())
    {
      this->SlamAlgo->SetKeyPointsExtractor(templateKe);
      this->SlamAlgo->GetKeyPointsExtractor()->SetNbThreads(this->SlamAlgo->GetNbThreads());
    }
    return;
  }

  // MultiLidar setup
  for (auto& [deviceId, state] : this->MultiLidarState)
  {
    if (!rebuildExtractor && this->SlamAlgo->GetKeyPointsExtractor(deviceId))
      continue;
    // Set a keypoint extractor for each LiDAR device
    auto cloneKe = templateKe->Clone();
    cloneKe->SetNbThreads(this->SlamAlgo->GetNbThreads());
    this->SlamAlgo->SetKeyPointsExtractor(cloneKe, deviceId);
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::InitOutput(vtkPolyData* input, vtkPolyData* output)
{
  vtkNew<vtkPoints> points;
  points->SetDataTypeToDouble();
  output->ShallowCopy(input);
  output->SetPoints(points);
  points->SetNumberOfPoints(input->GetPoints()->GetNumberOfPoints());
}

//-----------------------------------------------------------------------------
bool vtkSlam::ValidateAndLoadInputs(vtkInformationVector** inputVector, unsigned int& cdsSize, vtkSmartPointer<vtkCompositeDataSet>& cds)
{
  // Load optional external sensor input
  // If an optional external sensor input is connected, and has changed,
  // (re)load its FieldData to feed the SLAM external sensor managers.
  vtkTable* table = vtkTable::GetData(inputVector[EXTERNAL_SENSOR_INPUT_PORT], 0);
  if (table)
  {
    vtkMTimeType currentMTime = table->GetMTime();
    // Check whether the external sensor input modification time is changed
    if (currentMTime > this->ExternalSensorInputMTime)
    {
      vtkFieldData* measFD = table->GetRowData();
      vtkFieldData* calibFD = table->GetFieldData();

      this->LoadExternalSensorDataFromFieldData(measFD, calibFD);
      this->ExternalSensorInputMTime = currentMTime;
    }
  }
  else
  {
    vtkDataObject* extObj = vtkDataObject::GetData(inputVector[EXTERNAL_SENSOR_INPUT_PORT], 0);
    if (extObj)
    {
      vtkWarningMacro(<< "External sensor input must be a vtkTable. Got: "
                      << extObj->GetClassName()
                      << ". SLAM will be launched without external sensor data.");
    }
  }

  // Check if input is a multiblock
  cds = vtkCompositeDataSet::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
  // If the input is not a vtkCompositeDataSet check if it is a polydata
  if (!cds)
  {
    // Get the input
    vtkPolyData* input = vtkPolyData::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
    if (!input)
    {
      vtkErrorMacro(<< "Unable to cast input into a vtkPolyData");
      return false;
    }
    vtkNew<vtkPartitionedDataSet> pds;
    pds->SetNumberOfPartitions(1);
    pds->SetPartition(0, input);
    cds = pds;
  }
  // If the vtkCompositeDataSet doesn't contain exclusively polydata then return
  vtkSmartPointer<vtkCompositeDataIterator> itInput = vtk::TakeSmartPointer(cds->NewIterator());
  itInput->InitTraversal();
  itInput->GoToFirstItem();
  while (!itInput->IsDoneWithTraversal())
  {
    vtkPolyData* input = vtkPolyData::SafeDownCast(itInput->GetCurrentDataObject());
    if (!input)
    {
      vtkErrorMacro(<< "Unable to cast input into a vtkPolyData");
      return false;
    }
    ++cdsSize;
    itInput->GoToNextItem();
  }

  // Check input is not empty
  vtkIdType nbPoints = cds->GetNumberOfPoints();
  if (nbPoints == 0)
  {
    vtkErrorMacro(<< "Empty input data. Abort.");
    return false;
  }

  // Check if vectors need to be resized
  // It happends when a Lidar is turned off or turned on during the session
  if (this->TimeArrayName.size() != cdsSize || this->IntensityArrayName.size() != cdsSize ||
    this->LaserIdArrayName.size() != cdsSize || this->ArePointsValid.size() != cdsSize)
  {
    this->TimeArrayName.resize(cdsSize);
    this->IntensityArrayName.resize(cdsSize);
    this->LaserIdArrayName.resize(cdsSize);
    this->ArePointsValid.resize(cdsSize);
  }

  // Check input format
  itInput->InitTraversal();
  itInput->GoToFirstItem();
  unsigned int index = 0;
  while (!itInput->IsDoneWithTraversal())
  {
    vtkPolyData* input = vtkPolyData::SafeDownCast(itInput->GetCurrentDataObject());
    if (input->GetNumberOfPoints() && !this->IdentifyInputArrays(input, index++))
    {
      vtkErrorMacro(<< "Unable to identify LiDAR arrays to use or fail to compute time to second factor. "
                       "Please define them manually before processing the frame.");
      return false;
    }
    itInput->GoToNextItem();
  }

  return true;
}

//-----------------------------------------------------------------------------
int vtkSlam::RequestData(vtkInformation* vtkNotUsed(request), vtkInformationVector** inputVector,
  vtkInformationVector* outputVector)
{
  IF_VERBOSE(1, Utils::Timer::Init("vtkSlam"));

  // If frames are not processed by slam, keep last valid outputs
  this->OutputCacheShallow.resize(this->GetNumberOfOutputPorts());  // Ensure cache is sized
  for (int i = 0; i < this->GetNumberOfOutputPorts(); ++i)
  {
    vtkDataObject* output = vtkDataObject::GetData(outputVector, i);
    if (!this->OutputCacheShallow[i])
      continue;
    if (i == SLAM_FRAME_OUTPUT_PORT)
    {
      output->DeepCopy(this->OutputCacheShallow[i]);
    }
    else
    {
      output->ShallowCopy(this->OutputCacheShallow[i]);
    }
  }

  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : input conversions"));

  // Check and load inputs
  unsigned int cdsSize = 0;
  vtkSmartPointer<vtkCompositeDataSet> cdsInput;
  if (!this->ValidateAndLoadInputs(inputVector, cdsSize, cdsInput))
  {
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));
    IF_VERBOSE(1, Utils::Timer::StopAndDisplay("vtkSlam"));
    return 1;
  }

  // Get frame packet reception time
  this->FrameReceptionPOSIXTime = 0.;
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  if (inInfo->Has(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP()))
    this->FrameReceptionPOSIXTime = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());

  // Update multi-lidar state and get input polydata from different lidar devices
  std::vector<vtkSmartPointer<vtkPolyData>> vecInputPolydata;
  vecInputPolydata.reserve(cdsSize);
  this->UpdateMultiLidarState(cdsInput, vecInputPolydata);
  // Update keypoint extractors when necessary
  this->UpdateKeypointsExtractors();

  // Device id sorted by newest time first order
  std::vector<std::string> sortedDeviceId = this->GetDeviceIdsSortedByTime();
  if (sortedDeviceId.empty())
  {
    vtkErrorMacro(<< "No valid LiDAR device. Abort");
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));
    IF_VERBOSE(1, Utils::Timer::StopAndDisplay("vtkSlam"));
    return 1;
  }

  // Check whether all frames are received
  bool areAllFramesUpdated = this->AreAllFramesUpdated();

  // Check time difference between frames
  bool timeDifferenceConsistent = this->CheckMultiLidarFramesTimeDifference();

  // Update frame time
  this->FrameTime = this->MultiLidarState.at(sortedDeviceId[0]).Time;
  double waitingTime = this->FrameTime - this->LastFrameTime;

  // Get total number of points
  vtkIdType nbPoints = cdsInput->GetNumberOfPoints();

  // Frames can be processed by slam when
  // all frames are updated or timeout reached
  // internal frames time difference is consistent
  // and the current frame is newer than the previous frame
  // and frames contain enough points
  bool canBeProcessed = (areAllFramesUpdated || waitingTime > 0.2)
                        && timeDifferenceConsistent
                        && (this->LastFrameTime <= this->FrameTime) && (nbPoints >= 100);

  // Log messages for different status
  if (!areAllFramesUpdated && waitingTime <= 0.2)
    vtkDebugMacro(<< "Not all LiDAR frames updated yet. Waiting for remaining frames");
  if (std::abs(this->LastFrameTime - this->FrameTime) < 1e-6)
    vtkDebugMacro(<< "Timestamp has not changed. Skipping frame");
  if (this->LastFrameTime > this->FrameTime)
    vtkErrorMacro(<< "Received older frame than last processed one. Skipping frame");
  if (!timeDifferenceConsistent)
    vtkWarningMacro(<< "Too much time difference between frames from different LiDARs. Skipping frame");
  if (nbPoints < 100)
    vtkWarningMacro(<< "Input point cloud has too few points. Skipping frame");

  // If frames are not processed by slam, keep last valid outputs
  if (!canBeProcessed)
  {
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));
    IF_VERBOSE(1, Utils::Timer::StopAndDisplay("vtkSlam"));
    return 1;
  }

  // Conversion vtkPolyData -> PCL pointcloud
  std::vector<LidarSlam::Slam::PointCloud::Ptr> pc;
  pc.reserve(cdsSize);
  for (const auto& deviceId : sortedDeviceId)
  {
    int inputId = this->MultiLidarState.at(deviceId).InputId;
    LidarSlam::Slam::PointCloud::Ptr cloud(new LidarSlam::Slam::PointCloud);
    if (this->MultiLidarState.at(deviceId).HasFrame &&
        vecInputPolydata[inputId]->GetNumberOfPoints() > 0)
    {
      this->PolyDataToPointCloud(vecInputPolydata[inputId], cloud, deviceId);
      pc.push_back(cloud);
    }
  }

  // Check sensor time offset if external sensors are set to be synchronized on network time
  if (this->SynchronizeOnPacket)
  {
    // Get the first point time from the newest frame in vendor format
    int frameInputId = this->MultiLidarState.at(sortedDeviceId.front()).InputId;
    vtkDataArray *timeArray = vecInputPolydata[frameInputId]->GetPointData()->GetArray(this->TimeArrayName[frameInputId].c_str());
    double frameFirstPointTime = timeArray->GetRange()[0] * this->TimeToSecondsFactor;
    if (this->PointTimeRelativeToFrame)
      frameFirstPointTime += this->MultiLidarState.at(sortedDeviceId.front()).Time;

    // Get the frame reception posix time from the field data of the newest frame when sensor data is from streaming
    if (this->IsSensorDataLive)
    {
      bool foundTimestamp = false;
      vtkFieldData* fieldData = vecInputPolydata[frameInputId]->GetFieldData();
      if (fieldData)
      {
        vtkDataArray* timestampArray = fieldData->GetArray("Timestamp");
        if (timestampArray && timestampArray->GetNumberOfTuples() > 0)
        {
          this->FrameReceptionPOSIXTime = timestampArray->GetTuple1(0);
          foundTimestamp = true;
        }
      }
      if (!foundTimestamp)
      {
        vtkWarningMacro(<< "Reception timestamp missing: time offset with external sensor may be inaccurate");
      }
    }

    // Compare potential offset with current offset
    double absCurrentOffset = std::abs(this->SlamAlgo->GetSensorTimeOffset());
    double potentialOffset = frameFirstPointTime - this->FrameReceptionPOSIXTime;
    // We exclude the first frame cause frameReceptionPOSIXTime can be badly
    if (this->SlamAlgo->GetNbrFrameProcessed() > 0 &&
        (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset))
      this->SlamAlgo->SetSensorTimeOffset(potentialOffset);
  }
  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : input conversions"));

  // Run SLAM
  this->SlamAlgo->AddFrames(pc);
  // Reset lidar update status after slam process
  for (auto& lidarStatus : this->MultiLidarState)
  {
    lidarStatus.second.Updated = false;
  }
  // Update last processed frame time
  this->LastFrameTime = this->FrameTime;

  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : basic output conversions"));
  // ===== SLAM frame =====
  // Output : Current undistorted LiDAR frame in world coordinates
  vtkSmartPointer<vtkCompositeDataSet> cdsOutput =
    vtkCompositeDataSet::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
  // If the output is not a vtkCompositeDataSet check if it is a polydata
  bool outputIsPolyData = 0;
  if (!cdsOutput)
  {
    outputIsPolyData = 1;
    vtkNew<vtkPartitionedDataSet> pds;
    vtkNew<vtkPolyData> output;
    pds->SetNumberOfPartitions(1);
    pds->SetPartition(0, output);
    cdsOutput = pds;
  }
  else
  {
    cdsOutput->CompositeShallowCopy(cdsInput);
  }

  vtkSmartPointer<vtkCompositeDataIterator> itOutput = vtk::TakeSmartPointer(cdsOutput->NewIterator());
  itOutput->InitTraversal();
  itOutput->GoToFirstItem();
  // Create an output polydata for each element of the input
  for (auto input : vecInputPolydata)
  {
    vtkNew<vtkPolyData> output;
    if (input && input->GetNumberOfPoints() > 0)
      this->InitOutput(input, output);
    cdsOutput->SetDataSet(itOutput, output);
    itOutput->GoToNextItem();
  }
  if (this->OutputUndistortedFrame)
  {
    auto worldFrame = this->SlamAlgo->GetRegisteredFrame();

    // Iterate over the composite dataset and sets the registered point coordinates
    if (!worldFrame->empty())
    {
      unsigned int validFrameIndex = 0;
      for (const auto& deviceId : sortedDeviceId)
      {
        if (!this->MultiLidarState.at(deviceId).HasFrame)
          continue;
        int inputId = this->MultiLidarState.at(deviceId).InputId;
        if (vecInputPolydata[inputId]->GetNumberOfPoints() == 0)
          continue;

        // Get the iterator of the output corresponding to the current device Id
        itOutput->GoToFirstItem();
        for (int i = 0; i < inputId; ++i)
          itOutput->GoToNextItem();

        vtkPolyData* output = vtkPolyData::SafeDownCast(itOutput->GetCurrentDataObject());
        for (unsigned int pId = 0; pId < vecInputPolydata[inputId]->GetNumberOfPoints(); ++pId)
        {
          // Modify point only if valid
          double pos[3];
          if (this->ArePointsValid[inputId][pId])
          {
            const auto& p = worldFrame->points[validFrameIndex++];
            output->GetPoints()->SetPoint(pId, p.data);
          }
          else
            output->GetPoints()->SetPoint(pId, pos);
        }
      }
    }
  }
  else
  {
    auto currentPose = this->SlamAlgo->GetLastStates().back().Isometry;
    for (const auto& deviceId : sortedDeviceId)
    {
      if (!this->MultiLidarState.at(deviceId).HasFrame)
        continue;
      int inputId = this->MultiLidarState.at(deviceId).InputId;
      if (vecInputPolydata[inputId]->GetNumberOfPoints() == 0)
        continue;

      // Get the iterator of the output corresponding to the current device Id
      itOutput->GoToFirstItem();
      for (int i = 0; i < inputId; ++i)
        itOutput->GoToNextItem();

      vtkPolyData* output = vtkPolyData::SafeDownCast(itOutput->GetCurrentDataObject());
      vtkPolyData* input = vecInputPolydata[inputId];
      auto base2Lidar = this->SlamAlgo->GetBaseToLidarOffset(deviceId);
      for (unsigned int pId = 0; pId < vecInputPolydata[inputId]->GetNumberOfPoints(); ++pId)
      {
        // Modify point only if valid
        Eigen::Vector3d pos;
        if (this->ArePointsValid[inputId][pId])
        {
          input->GetPoint(pId, pos.data());
          pos = currentPose * base2Lidar * pos;
          output->GetPoints()->SetPoint(pId, pos.data());
        }
        else
          output->GetPoints()->SetPoint(pId, pos.data());
      }
    }
  }

  // If output is a polydata, ShallowCopy the polydata of the compositeDataSet
  // in the output polydata
  if (outputIsPolyData)
  {
    // Get the output
    vtkPolyData* output = vtkPolyData::GetData(outputVector, LIDAR_FRAME_INPUT_PORT);
    if (!output)
    {
      vtkErrorMacro(<< "Unable to cast output into a vtkPolyData");
      return 0;
    }
    itOutput->GoToFirstItem();
    vtkPolyData* poly = vtkPolyData::SafeDownCast(itOutput->GetCurrentDataObject());
    output->ShallowCopy(poly);
  }
  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : basic output conversions"));

  // ===== Aggregated Keypoints maps =====
  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output keypoints maps"));

  // Get the previous outputs
  auto* edgeMap          = vtkPolyData::GetData(outputVector, EDGE_MAP_OUTPUT_PORT);
  auto* planarMap        = vtkPolyData::GetData(outputVector, PLANE_MAP_OUTPUT_PORT);
  auto* intensityEdgeMap = vtkPolyData::GetData(outputVector, INTENSITY_EDGE_MAP_OUTPUT_PORT);

  if ((this->SlamAlgo->GetNbrFrameProcessed() - 1) % this->MapsUpdateStep == 0)
  {
    // Cache maps to update them only every MapsUpdateStep frames
    for (auto k : LidarSlam::KeypointTypes)
      this->CacheMaps[k] = vtkSmartPointer<vtkPolyData>::New();

    // The expected maps can be the whole maps or the submaps
    // If the maps is fixed by the user, the whole map and the submap are equal but the submap is outputed (faster)
    switch (this->OutputKeypointsMaps)
    {
      // Output the whole maps that are available
      case OutputKeypointsMapsMode::FULL_MAPS :
        for (auto k : LidarSlam::KeypointTypes)
        {
          if (this->SlamAlgo->KeypointTypeEnabled(k))
            this->PointCloudToPolyData(this->SlamAlgo->GetMap(k), this->CacheMaps[k]);
        }
        break;

      // Output the submaps that are available
      case OutputKeypointsMapsMode::SUB_MAPS :
        for (auto k : LidarSlam::KeypointTypes)
        {
          if (this->SlamAlgo->KeypointTypeEnabled(k))
            this->PointCloudToPolyData(this->SlamAlgo->GetTargetSubMap(k), this->CacheMaps[k]);
        }
        break;

      // If no map should be outputed, let the maps empty
      case OutputKeypointsMapsMode::NONE :
        break;

      default:
        break;
    }
  }

  // Fill outputs from cache
  edgeMap->ShallowCopy(this->CacheMaps[LidarSlam::EDGE]);
  planarMap->ShallowCopy(this->CacheMaps[LidarSlam::PLANE]);
  intensityEdgeMap->ShallowCopy(this->CacheMaps[LidarSlam::INTENSITY_EDGE]);

  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output keypoints maps"));

  // ===== Extracted keypoints from current frame =====
  if (this->OutputCurrentKeypoints)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output current keypoints"));
    for (auto k : LidarSlam::KeypointTypes)
    {
      int port = EDGE_KEYPOINTS_OUTPUT_PORT + static_cast<int>(k);
      if (port >= OUTPUT_PORT_COUNT)
        continue;
      auto* keyoints = vtkPolyData::GetData(outputVector, port);
      if (this->SlamAlgo->KeypointTypeEnabled(k))
        this->PointCloudToPolyData(this->SlamAlgo->GetKeypoints(k, this->OutputKeypointsInWorldCoordinates), keyoints);
      else
        this->PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr(new LidarSlam::Slam::PointCloud), keyoints);
    }
    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output current keypoints"));
  }

  // Add debug information if advanced return mode is enabled
  if (this->AdvancedReturnMode)
  {
    IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : add advanced return arrays"));

    // Keypoints extraction debug array (curvatures, depth gap, intensity gap...)
    // Arrays added to WORLD transformed frame output
    auto* slamFrame = vtkPolyData::GetData(outputVector, SLAM_FRAME_OUTPUT_PORT);
    auto keypointsExtractionDebugArray = this->SlamAlgo->GetKeyPointsExtractor()->GetDebugArray();
    for (const auto& it : keypointsExtractionDebugArray)
    {
      auto array = Utils::CreateArray<vtkFloatArray>(it.first.c_str(), 1, nbPoints);
      slamFrame->GetPointData()->AddArray(array);

      // Fill array values from debug data
      // memcpy is a better alternative than looping on all tuples
      // but can only be used if the arrays use continuous storage
      std::memcpy(array->GetVoidPointer(0), it.second.data(), sizeof(float) * it.second.size());
    }

    // ICP keypoints matching results for ego-motion registration or localization steps
    // Arrays added to keypoints extracted from current frame outputs
    if (this->OutputCurrentKeypoints)
    {
      std::unordered_map<std::string, vtkPolyData*> outputMap;
      if (this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE))
      {
        auto* edgePoints = vtkPolyData::GetData(outputVector, EDGE_KEYPOINTS_OUTPUT_PORT);
        outputMap["EgoMotion: edge matches"]     = edgePoints;
        outputMap["EgoMotion: edge weights"]     = edgePoints;
        outputMap["Localization: edge matches"]  = edgePoints;
        outputMap["Localization: edge weights"]  = edgePoints;
      }
      if (this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE))
      {
        auto* planarPoints = vtkPolyData::GetData(outputVector, PLANE_KEYPOINTS_OUTPUT_PORT);
        outputMap["EgoMotion: plane matches"]    = planarPoints;
        outputMap["EgoMotion: plane weights"]    = planarPoints;
        outputMap["Localization: plane matches"] = planarPoints;
        outputMap["Localization: plane weights"] = planarPoints;

      }

      auto debugArray = this->SlamAlgo->GetDebugArray();
      for (const auto& it : outputMap)
      {
        auto array = Utils::CreateArray<vtkDoubleArray>(it.first.c_str(), 1, debugArray[it.first].size());
        // memcpy is a better alternative than looping on all tuples
        std::memcpy(array->GetVoidPointer(0), debugArray[it.first].data(), sizeof(double) * debugArray[it.first].size());
        it.second->GetPointData()->AddArray(array);
      }
    }

    IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : add advanced return arrays"));
  }

  IF_VERBOSE(3, Utils::Timer::Init("vtkSlam : output trajectory"));
  // Update trajectory
  // If SLAM had failed before
  if (this->SlamAlgo->IsRecovery())
  {
    // TMP : in the future, the user should have a look
    // at the result to validate recovery
    // Check if the SLAM can go on and pose has to be displayed
    if (this->SlamAlgo->GetOverlapEstimation() > 0.2f &&
        this->SlamAlgo->GetPositionErrorStd()  < 0.1f)
    {
      vtkWarningMacro(<< "Getting out of recovery mode");
      // Frame is relocalized, reset params
      this->SlamAlgo->EndRecovery();
      this->AddLastPosesToTrajectory();
    }
    else
      vtkWarningMacro(<< "Still waiting for recovery");
  }
  // Checking failure and add or not the poses
  else if (this->SlamAlgo->HasFailed())
  {
    vtkErrorMacro(<< "SLAM has failed : entering recovery mode :\n"
      << "\t -Maps will not be updated\n"
      << "\t -Egomotion and undistortion are disabled\n"
      << "\t -The number of ICP iterations is increased\n"
      << "\t -The maximum distance between a frame point and a map target point is increased");
    // Enable recovery mode :
    // Last frames are removed
    // Maps are not updated
    // Param are tuned to handle bigger motions
    // Warning : real time is not ensured
    this->SlamAlgo->StartRecovery(this->RecoveryTime);
    // Remove newest trajectory poses
    this->ResetTrajectory(this->SlamAlgo->GetLogStates().back().Time);
  }
  // Check if a PGO has been triggered in slam
  else if (this->GetPGOHasBeenTriggered())
  {
    // Update pv trajectory and reset PGOHasBeenTriggered parameter
    this->UpdatePVTrajectory(false);
    this->SetPGOHasBeenTriggered(false);
  }
  else
    this->AddLastPosesToTrajectory();

  // Output : SLAM Trajectory
  auto* slamTrajectory = vtkPolyData::GetData(outputVector, SLAM_TRAJECTORY_OUTPUT_PORT);
  slamTrajectory->DeepCopy(this->Trajectory);
  IF_VERBOSE(3, Utils::Timer::StopAndDisplay("vtkSlam : output trajectory"));


  // Update output cache
  for (int i = 0; i < this->GetNumberOfOutputPorts(); ++i)
  {
    vtkDataObject* output = vtkDataObject::GetData(outputVector, i);
    if (!output)
      continue;

    if (!this->OutputCacheShallow[i])
    {
      vtkErrorMacro("OutputCache is not allocated! Could not copy output in cache.");
      continue;
    }
    if (i == SLAM_FRAME_OUTPUT_PORT)
    {
      this->OutputCacheShallow[i]->DeepCopy(output);
    }
    else
    {
      this->OutputCacheShallow[i]->ShallowCopy(output);
    }
  }

  IF_VERBOSE(1, Utils::Timer::StopAndDisplay("vtkSlam"));

  return 1;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetImuGravity(double x, double y, double z)
{
  vtkDebugMacro(<< "Setting ImuGravity to " << x << " " << y << " " << z);
  this->SlamAlgo->SetImuGravity(Eigen::Vector3d({x, y, z}));
  // refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetPoseCalibration(const vtkTransform& transform)
{
  vtkMatrix4x4* vtkMat = const_cast<vtkTransform&>(transform).GetMatrix();
  Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor>> eigMat(&vtkMat->Element[0][0]);

  Eigen::Isometry3d calibration = Eigen::Isometry3d::Identity();
  calibration.matrix() = eigMat;
  this->SlamAlgo->SetPoseCalibration(calibration);
}

//-----------------------------------------------------------------------------
Eigen::Isometry3d vtkSlam::GetExtPose(double stateTime) const
{
  return this->SlamAlgo->GetExtPose(stateTime);
}

//-----------------------------------------------------------------------------
bool vtkSlam::GetCalibrationMatrix(const std::string& fileName, Eigen::Isometry3d& calibration) const
{
  // Look for file
  std::ifstream fin (fileName);
  calibration = Eigen::Isometry3d::Identity();
  if (fin.is_open())
  {
    // Parse elements
    int idxElement = 0;
    while (fin.good() && idxElement < 16)
    {
      std::string elementString;
      fin >> elementString;
      try
      {
        calibration.matrix()(idxElement) = std::stof(elementString);
      }
      catch (...)
      {
        vtkWarningMacro(<< "Calibration file not well formed"
                        << " -> calibration is set to identity");
        calibration = Eigen::Isometry3d::Identity();
        return false;
      }
      ++idxElement;
    }
    if (idxElement < 16)
    {
        vtkWarningMacro(<< "Calibration file not well formed"
                        << " -> calibration is set to identity");
        calibration = Eigen::Isometry3d::Identity();
        return false;
    }
    calibration.matrix().transposeInPlace();
  }
  else
  {
    vtkWarningMacro(<< "No calibration file named "
                    << fileName << " was found");
    return false;
  }

  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::LoadWheelOdomMeasurements(vtkDataArray* timeArray,
                                        vtkDataSetAttributes* rowData,
                                        const Eigen::Isometry3d& base2Sensor)
{
  vtkDataArray* distanceArray = rowData->GetArray("odom");
  if (!distanceArray)
  {
    return false;
  }

  this->SlamAlgo->SetWheelOdomCalibration(base2Sensor);

  for (vtkIdType i = 0; i < timeArray->GetNumberOfTuples(); ++i)
  {
    LidarSlam::ExternalSensors::WheelOdomMeasurement odomMeasurement;
    odomMeasurement.Time = timeArray->GetTuple1(i);
    odomMeasurement.Distance = distanceArray->GetTuple1(i);
    this->SlamAlgo->AddWheelOdomMeasurement(odomMeasurement);
  }
  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::LoadImuMeasurements(vtkDataArray* timeArray,
                                  vtkDataSetAttributes* rowData,
                                  const Eigen::Isometry3d& base2Sensor)
{
  vtkDataArray* accXArray = rowData->GetArray("acc_x");
  vtkDataArray* accYArray = rowData->GetArray("acc_y");
  vtkDataArray* accZArray = rowData->GetArray("acc_z");
  vtkDataArray* gyroXArray = rowData->GetArray("w_x");
  vtkDataArray* gyroYArray = rowData->GetArray("w_y");
  vtkDataArray* gyroZArray = rowData->GetArray("w_z");

  if (!accXArray || !accYArray || !accZArray || !gyroXArray || !gyroYArray || !gyroZArray)
  {
    return false;
  }

  std::map<int, int> freqHistogram;
  for (vtkIdType i = 1; i < timeArray->GetNumberOfTuples(); ++i)
  {
    int freq = std::round(1. / (timeArray->GetTuple1(i) - timeArray->GetTuple1(i - 1)));
    ++freqHistogram[freq];
  }

  int frequency = std::max_element(freqHistogram.begin(), freqHistogram.end(),
                                      [](const auto& lhs, const auto& rhs)
                                      { return lhs.second < rhs.second; })->first;

  this->SlamAlgo->SetImuFrequency(frequency);
  this->SlamAlgo->SetImuCalibration(base2Sensor);

  for (vtkIdType i = 0; i < timeArray->GetNumberOfTuples(); ++i)
  {
    LidarSlam::ExternalSensors::ImuMeasurement imuMeasurement;
    imuMeasurement.Time = timeArray->GetTuple1(i);
    imuMeasurement.Acceleration.x() = accXArray->GetTuple1(i);
    imuMeasurement.Acceleration.y() = accYArray->GetTuple1(i);
    imuMeasurement.Acceleration.z() = accZArray->GetTuple1(i);
    imuMeasurement.AngleVelocity.x() = gyroXArray->GetTuple1(i);
    imuMeasurement.AngleVelocity.y() = gyroYArray->GetTuple1(i);
    imuMeasurement.AngleVelocity.z() = gyroZArray->GetTuple1(i);
    this->SlamAlgo->AddImuMeasurement(imuMeasurement);
  }
  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::LoadPoseMeasurementsFromRpy(vtkDataArray* timeArray,
                                          vtkDataSetAttributes* rowData,
                                          const Eigen::Isometry3d& base2Sensor)
{
  vtkDataArray* xArray = rowData->GetArray("X");
  vtkDataArray* yArray = rowData->GetArray("Y");
  vtkDataArray* zArray = rowData->GetArray("Z");
  vtkDataArray* rollArray = rowData->GetArray("Rx(Roll)");
  vtkDataArray* pitchArray = rowData->GetArray("Ry(Pitch)");
  vtkDataArray* yawArray = rowData->GetArray("Rz(Yaw)");

  if (!xArray || !yArray || !zArray || !rollArray || !pitchArray || !yawArray)
  {
    return false;
  }

  vtkDataArray* covarianceArray = rowData->GetArray("Covariance");
  const bool hasCovariance = covarianceArray && covarianceArray->GetNumberOfComponents() == 36;

  vtkDataArray* errXArray = rowData->GetArray("errX");
  vtkDataArray* errYArray = rowData->GetArray("errY");
  vtkDataArray* errZArray = rowData->GetArray("errZ");
  vtkDataArray* errRollArray = rowData->GetArray("errRoll");
  vtkDataArray* errPitchArray = rowData->GetArray("errPitch");
  vtkDataArray* errYawArray = rowData->GetArray("errYaw");
  const bool hasErrors = errXArray && errYArray && errZArray &&
                         errRollArray && errPitchArray && errYawArray;

  this->SlamAlgo->SetPoseCalibration(base2Sensor);
  this->SlamAlgo->SetPoseCovarianceRotation((hasCovariance || hasErrors));

  vtkIdType startId = 0;
  vtkIdType nTimeArray = timeArray->GetNumberOfTuples();
  // Get the start indice for the latest pose measurements if the input is from live source
  if (this->IsSensorDataLive && this->SlamAlgo->PoseHasData())
  {
    double lastMeasTime = this->SlamAlgo->GetLastExtPose().Time;
    startId = nTimeArray;
    for (vtkIdType i = nTimeArray - 1; i >= 0; --i)
    {
      if (timeArray->GetTuple1(i) < lastMeasTime || std::abs(timeArray->GetTuple1(i) - lastMeasTime) < 1e-6)
      {
        break;
      }
      startId = i;
    }
  }

  for (vtkIdType i = startId; i < nTimeArray; ++i)
  {
    LidarSlam::ExternalSensors::PoseMeasurement meas;
    meas.Time = timeArray->GetTuple1(i);
    // Derive Isometry
    meas.Pose.linear() = Eigen::Matrix3d(
                         Eigen::AngleAxisd(yawArray->GetTuple1(i),   Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitchArray->GetTuple1(i), Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(rollArray->GetTuple1(i),  Eigen::Vector3d::UnitX())
                        );
    meas.Pose.translation() = Eigen::Vector3d(xArray->GetTuple1(i), yArray->GetTuple1(i), zArray->GetTuple1(i));
    meas.Pose.makeAffine();

    if (hasCovariance)
    {
      covarianceArray->GetTuple(i, meas.Covariance.data());
      if (!LidarSlam::Utils::isCovarianceValid(meas.Covariance))
        meas.Covariance = LidarSlam::Utils::CreateDefaultCovariance();
    }
    else if (hasErrors)
    {
      meas.Covariance = LidarSlam::Utils::CreateCovarianceMatrix(errXArray->GetTuple1(i),  errYArray->GetTuple1(i),  errZArray->GetTuple1(i),
                                                                 errRollArray->GetTuple1(i), errPitchArray->GetTuple1(i), errYawArray->GetTuple1(i));
    }
    else
    {
      meas.Covariance = LidarSlam::Utils::CreateDefaultCovariance();
    }

    this->SlamAlgo->AddPoseMeasurement(meas);
  }
  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::LoadGravityMeasurements(vtkDataArray* timeArray,
                                      vtkDataSetAttributes* rowData,
                                      const Eigen::Isometry3d& base2Sensor)
{
  vtkDataArray* accXArray = rowData->GetArray("acc_x");
  vtkDataArray* accYArray = rowData->GetArray("acc_y");
  vtkDataArray* accZArray = rowData->GetArray("acc_z");

  if (!accXArray || !accYArray || !accZArray)
  {
    return false;
  }

  this->SlamAlgo->SetGravityCalibration(base2Sensor);

  for (vtkIdType i = 0; i < timeArray->GetNumberOfTuples(); ++i)
  {
    LidarSlam::ExternalSensors::GravityMeasurement gravityMeasurement;
    gravityMeasurement.Time = timeArray->GetTuple1(i);
    gravityMeasurement.Acceleration.x() = accXArray->GetTuple1(i);
    gravityMeasurement.Acceleration.y() = accYArray->GetTuple1(i);
    gravityMeasurement.Acceleration.z() = accZArray->GetTuple1(i);
    this->SlamAlgo->AddGravityMeasurement(gravityMeasurement);
  }
  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::LoadPoseMeasurementsFromMatrix(vtkDataArray* timeArray,
                                             vtkDataSetAttributes* rowData,
                                             const Eigen::Isometry3d& base2Sensor)
{
  auto fetchArray = [rowData](const char* name) -> vtkDataArray* {
    return rowData->GetArray(name);
  };

  vtkDataArray* xArray = fetchArray("x");
  vtkDataArray* yArray = fetchArray("y");
  vtkDataArray* zArray = fetchArray("z");
  vtkDataArray* x0Array = fetchArray("x0");
  vtkDataArray* x1Array = fetchArray("x1");
  vtkDataArray* x2Array = fetchArray("x2");
  vtkDataArray* y0Array = fetchArray("y0");
  vtkDataArray* y1Array = fetchArray("y1");
  vtkDataArray* y2Array = fetchArray("y2");
  vtkDataArray* z0Array = fetchArray("z0");
  vtkDataArray* z1Array = fetchArray("z1");
  vtkDataArray* z2Array = fetchArray("z2");

  if (!xArray || !yArray || !zArray || !x0Array || !x1Array || !x2Array || !y0Array ||
    !y1Array || !y2Array || !z0Array || !z1Array || !z2Array)
  {
    return false;
  }

  this->SlamAlgo->SetPoseCalibration(base2Sensor);

  for (vtkIdType i = 0; i < timeArray->GetNumberOfTuples(); ++i)
  {
    LidarSlam::ExternalSensors::PoseMeasurement meas;
    meas.Time = timeArray->GetTuple1(i);
    // Derive Isometry
    meas.Pose.matrix() << x0Array->GetTuple1(i), x1Array->GetTuple1(i), x2Array->GetTuple1(i), xArray->GetTuple1(i),
                          y0Array->GetTuple1(i), y1Array->GetTuple1(i), y2Array->GetTuple1(i), yArray->GetTuple1(i),
                          z0Array->GetTuple1(i), z1Array->GetTuple1(i), z2Array->GetTuple1(i), zArray->GetTuple1(i),
                          0, 0, 0, 1;
    meas.Covariance = LidarSlam::Utils::CreateDefaultCovariance();
    this->SlamAlgo->AddPoseMeasurement(meas);
  }
  return true;
}

//-----------------------------------------------------------------------------
bool vtkSlam::LoadGpsMeasurements(vtkDataArray* timeArray,
                                  vtkDataSetAttributes* rowData,
                                  const Eigen::Isometry3d& base2Sensor)
{
  vtkDataArray* xArray = rowData->GetArray("X");
  vtkDataArray* yArray = rowData->GetArray("Y");
  vtkDataArray* zArray = rowData->GetArray("Z");

  if (!xArray || !yArray || !zArray)
  {
    return false;
  }

  this->SlamAlgo->SetGpsCalibration(base2Sensor);

  for (vtkIdType i = 0; i < timeArray->GetNumberOfTuples(); ++i)
  {
    LidarSlam::ExternalSensors::GpsMeasurement meas;
    meas.Time = timeArray->GetTuple1(i);
    // Position
    meas.Position = Eigen::Vector3d(xArray->GetTuple1(i), yArray->GetTuple1(i), zArray->GetTuple1(i));
    // Default covariance
    meas.Covariance = 1e-4 * Eigen::Matrix3d::Identity();
    this->SlamAlgo->AddGpsMeasurement(meas);
  }
  return true;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSensorData(const std::string& fileName)
{
  vtkDebugMacro(<< "Setting sensor data from " << fileName);
  this->ExtSensorFileName = fileName;
  // Empty current measurements and reset local sensor params
  this->SlamAlgo->ResetSensors(true);

  std::string delimiter = " ;,";
  vtkSmartPointer<vtkDelimitedTextReader> reader = Utils::CreateCSVLoader(fileName, delimiter);
  if (!reader)
     return;
  vtkTable* csvTable = reader->GetOutput();

  // Check if time exists and extract it
  if (!Utils::CheckTableFields(csvTable, {"Time"}))
  {
    vtkErrorMacro(<< "No time found in external sensor file, loading aborted");
    return;
  }
  auto arrayTime = csvTable->GetRowData()->GetArray("Time");
  if (arrayTime->GetNumberOfTuples() == 0)
  {
    vtkErrorMacro(<< "No measure found in external sensor file");
    return;
  }
  // Set the maximum number of measurements stored in the SLAM filter
  this->SlamAlgo->SetSensorMaxMeasures(arrayTime->GetNumberOfTuples());

  // Look for a calibration file next to first file
  boost::filesystem::path path(fileName);
  std::string calibFileName = (path.parent_path() / "calibration_external_sensor.mat").string();
  // Set calibration
  Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();
  if (this->GetCalibrationMatrix(calibFileName, base2Sensor))
    vtkWarningMacro(<< this->GetClassName() << " (" << this
                    << "): Calibration loaded : \n"
                    << base2Sensor.matrix());
  else
    vtkWarningMacro(<< this->GetClassName() << " (" << this
                    << "): Calibration has not been loaded. Make sure to calibrate your sensor before using it.");

  bool extSensorFit = false;

  // Process wheel odometer data
  if (Utils::CheckTableFields(csvTable, {"odom"}))
  {
    if (this->LoadWheelOdomMeasurements(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("Odometry data successfully loaded")
      extSensorFit = true;
    }
  }

  // Process external pose data
  bool hasExtPose = false;
  // 1_ Format XYZRPY with position and orientation errors (in radians)
  if (Utils::CheckTableFields(csvTable, {"X", "Y", "Z",
                                         "Rx(Roll)", "Ry(Pitch)", "Rz(Yaw)",
                                         "errX", "errY", "errZ",
                                         "errRoll", "errPitch", "errYaw"}))
  {
    if (this->LoadPoseMeasurementsFromRpy(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("Pose data successfully loaded")
      extSensorFit = true;
      hasExtPose = true;
    }
  }
  // 1_ Format XYZRPY
  else if (Utils::CheckTableFields(csvTable, {"X", "Y", "Z", "Rx(Roll)", "Ry(Pitch)", "Rz(Yaw)"}))
  {
    if (this->LoadPoseMeasurementsFromRpy(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("Pose data successfully loaded")
      extSensorFit = true;
      hasExtPose = true;
    }
  }
  // 2_ Matrix format
  else if (Utils::CheckTableFields(csvTable, {"x", "y", "z",
                                              "x0", "x1", "x2",
                                              "y0", "y1", "y2",
                                              "z0", "z1", "z2"}))
  {
    if (this->LoadPoseMeasurementsFromMatrix(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("Pose data successfully loaded")
      extSensorFit = true;
      hasExtPose = true;
    }
  }

  // Process IMU data
#ifdef USE_GTSAM
  if (!hasExtPose && Utils::CheckTableFields(csvTable, {"acc_x", "acc_y", "acc_z", "w_x", "w_y", "w_z"}))
  {
    if (this->LoadImuMeasurements(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("IMU data successfully loaded");
      extSensorFit = true;
    }
  }
  else if (Utils::CheckTableFields(csvTable, {"acc_x", "acc_y", "acc_z"}))
  {
#else
  if (Utils::CheckTableFields(csvTable, {"acc_x", "acc_y", "acc_z"}))
  {
#endif
    if (this->LoadGravityMeasurements(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("IMU data successfully loaded for gravity integration");
      extSensorFit = true;
    }
  }

  // Process GPS data
  if (!hasExtPose && Utils::CheckTableFields(csvTable, {"X", "Y", "Z"}))
  {
    if (this->LoadGpsMeasurements(arrayTime, csvTable->GetRowData(), base2Sensor))
    {
      PRINT_INFO("GPS data successfully loaded")
      extSensorFit = true;
    }
  }

  if (!extSensorFit)
    vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No usable data found in the external sensor file");

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::LoadExternalSensorDataFromFieldData(vtkFieldData* measurementsFD, vtkFieldData* calibrationFD)
{
  if (!measurementsFD)
  {
    return;
  }

  // All external sensors require a time array
  vtkDataArray* arrayTime = measurementsFD->GetArray("Time");
  if (!arrayTime)
  {
    vtkWarningMacro(<< "No time found in external sensor FieldData, loading aborted");
    return;
  }
  if (arrayTime->GetNumberOfTuples() == 0)
  {
    vtkWarningMacro(<< "No measure found in external sensor FieldData");
    return;
  }

  if (!this->IsSensorDataLive)
  {
    // Empty current measurements and reset local sensor params
    this->SlamAlgo->ResetSensors(true);
    // Set the maximum number of measurements stored in the SLAM filter
    this->SlamAlgo->SetSensorMaxMeasures(arrayTime->GetNumberOfTuples());
  }

  bool extSensorFit = false;

  vtkDataSetAttributes* measurementsDSA = vtkDataSetAttributes::SafeDownCast(measurementsFD);
  if (!measurementsDSA)
  {
    vtkWarningMacro(<< this->GetClassName() << " (" << this << "): Unsupported external sensor FieldData type");
    return;
  }

  if (Utils::CheckTableFields(measurementsDSA, {"odom"}))
  {
    Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();
    if (!LoadCalibrationFromFD(calibrationFD, "Calibration_Odometry", base2Sensor))
    {
      vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No calibration found for Odometry in FieldData. Using identity calibration.");
    }

    extSensorFit = this->LoadWheelOdomMeasurements(arrayTime, measurementsDSA, base2Sensor);
  }

  bool hasExtPose = false;
  if (Utils::CheckTableFields(measurementsDSA, {"X", "Y", "Z", "Rx(Roll)", "Ry(Pitch)", "Rz(Yaw)"}))
  {
    Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();
    if (!LoadCalibrationFromFD(calibrationFD, "Calibration_INS", base2Sensor))
    {
      vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No calibration found for INS in FieldData. Using identity calibration.");
    }

    extSensorFit = this->LoadPoseMeasurementsFromRpy(arrayTime, measurementsDSA, base2Sensor);
    hasExtPose = extSensorFit;
  }

#ifdef USE_GTSAM
  if (!hasExtPose && Utils::CheckTableFields(measurementsDSA, {"acc_x", "acc_y", "acc_z", "w_x", "w_y", "w_z"}))
  {
    Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();
    if (!LoadCalibrationFromFD(calibrationFD, "Calibration_IMU", base2Sensor))
    {
      vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No calibration found for IMU in FieldData. Using identity calibration.");
    }

    extSensorFit = this->LoadImuMeasurements(arrayTime, measurementsDSA, base2Sensor);
  }
  else if (Utils::CheckTableFields(measurementsDSA, {"acc_x", "acc_y", "acc_z"}))
  {
#else
  if (Utils::CheckTableFields(measurementsDSA, {"acc_x", "acc_y", "acc_z"}))
  {
#endif
    Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();
    if (!LoadCalibrationFromFD(calibrationFD, "Calibration_IMU", base2Sensor))
    {
      vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No calibration found for IMU in FieldData. Using identity calibration.");
    }

    extSensorFit = this->LoadGravityMeasurements(arrayTime, measurementsDSA, base2Sensor);
  }

  if (!extSensorFit && Utils::CheckTableFields(measurementsDSA, {"X", "Y", "Z"}))
  {
    Eigen::Isometry3d base2Sensor = Eigen::Isometry3d::Identity();

    if (!LoadCalibrationFromFD(calibrationFD, "Calibration_GNSS", base2Sensor))
    {
      vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No calibration found for GNSS in FieldData. Using identity calibration.");
    }

    extSensorFit = this->LoadGpsMeasurements(arrayTime, measurementsDSA, base2Sensor);
  }

  if (!extSensorFit)
  {
    vtkWarningMacro(<< this->GetClassName() << " (" << this
                    << "): No usable data found in external sensor FieldData; input may lack required arrays for supported sensors.");
  }

  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::Calibrate()
{
  if (!this->SlamAlgo->PoseHasData() &&
      !this->SlamAlgo->GpsHasData())
    vtkWarningMacro(<< this->GetClassName() << " (" << this << "): No external poses, no GPS -> cannot estimate any calibration");

  if (this->SlamAlgo->PoseHasData())
    this->SlamAlgo->CalibrateWithExtPoses(this->CalibrationWindow, this->LeverArm, true, this->PlanarTrajectory); // true := reset calibration
  else
    this->SlamAlgo->CalibrateWithGps(this->CalibrationWindow, this->LeverArm, true, this->PlanarTrajectory); // true := reset calibration

}

//-----------------------------------------------------------------------------
void vtkSlam::SetTrajectory(const std::string& fileName)
{
  std::string delimiter = " ;,";
  vtkSmartPointer<vtkDelimitedTextReader> reader = Utils::CreateCSVLoader(fileName, delimiter);
  if (!reader)
     return;
  vtkTable* csvTable = reader->GetOutput();

  // Check if time exists and extract it
  if (!Utils::CheckTableFields(csvTable, {"Time"}))
  {
    vtkWarningMacro(<<"No time information in the trajectory file. Load trajectory failed.");
    return;
  }
  auto arrayTime = csvTable->GetRowData()->GetArray("Time");
  vtkIdType numPose = arrayTime->GetNumberOfTuples();
  if (numPose == 0)
  {
    vtkWarningMacro(<<"No valid data in the trajectory file. Load trajectory failed.");
    return;
  }

  // Initialize a pose manager to store the external trajectory
  // Enable Verbose is useful to know whether the new trajectory is loaded correctly.
  // Set DistanceThreshold and AngleThreshold by the same values used in slam for checking keyframes.
  LidarSlam::ExternalSensors::PoseManager trajectoryManager("new trajectory");
  trajectoryManager.SetVerbose(true);
  trajectoryManager.SetDistanceThreshold(std::max(2., 2*this->GetKfDistanceThreshold()));

  // Set default covariance
  float defaultPositionError = 1e-2; // 1cm
  float defaultAngleError = Utils::Deg2Rad(1.f); // 1°
  Eigen::Matrix6d defaultCovariance = Utils::CreateDefaultCovariance(defaultPositionError, defaultAngleError); //1cm, 1°
  // If 2D mode enabled, supply constant covariance for unevaluated variables
  if (this->GetTwoDMode())
  {
    defaultCovariance(2, 2) = std::pow(defaultPositionError, 2);
    defaultCovariance(3, 3) = std::pow(defaultAngleError,    2);
    defaultCovariance(4, 4) = std::pow(defaultAngleError,    2);
  }
  // Process Covariance data
  std::vector<Eigen::Matrix6d> newCovariances(numPose, defaultCovariance);
  bool hasCovariance = true;
  for (int nCov = 0; nCov < 36; ++nCov)
  {
    hasCovariance = hasCovariance && Utils::CheckTableFields(csvTable, {"Covariance:" + std::to_string(nCov)});
    if (!hasCovariance)
      break;
  }
  if (hasCovariance)
  {
    // If covariance exists, set CovarianceRotation true
    trajectoryManager.SetCovarianceRotation(true);
    // Load covariance matrix
    for (int nCov = 0; nCov < 36; ++nCov)
    {
      auto arrayCovariance =  csvTable->GetRowData()->GetArray(("Covariance:"+std::to_string(nCov)).c_str());
      for (vtkIdType poseIdx = 0; poseIdx < numPose; ++poseIdx)
        newCovariances[poseIdx](nCov) =  arrayCovariance->GetTuple1(poseIdx);
    }
  }

  // Process Pose data
  if (Utils::CheckTableFields(csvTable, {"X", "Y", "Z", "Rx(Roll)", "Ry(Pitch)", "Rz(Yaw)"}))
  {
    auto arrayX     = csvTable->GetRowData()->GetArray("X"    );
    auto arrayY     = csvTable->GetRowData()->GetArray("Y"    );
    auto arrayZ     = csvTable->GetRowData()->GetArray("Z"    );
    auto arrayRoll  = csvTable->GetRowData()->GetArray("Rx(Roll)" );
    auto arrayPitch = csvTable->GetRowData()->GetArray("Ry(Pitch)");
    auto arrayYaw   = csvTable->GetRowData()->GetArray("Rz(Yaw)"  );

    for (vtkIdType i = 0; i < numPose; ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Pose = Utils::XYZRPYtoIsometry(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i),
                                          arrayRoll->GetTuple1(i), arrayPitch->GetTuple1(i), arrayYaw->GetTuple1(i));
      meas.Time = arrayTime->GetTuple1(i);
      // If covariance data is available, check the validity and add to measurement.
      if(hasCovariance)
      {
        if (!Utils::isCovarianceValid(newCovariances[i]))
          newCovariances[i] = defaultCovariance;
        meas.Covariance = newCovariances[i];
      }
      trajectoryManager.AddMeasurement(meas);
    }
  }
  else if (Utils::CheckTableFields(csvTable, {"Orientation(AxisAngle):0", "Orientation(AxisAngle):1",
                                              "Orientation(AxisAngle):2", "Orientation(AxisAngle):3",
                                              "Points:0", "Points:1", "Points:2"}))
  {
    auto arrayAxisX = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):0");
    auto arrayAxisY = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):1");
    auto arrayAxisZ = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):2");
    auto arrayAngle = csvTable->GetRowData()->GetArray("Orientation(AxisAngle):3");
    auto arrayX     = csvTable->GetRowData()->GetArray("Points:0"                );
    auto arrayY     = csvTable->GetRowData()->GetArray("Points:1"                );
    auto arrayZ     = csvTable->GetRowData()->GetArray("Points:2"                );
    for (vtkIdType i = 0; i < numPose; ++i)
    {
      LidarSlam::ExternalSensors::PoseMeasurement meas;
      meas.Pose = Utils::XYZAngleAxistoIsometry(arrayX->GetTuple1(i), arrayY->GetTuple1(i), arrayZ->GetTuple1(i),
                                                arrayAngle->GetTuple1(i),
                                                arrayAxisX->GetTuple1(i), arrayAxisY->GetTuple1(i), arrayAxisZ->GetTuple1(i));
      meas.Time = arrayTime->GetTuple1(i);
      // If covariance data is available, check the validity and add to measurement.
      if(hasCovariance)
      {
        if (!Utils::isCovarianceValid(newCovariances[i]))
          newCovariances[i] = defaultCovariance;
        meas.Covariance = newCovariances[i];
      }
      trajectoryManager.AddMeasurement(meas);
    }
  }

  // Reload LogStates in Slam with new trajectory
  this->SlamAlgo->ResetStatePoses(trajectoryManager);
  PRINT_INFO("Trajectory successfully loaded.");

  // Update PV trajectory poses that have been modified by the SLAM
  this->UpdatePVTrajectory(false);

  // Clear loop detections
  this->ClearLoopDetections();
  PRINT_INFO("Loop indices are cleared!");

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSensorTimeSynchronization(int mode)
{
  if (mode > 1)
  {
    vtkErrorMacro(<< "Invalid time synchronization mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting SensorTimeSynchronization to " << mode);
  this->SynchronizeOnPacket = (mode == 0);

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Slam parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->SlamAlgo->Get##param() << std::endl;

  PrintParameter(Undistortion)
  PrintParameter(NbThreads)
  PrintParameter(Verbosity)

  for (auto& k : LidarSlam::KeypointTypes)
  {
    if (this->SlamAlgo->KeypointTypeEnabled(k))
      os << LidarSlam::KeypointTypeNames.at(k) << " enabled" << std::endl;
  }

  PrintParameter(EgoMotionICPMaxIter)
  PrintParameter(EgoMotionLMMaxIter)
  PrintParameter(EgoMotionMaxNeighborsDistance)
  PrintParameter(EgoMotionEdgeNbNeighbors)
  PrintParameter(EgoMotionEdgeMinNbNeighbors)
  PrintParameter(EgoMotionEdgeMaxModelError)
  PrintParameter(EgoMotionPlaneNbNeighbors)
  PrintParameter(EgoMotionPlaneMaxModelError)
  PrintParameter(EgoMotionPlanarityThreshold)
  PrintParameter(EgoMotionInitSaturationDistance)
  PrintParameter(EgoMotionFinalSaturationDistance)

  PrintParameter(LocalizationICPMaxIter)
  PrintParameter(LocalizationLMMaxIter)
  PrintParameter(LocalizationMaxNeighborsDistance)
  PrintParameter(LocalizationEdgeNbNeighbors)
  PrintParameter(LocalizationEdgeMinNbNeighbors)
  PrintParameter(LocalizationEdgeMaxModelError)
  PrintParameter(LocalizationPlaneNbNeighbors)
  PrintParameter(LocalizationPlanarityThreshold)
  PrintParameter(LocalizationPlaneMaxModelError)
  PrintParameter(LocalizationBlobNbNeighbors)
  PrintParameter(LocalizationInitSaturationDistance)
  PrintParameter(LocalizationFinalSaturationDistance)

  this->GetKeyPointsExtractor()->PrintSelf(os, indent);
}

//-----------------------------------------------------------------------------
int vtkSlam::FillInputPortInformation(int port, vtkInformation* info)
{
  // Pointcloud data
  if (port == LIDAR_FRAME_INPUT_PORT)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData");
    return 1;
  }

  if (port == EXTERNAL_SENSOR_INPUT_PORT)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTable");
    info->Set(vtkAlgorithm::INPUT_IS_OPTIONAL(), 1);
    return 1;
  }
  return 0;
}

//-----------------------------------------------------------------------------
vtkMTimeType vtkSlam::GetMTime()
{
  return std::max(this->Superclass::GetMTime(), this->ParametersModificationTime.GetMTime());
}

// =============================================================================
//   Useful helpers
// =============================================================================

//-----------------------------------------------------------------------------
bool vtkSlam::IdentifyInputArrays(vtkPolyData* poly)
{
  if (this->TimeArrayName.empty() || this->IntensityArrayName.empty() ||
    this->LaserIdArrayName.empty())
  {
    this->TimeArrayName.resize(1);
    this->IntensityArrayName.resize(1);
    this->LaserIdArrayName.resize(1);
  }
  return IdentifyInputArrays(poly, 0);
}

//-----------------------------------------------------------------------------
bool vtkSlam::IdentifyInputArrays(vtkPolyData* poly, unsigned int index)
{
  // Check if requested lidar scan arrays exist and set them if they are valid
  if (!this->AutoDetectInputArrays)
  {
    for (int i = 0; i < 3; i++)
    {
      if (!this->GetInputArrayToProcess(i, poly))
      {
        vtkWarningMacro(<< "Failed to get input array to process.");
        return false;
      }
    }
    this->TimeArrayName[index] = this->GetInputArrayToProcess(0, poly)->GetName();
    this->IntensityArrayName[index] = this->GetInputArrayToProcess(1, poly)->GetName();
    this->LaserIdArrayName[index] = this->GetInputArrayToProcess(2, poly)->GetName();
  }
  else
  {
    auto checkAndSetScanArray = [&](const char* name, std::string& member)
    {
      if (poly->GetPointData()->HasArray(name))
        member = name;
    };

    checkAndSetScanArray("time", this->TimeArrayName[index]);
    checkAndSetScanArray("times", this->TimeArrayName[index]);
    checkAndSetScanArray("timestamp", this->TimeArrayName[index]);
    checkAndSetScanArray("adjustedTime", this->TimeArrayName[index]);
    if (this->TimeArrayName[index] == "")
      return false;

    checkAndSetScanArray("reflectivity", this->IntensityArrayName[index]);
    checkAndSetScanArray("Signal Photons", this->IntensityArrayName[index]);
    checkAndSetScanArray("intensity", this->IntensityArrayName[index]);
    if (this->IntensityArrayName[index] == "")
      return false;

    checkAndSetScanArray("laser_id", this->LaserIdArrayName[index]);
    checkAndSetScanArray("ring", this->LaserIdArrayName[index]);
    if (this->LaserIdArrayName[index] == "")
      return false;
  }

  // Estimate the factor to convert times to seconds
  auto arrayTime = poly->GetPointData()->GetArray(this->TimeArrayName[index].c_str());
  double* range = arrayTime->GetRange();
  double duration = std::abs(range[1] - range[0]);

  // We suppose the duration time is contained between 20ms and 0.9s.
  // Estimate time to seconds factor when it is not set manually
  while (this->AutoDetectInputArrays && this->TimeToSecondsFactor * duration > 0.9) // Min = ~1 Hz
  {
    double midTime = arrayTime->GetTuple1(arrayTime->GetNumberOfTuples() / 2);
    // Check if median time is balanced between min and max time values.
    // If the difference between the distances to min and max exceeds 50% of the duration,
    // it indicates a "time jump" in the frame, preventing automatic factor calculation.
    if (std::abs((range[1] - midTime) - (midTime - range[0])) > duration * 0.5)
    {
      PRINT_WARNING("Automatic time factor estimation failed: detected unbalanced frame timestamps (time jump). Skipping frame");
      return false;
    }
    this->TimeToSecondsFactor *= 1e-3;
    PRINT_INFO("Time factor estimated to " << this->TimeToSecondsFactor)
  }

  return true;
}

//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkSlam::CreateInitTrajectory()
{
  vtkSmartPointer<vtkPolyData> traj = vtkSmartPointer<vtkPolyData>::New();
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetDataTypeToDouble();
  traj->SetPoints(pts);
  auto cellArray = vtkSmartPointer<vtkCellArray>::New();
  traj->SetLines(cellArray);
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Time", 1));
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(Quaternion)", 4));
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Orientation(AxisAngle)", 4));
  traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>("Covariance", 36));
  // Add debug arrays if required
  auto debugInfo = this->SlamAlgo->GetDebugInformation();
  if (this->AdvancedReturnMode)
  {
    for (const auto& it : debugInfo)
      traj->GetPointData()->AddArray(Utils::CreateArray<vtkDoubleArray>(it.first, 1));
  }
  return traj;
}

//-----------------------------------------------------------------------------
void vtkSlam::ResetTrajectory(double endTime)
{
  // By default reset the output SLAM trajectory
  if (endTime < 0)
  {
    this->Trajectory = this->CreateInitTrajectory();
    return;
  }
  // Create a temporary trajectory to save the trajectory before endTime
  vtkSmartPointer<vtkPolyData> trajectoryTmp = this->CreateInitTrajectory();

  auto pointData = this->Trajectory->GetPointData();
  auto arrayTime = pointData->GetArray("Time");
  for (vtkIdType idx = 0; idx < arrayTime->GetNumberOfTuples(); ++idx)
  {
    if (*arrayTime->GetTuple(idx) < endTime)
    {
      double *translation = this->Trajectory->GetPoint(idx);
      trajectoryTmp->GetPoints()->InsertNextPoint(translation);

      for (vtkIdType idxArray = 0; idxArray < pointData->GetNumberOfArrays(); ++idxArray)
      {
        char *fieldName = pointData->GetArray(idxArray)->GetName();
        auto arrayTmp = trajectoryTmp->GetPointData()->GetArray(fieldName);
        if (arrayTmp)
        {
          double *value = pointData->GetArray(idxArray)->GetTuple(idx);
          arrayTmp->InsertNextTuple(value);
        }
      }

      // Add line linking 2 successive points
      vtkIdType nPoints = trajectoryTmp->GetNumberOfPoints();
      if (nPoints >= 2)
      {
        vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
        line->GetPointIds()->SetId(0, nPoints - 2);
        line->GetPointIds()->SetId(1, nPoints - 1);
        trajectoryTmp->GetLines()->InsertNextCell(line);
      }
    }
    else
      break;
  }

  // Copy temporary trajectory to Trajectory
  this->Trajectory->ShallowCopy(trajectoryTmp);

}

//-----------------------------------------------------------------------------
void vtkSlam::AddPoseToTrajectory(const LidarSlam::LidarState& state)
{
  // Add position
  Eigen::Vector3d translation = state.Isometry.translation();
  this->Trajectory->GetPoints()->InsertNextPoint(translation.x(), translation.y(), translation.z());

  // Add orientation as quaternion
  Eigen::Quaterniond quaternion(state.Isometry.linear());
  double wxyz[] = {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
  this->Trajectory->GetPointData()->GetArray("Orientation(Quaternion)")->InsertNextTuple(wxyz);

  // Add orientation as axis angle
  Eigen::AngleAxisd angleAxis(state.Isometry.linear());
  Eigen::Vector3d axis = angleAxis.axis();
  double xyza[] = {axis.x(), axis.y(), axis.z(), angleAxis.angle()};
  this->Trajectory->GetPointData()->GetArray("Orientation(AxisAngle)")->InsertNextTuple(xyza);

  // Add pose time and covariance
  this->Trajectory->GetPointData()->GetArray("Time")->InsertNextTuple(&state.Time);
  this->Trajectory->GetPointData()->GetArray("Covariance")->InsertNextTuple(state.Covariance.data());

  // Add line linking 2 successive points
  vtkIdType nPoints = this->Trajectory->GetNumberOfPoints();
  if (nPoints >= 2)
  {
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0, nPoints - 2);
    line->GetPointIds()->SetId(1, nPoints - 1);
    this->Trajectory->GetLines()->InsertNextCell(line);
  }

  if (this->AdvancedReturnMode)
  {
    // General SLAM info (number of keypoints used in ICP and optimization, max variance, ...)
    // Arrays added to trajectory output
    auto debugInfo = this->SlamAlgo->GetDebugInformation();
    for (const auto& it : debugInfo)
    {
      auto point = this->Trajectory->GetPointData();
      if (!point)
        continue;
      auto array = point->GetArray(it.first.c_str());
      if (!array)
        continue;
      array->InsertNextTuple1(it.second);
    }
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::AddLastPosesToTrajectory()
{
  // Get current SLAM pose in WORLD coordinates
  std::vector<LidarSlam::LidarState> lastStates = this->SlamAlgo->GetLastStates(this->TrajFrequency);

  for (const auto& state : lastStates)
    this->AddPoseToTrajectory(state);
}

//-----------------------------------------------------------------------------
void vtkSlam::UpdatePVTrajectory(bool resetPrevPoses)
{
  // Update trajectory for visualisation
  const std::list<LidarSlam::LidarState>& lidarStates = this->SlamAlgo->GetLogStates();
  if (!lidarStates.empty())
  {
    double endTime = resetPrevPoses ? -1 : lidarStates.front().Time;
    this->ResetTrajectory(endTime);
    for (auto const& state: lidarStates)
      this->AddPoseToTrajectory(state);
  }

  // Update the output cache when trajectory is updated.
  if (this->OutputCacheShallow.size() > SLAM_TRAJECTORY_OUTPUT_PORT
    && this->OutputCacheShallow[SLAM_TRAJECTORY_OUTPUT_PORT])
  {
    this->OutputCacheShallow[SLAM_TRAJECTORY_OUTPUT_PORT]->ShallowCopy(this->Trajectory);
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::PolyDataToPointCloud(vtkPolyData *poly,
                                   LidarSlam::Slam::PointCloud::Ptr pc)
{
  PolyDataToPointCloud(poly, pc, "mainLidar");
}

//-----------------------------------------------------------------------------
void vtkSlam::PolyDataToPointCloud(vtkPolyData *poly,
                                   LidarSlam::Slam::PointCloud::Ptr pc,
                                   const std::string& deviceId)
{
  const vtkIdType nbPoints = poly->GetNumberOfPoints();
  int inputId = this->MultiLidarState.at(deviceId).InputId;

  // Get pointers to arrays
  auto arrayTime = poly->GetPointData()->GetArray(this->TimeArrayName[inputId].c_str());
  auto arrayLaserId = poly->GetPointData()->GetArray(this->LaserIdArrayName[inputId].c_str());
  auto arrayIntensity = poly->GetPointData()->GetArray(this->IntensityArrayName[inputId].c_str());
  double frameEndTime = this->MultiLidarState.at(deviceId).Time;
  // Loop over points data
  pc->reserve(nbPoints);
  pc->header.stamp = frameEndTime * 1e6; // max time in microseconds
  pc->header.frame_id = deviceId;
  this->ArePointsValid[inputId].resize(nbPoints);
  for (vtkIdType i = 0; i < nbPoints; i++)
  {
    // Get point coordinates
    Eigen::Vector3d pos;
    poly->GetPoint(i, pos.data());
    this->ArePointsValid[inputId][i] = false;
    // Check that points coordinates are not null before adding point
    if (!Utils::IsPointValid(pos))
      continue;
    LidarSlam::Slam::Point p;
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    p.time = this->PointTimeRelativeToFrame ? arrayTime->GetTuple1(i) * this->TimeToSecondsFactor
              : arrayTime->GetTuple1(i) * this->TimeToSecondsFactor - frameEndTime; // time in seconds
    p.laser_id = arrayLaserId->GetTuple1(i);
    p.intensity = arrayIntensity->GetTuple1(i);
    if (Utils::HasNanField(p))
      continue;
    pc->push_back(p);
    this->ArePointsValid[inputId][i] = true;
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::PointCloudToPolyData(LidarSlam::Slam::PointCloud::Ptr pc, vtkPolyData* poly) const
{
  const vtkIdType nbPoints = pc->size();

  // Init and register points
  vtkNew<vtkPoints> pts;
  pts->SetDataTypeToDouble();
  pts->SetNumberOfPoints(nbPoints);
  poly->SetPoints(pts);
  auto intensityArray =
    Utils::CreateArray<vtkDoubleArray>(this->IntensityArrayName[0].c_str(), 1, nbPoints);
  poly->GetPointData()->AddArray(intensityArray);

  // Init and register cells
  vtkNew<vtkIdTypeArray> connectivity;
  connectivity->SetNumberOfValues(nbPoints);
  vtkNew<vtkCellArray> cellArray;
  cellArray->SetData(1 , connectivity);
  poly->SetVerts(cellArray);

  // Fill points and cells values
  for (vtkIdType i = 0; i < nbPoints; ++i)
  {
    // Set point
    const auto& p = pc->points[i];
    pts->SetPoint(i, p.x, p.y, p.z);
    intensityArray->SetTuple1(i, p.intensity);
    // TODO : add other fields (time, laserId)?

    connectivity->SetValue(i, i); //TODO can we iota this thing
  }
}

// =============================================================================
//   Getters / setters
// =============================================================================

bool vtkSlam::areEdgesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE);
  if (enabled)
    vtkDebugMacro(<< "Edges are enabled");
  else
    vtkDebugMacro(<< "Edges are disabled");
  return enabled;
}

bool vtkSlam::areIntensityEdgesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::INTENSITY_EDGE);
  if (enabled)
    vtkDebugMacro(<< "Intensity edges are enabled");
  else
    vtkDebugMacro(<< "Intensity edges are disabled");
  return enabled;
}

bool vtkSlam::arePlanesEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE);
  if (enabled)
    vtkDebugMacro(<< "Planes are enabled");
  else
    vtkDebugMacro(<< "Planes are disabled");
  return enabled;
}

bool vtkSlam::areBlobsEnabled()
{
  bool enabled = this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB);
  if (enabled)
    vtkDebugMacro(<< "Blobs are enabled");
  else
    vtkDebugMacro(<< "Blobs are disabled");
  return enabled;
}

//-----------------------------------------------------------------------------
void vtkSlam::EnableEdges(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling edges");
  else
  {
    vtkDebugMacro(<< "Disabling edges");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::EDGE, enabled);
  // Reset trajectory to add/remove confidence estimators related to edge keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

void vtkSlam::EnableIntensityEdges(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling intensity edges");
  else
  {
    vtkDebugMacro(<< "Disabling intensity edges");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::INTENSITY_EDGE, enabled);
  // Reset trajectory to add/remove confidence estimators related to intensity edge keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

void vtkSlam::EnablePlanes(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling planes");
  else
  {
    vtkDebugMacro(<< "Disabling planes");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::BLOB))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::PLANE, enabled);
  // Reset trajectory to add/remove confidence estimators related to plane keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

void vtkSlam::EnableBlobs(bool enabled)
{
  if (enabled)
    vtkDebugMacro(<< "Enabling blobs");
  else
  {
    vtkDebugMacro(<< "Disabling blobs");
    if (!this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::EDGE) &&
        !this->SlamAlgo->KeypointTypeEnabled(LidarSlam::Keypoint::PLANE))
      vtkWarningMacro(<< "No keypoint selected !");
  }
  this->SlamAlgo->EnableKeypointType(LidarSlam::Keypoint::BLOB, enabled);
  // Reset trajectory to add/remove confidence estimators related to blob keypoints
  if (this->AdvancedReturnMode)
    this->ResetTrajectory(this->FrameTime);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetFailureDetectionEnabled(bool faildetect)
{
  // If failure detection is being activated
  if (faildetect)
  {
    // Enable overlap computation
    this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
    // Enable motion metrics and averages/derivatives computation
    this->SlamAlgo->SetConfidenceWindow(this->ConfidenceWindow);
  }
  // If failure detection is being disabled
  else
  {
    // End recovery mode
    if (this->SlamAlgo->IsRecovery())
    {
      this->SlamAlgo->EndRecovery();
      vtkWarningMacro(<< "Getting out of recovery mode");
    }

    if (!this->AdvancedReturnMode)
    {
      // Disable overlap computation
      this->SlamAlgo->SetOverlapSamplingRatio(0.);
      // Disable motion metrics and averages/derivatives computation
      this->SlamAlgo->SetConfidenceWindow(0);
    }
  }
  this->SlamAlgo->SetFailureDetectionEnabled(faildetect);
}

//-----------------------------------------------------------------------------
void vtkSlam::SetAdvancedReturnMode(bool _arg)
{
  vtkDebugMacro(<< "Setting AdvancedReturnMode to " << _arg);
  if (this->AdvancedReturnMode != _arg)
  {
    auto debugInfo = this->SlamAlgo->GetDebugInformation();

    // If AdvancedReturnMode is being activated
    if (_arg)
    {
      // Add new optional arrays to trajectory, and init past values to 0.
      for (const auto& it : debugInfo)
      {
        auto array = Utils::CreateArray<vtkDoubleArray>(it.first, 1, this->Trajectory->GetNumberOfPoints());
        for (vtkIdType i = 0; i < this->Trajectory->GetNumberOfPoints(); i++)
          array->SetTuple1(i, 0.);
        this->Trajectory->GetPointData()->AddArray(array);
      }
      // Enable overlap computation
      this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
      // Enable motion metrics and averages/derivatives computation
      this->SlamAlgo->SetConfidenceWindow(this->ConfidenceWindow);
    }

    // If AdvancedReturnMode is being disabled
    else
    {
      // Delete optional arrays
      for (const auto& it : debugInfo)
        this->Trajectory->GetPointData()->RemoveArray(it.first.c_str());
      if (!this->SlamAlgo->GetFailureDetectionEnabled())
      {
        // Disable overlap computation
        this->SlamAlgo->SetOverlapSamplingRatio(0.);
        // Disable motion metrics and averages/derivatives computation
        this->SlamAlgo->SetConfidenceWindow(0);
      }
    }

    this->AdvancedReturnMode = _arg;
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetOutputKeypointsMaps()
{
  int outputMaps = static_cast<int>(this->OutputKeypointsMaps);
  vtkDebugMacro(<< "Returning output keypoints maps mode : " << outputMaps);
  return outputMaps;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetOutputKeypointsMaps(int mode)
{
  OutputKeypointsMapsMode outputMaps = static_cast<OutputKeypointsMapsMode>(mode);
  if (outputMaps != OutputKeypointsMapsMode::NONE      &&
      outputMaps != OutputKeypointsMapsMode::FULL_MAPS &&
      outputMaps != OutputKeypointsMapsMode::SUB_MAPS)
  {
    vtkErrorMacro(<< "Invalid output keypoints maps mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting output keypoints maps mode to " << mode);
  if (this->OutputKeypointsMaps != outputMaps)
  {
    this->OutputKeypointsMaps = outputMaps;
    this->ParametersModificationTime.Modified();
  }
  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
int vtkSlam::GetEgoMotion()
{
  int egoMotion = static_cast<int>(this->SlamAlgo->GetEgoMotion());
  vtkDebugMacro(<< "Returning Ego-Motion of " << egoMotion);
  return egoMotion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetEgoMotion(int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change ego motion in recovery mode! This param might be falsely set afterwards");
    return;
  }
  LidarSlam::EgoMotionMode egoMotion = static_cast<LidarSlam::EgoMotionMode>(mode);
  if (egoMotion != LidarSlam::EgoMotionMode::NONE                 &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION &&
      egoMotion != LidarSlam::EgoMotionMode::REGISTRATION         &&
      egoMotion != LidarSlam::EgoMotionMode::MOTION_EXTRAPOLATION_AND_REGISTRATION &&
      egoMotion != LidarSlam::EgoMotionMode::EXTERNAL &&
      egoMotion != LidarSlam::EgoMotionMode::EXTERNAL_OR_MOTION_EXTRAPOLATION)
  {
    vtkErrorMacro(<< "Invalid ego-motion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting Ego-Motion to " << mode);
  if (this->SlamAlgo->GetEgoMotion() != egoMotion)
  {
    this->SlamAlgo->SetEgoMotion(egoMotion);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetUndistortion()
{
  int undistortion = static_cast<int>(this->SlamAlgo->GetUndistortion());
  vtkDebugMacro(<< "Returning Undistortion of " << undistortion);
  return undistortion;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUndistortion(int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change undistortion in recovery mode! This param might be falsely set afterwards");
    return;
  }
  LidarSlam::UndistortionMode undistortion = static_cast<LidarSlam::UndistortionMode>(mode);
  if (undistortion != LidarSlam::UndistortionMode::NONE &&
      undistortion != LidarSlam::UndistortionMode::ONCE &&
      undistortion != LidarSlam::UndistortionMode::REFINED &&
      undistortion != LidarSlam::UndistortionMode::EXTERNAL)
  {
    vtkErrorMacro(<< "Invalid undistortion mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting Undistortion to " << mode);
  if (this->SlamAlgo->GetUndistortion() != undistortion)
  {
    this->SlamAlgo->SetUndistortion(undistortion);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
int vtkSlam::GetInterpolation()
{
  int interpoModel = static_cast<int>(this->SlamAlgo->GetInterpolation());
  vtkDebugMacro(<< this->GetClassName() << "(" << this << "): returning Interpolation Model of " << interpoModel);
  return interpoModel;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInterpolation(int model)
{
  LidarSlam::Interpolation::Model interpoModel = static_cast<LidarSlam::Interpolation::Model>(model);
  if (interpoModel != LidarSlam::Interpolation::Model::LINEAR &&
      interpoModel != LidarSlam::Interpolation::Model::QUADRATIC &&
      interpoModel != LidarSlam::Interpolation::Model::CUBIC)
  {
    vtkErrorMacro("Invalid Interpolation Model (" << model << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< this->GetClassName() << "(" << this << "): setting Interpolation Model to " << model);
  if (this->SlamAlgo->GetInterpolation() != interpoModel)
  {
    this->SlamAlgo->SetInterpolation(interpoModel);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetInitialPose(std::string filename)
{
  if (filename.empty())
    return;

  Eigen::Isometry3d newPose;
  this->GetCalibrationMatrix(filename, newPose);
  vtkDebugMacro(<< "Setting Initial pose to \n" << newPose.matrix() << "\n");
  // Move odom so the initial pose corresponds to the newPose
  this->SlamAlgo->SetInitialPose(newPose);

  // Update PV trajectory poses that have been modified by SLAM
  this->UpdatePVTrajectory();

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetCurrentPose(std::string filename)
{
  if (filename.empty())
    return;

  Eigen::Isometry3d newPose;
  this->GetCalibrationMatrix(filename, newPose);
  vtkDebugMacro(<< "Setting current pose to \n" << newPose.matrix() << "\n");
  // Move odom so the current pose corresponds to the newPose
  this->SlamAlgo->SetCurrentPose(newPose);

  // Update PV trajectory poses that have been modified by SLAM
  this->UpdatePVTrajectory();

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetKeyPointsExtractor(vtkSpinningSensorKeypointExtractor* _arg)
{
  vtkSetObjectBodyMacro(KeyPointsExtractor, vtkSpinningSensorKeypointExtractor, _arg);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
unsigned int vtkSlam::GetMapUpdate()
{
  unsigned int mapUpdate = static_cast<unsigned int>(this->SlamAlgo->GetMapUpdate());
  vtkDebugMacro(<< "Returning mapping mode of " << mapUpdate);
  return mapUpdate;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetMapUpdate(unsigned int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change map update in recovery mode! This param might be falsely displayed afterwards");
    return;
  }
  LidarSlam::MappingMode mapUpdate = static_cast<LidarSlam::MappingMode>(mode);
  if (mapUpdate != LidarSlam::MappingMode::NONE         &&
      mapUpdate != LidarSlam::MappingMode::ADD_KPTS_TO_FIXED_MAP &&
      mapUpdate != LidarSlam::MappingMode::UPDATE)
  {
    vtkErrorMacro(<< "Invalid mapping mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting mapping mode to " << mode);
  if (this->SlamAlgo->GetMapUpdate() != mapUpdate)
  {
    this->SlamAlgo->SetMapUpdate(mapUpdate);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
unsigned int vtkSlam::GetSubmapMode()
{
  unsigned int submapMode = static_cast<unsigned int>(this->SlamAlgo->GetSubmapMode());
  vtkDebugMacro(<< "Returning mapping mode of " << submapMode);
  return submapMode;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetSubmapMode(unsigned int mode)
{
  if (this->SlamAlgo->IsRecovery())
  {
    vtkErrorMacro(<< "Cannot change submap mode in recovery mode! This param might be falsely displayed afterwards");
    return;
  }
  LidarSlam::PreSearchMode submapMode = static_cast<LidarSlam::PreSearchMode>(mode);
  if (submapMode != LidarSlam::PreSearchMode::BOUNDING_BOX &&
      submapMode != LidarSlam::PreSearchMode::PROFILE)
  {
    vtkErrorMacro(<< "Invalid submap mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting submap mode to " << mode);
  if (this->SlamAlgo->GetSubmapMode() != submapMode)
  {
    this->SlamAlgo->SetSubmapMode(submapMode);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridLeafSize(LidarSlam::Keypoint k, double s)
{
  // The setting of this parameter is only possible if the relative map exists
  // the enabling step will create the map on which the sampling mode parameter can be set
  // The user can call this setter function only if the keypoint type has been enabled (see xml)
  // So, this function must not be called before clicking on enabled
  // However, clicking on Apply call all the setters with default values.
  // Therefore, the on/off state is checked but no warning can be raised
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
    return;

  vtkDebugMacro(<< "Setting VoxelGridLeafSize to " << s);
  this->SlamAlgo->SetVoxelGridLeafSize(k, s);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
double vtkSlam::GetVoxelGridLeafSize(LidarSlam::Keypoint k) const
{
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
  {
    vtkErrorMacro(<< "Cannot get leaf size, " << LidarSlam::KeypointTypeNames.at(k) << " keypoints are not enabled.");
    return -1.;
  }
  double leafSize = this->SlamAlgo->GetVoxelGridLeafSize(k);
  vtkDebugMacro(<< "Returning sampling mode : " << leafSize);
  return leafSize;
}

//-----------------------------------------------------------------------------
int vtkSlam::GetVoxelGridSamplingMode(LidarSlam::Keypoint k) const
{
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
  {
    vtkErrorMacro(<< "Cannot get sampling mode, " << LidarSlam::KeypointTypeNames.at(k) << " keypoints are not enabled.");
    return -1;
  }
  LidarSlam::SamplingMode sampling = this->SlamAlgo->GetVoxelGridSamplingMode(k);
  int sm = static_cast<int>(sampling);
  vtkDebugMacro(<< "Returning sampling mode : " << sm);
  return sm;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVoxelGridSamplingMode(LidarSlam::Keypoint k, int mode)
{
  // The setting of this parameter is only possible if the relative map exists
  // the enabling step will create the map on which the sampling mode parameter can be set
  // The user can call this setter function only if the keypoint type has been enabled (see xml)
  // So, this function must not be called before clicking on enabled
  // However, clicking on Apply call all the setters with default values.
  // Therefore, the on/off state is checked but no warning can be raised
  if (!this->SlamAlgo->KeypointTypeEnabled(k))
    return;

  LidarSlam::SamplingMode sampling = static_cast<LidarSlam::SamplingMode>(mode);
  if (sampling != LidarSlam::SamplingMode::FIRST         &&
      sampling != LidarSlam::SamplingMode::LAST          &&
      sampling != LidarSlam::SamplingMode::MAX_INTENSITY &&
      sampling != LidarSlam::SamplingMode::CENTER_POINT  &&
      sampling != LidarSlam::SamplingMode::CENTROID)
  {
    vtkErrorMacro(<< "Invalid sampling mode (" << mode << "), ignoring setting.");
    return;
  }
  vtkDebugMacro(<< "Setting sampling mode to " << mode);
  if (this->SlamAlgo->GetVoxelGridSamplingMode(k) != sampling)
  {
    this->SlamAlgo->SetVoxelGridSamplingMode(k, sampling);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetOverlapSamplingRatio(double ratio)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< "Setting OverlapSamplingRatio to " << ratio);
  if (ratio < 0 || ratio > 1)
  {
    vtkWarningMacro(<< "Overlap sampling ratio should be contained between 0 and 1"
                    << "Input value is : " << ratio
                    << "It is set to default value : 0.25");
    ratio = 0.25;
  }
  if (this->OverlapSamplingRatio != ratio)
  {
    this->OverlapSamplingRatio = ratio;
    // Forward this parameter change to SLAM if it is to be used in the interface
    if (this->AdvancedReturnMode || this->SlamAlgo->GetFailureDetectionEnabled())
      this->SlamAlgo->SetOverlapSamplingRatio(this->OverlapSamplingRatio);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetConfidenceWindow(unsigned int window)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< "Setting ConfidenceWindow to " << window);
  if (window == 1)
  {
    vtkWarningMacro(<< "Some confidence metrics will not be computed, "
                    << "please increase Confidence window value if you want to use it");
    window = 0;
  }
  if (this->ConfidenceWindow != window)
  {
    this->ConfidenceWindow = window;
    // Forward this parameter change to SLAM if it is to be used in the interface
    if (this->AdvancedReturnMode || this->SlamAlgo->GetFailureDetectionEnabled())
      this->SlamAlgo->SetConfidenceWindow(this->ConfidenceWindow);
    this->ParametersModificationTime.Modified();
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::SetAccelerationLimits(float linearAcc, float angularAcc)
{
  vtkDebugMacro(<< "Setting AccelerationLimits to " << linearAcc << " " << angularAcc);
  Eigen::Array2f accLim = {linearAcc, angularAcc};
  this->SlamAlgo->SetAccelerationLimits(accLim);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetVelocityLimits(float linearVel, float angularVel)
{
  vtkDebugMacro(<< "Setting VelocityLimits to " << linearVel << " " << angularVel);
  Eigen::Array2f velLim = {linearVel, angularVel};
  this->SlamAlgo->SetVelocityLimits(velLim);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetPoseLimits(float position, float orientation)
{
  vtkDebugMacro(<< "Setting PoseLimits to " << position << " " << orientation);
  Eigen::Array2f posLim = {position, orientation};
  this->SlamAlgo->SetPoseLimits(posLim);
  this->ParametersModificationTime.Modified();
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoggingTimeout(double loggingTimeout)
{
  // Change parameter value if it is modified
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting LoggingTimeout to " << loggingTimeout);
  if (this->SlamAlgo->GetLoggingTimeout() != loggingTimeout)
  {
    // Forward this parameter change to SLAM
    this->SlamAlgo->SetLoggingTimeout(loggingTimeout);
    this->ParametersModificationTime.Modified();
  }

  // If UsePoseGraph is enabled, check LoggingTimeout and return a warning if LoggingTimeout is 0
  if (this->UsePoseGraph && loggingTimeout <= 1e-6)
    vtkWarningMacro(<< "Pose graph is required but the logging timeout is null : "
                       "no pose can be used to build the graph, please increase the logging timeout.");
}

//-----------------------------------------------------------------------------
void vtkSlam::SetUsePoseGraph(bool usePoseGraph)
{
  if (this->UsePoseGraph != usePoseGraph)
  {
    this->UsePoseGraph = usePoseGraph;
    this->ParametersModificationTime.Modified();

    // If UsePoseGraph is enabled, check LoggingTimeout and return a warning if LoggingTimeout is 0
    if (this->UsePoseGraph && this->GetLoggingTimeout() <= 1e-6)
      vtkWarningMacro(<< "Pose graph is required but the logging timeout is null : "
                         "no pose can be used to build the graph, please increase the logging timeout.");
  }
}

//-----------------------------------------------------------------------------
double* vtkSlam::GetLoopClosurePosition()
{
  Eigen::Vector3d revistedPosition = this->SlamAlgo->GetStatePosition(this->LastLoopInfo.RevisitedIdx);
  this->LastLoopClosurePosition[0] = revistedPosition.x();
  this->LastLoopClosurePosition[1] = revistedPosition.y();
  this->LastLoopClosurePosition[2] = revistedPosition.z();
  return this->LastLoopClosurePosition;
}

//-----------------------------------------------------------------------------
int vtkSlam::GetLoopDetector()
{
  int loopClosureDetector = static_cast<int>(this->SlamAlgo->GetLoopDetector());
  vtkDebugMacro(<< "Returning loop closure detection of " << loopClosureDetector);
  return loopClosureDetector;
}

//-----------------------------------------------------------------------------
void vtkSlam::SetLoopDetector(int detector)
{
  LidarSlam::LoopClosureDetector loopClosureDetector = static_cast<LidarSlam::LoopClosureDetector>(detector);
  if (loopClosureDetector != LidarSlam::LoopClosureDetector::EXTERNAL   &&
      loopClosureDetector != LidarSlam::LoopClosureDetector::TEASERPP   &&
      loopClosureDetector != LidarSlam::LoopClosureDetector::NEAREST_POSE)
  {
    vtkErrorMacro(<< "Invalid loop closure detector (" << detector << "), ignoring setting.");
    return;
  }
  #ifndef USE_TEASERPP
  if (loopClosureDetector == LidarSlam::LoopClosureDetector::TEASERPP)
  {
    vtkErrorMacro(<< "Automatic loop closure detection requires TEASER++, but it was not found.");
    return;
  }
  #endif

  vtkDebugMacro(<< "Setting loop closure detector to " << static_cast<int>(loopClosureDetector));
  if (this->SlamAlgo->GetLoopDetector() != loopClosureDetector)
  {
    this->SlamAlgo->SetLoopDetector(loopClosureDetector);
    this->ParametersModificationTime.Modified();
  }

  // If teaser detector is enabled, the detection is performed on current frame.
  // As there is no mid submap AFTER current frame, we force the submaps to be built upon previous frames.
  if (loopClosureDetector == LidarSlam::LoopClosureDetector::TEASERPP)
  {
    this->SlamAlgo->SetLoopQueryMapEndRange(0);
    this->SlamAlgo->SetLoopRevisitedMapEndRange(0);
  }
}

//-----------------------------------------------------------------------------
void vtkSlam::LoadLoopDetectionIndices(const std::string& fileName)
{
  if (static_cast<LidarSlam::LoopClosureDetector>(this->GetLoopDetector()) != LidarSlam::LoopClosureDetector::EXTERNAL)
  {
    vtkWarningMacro(<< "Loading loop indices from external source is disabled!");
    return;
  }

  // Reset loop indices before loading new file
  this->ClearLoopDetections();

  std::string delimiter = " ;,";
  vtkSmartPointer<vtkDelimitedTextReader> reader = Utils::CreateCSVLoader(fileName, delimiter);
  if (!reader)
     return;
  vtkTable* csvTable = reader->GetOutput();

  // Check if loop closure information exists
  if (!Utils::CheckTableFields(csvTable, {"queryIdx", "revisitedIdx"}))
  {
    vtkWarningMacro(<<"No loop closure information in the file. Load loop closure indices failed.");
    return;
  }

  auto arrayQueryIdx     = csvTable->GetRowData()->GetArray("queryIdx"    );
  auto arrayRevisitedIdx = csvTable->GetRowData()->GetArray("revisitedIdx");
  vtkIdType numLoops     = arrayQueryIdx->GetNumberOfTuples();
  if (numLoops == 0)
  {
    vtkWarningMacro(<<"No valid data in the loop closure indices file. Load loop closure indices failed.");
    return;
  }

  // Process query frame indices and revisited frame indices
  for (vtkIdType i = 0; i < numLoops; ++i)
  {
    LidarSlam::LoopClosure::LoopInfo loop(arrayQueryIdx->GetTuple1(i), arrayRevisitedIdx->GetTuple1(i), -1);
    this->SlamAlgo->AddLoopClosureIndices(loop);
  }

  PRINT_INFO("Loop closure indices are loaded successfully from external source!");

  // Refresh view
  this->ParametersModificationTime.Modified();
}

//------------------------------------------------------------------------------
int vtkSlam::RequestDataObject(
  vtkInformation*, vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  if (this->GetNumberOfInputPorts() == 0 || this->GetNumberOfOutputPorts() == 0)
  {
    return 1;
  }

  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  if (!inInfo)
  {
    return 0;
  }
  vtkDataObject* input = inInfo->Get(vtkDataObject::DATA_OBJECT());

  this->OutputCacheShallow.resize(this->GetNumberOfOutputPorts());  // Ensure cache is sized
  if (input)
  {
    // for each output
    for (int i = 0; i < this->GetNumberOfOutputPorts(); ++i)
    {
      vtkInformation* info = outputVector->GetInformationObject(i);
      vtkDataObject* output = info->Get(vtkDataObject::DATA_OBJECT());

      if ((!output || !output->IsA(input->GetClassName())) && i == SLAM_FRAME_OUTPUT_PORT)
      {
        vtkDataObject* newOutput = input->NewInstance();
        info->Set(vtkDataObject::DATA_OBJECT(), newOutput);
        newOutput->Delete();
      }
      else if (!output || !output->IsA(input->GetClassName()))
      {
        vtkNew<vtkPolyData> newOutput;
        info->Set(vtkDataObject::DATA_OBJECT(), newOutput);
      }

      output = info->Get(vtkDataObject::DATA_OBJECT());
      if (!this->OutputCacheShallow[i])
        this->OutputCacheShallow[i].TakeReference(output->NewInstance());
    }
    return 1;
  }
  return 0;
}

//------------------------------------------------------------------------------
void vtkSlam::SetTimeArrayName(const std::vector<std::string>& names)
{
  this->TimeArrayName = names;
}

//------------------------------------------------------------------------------
const std::vector<std::string>& vtkSlam::GetTimeArrayName() const
{
  return this->TimeArrayName;
}
