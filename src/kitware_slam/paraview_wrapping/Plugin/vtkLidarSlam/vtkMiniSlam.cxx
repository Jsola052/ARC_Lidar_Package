//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Tong Fu (Kitware SAS)
// Creation date: 2024-11-29
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

#include "vtkMiniSlam.h"

#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkStreamingDemandDrivenPipeline.h>
#include <vtkTransform.h>

#include <sstream>

// vtkMiniSlam filter input ports (vtkPolyData and vtkTable)
#define LIDAR_FRAME_INPUT_PORT 0
#define INPUT_PORT_COUNT 1

#define SLAM_FRAMES_OUTPUT_PORT 0       ///< Current transformed SLAM frames
#define OUTPUT_PORT_COUNT 1

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkMiniSlam)

//----------------------------------------------------------------------------
vtkMiniSlam::vtkMiniSlam()
{
  // One input port
  this->SetNumberOfInputPorts(INPUT_PORT_COUNT);

  // The accumulation of stabilized frames
  this->SetNumberOfOutputPorts(OUTPUT_PORT_COUNT);

  this->SetProgressText("Computing slam");
}

//----------------------------------------------------------------------------
void vtkMiniSlam::ResetMiniSlam()
{
  this->SlamPoses.clear();
  this->SlamFrames.clear();
  vtkSlam::Reset();
}

//----------------------------------------------------------------------------
void vtkMiniSlam::PrintSelf(std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "MiniSlam parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->SlamAlgo->Get##param() << std::endl;

}

//----------------------------------------------------------------------------
int vtkMiniSlam::FillOutputPortInformation(int port, vtkInformation* info)
{
  if (port == 0)
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMultiBlockDataSet");
    return 1;
  }

  return 0;
}

//----------------------------------------------------------------------------
int vtkMiniSlam::RequestData(vtkInformation* request,
                             vtkInformationVector** inputVector,
                             vtkInformationVector* outputVector)
{
  // ===== Get input/output =====
  vtkPolyData* input = vtkPolyData::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
  vtkMultiBlockDataSet* output = vtkMultiBlockDataSet::GetData(outputVector);
  // Check if input is a multiblock
  if (!input)
  {
    vtkMultiBlockDataSet* mb = vtkMultiBlockDataSet::GetData(inputVector[LIDAR_FRAME_INPUT_PORT], 0);
    // Extract first block if it is a vtkPolyData
    if (mb)
        input = vtkPolyData::SafeDownCast(mb->GetBlock(0));
  }
  // If the input could not be cast, return
  if (!input)
  {
    vtkErrorMacro(<< "Unable to cast input into a vtkPolyData");
    return 0;
  }
  // Check input is not empty
  vtkIdType nbPoints = input->GetNumberOfPoints();
  if (nbPoints == 0)
  {
      vtkErrorMacro(<< "Empty input data. Abort.");
      return 0;
  }
  // Check input format
  if (!vtkSlam::IdentifyInputArrays(input))
  {
    vtkErrorMacro(<< "Unable to identify LiDAR arrays to use. Please define them manually before processing the frame.");
    return 0;
  }

  // ===== Apply SLAM =====
  if (vtkSlam::GetTimeArrayName().empty())
  {
    vtkErrorMacro(<< "Internal Error : vtkSlam::GetTimeArrayName() returned an empty array");
    return 0;
  }

  std::string timeArrayName = vtkSlam::GetTimeArrayName().front();
  auto arrayTime = input->GetPointData()->GetArray(timeArrayName.c_str());

  vtkSlam::SetLastFrameTime(vtkSlam::GetFrameTime());
  vtkSlam::SetFrameTime(arrayTime->GetRange()[1]);
  if (vtkSlam::GetLastFrameTime() == vtkSlam::GetFrameTime())
      vtkDebugMacro(<< "Timestamp has not changed. Skipping frame.");
  if (nbPoints < 100)
      vtkErrorMacro(<< "Input point cloud does not contain enough points. Skipping frame");

  // Conversion vtkPolyData -> PCL pointcloud
  if (vtkSlam::GetLastFrameTime() != vtkSlam::GetFrameTime() && nbPoints >= 100)
  {
    LidarSlam::Slam::PointCloud::Ptr pc(new LidarSlam::Slam::PointCloud);
    vtkSlam::PolyDataToPointCloud(input, pc);

    // Get frame first point time in vendor format
    double* range = arrayTime->GetRange();
    double frameFirstPointTime = range[0] * vtkSlam::GetTimeToSecondsFactor();
    if (vtkSlam::GetSynchronizeOnPacket())
    {
      // Get first frame packet reception time
      vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
      double frameReceptionPOSIXTime = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_TIME_STEP());
      double absCurrentOffset = std::abs(vtkSlam::SlamAlgo->GetSensorTimeOffset());
      double potentialOffset = frameFirstPointTime - frameReceptionPOSIXTime;
      // We exclude the first frame cause frameReceptionPOSIXTime can be badly set
      if (vtkSlam::SlamAlgo->GetNbrFrameProcessed() > 0 && (absCurrentOffset < 1e-6 || std::abs(potentialOffset) < absCurrentOffset))
          vtkSlam::SlamAlgo->SetSensorTimeOffset(potentialOffset);
    }
    // Run SLAM
    vtkSlam::SlamAlgo->AddFrame(pc);
  }

  // ===== Ouput slam =====
  // Output: slam pose
  this->AddSlamPose(vtkSlam::SlamAlgo->GetLastState().Isometry);
  // Output: Current undistorted LiDAR frame in world coordinates
  vtkNew<vtkPolyData> currentFrame;
  currentFrame->ShallowCopy(input);
  auto worldFrame = vtkSlam::SlamAlgo->GetRegisteredFrame();
  // Modify only points coordinates to keep input arrays
  auto registeredPoints = vtkSmartPointer<vtkPoints>::New();
  registeredPoints->SetNumberOfPoints(nbPoints);
  currentFrame->SetPoints(registeredPoints);

  std::vector<bool> arePointsValid = vtkSlam::GetArePointsValid();
  if (!worldFrame->empty())
  {
    unsigned int validFrameIndex = 0;
    for (vtkIdType i = 0; i < nbPoints; i++)
    {
      // Modify point only if valid
      double pos[3];
      input->GetPoint(i, pos);
      if (arePointsValid[i])
      {
        const auto& p = worldFrame->points[validFrameIndex++];
        registeredPoints->SetPoint(i, p.data);
      }
      else
        registeredPoints->SetPoint(i, pos);
    }
  }

  // Add current frame into frame cache
  this->SlamFrames.push_back(currentFrame);
  if (this->SlamFrames.size() > this->NumberOfSlamFrames)
    this->SlamFrames.pop_front();
  // Set output
  output->Initialize();
  output->SetNumberOfBlocks(this->NumberOfSlamFrames);
  for (auto frameId = 0; frameId < this->SlamFrames.size(); ++frameId)
    output->SetBlock(static_cast<unsigned int>(frameId), this->SlamFrames[frameId]);

  // Transform frames to origin
  if (this->DisplayAtOrigin)
    this->TransformFramesToOrigin(output);

  return 1;
}

//----------------------------------------------------------------------------
void vtkMiniSlam::AddSlamPose(const Eigen::Isometry3d& pose)
{
  this->SlamPoses.push_back(pose);
  if (this->SlamPoses.size() > this->NumberOfSlamFrames)
    this->SlamPoses.pop_front();
}

//----------------------------------------------------------------------------
void vtkMiniSlam::TransformFramesToOrigin(vtkMultiBlockDataSet* outputBlocks)
{
  if (this->SlamFrames.size() == 0)
    return;
  // Get transform from estimated pose of slam
  Eigen::Isometry3d refTransform = this->RefFrame == ReferenceFrame::OLDEST_FRAME ?
                                   this->SlamPoses.front() : this->SlamPoses.back();
  refTransform = refTransform.inverse();
  // Convert matrix from column major (Eigen) to row major (VTK)
  refTransform.matrix().transposeInPlace();
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Concatenate(refTransform.matrix().data());

  for (unsigned int idx = 0; idx < outputBlocks->GetNumberOfBlocks(); idx++)
  {
    vtkPolyData* frame = vtkPolyData::SafeDownCast(outputBlocks->GetBlock(idx));
    if (frame)
    {
      vtkSmartPointer<vtkPolyData> newFrame = vtkSmartPointer<vtkPolyData>::New();
      vtkSmartPointer<vtkPoints> newPts = vtkSmartPointer<vtkPoints>::New();
      newFrame->DeepCopy(frame);
      newPts->Allocate(newFrame->GetNumberOfPoints());
      newPts->GetData()->SetName(newFrame->GetPoints()->GetData()->GetName());
      transform->TransformPoints(newFrame->GetPoints(), newPts);
      newFrame->SetPoints(newPts);
      outputBlocks->SetBlock(idx, newFrame);
    }
  }
}