//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Authors: Guilbert Pierre (Kitware SAS)
//          Laurenson Nick (Kitware SAS)
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

#include "vtkSpinningSensorKeypointExtractor.h"

#include <vtkObjectFactory.h>

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSpinningSensorKeypointExtractor)

//-----------------------------------------------------------------------------
vtkSpinningSensorKeypointExtractor::vtkSpinningSensorKeypointExtractor()
{
  if (this->Mode != LidarSlam::KeypointExtractorMode::DENSE)
    this->Extractor = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();
  else
    this->Extractor = std::make_shared<LidarSlam::DenseSpinningSensorKeypointExtractor>();
}

//-----------------------------------------------------------------------------
void vtkSpinningSensorKeypointExtractor::PrintSelf(std::ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "SpinningSensorKeypointExtractor parameters: " << std::endl;
  vtkIndent paramIndent = indent.GetNextIndent();
  #define PrintParameter(param) os << paramIndent << #param << "\t" << this->Extractor->Get##param() << std::endl;

  PrintParameter(NbThreads)
  PrintParameter(MaxPoints)
  PrintParameter(VoxelResolution)
  PrintParameter(InputSamplingRatio)
  PrintParameter(MinNeighNb)
  PrintParameter(MinNeighRadius)
  PrintParameter(MinDistanceToSensor)
  PrintParameter(AzimuthMin)
  PrintParameter(AzimuthMax)
  PrintParameter(PlaneAngleThreshold)
  PrintParameter(EdgeAngleThreshold)
  PrintParameter(EdgeDepthGapThreshold)
  PrintParameter(EdgeIntensityGapThreshold)
  PrintParameter(EdgeNbGapPoints)
  PrintParameter(NbLaserRings)
  PrintParameter(AzimuthalResolution)
}

//-----------------------------------------------------------------------------
void vtkSpinningSensorKeypointExtractor::SetMode(int mode)
{
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting mode to " << mode);
  LidarSlam::KeypointExtractorMode extractMode = static_cast<LidarSlam::KeypointExtractorMode>(mode);
  if (this->Mode == extractMode)
    return;

  auto oldExtractor = this->Extractor;

  this->Mode = extractMode;
  if (this->Mode != LidarSlam::DENSE)
    this->Extractor = std::make_shared<LidarSlam::SpinningSensorKeypointExtractor>();
  else
    this->Extractor = std::make_shared<LidarSlam::DenseSpinningSensorKeypointExtractor>();

  this->Extractor->SetNbThreads(oldExtractor->GetNbThreads());
  this->Extractor->SetMaxPoints(oldExtractor->GetMaxPoints());
  this->Extractor->SetVoxelResolution(oldExtractor->GetVoxelResolution());
  this->Extractor->SetInputSamplingRatio(oldExtractor->GetInputSamplingRatio());
  this->Extractor->SetMinNeighNb(oldExtractor->GetMinNeighNb());
  this->Extractor->SetMinNeighRadius(oldExtractor->GetMinNeighRadius());
  this->Extractor->SetMinDistanceToSensor(oldExtractor->GetMinDistanceToSensor());
  this->Extractor->SetMaxDistanceToSensor(oldExtractor->GetMaxDistanceToSensor());
  this->Extractor->SetAzimuthMin(oldExtractor->GetAzimuthMin());
  this->Extractor->SetAzimuthMax(oldExtractor->GetAzimuthMax());
  this->Extractor->SetMinBeamSurfaceAngle(oldExtractor->GetMinBeamSurfaceAngle());
  this->Extractor->SetPlaneAngleThreshold(oldExtractor->GetPlaneAngleThreshold());
  this->Extractor->SetEdgeAngleThreshold(oldExtractor->GetEdgeAngleThreshold());
  this->Extractor->SetEdgeDepthGapThreshold(oldExtractor->GetEdgeDepthGapThreshold());
  this->Extractor->SetEdgeIntensityGapThreshold(oldExtractor->GetEdgeIntensityGapThreshold());
  this->Extractor->SetEdgeNbGapPoints(oldExtractor->GetEdgeNbGapPoints());

  this->Modified();
}