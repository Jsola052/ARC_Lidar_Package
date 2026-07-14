//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Timothée Couble (Kitware SAS)
//         Julia Sanchez (Kitware SAS)
// Creation date: 2024-01-25
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

#include "vtkSlamFinder.h"

#include <vtkObjectFactory.h>
#include <vtkSMProxy.h>

#include <cstring>

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkSlamFinder);

//----------------------------------------------------------------------------
bool vtkSlamFinder::isSlamFilter(vtkSMProxy* proxy)
{
  if (!proxy)
  {
    return false;
  }
  return vtkSlamFinder::isSlamFilter(proxy->GetXMLGroup(), proxy->GetXMLName());
}

//----------------------------------------------------------------------------
bool vtkSlamFinder::isSlamFilter(const char* smXMLGroup, const char* smXMLName)
{
  return std::strcmp(smXMLGroup, vtkSlamFinder::SLAM_XML_GROUP()) == 0 &&
    std::strcmp(smXMLName, vtkSlamFinder::SLAM_XML_NAME()) == 0;
}

//----------------------------------------------------------------------------
void vtkSlamFinder::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
