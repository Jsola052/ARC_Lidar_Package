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

#ifndef vtkSlamFinder_h
#define vtkSlamFinder_h

#include <vtkObject.h>

#include "vtkSlamUtilitiesModule.h" // For export macro

class vtkSMProxy;

class VTKSLAMUTILITIES_EXPORT vtkSlamFinder : public vtkObject
{
public:
  static vtkSlamFinder* New();
  vtkTypeMacro(vtkSlamFinder, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static bool isSlamFilter(vtkSMProxy* proxy);
  static bool isSlamFilter(const char* smXMLGroup, const char* smXMLName);

  static constexpr const char* SLAM_XML_GROUP() { return "filters"; }
  static constexpr const char* SLAM_XML_NAME() { return "SlamOnline"; }

protected:
  vtkSlamFinder() = default;
  ~vtkSlamFinder() override = default;

private:
  vtkSlamFinder(const vtkSlamFinder&) = delete;
  void operator=(const vtkSlamFinder&) = delete;
};

#endif
