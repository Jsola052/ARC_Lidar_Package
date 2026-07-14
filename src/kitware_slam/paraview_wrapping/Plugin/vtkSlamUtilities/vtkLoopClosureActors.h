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

#ifndef vtkLoopClosureActors_h
#define vtkLoopClosureActors_h

#include <vtkActor.h>
#include <vtkObject.h>
#include <vtkSmartPointer.h>

#include "vtkSlamUtilitiesModule.h" // For export macro

class vtkSphereSource;
class vtkCubeSource;

class VTKSLAMUTILITIES_EXPORT vtkLoopClosureActors : public vtkObject
{
public:
  vtkTypeMacro(vtkLoopClosureActors, vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  static vtkLoopClosureActors* New();

  vtkGetObjectMacro(CenterPointActor, vtkActor);
  vtkGetObjectMacro(BoundingBoxActor, vtkActor);

  void SetSize(double size);
  void SetCenter(double* pos);

protected:
  vtkLoopClosureActors();
  ~vtkLoopClosureActors() override = default;

private:
  vtkLoopClosureActors(const vtkLoopClosureActors&) = delete;
  void operator=(const vtkLoopClosureActors&) = delete;

  vtkSmartPointer<vtkSphereSource> SphereSource;
  vtkSmartPointer<vtkSphereSource> CubeSource;
  vtkSmartPointer<vtkActor> CenterPointActor;
  vtkSmartPointer<vtkActor> BoundingBoxActor;
};

#endif
