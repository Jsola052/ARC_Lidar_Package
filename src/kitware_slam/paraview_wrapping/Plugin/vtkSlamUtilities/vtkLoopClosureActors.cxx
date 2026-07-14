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

#include "vtkLoopClosureActors.h"

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkCubeSource.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

vtkStandardNewMacro(vtkLoopClosureActors);

//------------------------------------------------------------------------------
vtkLoopClosureActors::vtkLoopClosureActors()
{
  this->SphereSource = vtkSmartPointer<vtkSphereSource>::New();
  this->SphereSource->SetRadius(0.5);
  vtkNew<vtkPolyDataMapper> centerPointMapper;
  centerPointMapper->SetInputConnection(this->SphereSource->GetOutputPort());

  this->CenterPointActor = vtkSmartPointer<vtkActor>::New();
  this->CenterPointActor->SetMapper(centerPointMapper);
  double darkOrangeColor[3] = { 0.76, 0.18, 0.08 };
  this->CenterPointActor->GetProperty()->SetColor(darkOrangeColor);

  this->CubeSource = vtkSmartPointer<vtkSphereSource>::New();
  vtkNew<vtkPolyDataMapper> boundingBoxMapper;
  boundingBoxMapper->SetInputConnection(this->CubeSource->GetOutputPort());

  this->BoundingBoxActor = vtkSmartPointer<vtkActor>::New();
  this->BoundingBoxActor->SetMapper(boundingBoxMapper);
  vtkProperty* boxProperty = this->BoundingBoxActor->GetProperty();
  double lightOrangeColor[3] = { 0.88, 0.53, 0.26 };
  boxProperty->SetRepresentationToWireframe();
  boxProperty->SetColor(lightOrangeColor);
  boxProperty->SetLineWidth(2);
}

//------------------------------------------------------------------------------
void vtkLoopClosureActors::SetSize(double size)
{
  if (size == 0)
  {
    return;
  }
  this->CubeSource->SetRadius(size);
}

//------------------------------------------------------------------------------
void vtkLoopClosureActors::SetCenter(double* position)
{
  this->SphereSource->SetCenter(position);
  this->CubeSource->SetCenter(position);
}

//------------------------------------------------------------------------------
void vtkLoopClosureActors::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "SphereSource:\n";
  this->SphereSource->PrintSelf(os, indent.GetNextIndent());
  os << indent << "CubeSource:\n";
  this->CubeSource->PrintSelf(os, indent.GetNextIndent());
}
