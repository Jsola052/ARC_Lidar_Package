//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
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

#include "sqOptionalInputDecorator.h"

#include <pqCoreUtilities.h>
#include <pqPropertyWidget.h>

#include <vtkCommand.h>
#include <vtkPVXMLElement.h>
#include <vtkSMInputProperty.h>
#include <vtkSMProperty.h>
#include <vtkSMProxy.h>

#include <QDebug>

//-----------------------------------------------------------------------------
sqOptionalInputDecorator::sqOptionalInputDecorator(
  vtkPVXMLElement* config, pqPropertyWidget* parentObject)
  : Superclass(config, parentObject)
{
  const char* propName = config && config->GetAttribute("input_property")
    ? config->GetAttribute("input_property")
    : "External Sensor Data";
  this->InputPropertyName = QString::fromUtf8(propName);

  // Optional inverse attribute to invert logic
  if (config && config->GetAttribute("inverse"))
  {
    const char* inv = config->GetAttribute("inverse");
    this->Inverse = (QString::fromUtf8(inv) == QLatin1String("1"));
  }

  vtkSMProxy* proxy = parentObject ? parentObject->proxy() : nullptr;
  vtkSMProperty* prop = proxy ? proxy->GetProperty(this->InputPropertyName.toUtf8().data()) : nullptr;
  vtkSMInputProperty* inputProp = vtkSMInputProperty::SafeDownCast(prop);
  if (!inputProp)
  {
    qWarning() << "sqOptionalInputDecorator could not find input property"
               << this->InputPropertyName << "on proxy";
    return;
  }

  this->ObservedObject = inputProp;
  this->ObserverId = pqCoreUtilities::connect(
    inputProp, vtkCommand::ModifiedEvent, this, SIGNAL(visibilityChanged()));
  this->UncheckedObserverId = pqCoreUtilities::connect(
    inputProp, vtkCommand::UncheckedPropertyModifiedEvent, this, SIGNAL(visibilityChanged()));

  emit this->visibilityChanged();
}

//-----------------------------------------------------------------------------
sqOptionalInputDecorator::~sqOptionalInputDecorator()
{
  if (this->ObservedObject && this->ObserverId)
  {
    this->ObservedObject->RemoveObserver(this->ObserverId);
  }
  if (this->ObservedObject && this->UncheckedObserverId)
  {
    this->ObservedObject->RemoveObserver(this->UncheckedObserverId);
  }
}

//-----------------------------------------------------------------------------
bool sqOptionalInputDecorator::canShowWidget(bool show_advanced) const
{
  const bool hasInput = this->hasOptionalInput();
  // Default behavior (Inverse == false): show when input is present
  // Inverse == true: show only when input is absent
  bool condition = this->Inverse ? !hasInput : hasInput;
  if (!condition)
  {
    return false;
  }
  return this->Superclass::canShowWidget(show_advanced);
}

//-----------------------------------------------------------------------------
bool sqOptionalInputDecorator::hasOptionalInput() const
{
  pqPropertyWidget* parentObj = this->parentWidget();
  vtkSMProxy* proxy = parentObj ? parentObj->proxy() : nullptr;
  vtkSMInputProperty* inputProp = proxy
    ? vtkSMInputProperty::SafeDownCast(proxy->GetProperty(this->InputPropertyName.toUtf8().data()))
    : nullptr;

  if (!inputProp)
  {
    return false;
  }

  unsigned int numberOfConnections = inputProp->GetNumberOfProxies();
  for (unsigned int cc = 0; cc < numberOfConnections; ++cc)
  {
    if (inputProp->GetProxy(cc) != nullptr)
    {
      return true;
    }
  }
  return false;
}
