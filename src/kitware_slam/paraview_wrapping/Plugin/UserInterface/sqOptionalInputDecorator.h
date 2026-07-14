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

#ifndef sqOptionalInputDecorator_h
#define sqOptionalInputDecorator_h

#include <pqPropertyWidgetDecorator.h>

#include <QString>

class vtkSMProperty;

/**
 * Decorator used to hide the external sensor file chooser when an optional
 * external sensor input connection is provided on the proxy.
 */
class sqOptionalInputDecorator : public pqPropertyWidgetDecorator
{
  Q_OBJECT
  typedef pqPropertyWidgetDecorator Superclass;

public:
  sqOptionalInputDecorator(vtkPVXMLElement* config, pqPropertyWidget* parentObject);
  ~sqOptionalInputDecorator() override;

  /**
   * By default, return true when external sensor input is present.
   * If inverse="1" is set in XML, the logic is inverted: return true
   * only when external sensor input is absent.
   */
  bool canShowWidget(bool show_advanced) const override;

private:
  bool hasOptionalInput() const;

  QString InputPropertyName;
  bool Inverse = false;
  vtkSMProperty* ObservedObject = nullptr;
  unsigned long ObserverId = 0;
  unsigned long UncheckedObserverId = 0;
};

#endif
