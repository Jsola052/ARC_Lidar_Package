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

#ifndef sqAppendAnchorPropertyWidget_h
#define sqAppendAnchorPropertyWidget_h

#include "pqPropertyWidget.h"

class vtkSMProperty;
class vtkSMProxy;

class QPushButton;
class QLineEdit;

/**
 * Appends a manual anchor row [id x y z] to a target table property.
 * Reads the id and XYZ from properties named via XML hints.
 *
 * Hints (set on this property):
 * - WriteProperty: target table property name (required)
 * - IdProperty: scalar id source (required; must be > 0)
 * - SeedProperty: 3-tuple x,y,z source (optional; defaults 0,0,0)
 * - IndexPort: validate id <= nPoints-1 on that port (optional)
 * - ButtonText: override button label (optional; default "Append to Table")
 *
 * The target property’s unchecked values are updated immediately; apply via the usual Apply/OK.
 */
class sqAppendAnchorPropertyWidget : public pqPropertyWidget {
  Q_OBJECT
  typedef pqPropertyWidget Superclass;

public:
  sqAppendAnchorPropertyWidget(
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject = nullptr);
  ~sqAppendAnchorPropertyWidget() override;

private Q_SLOTS:
  void appendClicked();

private:
  bool appendAnchor(double id, double x, double y, double z);
  bool readId(double& id) const;
  void readSeedXYZ(double& x, double& y, double& z) const;
  /**
   * Validate current picked index against max index on a configured output port.
   * Returns true when valid or when no port is configured; false when invalid and a warning is shown.
   */
  bool validateIndexForPort(double id) const;

  QPushButton *PickButton = nullptr;
  QString TargetPropertyName;
};

#endif // sqAppendAnchorPropertyWidget_h
