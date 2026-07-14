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

#ifndef sqPickTrajectoryIdPropertyWidget_h
#define sqPickTrajectoryIdPropertyWidget_h

#include "pqPropertyWidget.h"

class vtkSMProperty;
class vtkSMProxy;

class QPushButton;
class QLineEdit;
class QMetaObject;

/**
 * Picks a point's array value from Trajectory and writes it to a target property.
 *
 * Hints (set on this property):
 * - ArrayName: point-data array name to read (required)
 * - WriteProperty: target property name (required)
 * - AppendMode: optional integer flag controlling write mode for
 *   target DoubleVectorProperty.
 *   * AppendMode=1: append picked scalar to the end of current values.
 *   * AppendMode missing/0: overwrite element 0 (legacy behavior).
 * - ButtonText: override button label (optional; default "Pick (Click)")
 *
 * The picked scalar is parsed from the render-view tooltip and written to the
 * target property.
 */
class sqPickTrajectoryIdPropertyWidget : public pqPropertyWidget {
  Q_OBJECT
  typedef pqPropertyWidget Superclass;

public:
  sqPickTrajectoryIdPropertyWidget(
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject = nullptr);
  ~sqPickTrajectoryIdPropertyWidget() override;

private Q_SLOTS:
  void pickValue();
  void onSelectionChanged(class pqOutputPort*);

private:
  void setActionChecked(const char* objectName, bool checked);

  QPushButton* PickButton = nullptr;
  QLineEdit* Display = nullptr;
  QMetaObject::Connection SelectionConn;
  QString InputArrayName;
  std::string TargetPropertyName;
  bool AppendMode = false;
};

#endif // sqPickTrajectoryIdPropertyWidget_h
