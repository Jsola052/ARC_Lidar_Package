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

#ifndef sqSeedHandlePropertyWidget_h
#define sqSeedHandlePropertyWidget_h

#include <pqInteractivePropertyWidget.h>
#include <QPointer>
#include <QCheckBox>
#include <vtkSmartPointer.h>

class pqView;
class vtkSMProxy;
class vtkSMProperty;
class vtkSMPropertyGroup;
class QPushButton;

/**
 * SqSeedHandlePropertyWidget provides an interactive 3D seed handle that
 * writes the handle world position (x, y, z) to a target 3‑tuple property.
 *
 * XML usage (property widget): set panel_widget="slam_seed_handle" on the
 * vtkSMDoubleVectorProperty representing the world-space (x, y, z) position.
 * This widget links its 3 line-edits to the property elements and
 * drives a 3D HandleWidgetRepresentation in the active view.
 *
 * Implementation note: although this is registered as a single-property
 * widget (KIND WIDGET), it adapts to pqInteractivePropertyWidget by creating
 * a synthetic vtkSMPropertyGroup that maps function "WorldPosition" to the
 * bound property so the underlying 3D widget can link as expected.
 *
 * UI controls: the checkbox “Show interactive handle” toggles the 3D widget
 * visibility; the button “Recenter handle” moves the handle to the current
 * render view’s camera focal point.
 */
class sqSeedHandlePropertyWidget : public pqInteractivePropertyWidget {
  Q_OBJECT
  typedef pqInteractivePropertyWidget Superclass;

public:
  explicit sqSeedHandlePropertyWidget(
    vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parent = nullptr);
  ~sqSeedHandlePropertyWidget() override;

protected:
  void placeWidget() override;

private:
  Q_DISABLE_COPY(sqSeedHandlePropertyWidget)
  /**
   * Holds a synthetic property-group that exposes the bound 3-tuple property
   * under function name "WorldPosition" so the base class can link the 3D widget.
   */
  vtkSmartPointer<vtkSMPropertyGroup> SMGroup;

  /**
   * Raw pointer used only to call the base constructor; immediately adopted
   * by SMGroup via TakeReference() to manage lifetime safely.
   */
  vtkSMPropertyGroup* SMGroupRaw = nullptr;

private Q_SLOTS:
  void centerOnView();

private:
  void setWorldPosition(double wx, double wy, double wz);
};

#endif // sqSeedHandlePropertyWidget_h
