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

#include "sqSeedHandlePropertyWidget.h"

#include "ui_sqSeedHandlePropertyWidget.h"

#include <pqView.h>
#include <pqActiveObjects.h>
#include <pqRenderView.h>

#include <vtkSMNewWidgetRepresentationProxy.h>
#include <vtkSMProperty.h>
#include <vtkSMPropertyGroup.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMProxy.h>
#include <vtkSmartPointer.h>
#include <vtkSMViewProxy.h>
#include <vtkSMRenderViewProxy.h>
#include <vtkCamera.h>

#include <QKeySequence>
#include <QCheckBox>
#include <QString>
#include <QPushButton>

namespace
{
// Adapt a single 3-tuple property into a vtkSMPropertyGroup so it can be
// linked by pqInteractivePropertyWidget. The property is exposed under the
// function name "WorldPosition" to match the HandleWidgetRepresentation API.
vtkSMPropertyGroup* MakeWorldPositionGroup(
  vtkSMProperty* smproperty, vtkSMPropertyGroup** outRaw)
{
  auto* grp = vtkSMPropertyGroup::New();
  grp->AddProperty("WorldPosition", smproperty, smproperty ? smproperty->GetXMLName() : nullptr);
  if (outRaw)
  {
    *outRaw = grp;
  }
  return grp;
}
}

//-----------------------------------------------------------------------------
sqSeedHandlePropertyWidget::sqSeedHandlePropertyWidget(
  vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parent)
  : Superclass(
      "representations", "HandleWidgetRepresentation", smproxy, MakeWorldPositionGroup(smproperty, &this->SMGroupRaw), parent)
{
  this->SMGroup.TakeReference(this->SMGroupRaw);
  Ui::SqSeedHandlePropertyWidget ui;
  ui.setupUi(this);
  this->setShowLabel(false);

  // Ensure the 3D widget is attached to the active view and selected
  if (auto* av = pqActiveObjects::instance().activeView())
  {
    this->setView(av);
  }
  this->select();

  if (vtkSMProperty* worldPosition = smproperty)
  {
    this->addPropertyLink(
      ui.worldPositionX, "text2", SIGNAL(textChangedAndEditingFinished()), worldPosition, 0);
    this->addPropertyLink(
      ui.worldPositionY, "text2", SIGNAL(textChangedAndEditingFinished()), worldPosition, 1);
    this->addPropertyLink(
      ui.worldPositionZ, "text2", SIGNAL(textChangedAndEditingFinished()), worldPosition, 2);
  }

  // Button to center handle on current view center
  if (auto* centerBtn = this->findChild<QPushButton*>(QStringLiteral("useViewCenter")))
  {
    QObject::connect(centerBtn, &QPushButton::clicked, this,
      &sqSeedHandlePropertyWidget::centerOnView);
  }

  if (auto* cb = this->findChild<QCheckBox*>(QStringLiteral("show3DWidget")))
  {
    this->setWidgetVisible(cb->isChecked());
    QObject::connect(cb, &QCheckBox::toggled, this,
      [this](bool on) { this->setWidgetVisible(on); });
  }
  else
  {
    this->setWidgetVisible(true);
  }
}

//-----------------------------------------------------------------------------
sqSeedHandlePropertyWidget::~sqSeedHandlePropertyWidget() = default;

//-----------------------------------------------------------------------------
void sqSeedHandlePropertyWidget::placeWidget(){}

//-----------------------------------------------------------------------------
void sqSeedHandlePropertyWidget::setWorldPosition(double wx, double wy, double wz)
{
  if (auto* wproxy = this->widgetProxy())
  {
    double pos[3] = { wx, wy, wz };
    vtkSMPropertyHelper(wproxy, "WorldPosition").Set(pos, 3);
    wproxy->UpdateVTKObjects();
    Q_EMIT this->changeAvailable();
    this->render();
  }
}

//-----------------------------------------------------------------------------
void sqSeedHandlePropertyWidget::centerOnView()
{
  auto* rview = qobject_cast<pqRenderView*>(this->view());
  vtkSMRenderViewProxy* rvp = rview->getRenderViewProxy();
  vtkCamera* cam = rvp->GetActiveCamera();

  double fp[3] = {0., 0., 0.};
  cam->GetFocalPoint(fp);
  this->setWorldPosition(fp[0], fp[1], fp[2]);
}
