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

#include "sqSubsetPropertiesDialog.h"
#include "sqSelectTablePortDialog.h"
#include "vtkSlamFinder.h"

#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqCoreUtilities.h>
#include <pqOutputPort.h>
#include <pqPipelineFilter.h>
#include <pqPipelineSource.h>
#include <pqProxyWidgetDialog.h>
#include <vtkSMInputProperty.h>
#include <vtkSMSourceProxy.h>

#include <QHBoxLayout>
#include <QPointer>
#include <QPushButton>
#include <QStringList>

namespace
{
constexpr const char* ADD_EXTERNAL_SENSOR_TITLE = "Add External Sensor Dialog";
const QStringList ADD_EXTERNAL_SENSOR_PROPERTIES = { "External sensors data file", "Calibrate",
  "Ego-Motion mode", "Undistortion mode", "Wheel odom weight", "Wheel odometry relative mode",
  "IMU gravity weight", "IMU weight", "Gravity", "Pose weight", "Imu reset threshold", "Imu update",
  "Time synchronization" };

constexpr const char* OPTIMIZE_GRAPH_TITLE = "Optimize Graph Dialog";
const QStringList OPTIMIZE_GRAPH_PROPERTIES = { "Optimize Graph", "Logging timeout",
  "Use loop closure constraint", "Use GPS constraint", "Use external pose constraint",
  "Enable external pose offset", "Use manual position constraint", "Fixed anchor points",
  "Manual position anchors", "Pick Point Id Tools", "Append Manual Anchor",
  "Define anchor points", "Pick Pose Anchor Tools", "Seed position", "Loop closure detector",
  "Load loop indices", "Detect Loop" };
}

//-----------------------------------------------------------------------------
sqSubsetPropertiesDialog::sqSubsetPropertiesDialog(QAction* parentObject, PropertiesSubset type)
  : Superclass(parentObject)
  , Type(type)
{
}

//-----------------------------------------------------------------------------
sqSubsetPropertiesDialog::~sqSubsetPropertiesDialog() = default;

//-----------------------------------------------------------------------------
void sqSubsetPropertiesDialog::showSubsetPropertiesDialog(
  const QString& title, const QStringList& properties)
{
  pqPipelineFilter* filter =
    qobject_cast<pqPipelineFilter*>(pqActiveObjects::instance().activeSource());

  if (filter && vtkSlamFinder::isSlamFilter(filter->getSourceProxy()))
  {
    if (!this->Dialogs[this->Type])
    {
      this->Dialogs[this->Type] = std::make_unique<pqProxyWidgetDialog>(
        filter->getProxy(), properties, pqCoreUtilities::mainWidget());
      this->Dialogs[this->Type]->setWindowTitle(title);
    }

    if (this->Type == PropertiesSubset::ADD_EXTERNAL_SENSOR)
    {
      this->setupExternalSensorButton(filter);
    }

    this->Dialogs[this->Type]->show();
    this->Dialogs[this->Type]->raise();
    this->Dialogs[this->Type]->activateWindow();
  }
}

//-----------------------------------------------------------------------------
void sqSubsetPropertiesDialog::setupExternalSensorButton(pqPipelineFilter* filter)
{
  // Get proxy widget dialog for this subset type
  pqProxyWidgetDialog* dialog = this->Dialogs[this->Type].get();
  if (!dialog)
  {
    return;
  }

  // Find the panel that contains the standard Apply/Reset buttons
  QWidget* buttonPanel = dialog->findChild<QWidget*>("widget");
  if (!buttonPanel)
  {
    return;
  }

  // Either reuse the existing button or create a new one if missing.
  auto* buttonLayout = qobject_cast<QHBoxLayout*>(buttonPanel->layout());
  auto* loadButton = buttonPanel->findChild<QPushButton*>("ReadExternalSensorDataButton");
  if (!loadButton)
  {
    // Create the "Read External Sensor Data" button and insert it next to Apply/Reset.
    loadButton = new QPushButton(tr("Read External Sensor Data"), buttonPanel);
    loadButton->setObjectName("ReadExternalSensorDataButton");
    loadButton->setToolTip(
      tr("Import measurements and calibration from an existing pipeline source."));
    buttonLayout->insertWidget(2, loadButton);
  }
  else
  {
    loadButton->disconnect();
  }

  // Connect the button to open a selection dialog when clicked.
  QPointer<pqPipelineFilter> filterPtr(filter);
  QObject::connect(loadButton, &QPushButton::clicked, dialog,
    [filterPtr, dialog]()
    {
      if (!filterPtr)
      {
        return;
      }

      sqSelectTablePortDialog selectionDialog(filterPtr, dialog);
      if (selectionDialog.exec() != QDialog::Accepted)
      {
        return;
      }

      pqOutputPort* selectedPort = selectionDialog.selectedPort();
      if (!selectedPort)
      {
        return;
      }

      // Get the "External Sensor Data" input property on the SLAM filter proxy.
      vtkSMInputProperty* inputProperty = vtkSMInputProperty::SafeDownCast(
        filterPtr->getProxy()->GetProperty("External Sensor Data"));

      std::vector<vtkSMProxy*> proxies;
      std::vector<unsigned int> ports;
      proxies.push_back(selectedPort->getSource()->getProxy());
      ports.push_back(selectedPort->getPortNumber());

      // Update the input property with the selected source,
      inputProperty->SetProxies(
        static_cast<unsigned int>(proxies.size()), proxies.data(), ports.data());

      // Push the change to the server manager,
      filterPtr->getProxy()->UpdateVTKObjects();

      // Request a pipeline update and refresh all views.
      filterPtr->updatePipeline();
      pqApplicationCore::instance()->render();
    });
}

//-----------------------------------------------------------------------------
void sqSubsetPropertiesDialog::onTriggered()
{
  switch (this->Type)
  {
    case PropertiesSubset::ADD_EXTERNAL_SENSOR:
      this->showSubsetPropertiesDialog(
        ::ADD_EXTERNAL_SENSOR_TITLE, ::ADD_EXTERNAL_SENSOR_PROPERTIES);
      break;

    case PropertiesSubset::OPTIMIZE_GRAPH:
      this->showSubsetPropertiesDialog(::OPTIMIZE_GRAPH_TITLE, ::OPTIMIZE_GRAPH_PROPERTIES);
      break;

    default:
      qWarning("Invalid properties subset type.");
      break;
  }
}
