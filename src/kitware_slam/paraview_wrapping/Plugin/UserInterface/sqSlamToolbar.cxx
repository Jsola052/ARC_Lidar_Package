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

#include "sqSlamToolbar.h"
#include "sqAddSlamReaction.h"
#include "sqInitializeDialogReaction.h"
#include "sqPresetDialogReaction.h"
#include "sqSubsetPropertiesDialog.h"
#include "sqTriggerSlamCommandReaction.h"
#include "vtkSlamFinder.h"

#include "ui_sqSlamToolbar.h"

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QMessageBox>
#include <QScopedPointer>
#include <QStyle>

#include <pqActiveObjects.h>
#include <pqPipelineSource.h>

#include <vtkSMSourceProxy.h>

//-----------------------------------------------------------------------------
struct sqSlamToolbar::sqInternals
{
  sqInternals()
    : Ui(new Ui::sqSlamToolbar)
  {
  }

  QScopedPointer<Ui::sqSlamToolbar> Ui;
};

//-----------------------------------------------------------------------------
sqSlamToolbar::sqSlamToolbar(const QString& title, QWidget* parentW)
  : Superclass(title, parentW)
  , Internals(new sqInternals())
{
  this->constructor();
}

//-----------------------------------------------------------------------------
sqSlamToolbar::sqSlamToolbar(QWidget* parentW)
  : Superclass(parentW)
  , Internals(new sqInternals())
{
  this->setWindowTitle("LiDAR SLAM Toolbar");
  this->constructor();
}

//-----------------------------------------------------------------------------
sqSlamToolbar::~sqSlamToolbar() = default;

//-----------------------------------------------------------------------------
void sqSlamToolbar::constructor()
{
  this->Internals->Ui->setupUi(this);
  auto& ui = this->Internals->Ui;

  QObject::connect(&pqActiveObjects::instance(), &pqActiveObjects::sourceChanged, this,
    &sqSlamToolbar::updateEnableState);

  new sqAddSlamReaction(ui->actionAddSlamFilter);
  new sqPresetDialogReaction(ui->actionSlamPresetSettings);
  new sqInitializeDialogReaction(ui->actionInitialize);
  new sqSubsetPropertiesDialog(
    ui->actionAddExternalSensor, sqSubsetPropertiesDialog::ADD_EXTERNAL_SENSOR);
  new sqSubsetPropertiesDialog(ui->actionOptimizeGraph, sqSubsetPropertiesDialog::OPTIMIZE_GRAPH);
  new sqTriggerSlamCommandReaction(ui->actionResetSlam, "Reset state");

  this->updateEnableState();
}

//-----------------------------------------------------------------------------
void sqSlamToolbar::updateEnableState()
{
  pqPipelineSource* source = pqActiveObjects::instance().activeSource();
  if (source == nullptr)
  {
    this->setEnabled(false);
    return;
  }
  this->setEnabled(true);

  auto& ui = this->Internals->Ui;
  bool isSlamFilter = vtkSlamFinder::isSlamFilter(source->getSourceProxy());

  ui->actionAddSlamFilter->setEnabled(!isSlamFilter);
  ui->actionSlamPresetSettings->setEnabled(isSlamFilter);
  ui->actionInitialize->setEnabled(isSlamFilter);
  ui->actionAddExternalSensor->setEnabled(isSlamFilter);
  ui->actionOptimizeGraph->setEnabled(isSlamFilter);
  ui->actionResetSlam->setEnabled(isSlamFilter);
}
