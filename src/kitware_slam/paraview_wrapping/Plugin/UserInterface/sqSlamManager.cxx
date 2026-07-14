//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware SAS
// Author: Timothée Couble (Kitware SAS)
//         Julia Sanchez (Kitware SAS)
// Creation date: 2023-01-23
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

// Local Includes.
#include "sqSlamManager.h"
#include "vtkLoopClosureActors.h"

// Qt Includes.
#include <QAction>
#include <QApplication>
#include <QDebug>
#include <QMessageBox>
#include <QPushButton>
#include <QStyle>

// ParaView Includes.
#include <pqActiveObjects.h>
#include <pqAnimationManager.h>
#include <pqAnimationScene.h>
#include <pqCoreUtilities.h>
#include <pqPVApplicationCore.h>
#include <pqPipelineSource.h>
#include <pqView.h>

// VTK Includes
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkSMProperty.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMProxy.h>
#include <vtkSMSourceProxy.h>
#include <vtkSMViewProxy.h>

#include <iostream>

//-----------------------------------------------------------------------------
sqSlamManager::sqSlamManager(QObject* p /*=0*/)
  : QObject(p)
  , MessageDialog(new QMessageBox(pqCoreUtilities::mainWidget()))
{
  QStyle* appStyle = QApplication::style();
  this->MessageDialog->setWindowTitle("Loop closure detection");
  this->MessageDialog->setText("Loop closure detected! Do you confirm the trajectory got back to the orange sphere?");
  auto *cancelButton = new QPushButton(appStyle->standardIcon(QStyle::SP_DialogDiscardButton), "Discard", this->MessageDialog);
  auto *okButton = new QPushButton(appStyle->standardIcon(QStyle::SP_DialogApplyButton), "Keep it for later", this->MessageDialog);
  auto *applyButton = new QPushButton(appStyle->standardIcon(QStyle::SP_DialogYesButton), "Close loop now", this->MessageDialog);
  this->MessageDialog->addButton(cancelButton, QMessageBox::RejectRole);
  this->MessageDialog->addButton(okButton, QMessageBox::AcceptRole);
  this->MessageDialog->addButton(applyButton, QMessageBox::ApplyRole);
  this->MessageDialog->setDefaultButton(okButton);
  this->MessageDialog->setModal(false);

  this->LoopClosureActors = vtkSmartPointer<vtkLoopClosureActors>::New();
}

//-----------------------------------------------------------------------------
sqSlamManager::~sqSlamManager()
{
  this->MessageDialog->done(sqSlamManager::Destroyed);
}

//-----------------------------------------------------------------------------
void sqSlamManager::onModifiedStateChange()
{
  pqPipelineSource* source = pqActiveObjects::instance().activeSource();
  if (source)
  {
    vtkSMProxy* proxy = source->getProxy();
    if (proxy && proxy->GetProperty("LoopDetectorInfo"))
    {
      vtkSMProperty* loopDetectorProp = proxy->GetProperty("LoopDetectorInfo", true);
      proxy->UpdatePropertyInformation(loopDetectorProp);
      int detector = vtkSMPropertyHelper(loopDetectorProp).GetAsInt();

      // If detector is Teaserpp the loop can be searched automatically
      // Note: if the user deactivates pose graph it doesn't change the loop detector
      // and therefore the plugin manager is still active
      if (detector == 1 || detector == 2)
      {
        QObject::connect(source, QOverload<pqPipelineSource*>::of(&pqPipelineSource::dataUpdated),
          this, &sqSlamManager::onDataUpdated);
      }
      else
      {
        QObject::disconnect(source,
          QOverload<pqPipelineSource*>::of(&pqPipelineSource::dataUpdated), this,
          &sqSlamManager::onDataUpdated);
      }
    }
  }
}

//-----------------------------------------------------------------------------
void sqSlamManager::onDataUpdated(pqPipelineSource* src)
{
  // If the message dialog is already visible do nothing.
  if (this->MessageDialog->isVisible())
  {
    return;
  }

  vtkSMProxy* proxy = src->getProxy();
  if (!proxy)
  {
    return;
  }
  pqAnimationManager* manager = pqPVApplicationCore::instance()->animationManager();
  pqAnimationScene* scene = manager->getActiveScene();
  if (!scene)
  {
    return;
  }

  vtkSMProperty* loopDetectedProp = proxy->GetProperty("LoopDetectedInfo", true);
  proxy->UpdatePropertyInformation(loopDetectedProp);
  bool isDetected = vtkSMPropertyHelper(loopDetectedProp).GetAsInt();

  if (!isDetected)
  {
    return;
  }

  bool isCurrentlyPlaying = manager->animationPlaying();
  if (isCurrentlyPlaying)
  {
    // Pause scene when the message is displayed
    scene->pause();
  }

  pqView* view = pqActiveObjects::instance().activeView();
  vtkSMViewProxy* viewProxy = view->getViewProxy();
  if (!viewProxy)
  {
    qWarning() << "A loop was detected, but cannot find view proxy!";
    return;
  }
  vtkRenderWindow* renderWin = viewProxy->GetRenderWindow();
  if (!renderWin)
  {
    qWarning() << "A loop was detected, but cannot find the render window!";
    return;
  }

  vtkRenderer* renderer = renderWin->GetRenderers()->GetFirstRenderer();
  if (!renderer)
  {
    qWarning() << "A loop was detected, but cannot find the renderer!";
    return;
  }

  // Get loop closure infos from proxy and display it on the view
  vtkSMProperty* detectionSizePro = proxy->GetProperty("LoopDetectionSizeInfo", true);
  vtkSMProperty* loopClosurePositionProp = proxy->GetProperty("LoopClosurePositionInfo", true);
  proxy->UpdatePropertyInformation(detectionSizePro);
  proxy->UpdatePropertyInformation(loopClosurePositionProp);
  double voxelSize = vtkSMPropertyHelper(detectionSizePro).GetAsDouble();
  auto position = vtkSMPropertyHelper(loopClosurePositionProp).GetDoubleArray();
  this->LoopClosureActors->SetSize(voxelSize);
  this->LoopClosureActors->SetCenter(position.data());
  renderer->AddActor(this->LoopClosureActors->GetCenterPointActor());
  renderer->AddActor(this->LoopClosureActors->GetBoundingBoxActor());

  proxy->InvokeCommand("RebuildMaps");

  this->MessageDialog->disconnect();
  QObject::connect(this->MessageDialog, &QMessageBox::finished,
    [=](int retcode)
    {
      switch (retcode)
      {
        case sqSlamManager::Accepted:
          proxy->InvokeCommand("AddLoopDetectionButton");
          break;
        case sqSlamManager::Applied:
          proxy->InvokeCommand("AddLoopDetectionButton");
          proxy->InvokeCommand("Optimize Graph");
          break;
        case sqSlamManager::Rejected:
          vtkSMPropertyHelper(proxy, "LoopDetected").Set(false);
          proxy->UpdateProperty("LoopDetected", true);
          break;
        default:
          break;
      }

      if (renderer)
      {
        renderer->RemoveActor(this->LoopClosureActors->GetCenterPointActor());
        renderer->RemoveActor(this->LoopClosureActors->GetBoundingBoxActor());
      }
      // Unpause the scene if the player was playing before pop-up
      if (isCurrentlyPlaying &&
        (retcode == sqSlamManager::Accepted || retcode == sqSlamManager::Rejected))
      {
        scene->play();
      }
    });

  this->MessageDialog->show();
}
