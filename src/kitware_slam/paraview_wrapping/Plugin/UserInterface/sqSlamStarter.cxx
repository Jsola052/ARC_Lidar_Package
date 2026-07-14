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
#include "sqSlamStarter.h"
#include "sqSlamManager.h"
#include "vtkSlamFinder.h"

// Qt Includes.
#include <QApplication>
#include <QDebug>

// ParaView Includes.
#include <pqApplicationCore.h>
#include <pqPipelineSource.h>
#include <pqServerManagerModel.h>

// VTK Includes.
#include <vtkSMSourceProxy.h>

#include <iostream>

//-----------------------------------------------------------------------------
sqSlamStarter::sqSlamStarter(QObject* p /*=0*/)
  : QObject(p)
{
}

//-----------------------------------------------------------------------------
sqSlamStarter::~sqSlamStarter() = default;

//-----------------------------------------------------------------------------
void sqSlamStarter::onStartup()
{
  pqServerManagerModel* smModel = pqApplicationCore::instance()->getServerManagerModel();
  this->connect(
    smModel, SIGNAL(sourceAdded(pqPipelineSource*)), SLOT(onSourceAdded(pqPipelineSource*)));
  this->connect(
    smModel, SIGNAL(preSourceRemoved(pqPipelineSource*)), SLOT(onSourceRemoved(pqPipelineSource*)));
}

//-----------------------------------------------------------------------------
void sqSlamStarter::onShutdown()
{
  this->SlamFiltersMap.clear();
}

//-----------------------------------------------------------------------------
void sqSlamStarter::onSourceAdded(pqPipelineSource* src)
{
  vtkSMSourceProxy* sourceProxy = src->getSourceProxy();
  if (vtkSlamFinder::isSlamFilter(sourceProxy))
  {
    QPointer<sqSlamManager> manager(new sqSlamManager());
    QObject::connect(
      src, &pqProxy::modifiedStateChanged, manager, &sqSlamManager::onModifiedStateChange);
    this->SlamFiltersMap.insert(sourceProxy->GetGlobalID(), manager);
  }
}

//-----------------------------------------------------------------------------
void sqSlamStarter::onSourceRemoved(pqPipelineSource* src)
{
  vtkSMSourceProxy* sourceProxy = src->getSourceProxy();
  if (vtkSlamFinder::isSlamFilter(sourceProxy))
  {
    auto it = this->SlamFiltersMap.find(sourceProxy->GetGlobalID());
    if (it != this->SlamFiltersMap.end())
    {
      delete it.value();
      this->SlamFiltersMap.erase(it);
    }
  }
}
