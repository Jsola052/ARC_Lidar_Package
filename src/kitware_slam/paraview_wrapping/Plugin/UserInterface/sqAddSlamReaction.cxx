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

#include "sqAddSlamReaction.h"
#include "vtkSlamFinder.h"

#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqObjectBuilder.h>
#include <pqPVApplicationCore.h>
#include <pqPipelineSource.h>
#include <pqServer.h>

//-----------------------------------------------------------------------------
sqAddSlamReaction::sqAddSlamReaction(QAction* parentObject)
  : Superclass(parentObject)
{
}

//-----------------------------------------------------------------------------
void sqAddSlamReaction::onTriggered()
{
  pqPipelineSource* source = pqActiveObjects::instance().activeSource();
  if (source == nullptr)
  {
    return;
  }

  pqServer* server = pqActiveObjects::instance().activeServer();
  pqObjectBuilder* builder = pqApplicationCore::instance()->getObjectBuilder();
  if (server && builder)
  {
    builder->createFilter(vtkSlamFinder::SLAM_XML_GROUP(), vtkSlamFinder::SLAM_XML_NAME(), source);
    pqPVApplicationCore::instance()->triggerApply();
  }
}
