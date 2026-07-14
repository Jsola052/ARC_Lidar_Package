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

#include "sqAboutSlamAction.h"
#include "sqAboutSlamDialog.h"

#include <pqCoreUtilities.h>

#include <QAction>

#include <iostream>

//-----------------------------------------------------------------------------
sqAboutSlamAction::sqAboutSlamAction(QObject* p)
  : QActionGroup(p)
{
  QAction* aboutSlamAction = new QAction(this);
  aboutSlamAction->setText("About SLAM plugin...");

  this->connect(
    aboutSlamAction, &QAction::triggered, this, &sqAboutSlamAction::showAboutSlamDialog);

  this->addAction(aboutSlamAction);
}

//-----------------------------------------------------------------------------
void sqAboutSlamAction::showAboutSlamDialog()
{
  sqAboutSlamDialog* aboutDialog = new sqAboutSlamDialog(pqCoreUtilities::mainWidget());
  aboutDialog->setObjectName("SlamAboutDialog");
  aboutDialog->exec();
}
