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

#ifndef sqTriggerSlamCommandReaction_h
#define sqTriggerSlamCommandReaction_h

#include <pqReaction.h>

class sqPresetDialog;

class sqTriggerSlamCommandReaction : public pqReaction
{
  Q_OBJECT
  typedef pqReaction Superclass;

public:
  sqTriggerSlamCommandReaction(QAction* parent, const char* commandName);

protected:
  /**
   * Called when the action is triggered (e.g the associated button is clicked).
   */
  void onTriggered() override;

private:
  Q_DISABLE_COPY(sqTriggerSlamCommandReaction)

  const char* CommandName;
};

#endif
