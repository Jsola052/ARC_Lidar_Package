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

#ifndef sqSubsetPropertiesDialog_h
#define sqSubsetPropertiesDialog_h

#include <pqReaction.h>
#include <pqProxyWidgetDialog.h>

#include <memory>

class pqPipelineFilter;

class sqSubsetPropertiesDialog : public pqReaction
{
  Q_OBJECT
  typedef pqReaction Superclass;

public:
  enum PropertiesSubset
  {
    ADD_EXTERNAL_SENSOR = 0,
    OPTIMIZE_GRAPH,

    propertiesSubsetSize
  };

  sqSubsetPropertiesDialog(QAction* parent, PropertiesSubset type);
  ~sqSubsetPropertiesDialog() override;

protected:
  /**
   * Called when the action is triggered (e.g the associated button is clicked).
   */
  void onTriggered() override;

private:
  /**
   * Shows the subset properties dialog
   */
  void showSubsetPropertiesDialog(const QString& title, const QStringList& properties);

  /**
   * Adds a button to allow loading external sensor
   * data from an existing pipeline source.
   */
  void setupExternalSensorButton(pqPipelineFilter* filter);

private:
  PropertiesSubset Type;

  std::unique_ptr<pqProxyWidgetDialog> Dialogs[propertiesSubsetSize];
};

#endif
