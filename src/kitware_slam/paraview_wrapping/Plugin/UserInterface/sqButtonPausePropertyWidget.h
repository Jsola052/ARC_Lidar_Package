//==============================================================================
// Copyright 2018-2020 Kitware, Inc., Kitware
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

#ifndef sqButtonPausePropertyWidget_h
#define sqButtonPausePropertyWidget_h

#include <pqCommandPropertyWidget.h>

/**
 * A property widget with a push button for invoking a command on a proxy and
 * pause the animation scene if playing.
 *
 * To use this widget for a property add the 'panel_widget="command_pause_button"'
 * to the property's XML.
 */
class sqButtonPausePropertyWidget : public pqCommandPropertyWidget
{
  Q_OBJECT
  typedef pqCommandPropertyWidget Superclass;

public:
  explicit sqButtonPausePropertyWidget(
    vtkSMProxy* proxy, vtkSMProperty* property, QWidget* parent = nullptr);
  ~sqButtonPausePropertyWidget() override;

protected Q_SLOTS:
  void buttonClicked() override;
};

#endif // sqButtonPausePropertyWidget_h
