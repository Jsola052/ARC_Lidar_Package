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

#ifndef sqPresetDialog_h
#define sqPresetDialog_h

#include <QDialog>

#include <memory>

class sqPresetDialog : public QDialog
{
  Q_OBJECT
  typedef QDialog Superclass;

public:
  sqPresetDialog(QWidget* Parent = nullptr);
  ~sqPresetDialog() override;

private Q_SLOTS:
  void updateUIState();
  void onLoadFile();
  void onSaveCurrent();
  void onRemoveSelected();
  void onRemoveAll();
  void onApplySelected();

private:
  void populateTree();

private:
  struct sqInternals;
  std::unique_ptr<sqInternals> Internals;
};

#endif
