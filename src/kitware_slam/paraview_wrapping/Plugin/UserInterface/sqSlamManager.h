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

#ifndef sqSlamManager_h
#define sqSlamManager_h

#include <QDialog>
#include <QObject>

#include <vtkEventQtSlotConnect.h>
#include <vtkSmartPointer.h>

class pqPipelineSource;
class QMessageBox;
class vtkLoopClosureActors;

class sqSlamManager : public QObject
{
  Q_OBJECT
  typedef QObject Superclass;

  enum MessageBox
  {
    Rejected = QDialog::Rejected,
    Accepted = QDialog::Accepted,
    Applied = QDialog::Accepted + 1,
    Destroyed
  };

public:
  sqSlamManager(QObject* p = nullptr);
  ~sqSlamManager();

public Q_SLOTS:
  void onModifiedStateChange();

private Q_SLOTS:
  void onDataUpdated(pqPipelineSource* src);

private:
  Q_DISABLE_COPY(sqSlamManager);

  QMessageBox* MessageDialog;
  vtkSmartPointer<vtkLoopClosureActors> LoopClosureActors;
};

#endif
