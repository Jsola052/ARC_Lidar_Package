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

#ifndef sqSelectTablePortDialog_h
#define sqSelectTablePortDialog_h

#include <QDialog>
#include <QPointer>

class pqFlatTreeView;
class pqPipelineFilter;
class pqPipelineModel;
class pqOutputPort;
class QDialogButtonBox;
class QModelIndex;
class sqTablePipelineProxyModel;

/**
 * A dialog for selecting a pipeline output port that produces vtkTable data.
 *
 * This dialog wraps the pipeline model with a proxy model that
 * filters out non-vtkTable outputs. The tree view therefore only shows sources
 * and ports compatible with vtkTable. The OK button is enabled only when a
 * valid vtkTable-producing port is selected.
 */
class sqSelectTablePortDialog : public QDialog {
  Q_OBJECT
  typedef QDialog Superclass;

public:
  sqSelectTablePortDialog(pqPipelineFilter *filter,
                                   QWidget *parent = nullptr);
  ~sqSelectTablePortDialog() override;

  pqOutputPort *selectedPort() const;

private Q_SLOTS:
  /**
   * Update SelectedPort and enable/disable OK button based on user selection
   */
  void updateSelectedPort();
  void acceptIfValid(const QModelIndex &index);

private:
  void setupPipelineView();

  pqOutputPort *findPortForIndex(const QModelIndex &index) const;

  /**
   * Check if a port is acceptable (must be vtkTable)
   */
  bool isPortAcceptable(pqOutputPort *port) const;

  pqPipelineModel *PipelineModel = nullptr;
  sqTablePipelineProxyModel *FilteredPipelineModel = nullptr;
  pqFlatTreeView *PipelineView = nullptr;
  QDialogButtonBox *ButtonBox = nullptr;
  pqOutputPort *SelectedPort = nullptr;
};

#endif
