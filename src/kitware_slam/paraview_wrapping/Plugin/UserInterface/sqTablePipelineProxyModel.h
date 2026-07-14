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

#ifndef sqTablePipelineProxyModel_h
#define sqTablePipelineProxyModel_h

#include <QSortFilterProxyModel>

class pqOutputPort;
class QModelIndex;

/**
 * Helper used to expose only vtkTable-producing pipeline entries.
 */
class sqTablePipelineProxyModel : public QSortFilterProxyModel
{
  Q_OBJECT

public:
  explicit sqTablePipelineProxyModel(QObject* parent = nullptr);

  /**
   * Check if the given output port produces vtkTable data
   */
  static bool isTablePort(pqOutputPort* port);

protected:
  /**
   * Decides whether a row should be shown in the tree view
   */
  bool filterAcceptsRow(int sourceRow, const QModelIndex& sourceParent) const override;

private:
  /**
   * Checks whether a single item (an output port or a source) directly produces a vtkTable
   */
  bool isIndexAcceptable(const QModelIndex& sourceIndex) const;
};

#endif
