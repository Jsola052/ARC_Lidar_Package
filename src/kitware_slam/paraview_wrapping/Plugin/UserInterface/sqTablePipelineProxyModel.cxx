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

#include "sqTablePipelineProxyModel.h"

#include <pqOutputPort.h>
#include <pqPipelineModel.h>
#include <pqPipelineSource.h>
#include <vtkPVDataInformation.h>

#include <QModelIndex>
#include <QString>

namespace {
bool isTablePortInternal(pqOutputPort *port) 
{
  vtkPVDataInformation *dataInfo = port->getDataInformation();

  const char *datasetType = dataInfo->GetDataSetTypeAsString();
  return datasetType != nullptr && QString(datasetType) == "vtkTable";
}
} // namespace

//-----------------------------------------------------------------------------
sqTablePipelineProxyModel::sqTablePipelineProxyModel(QObject *parent)
    : QSortFilterProxyModel(parent) {}

//-----------------------------------------------------------------------------
bool sqTablePipelineProxyModel::isTablePort(pqOutputPort *port) 
{
  return ::isTablePortInternal(port);
}

//-----------------------------------------------------------------------------
bool sqTablePipelineProxyModel::filterAcceptsRow(
    int sourceRow, const QModelIndex &sourceParent) const 
{
  // Try to cast the source model into a pqPipelineModel
  auto *pipelineModel = qobject_cast<pqPipelineModel *>(this->sourceModel());
  if (!pipelineModel) {
    return QSortFilterProxyModel::filterAcceptsRow(sourceRow, sourceParent);
  }

  // Get the index for the current row in the pipeline model
  QModelIndex index = pipelineModel->index(sourceRow, 0, sourceParent);
  if (!index.isValid()) {
    return false;
  }

  // Check if the current index corresponds to a vtkTable-producing item
  if (this->isIndexAcceptable(index)) {
    return true;
  }

  // Otherwise, check children recursively
  const int childCount = pipelineModel->rowCount(index);
  for (int childRow = 0; childRow < childCount; ++childRow) {
    if (this->filterAcceptsRow(childRow, index)) {
      return true;
    }
  }

  // Neither this row nor its children are acceptable
  return false;
}

//-----------------------------------------------------------------------------
bool sqTablePipelineProxyModel::isIndexAcceptable(
    const QModelIndex &sourceIndex) const 
{
  auto *pipelineModel = qobject_cast<pqPipelineModel *>(this->sourceModel());
  if (!pipelineModel) {
    return true;
  }

  QObject *itemObject = pipelineModel->getItemFor(sourceIndex);

  // If the index corresponds to an output port
  if (auto *port = qobject_cast<pqOutputPort *>(itemObject)) {
    // Check if this port produces a vtkTable
    return ::isTablePortInternal(port);
  }

  // If the index corresponds to a pipeline source
  if (auto *source = qobject_cast<pqPipelineSource *>(itemObject)) {
    const int numberOfPorts = source->getNumberOfOutputPorts();
    for (int portIndex = 0; portIndex < numberOfPorts; ++portIndex) {
      // If any port produces a vtkTable, accept the index
      if (::isTablePortInternal(source->getOutputPort(portIndex))) {
        return true;
      }
    }
  }

  return false;
}
