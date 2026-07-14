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

#include "sqSelectTablePortDialog.h"
#include "sqTablePipelineProxyModel.h"

#include <pqActiveObjects.h>
#include <pqApplicationCore.h>
#include <pqFlatTreeView.h>
#include <pqOutputPort.h>
#include <pqPipelineFilter.h>
#include <pqPipelineModel.h>
#include <pqPipelineSource.h>
#include <pqServer.h>
#include <pqServerManagerModel.h>
#include <vtkSMProxy.h>

#include <QDialogButtonBox>
#include <QHeaderView>
#include <QItemSelectionModel>
#include <QLabel>
#include <QPushButton>
#include <QModelIndex>
#include <QString>
#include <QVBoxLayout>

//-----------------------------------------------------------------------------
sqSelectTablePortDialog::sqSelectTablePortDialog(
    pqPipelineFilter *filter, QWidget *parent)
    : Superclass(parent) {
  this->setWindowTitle(tr("Select External Sensor Data"));
  this->setObjectName("SelectExternalSensorDataDialog");
  this->setModal(true);
  this->setSizeGripEnabled(true);

  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(12, 12, 12, 12);

  auto *description =
      new QLabel(tr("Choose a pipeline source producing a vtkTable that "
                    "contains external sensor measurements."),
                 this);
  description->setWordWrap(true);
  layout->addWidget(description);

  // Initialize and insert the pipeline tree view
  this->setupPipelineView();
  layout->addWidget(this->PipelineView);

  this->ButtonBox = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  this->ButtonBox->button(QDialogButtonBox::Ok)->setText(tr("Read Data"));
  this->ButtonBox->button(QDialogButtonBox::Ok)->setEnabled(false);
  layout->addWidget(this->ButtonBox);

  QObject::connect(this->ButtonBox, &QDialogButtonBox::accepted, this,
                   &QDialog::accept);
  QObject::connect(this->ButtonBox, &QDialogButtonBox::rejected, this,
                   &QDialog::reject);

  // Update selected port when user changes selection
  if (auto *selectionModel = this->PipelineView->getSelectionModel()) {
    QObject::connect(selectionModel, &QItemSelectionModel::selectionChanged,
                     this,
                     &sqSelectTablePortDialog::updateSelectedPort);
  }

  // Accept dialog if user double-clicks a valid pipeline entry
  QObject::connect(this->PipelineView, &pqFlatTreeView::activated, this,
                   &sqSelectTablePortDialog::acceptIfValid);

  this->updateSelectedPort();
}

//-----------------------------------------------------------------------------
sqSelectTablePortDialog::~sqSelectTablePortDialog() = default;

//-----------------------------------------------------------------------------
pqOutputPort *sqSelectTablePortDialog::selectedPort() const {
  return this->SelectedPort;
}

//-----------------------------------------------------------------------------
void sqSelectTablePortDialog::setupPipelineView() {
  pqServerManagerModel *smModel =
      pqApplicationCore::instance()->getServerManagerModel();

  // Create a pipeline model and attach it to this dialog
  this->PipelineModel = new pqPipelineModel(*smModel, this);
  this->PipelineModel->setEditable(false);

  // Wrap the pipeline model with a proxy that only exposes vtkTable outputs
  this->FilteredPipelineModel = new sqTablePipelineProxyModel(this);
  this->FilteredPipelineModel->setSourceModel(this->PipelineModel);

  // Create a tree view to display the pipeline
  this->PipelineView = new pqFlatTreeView(this);
  this->PipelineView->setObjectName("ExternalSensorPipelineView");
  this->PipelineView->setModel(this->FilteredPipelineModel);
  this->PipelineView->setSelectionMode(pqFlatTreeView::SingleSelection);
  this->PipelineView->setSelectionBehavior(pqFlatTreeView::SelectRows);
  this->PipelineView->getHeader()->hide();
  this->PipelineView->getHeader()->hideSection(1);

  pqServer *server = pqActiveObjects::instance().activeServer();
  if (server) {
    QModelIndex rootIndex = this->PipelineModel->getIndexFor(server);
    if (this->FilteredPipelineModel)
    {
      rootIndex = this->FilteredPipelineModel->mapFromSource(rootIndex);
    }
    this->PipelineView->setRootIndex(rootIndex);
  }
  // Expand all visible nodes so users can immediately see available table sources
  this->PipelineView->expandAll();
}

//-----------------------------------------------------------------------------
void sqSelectTablePortDialog::updateSelectedPort() {
  this->SelectedPort = nullptr;

  QItemSelectionModel *selectionModel = this->PipelineView->getSelectionModel();
  if (!selectionModel) 
  {
    return;
  }

  const QModelIndexList indexes = selectionModel->selectedIndexes();
  for (const QModelIndex &index : indexes) {
    pqOutputPort *port = this->findPortForIndex(index);
    if (this->isPortAcceptable(port)) {
      this->SelectedPort = port;
      break;
    }
  }

  // Enable or disable the OK button depending on whether a valid port was found
  this->ButtonBox->button(QDialogButtonBox::Ok)
      ->setEnabled(this->SelectedPort != nullptr);
}

//-----------------------------------------------------------------------------
void sqSelectTablePortDialog::acceptIfValid(const QModelIndex &index) {
  pqOutputPort *port = this->findPortForIndex(index);
  if (!this->isPortAcceptable(port)) 
  {
    return;
  }

  if (this->ButtonBox->button(QDialogButtonBox::Ok)->isEnabled()) 
  {
    this->accept();
  }
}

//-----------------------------------------------------------------------------
pqOutputPort *sqSelectTablePortDialog::findPortForIndex(
    const QModelIndex &index) const {
  if (!index.isValid()) 
  {
    return nullptr;
  }

  QModelIndex sourceIndex = index;
  if (this->FilteredPipelineModel)
  {
    sourceIndex = this->FilteredPipelineModel->mapToSource(index);
  }

  if (!sourceIndex.isValid())
  {
    return nullptr;
  }

  QObject *itemObject = this->PipelineModel->getItemFor(sourceIndex);
  if (auto *port = qobject_cast<pqOutputPort *>(itemObject)) 
  {
    return port;
  }

  if (auto *source = qobject_cast<pqPipelineSource *>(itemObject)) 
  {
    return source->getOutputPort(0);
  }

  return nullptr;
}

//-----------------------------------------------------------------------------
bool sqSelectTablePortDialog::isPortAcceptable(
    pqOutputPort *port) const {
  return sqTablePipelineProxyModel::isTablePort(port);
}
