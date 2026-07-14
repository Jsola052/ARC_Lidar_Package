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

#include "sqInitializeDialogReaction.h"
#include "vtkSlamFinder.h"

#include <vtkSMProxy.h>
#include <vtkSMSourceProxy.h>

#include <pqActiveObjects.h>
#include <pqCoreUtilities.h>
#include <pqPipelineFilter.h>
#include <pqProxyWidget.h>

#include <QDialog>
#include <QDialogButtonBox>
#include <QPointer>
#include <QPushButton>
#include <QVBoxLayout>

namespace
{
constexpr const char* INITIALIZATION_TITLE = "SLAM Initialization Dialog";
// For some reason it requires properties names WITHOUT spaces
const QStringList INITIALIZATION_PROPERTIES = { "Initialmapfilesprefix", "InitialposeRPY",
  "InitialposeXYZ" };
}

//-----------------------------------------------------------------------------
class sqInitializeDialog : public QDialog
{
  typedef QDialog Superclass;

public:
  sqInitializeDialog(QWidget* widget, vtkSMProxy* proxy)
    : QDialog(widget)
  {
    this->Layout = new QVBoxLayout(this);

    this->ProxyWidget = new pqProxyWidget(proxy, ::INITIALIZATION_PROPERTIES, this);
    this->ProxyWidget->filterWidgets(true);
    this->Layout->addWidget(this->ProxyWidget);

    this->BBox = new QDialogButtonBox({ QDialogButtonBox::Apply, QDialogButtonBox::Close }, this);
    this->Layout->addWidget(this->BBox);

    QPushButton* applyButton = this->BBox->button(QDialogButtonBox::Apply);
    QPushButton* closeButton = this->BBox->button(QDialogButtonBox::Close);

    this->connect(applyButton, &QPushButton::clicked, this, &sqInitializeDialog::apply);
    this->connect(closeButton, &QPushButton::clicked, this, &QDialog::reject);

    applyButton->setEnabled(false);
    this->connect(this->ProxyWidget, &pqProxyWidget::changeAvailable,
      [this]() { this->BBox->button(QDialogButtonBox::Apply)->setEnabled(true); });

    this->setLayout(this->Layout);
    this->setWindowTitle(::INITIALIZATION_TITLE);
  }

protected:
  void done(int status) override
  {
    switch (status)
    {
      case QDialog::Accepted:
        this->apply();
        break;

      case QDialog::Rejected:
        this->ProxyWidget->reset();
        break;
    }
    this->Superclass::done(status);
  }

private Q_SLOTS:
  void apply()
  {
    this->ProxyWidget->apply();
    this->BBox->button(QDialogButtonBox::Apply)->setEnabled(false);
    this->ProxyWidget->proxy()->InvokeCommand("Init");
  }

private:
  QVBoxLayout* Layout;
  QDialogButtonBox* BBox;
  pqProxyWidget* ProxyWidget;
};

//-----------------------------------------------------------------------------
QPointer<sqInitializeDialog> sqInitializeDialogReaction::Dialog = nullptr;

//-----------------------------------------------------------------------------
sqInitializeDialogReaction::sqInitializeDialogReaction(QAction* parentObject)
  : Superclass(parentObject)
{
}

//-----------------------------------------------------------------------------
void sqInitializeDialogReaction::showInitializeDialog()
{
  pqPipelineFilter* filter =
    qobject_cast<pqPipelineFilter*>(pqActiveObjects::instance().activeSource());

  if (filter && vtkSlamFinder::isSlamFilter(filter->getSourceProxy()))
  {
    if (!sqInitializeDialogReaction::Dialog)
    {
      sqInitializeDialogReaction::Dialog =
        new sqInitializeDialog(pqCoreUtilities::mainWidget(), filter->getProxy());
      sqInitializeDialogReaction::Dialog->setObjectName("SlamInitializeDialog");
    }

    sqInitializeDialogReaction::Dialog->show();
    sqInitializeDialogReaction::Dialog->raise();
    sqInitializeDialogReaction::Dialog->activateWindow();
  }
}

//-----------------------------------------------------------------------------
void sqInitializeDialogReaction::onTriggered()
{
  sqInitializeDialogReaction::showInitializeDialog();
}
