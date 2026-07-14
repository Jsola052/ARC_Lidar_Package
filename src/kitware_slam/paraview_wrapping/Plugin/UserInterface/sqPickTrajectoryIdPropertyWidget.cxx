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

#include "sqPickTrajectoryIdPropertyWidget.h"

#include <vtkDataObject.h>
#include <vtkPVXMLElement.h>
#include <vtkSMDoubleVectorProperty.h>
#include <vtkSMProperty.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMProxy.h>
#include <vtkSMSourceProxy.h>
#include <vtkSMTooltipSelectionPipeline.h>

#include <pqActiveObjects.h>
#include <pqCoreUtilities.h>
#include <pqOutputPort.h>
#include <pqPVApplicationCore.h>
#include <pqRenderView.h>
#include <pqSelectionManager.h>

#include <QAction>
#include <QApplication>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>

#include <vector>

namespace
{
//-----------------------------------------------------------------------------
// Extract a single array value from a tooltip plain-text block.
// Regex breakdown:
//   (?:^|\n)        Match start of string or a newline (beginning of a line)
//   "  "            Two leading spaces (tooltip indentation)
//   <ArrayName>     Exact array name (escaped to avoid regex metacharacters)
//   ":"             Literal colon separator
//   \s*             Optional whitespace after the colon
//   ([^\n]+)        Capture group: value up to the end of the line
//
// Expects lines formatted like: "  <ArrayName>: <value>" and returns the
// trimmed <value> for the first match; otherwise returns an empty QString.

QString extractArrayValue(const QString& plain, const QString& arrayName)
{
  QRegularExpression rx(
    QString::fromLatin1("(?:^|\n)  %1:\\s*([^\n]+)").arg(QRegularExpression::escape(arrayName)));
  auto m = rx.match(plain);
  if (m.hasMatch())
  {
    return m.captured(1).trimmed();
  }
  return QString();
}
}

//-----------------------------------------------------------------------------
sqPickTrajectoryIdPropertyWidget::sqPickTrajectoryIdPropertyWidget(
  vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject)
  : Superclass(smproxy, parentObject)
{
  auto* mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->PickButton = new QPushButton(tr("Pick (Click)"), this);
  this->Display = new QLineEdit(this);
  this->Display->setReadOnly(true);

  if (auto* hints = smproperty->GetHints())
  {
    auto readTextOrValue = [](vtkPVXMLElement* element) -> QString
    {
      if (!element)
      {
        return QString();
      }
      if (const char* valueAttr = element->GetAttribute("value"))
      {
        return QString::fromUtf8(valueAttr);
      }
      if (const char* content = element->GetCharacterData())
      {
        return QString::fromUtf8(content).trimmed();
      }
      return QString();
    };

    this->InputArrayName = readTextOrValue(hints->FindNestedElementByName("ArrayName"));

    QString writeProperty = readTextOrValue(hints->FindNestedElementByName("WriteProperty"));
    this->TargetPropertyName = writeProperty.toStdString();

    QString appendMode = readTextOrValue(hints->FindNestedElementByName("AppendMode"));
    if (!appendMode.isEmpty())
    {
      bool parseOk = false;
      int appendAsInt = appendMode.toInt(&parseOk);
      if (parseOk)
      {
        this->AppendMode = appendAsInt != 0;
      }
    }

    QString buttonText = readTextOrValue(hints->FindNestedElementByName("ButtonText"));
    if (!buttonText.isEmpty())
    {
      this->PickButton->setText(buttonText);
    }
  }

  if (this->InputArrayName.isEmpty())
  {
    this->Display->setPlaceholderText(tr("Missing ArrayName hint"));
    this->PickButton->setEnabled(false);
  }
  else if (this->TargetPropertyName.empty())
  {
    this->Display->setPlaceholderText(tr("Missing TargetPropertyName hint"));
    this->PickButton->setEnabled(false);
  }
  else
  {
    this->Display->setPlaceholderText(tr("Preview selected value in Render View"));
    QObject::connect(
      this->PickButton, &QPushButton::clicked, this, &sqPickTrajectoryIdPropertyWidget::pickValue);
  }

  mainLayout->addWidget(this->PickButton);
  mainLayout->addWidget(this->Display, 1);

  this->setLayout(mainLayout);
  this->setShowLabel(false);
}

//-----------------------------------------------------------------------------
sqPickTrajectoryIdPropertyWidget::~sqPickTrajectoryIdPropertyWidget() = default;

//-----------------------------------------------------------------------------
void sqPickTrajectoryIdPropertyWidget::pickValue()
{
  if (auto selectionManager = pqPVApplicationCore::instance()->selectionManager())
  {
    selectionManager->clearSelection();
  }

  // Enable interactive select points mode
  this->setActionChecked("actionInteractiveSelectSurfacePoints", true);

  auto selectionManager = pqPVApplicationCore::instance()->selectionManager();

  if (this->SelectionConn)
  {
    QObject::disconnect(this->SelectionConn);
  }
  this->SelectionConn = QObject::connect(selectionManager, &pqSelectionManager::selectionChanged,
    this, &sqPickTrajectoryIdPropertyWidget::onSelectionChanged);
}

//-----------------------------------------------------------------------------
void sqPickTrajectoryIdPropertyWidget::onSelectionChanged(pqOutputPort* port)
{
  // one-shot
  if (this->SelectionConn)
  {
    QObject::disconnect(this->SelectionConn);
  }

  auto renderView = qobject_cast<pqRenderView*>(pqActiveObjects::instance().activeView());
  vtkSMSourceProxy* selectionProxy = port->getSelectionInput();
  auto representation = port->getRepresentation(renderView);
  auto* outputProperty = vtkSMDoubleVectorProperty::SafeDownCast(
    this->proxy()->GetProperty(this->TargetPropertyName.c_str()));
  if (!renderView || !port || !selectionProxy || !representation || !outputProperty)
  {
    this->Display->setText(tr("No selection"));
    this->setActionChecked("actionInteractiveSelectSurfacePoints", false);
    return;
  }

  auto renderViewProxy = renderView->getRenderViewProxy();
  auto pipeline = vtkSMTooltipSelectionPipeline::GetInstance();
  // Pass the representation itself; PreselectionPipeline extracts its Input internally
  pipeline->Show(
    vtkSMSourceProxy::SafeDownCast(representation->getProxy()), selectionProxy, renderViewProxy);

  std::string htmlText, plainText;
  if (!pipeline->GetTooltipInfo(vtkDataObject::FIELD_ASSOCIATION_POINTS, htmlText, plainText))
  {
    this->Display->setText(tr("No point info"));
  }
  else if (port->getPortName() != "Trajectory")
  {
    this->Display->setText(tr("No selection"));
    if (!this->AppendMode)
    {
      outputProperty->SetElement(0, -1);
    }
  }
  else
  {
    // Extract the requested array field
    QString pickedValue =
      extractArrayValue(QString::fromStdString(plainText), this->InputArrayName);
    // Write to a target property specified via XML hints
    bool parseOk = false;
    const double pickedScalar = pickedValue.toDouble(&parseOk);
    if (!pickedValue.isEmpty() && parseOk)
    {
      this->Display->setText(pickedValue);

      if (this->AppendMode)
      {
        vtkSMPropertyHelper propertyHelper(this->proxy(), this->TargetPropertyName.c_str());
        propertyHelper.SetUseUnchecked(true);
        const unsigned int elementCount = propertyHelper.GetNumberOfElements();
        std::vector<double> values;
        values.reserve(elementCount + 1);
        for (unsigned int idx = 0; idx < elementCount; ++idx)
        {
          values.push_back(propertyHelper.GetAsDouble(idx));
        }
        values.push_back(pickedScalar);
        propertyHelper.Set(values.data(), static_cast<unsigned int>(values.size()));
      }
      else
      {
        outputProperty->SetElement(0, pickedScalar);
      }
      this->proxy()->UpdateProperty(this->TargetPropertyName.c_str(), true);
    }
    else
    {
      this->Display->setText(tr("%1 not found").arg(this->InputArrayName));
    }
  }
  this->setActionChecked("actionInteractiveSelectSurfacePoints", false);
}

//-----------------------------------------------------------------------------
void sqPickTrajectoryIdPropertyWidget::setActionChecked(const char* objectName, bool checked)
{
  QWidget* mainWindow = pqCoreUtilities::mainWidget();
  if (!mainWindow)
  {
    return;
  }
  auto actionsList = mainWindow->findChildren<QAction*>(QLatin1String(objectName));
  if (actionsList.isEmpty())
  {
    return;
  }
  QAction* action = actionsList.first();
  if (action->isCheckable())
  {
    if (action->isChecked() != checked && action->isEnabled())
    {
      action->trigger();
    }
  }
  else if (checked)
  {
    if (action->isEnabled())
    {
      action->trigger();
    }
  }
}
