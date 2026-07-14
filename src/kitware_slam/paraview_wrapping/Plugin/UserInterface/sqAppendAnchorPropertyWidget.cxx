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

#include "sqAppendAnchorPropertyWidget.h"

#include <pqCoreUtilities.h>
#include <pqPVApplicationCore.h>

#include <vtkPVDataInformation.h>
#include <vtkPVXMLElement.h>
#include <vtkSMDoubleVectorProperty.h>
#include <vtkSMProperty.h>
#include <vtkSMPropertyHelper.h>
#include <vtkSMProxy.h>
#include <vtkSMSourceProxy.h>

#include <QApplication>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>

//-----------------------------------------------------------------------------
sqAppendAnchorPropertyWidget::sqAppendAnchorPropertyWidget(
  vtkSMProxy* smproxy, vtkSMProperty* smproperty, QWidget* parentObject)
  : Superclass(smproxy, parentObject)
{
  auto* mainLayout = new QHBoxLayout;
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->PickButton = new QPushButton(tr("Add to table"), this);

  if (smproperty)
  {
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
      // Target table property
      this->TargetPropertyName = readTextOrValue(hints->FindNestedElementByName("WriteProperty"));
      // Id buffer property
      QString idProperty = readTextOrValue(hints->FindNestedElementByName("IdProperty"));
      if (!idProperty.isEmpty())
      {
        this->setProperty("IdPropertyName", idProperty);
      }
      // Seed property for XYZ
      QString seedProperty = readTextOrValue(hints->FindNestedElementByName("SeedProperty"));
      if (!seedProperty.isEmpty())
      {
        this->setProperty("SeedPropertyName", seedProperty);
      }
      // Optional: target output port for index validation
      auto elem = hints->FindNestedElementByName("IndexPort");
      if (elem)
      {
        bool conversionSucceeded = false;
        int portIdx = readTextOrValue(elem).toInt(&conversionSucceeded);
        if (conversionSucceeded && portIdx >= 0)
        {
          this->setProperty("IndexPort", portIdx);
        }
      }

      // Button label override
      QString buttonText = readTextOrValue(hints->FindNestedElementByName("ButtonText"));
      if (!buttonText.isEmpty())
      {
        this->PickButton->setText(buttonText);
      }
    }
  }

  mainLayout->addWidget(this->PickButton);
  this->setLayout(mainLayout);
  this->setShowLabel(false);

  QObject::connect(
    this->PickButton, &QPushButton::clicked, this, &sqAppendAnchorPropertyWidget::appendClicked);
}

//-----------------------------------------------------------------------------
sqAppendAnchorPropertyWidget::~sqAppendAnchorPropertyWidget() = default;

//-----------------------------------------------------------------------------
// Read id from a 1-element property configured by IdProperty hint
bool sqAppendAnchorPropertyWidget::readId(double& t) const
{
  const QString idPropertyName = this->property("IdPropertyName").toString();
  if (idPropertyName.isEmpty())
  {
    return false;
  }
  if (auto* timeProperty = vtkSMDoubleVectorProperty::SafeDownCast(
        this->proxy()->GetProperty(idPropertyName.toUtf8().data())))
  {
    if (timeProperty->GetNumberOfUncheckedElements() >= 1)
    {
      t = timeProperty->GetUncheckedElement(0);
      return true;
    }
    if (timeProperty->GetNumberOfElements() >= 1)
    {
      t = timeProperty->GetElement(0);
      return true;
    }
  }
  return false;
}

//-----------------------------------------------------------------------------
// Read XYZ from seed property
void sqAppendAnchorPropertyWidget::readSeedXYZ(double& x, double& y, double& z) const
{
  QString seedPropertyName = this->property("SeedPropertyName").toString();
  if (auto* seedProperty = vtkSMDoubleVectorProperty::SafeDownCast(
        this->proxy()->GetProperty(seedPropertyName.toUtf8().data())))
  {
    if (seedProperty->GetNumberOfUncheckedElements() >= 3)
    {
      x = seedProperty->GetUncheckedElement(0);
      y = seedProperty->GetUncheckedElement(1);
      z = seedProperty->GetUncheckedElement(2);
      return;
    }
    if (seedProperty->GetNumberOfElements() >= 3)
    {
      x = seedProperty->GetElement(0);
      y = seedProperty->GetElement(1);
      z = seedProperty->GetElement(2);
      return;
    }
  }
  x = y = z = 0.0;
}

//-----------------------------------------------------------------------------
bool sqAppendAnchorPropertyWidget::appendAnchor(double id, double x, double y, double z)
{
  if (this->TargetPropertyName.isEmpty())
  {
    return false;
  }

  // Update visible UI table
  std::vector<double> fullUiValues;
  vtkSMProperty* visibleProperty =
    this->proxy()->GetProperty(this->TargetPropertyName.toUtf8().data());
  if (visibleProperty)
  {
    vtkSMPropertyHelper propertyHelperUI(this->proxy(), this->TargetPropertyName.toUtf8().data());
    propertyHelperUI.SetUseUnchecked(true);
    unsigned int elementCount = propertyHelperUI.GetNumberOfElements();
    fullUiValues.reserve(elementCount + 4);
    for (unsigned int idx = 0; idx < elementCount; ++idx)
    {
      fullUiValues.push_back(propertyHelperUI.GetAsDouble(idx));
    }
    fullUiValues.push_back(id);
    fullUiValues.push_back(x);
    fullUiValues.push_back(y);
    fullUiValues.push_back(z);
    // Pre-update UI
    propertyHelperUI.Set(fullUiValues.data(), static_cast<unsigned int>(fullUiValues.size()));
    this->proxy()->UpdateProperty(this->TargetPropertyName.toUtf8().data(), true);
  }
  return true;
}

//-----------------------------------------------------------------------------
void sqAppendAnchorPropertyWidget::appendClicked()
{
  double idValue = 0.0;
  if (!this->readId(idValue) || !(idValue > 0.0))
  {
    return;
  }
  double seedX = 0.0;
  double seedY = 0.0;
  double seedZ = 0.0;
  this->readSeedXYZ(seedX, seedY, seedZ);

  // Validate picked index against max index of configured output port
  if (!this->validateIndexForPort(idValue))
  {
    return;
  }
  this->appendAnchor(idValue, seedX, seedY, seedZ);
}

//-----------------------------------------------------------------------------
bool sqAppendAnchorPropertyWidget::validateIndexForPort(double idValue) const
{
  // Only perform validation if XML hint provided an IndexPort
  if (!this->property("IndexPort").isValid())
  {
    return true;
  }

  if (auto* src = vtkSMSourceProxy::SafeDownCast(this->proxy()))
  {
    src->UpdatePipelineInformation();
    int portIndex = this->property("IndexPort").toInt();
    if (auto* info = src->GetDataInformation(portIndex))
    {
      vtkTypeInt64 npts = info->GetNumberOfPoints();
      if (npts <= 0)
      {
        QMessageBox::warning(pqCoreUtilities::mainWidget(), tr("Invalid index"),
          tr("No points are available on output port %1.").arg(portIndex));
        return false;
      }
      const double maxIndex = static_cast<double>(npts - 1);
      if (idValue > maxIndex)
      {
        QMessageBox::warning(pqCoreUtilities::mainWidget(), tr("Invalid index"),
          tr("Picked index %1 exceeds max index %2 on output port %3.")
            .arg(idValue)
            .arg(maxIndex)
            .arg(portIndex));
        return false;
      }
    }
  }
  return true;
}
