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

// To enable if only a subset of properties must be saved.
// Should be only used to generate pre-defined presets.
#define SAVE_PROPERTIES_SUBSET 0

#include "sqPresetDialog.h"
#include "vtkSlamFinder.h"

#include "ui_sqPresetDialog.h"

#include <vtkLogger.h>
#include <vtkPVXMLElement.h>
#include <vtkPVXMLParser.h>
#include <vtkSMNamedPropertyIterator.h>
#include <vtkSMProperty.h>
#include <vtkSMSourceProxy.h>
#include <vtksys/FStream.hxx>

#include <pqActiveObjects.h>
#include <pqCoreUtilities.h>
#include <pqFileDialog.h>
#include <pqPVApplicationCore.h>
#include <pqPipelineFilter.h>
#include <pqServer.h>

#include <QByteArray>
#include <QDebug>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QInputDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QScopedPointer>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <string>
#include <vector>

namespace
{
#if SAVE_PROPERTIES_SUBSET
const std::vector<std::string> ENVIRONMENT_PROPERTIES = {
  "Use edges",
  "Use intensity edges",
  "Use planes",
  "Ego-Motion mode",
  "Undistortion mode",
  "Min laser beam to surface angle",
  "Plane threshold angle",
  "Edge threshold angle",
  "Edge min depth gap",
  "Edge min nb of missing points",
  "Edge min intensity gap",
  "Maximum keypoints number",
  "Edge nb min filtered neighbors",
  "Edge max model error",
  "Plane nb of neighbors",
  "Planarity threshold",
  "Final saturation distance",
  "Submap extraction mode",
  "Keyframe distance threshold",
  "Keyframe angle threshold",
  "Edges map resolution",
  "Planes map resolution",
  "Rolling grid dimension",
  "Rolling grid resolution",
  "Min number of frames per voxel",
};
const std::vector<std::string> LIDAR_MODEL_PROPERTIES = {
  "Mode",
  "Min neighbors nb",
  "Min distance to sensor",
  "Ratio of points",
  "Max neighbors distance",
  "Edge nb of neighbors",
};
#endif
}

//-----------------------------------------------------------------------------
struct sqPresetDialog::sqInternals
{
  sqInternals()
    : Ui(new Ui::PresetDialog)
  {
  }

  enum PresetType
  {
    ENVIRONMENT = 0,
    LIDAR_MODEL,
    LIDAR_SUPPORT,
    USER_CUSTOM,

    presetTypeSize
  };

  static constexpr int PRESET_COLUMN() { return 0; }
  static constexpr int PRESET_PATH_ROLE() { return Qt::UserRole + 1; }

  //-----------------------------------------------------------------------------
  static QString CUSTOM_PRESET_DIR()
  {
    return pqCoreUtilities::getParaViewUserDirectory() + "/SlamPresets";
  }

  //-----------------------------------------------------------------------------
  static void addItem(QTreeWidgetItem* parent, QString text, QString filePath)
  {
    QTreeWidgetItem* item = new QTreeWidgetItem(parent);
    item->setText(sqInternals::PRESET_COLUMN(), text);
    item->setData(sqInternals::PRESET_COLUMN(), sqInternals::PRESET_PATH_ROLE(), filePath);
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
  }

  //-----------------------------------------------------------------------------
  static void addItemsFromDirectory(QString path, QTreeWidgetItem* parent)
  {
    QDirIterator it(path, QDirIterator::Subdirectories);
    while (it.hasNext())
    {
      QString filePath = it.next();
      if (it.fileInfo().completeSuffix() == "xml")
      {
        sqInternals::addItem(parent, it.fileInfo().baseName(), filePath);
      }
    }
  }

  //-----------------------------------------------------------------------------
  static void tryCreatePresetDir()
  {
    QDir directory(sqInternals::CUSTOM_PRESET_DIR());
    if (!directory.exists())
    {
      directory.mkpath(".");
    }
  }

  //-----------------------------------------------------------------------------
  void handleMultiSelection()
  {
    auto items = this->Ui->presetTree->selectedItems();
    if (items.size() <= 1)
    {
      return;
    }

    bool doDeselect[sqInternals::presetTypeSize] = { false };
    // Handle custom preset case
    if (this->isCustomModelSelected())
    {
      // Keep the last category selected
      if (!this->isItemOfType(sqInternals::USER_CUSTOM, items.first()))
      {
        doDeselect[sqInternals::ENVIRONMENT] = true;
        doDeselect[sqInternals::LIDAR_MODEL] = true;
        doDeselect[sqInternals::LIDAR_SUPPORT] = true;
      }
      else
      {
        auto hasCustomModel = [this](QTreeWidgetItem* item) {
          return this->isItemOfType(sqInternals::USER_CUSTOM, item);
        };
        bool allCustom = std::all_of(items.cbegin(), items.cend(), hasCustomModel);
        doDeselect[sqInternals::USER_CUSTOM] = !allCustom;
      }
    }

    auto changeSelection = [this, &doDeselect](QTreeWidgetItem* selected) {
      for (unsigned int typeIdx = 0; typeIdx < sqInternals::presetTypeSize; typeIdx++)
      {
        auto type = static_cast<sqInternals::PresetType>(typeIdx);
        if (this->isItemOfType(type, selected))
        {
          if (doDeselect[typeIdx])
          {
            this->Ui->presetTree->setCurrentItem(selected, 0, QItemSelectionModel::Deselect);
          }
          doDeselect[typeIdx] = true;
        }
      }
    };

    this->Ui->presetTree->blockSignals(true);
    std::for_each(items.rbegin(), items.rend(), changeSelection);
    // Prevent bug where the selection focus is switched to last deselected
    this->Ui->presetTree->setCurrentItem(this->Ui->presetTree->selectedItems().last(), 0, QItemSelectionModel::Select);
    this->Ui->presetTree->blockSignals(false);
  }

  //-----------------------------------------------------------------------------
  bool isCustomModelSelected()
  {
    QList<QTreeWidgetItem*> items = this->Ui->presetTree->selectedItems();

    auto hasCustomModel = [this](QTreeWidgetItem* item) {
      return this->isItemOfType(sqInternals::USER_CUSTOM, item);
    };
    auto found = std::find_if(items.cbegin(), items.cend(), hasCustomModel);
    return found != items.cend();
  }

  //-----------------------------------------------------------------------------
  bool isItemOfType(sqInternals::PresetType type, QTreeWidgetItem* item)
  {
    return this->getTreeMainItem(type)->indexOfChild(item) != -1;
  }

  //-----------------------------------------------------------------------------
  QTreeWidgetItem* getTreeMainItem(PresetType type)
  {
    return this->Ui->presetTree->topLevelItem(type);
  }

  //-----------------------------------------------------------------------------
  void checkPresetValidity(vtkSMSourceProxy* proxy, vtkPVXMLElement* element)
  {
    unsigned int numElems = element->GetNumberOfNestedElements();
    for (unsigned int i = 0; i < numElems; i++)
    {
      vtkPVXMLElement* currentElement = element->GetNestedElement(i);
      const char* elementName = currentElement->GetName();
      if (elementName && strcmp(elementName, "Property") == 0)
      {
        const char* name = currentElement->GetAttribute("name");
        if (!name)
        {
          vtkLog(ERROR, "Cannot load property without a name.");
          continue;
        }
        vtkSMProperty* property = proxy->GetProperty(name);
        if (!property)
        {
          vtkLog(ERROR, << "Property " << name << " from preset does not exist in SLAM.");
        }
      }
    }
  }

  QScopedPointer<Ui::PresetDialog> Ui;
};

//-----------------------------------------------------------------------------
sqPresetDialog::sqPresetDialog(QWidget* Parent)
  : Superclass(Parent)
  , Internals(new sqInternals())

{
  this->Internals->Ui->setupUi(this);

  this->connect(&pqActiveObjects::instance(), &pqActiveObjects::sourceChanged, this,
    &sqPresetDialog::updateUIState);
  this->connect(this->Internals->Ui->presetTree, &QTreeWidget::itemSelectionChanged, this,
    &sqPresetDialog::updateUIState);

  QPushButton* applyButton = this->Internals->Ui->buttonBox->button(QDialogButtonBox::Apply);
  QPushButton* closeButton = this->Internals->Ui->buttonBox->button(QDialogButtonBox::Close);

  this->connect(
    this->Internals->Ui->addPreset, &QPushButton::clicked, this, &sqPresetDialog::onLoadFile);
  this->connect(this->Internals->Ui->saveCurrentPreset, &QPushButton::clicked, this,
    &sqPresetDialog::onSaveCurrent);
  this->connect(this->Internals->Ui->removePreset, &QPushButton::clicked, this,
    &sqPresetDialog::onRemoveSelected);
  this->connect(
    this->Internals->Ui->removeAll, &QPushButton::clicked, this, &sqPresetDialog::onRemoveAll);
  this->connect(applyButton, &QPushButton::clicked, this, &sqPresetDialog::onApplySelected);
  this->connect(closeButton, &QPushButton::clicked, this, &sqPresetDialog::close);

  // Trigger ParaView apply mechanism when applying preset
  this->connect(applyButton, &QPushButton::clicked, pqPVApplicationCore::instance(),
    &pqPVApplicationCore::triggerApply);

  this->populateTree();
}

//-----------------------------------------------------------------------------
sqPresetDialog::~sqPresetDialog() = default;

//----------------------------------------------------------------------------
void sqPresetDialog::populateTree()
{
  auto environmentItems = this->Internals->getTreeMainItem(sqInternals::ENVIRONMENT);
  auto lidarModelItems = this->Internals->getTreeMainItem(sqInternals::LIDAR_MODEL);
  auto lidarSupportItems = this->Internals->getTreeMainItem(sqInternals::LIDAR_SUPPORT);
  auto customItems = this->Internals->getTreeMainItem(sqInternals::USER_CUSTOM);
  customItems->setExpanded(true);

  sqInternals::addItemsFromDirectory(":/sqResources/Presets/Environment", environmentItems);
  sqInternals::addItemsFromDirectory(":/sqResources/Presets/LidarModel", lidarModelItems);
  sqInternals::addItemsFromDirectory(":/sqResources/Presets/LidarSupport", lidarSupportItems);
  sqInternals::addItemsFromDirectory(sqInternals::CUSTOM_PRESET_DIR(), customItems);

  this->Internals->Ui->presetTree->setCurrentItem(nullptr);
  this->updateUIState();
}

//-----------------------------------------------------------------------------
void sqPresetDialog::onLoadFile()
{
  pqServer* server = pqActiveObjects::instance().activeServer();
  pqFileDialog fileDialog(server, pqCoreUtilities::mainWidget(),
    QString("Open an existing SLAM preset file:"), QString(), QString("XML Files (*.xml)"), false);
  fileDialog.setObjectName("FileOpenDialog");
  fileDialog.setFileMode(pqFileDialog::ExistingFile);
  if (fileDialog.exec() != QDialog::Accepted)
  {
    return;
  }
  QString srcFilename = fileDialog.getSelectedFiles()[0];
  QFileInfo info(srcFilename);
  QString destFilename = sqInternals::CUSTOM_PRESET_DIR() + "/" + info.fileName();
  QFile srcFile(srcFilename);

  sqInternals::tryCreatePresetDir();
  if (!srcFile.copy(destFilename))
  {
    qCritical() << "Failed to copy " + info.fileName() + ".";
    return;
  }

  auto parent = this->Internals->getTreeMainItem(sqInternals::USER_CUSTOM);
  sqInternals::addItem(parent, info.baseName(), destFilename);
  this->updateUIState();
}

//-----------------------------------------------------------------------------
void sqPresetDialog::onSaveCurrent()
{
  pqPipelineFilter* filter =
    qobject_cast<pqPipelineFilter*>(pqActiveObjects::instance().activeSource());
  if (filter == nullptr)
  {
    return;
  }

  QString presetName =
    QInputDialog::getText(this, "Preset Name", "Please enter a name for your new SLAM preset:");
  if (presetName.isEmpty())
  {
    return;
  }
  auto parent = this->Internals->getTreeMainItem(sqInternals::USER_CUSTOM);
  for (int idx = 0; idx < parent->childCount(); idx++)
  {
    QString childName = parent->child(idx)->text(sqInternals::PRESET_COLUMN());
    if (childName == presetName)
    {
      qCritical() << "A preset with this name already exists!";
      return;
    }
  }

  vtkSMSourceProxy* proxy = filter->getSourceProxy();
  vtkSmartPointer<vtkPVXMLElement> root = vtkSmartPointer<vtkPVXMLElement>::New();
  root->SetName("SlamPresets");
  root->AddAttribute("name", presetName.toStdString().c_str());
#if SAVE_PROPERTIES_SUBSET
  vtkNew<vtkSMNamedPropertyIterator> subset;
  subset->SetProxy(proxy);
  // Choose one:
  subset->SetPropertyNames(::LIDAR_MODEL_PROPERTIES);
  // subset->SetPropertyNames(::ENVIRONMENT_PROPERTIES);
  proxy->SaveXMLState(root, subset);
#else
  proxy->SaveXMLState(root);
#endif

  sqInternals::tryCreatePresetDir();
  QString path = sqInternals::CUSTOM_PRESET_DIR() + "/" + presetName + ".xml";
  vtksys::ofstream os(path.toUtf8().data(), ios::out);
  if (!os.good())
  {
    qCritical() << "Failed to save " << path << ".";
    return;
  }
  root->PrintXML(os, vtkIndent());
  os.close();

  sqInternals::addItem(parent, presetName, path);
  this->updateUIState();
}

//-----------------------------------------------------------------------------
void sqPresetDialog::onRemoveSelected()
{
  auto selected = this->Internals->Ui->presetTree->selectedItems().first();
  QString presetName = selected->text(sqInternals::PRESET_COLUMN());
  QString message = "Are you sure you want to delete the " + presetName + " preset?";
  auto confirmation =
    QMessageBox::question(this, "Remove Preset", message, QMessageBox::Yes | QMessageBox::No);
  if (confirmation == QMessageBox::No)
  {
    return;
  }

  QString filepath =
    selected->data(sqInternals::PRESET_COLUMN(), sqInternals::PRESET_PATH_ROLE()).toString();
  QFile file(filepath);
  if (!file.remove())
  {
    qCritical() << "Failed to remove " << filepath << ".";
    return;
  }

  auto parent = this->Internals->getTreeMainItem(sqInternals::USER_CUSTOM);
  parent->removeChild(selected);
  this->updateUIState();
}

//-----------------------------------------------------------------------------
void sqPresetDialog::onRemoveAll()
{
  auto confirmation = QMessageBox::question(this, "Remove Preset",
    "Are you sure you want to delete all custom presets?", QMessageBox::Yes | QMessageBox::No);
  if (confirmation == QMessageBox::No)
  {
    return;
  }

  QDirIterator it(sqInternals::CUSTOM_PRESET_DIR(), QDirIterator::Subdirectories);
  while (it.hasNext())
  {
    QString filepath = it.next();

    if (it.fileInfo().completeSuffix() == "xml")
    {
      QFile file(filepath);
      if (!file.remove())
      {
        qCritical() << "Failed to remove " << filepath << ".";
      }
    }
  }

  auto parent = this->Internals->getTreeMainItem(sqInternals::USER_CUSTOM);
  parent->takeChildren();
  this->updateUIState();
}

//-----------------------------------------------------------------------------
void sqPresetDialog::onApplySelected()
{
  pqPipelineFilter* filter =
    qobject_cast<pqPipelineFilter*>(pqActiveObjects::instance().activeSource());
  if (filter == nullptr)
  {
    return;
  }
  vtkSMSourceProxy* proxy = filter->getSourceProxy();

  QList<QTreeWidgetItem*> items = this->Internals->Ui->presetTree->selectedItems();
  for (auto &item : items)
  {
    QString filename =
      item->data(sqInternals::PRESET_COLUMN(), sqInternals::PRESET_PATH_ROLE()).toString();
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      qCritical() << "Failed to open file: " << filename << ".";
      continue;
    }
    QByteArray byteArray = file.readAll();
    file.close();

    vtkSmartPointer<vtkPVXMLParser> parser = vtkSmartPointer<vtkPVXMLParser>::New();
    if (parser->Parse(byteArray.constData()) == 0)
    {
      qCritical() << "Invalid XML in file: " << filename << ".";
      continue;
    }
    vtkPVXMLElement* xmlStream = parser->GetRootElement();
    if (xmlStream == nullptr)
    {
      qCritical() << "Invalid XML in file: " << filename << ".";
      continue;
    }
    vtkPVXMLElement* root = xmlStream->GetNestedElement(0);
    this->Internals->checkPresetValidity(proxy, root);
    proxy->LoadXMLState(root, nullptr);
  }

  QPushButton* applyButton = this->Internals->Ui->buttonBox->button(QDialogButtonBox::Apply);
  applyButton->setEnabled(false);

  Q_EMIT pqPVApplicationCore::instance()->triggerApply();
}

//-----------------------------------------------------------------------------
void sqPresetDialog::updateUIState()
{
  QPushButton* applyButton = this->Internals->Ui->buttonBox->button(QDialogButtonBox::Apply);
  bool isSlamFilter = false;

  pqPipelineFilter* filter =
    qobject_cast<pqPipelineFilter*>(pqActiveObjects::instance().activeSource());
  if (filter != nullptr)
  {
    isSlamFilter = vtkSlamFinder::isSlamFilter(filter->getSourceProxy());
  }

  auto customItem = this->Internals->getTreeMainItem(sqInternals::USER_CUSTOM);
  bool hasSelection = !this->Internals->Ui->presetTree->selectedItems().isEmpty();
  bool isCustomSelected = false;
  bool hasCustomItems = customItem->childCount() != 0;

  if (hasSelection)
  {
    this->Internals->handleMultiSelection();
    isCustomSelected = this->Internals->isCustomModelSelected();
  }

  applyButton->setEnabled(isSlamFilter && hasSelection);
  customItem->setDisabled(!hasCustomItems);
  this->Internals->Ui->saveCurrentPreset->setEnabled(isSlamFilter);
  this->Internals->Ui->removePreset->setEnabled(isCustomSelected);
  this->Internals->Ui->removeAll->setEnabled(hasCustomItems);
}
