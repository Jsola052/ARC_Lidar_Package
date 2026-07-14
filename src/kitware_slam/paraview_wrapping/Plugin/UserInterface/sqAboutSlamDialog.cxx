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

#include "sqAboutSlamDialog.h"
#include "vtkSlamInfo.h"

#include "ui_sqAboutSlamDialog.h"

#include <pqCoreUtilities.h>
#include <pqFileDialog.h>

#include <QByteArray>
#include <QClipboard>
#include <QPushButton>
#include <QString>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <fstream>
#include <iostream>

//-----------------------------------------------------------------------------
struct sqAboutSlamDialog::sqInternals
{
  sqInternals()
    : Ui(new Ui::AboutSlamDialog)
  {
  }

  //-----------------------------------------------------------------------------
  void addItem(const QString& key, const QString& value)
  {
    QTreeWidget* tree = this->Ui->SlamInformation;
    QTreeWidgetItem* item = new QTreeWidgetItem(tree);
    item->setText(0, key);
    item->setText(1, value);
  }

  //-----------------------------------------------------------------------------
  void addBoolItem(const QString& key, bool value)
  {
    this->addItem(key, QString("%1").arg(value ? "On" : "Off"));
  }

  QScopedPointer<Ui::AboutSlamDialog> Ui;
};

//-----------------------------------------------------------------------------
sqAboutSlamDialog::sqAboutSlamDialog(QWidget* Parent)
  : Superclass(Parent)
  , Internals(new sqInternals())

{
  this->Internals->Ui->setupUi(this);

  this->connect(this->Internals->Ui->copyToClipBoardButton, &QPushButton::clicked, this,
    &sqAboutSlamDialog::copyToClipboard);
  this->connect(this->Internals->Ui->saveToFileButton, &QPushButton::clicked, this,
    &sqAboutSlamDialog::saveToFile);

  this->Internals->addItem("Slam version", SLAM_VERSION);
  this->Internals->addItem("Slam build type", SLAM_BUILD_TYPE);
  this->Internals->addItem("Boost version", SLAM_BOOST_VERSION);
  this->Internals->addItem("Nanoflann version", SLAM_NANOFLANN_VERSION);
  this->Internals->addItem("Eigen3 version", SLAM_EIGEN3_VERSION);
  this->Internals->addItem("Ceres version", SLAM_CERES_VERSION);
  this->Internals->addItem("PCL version", SLAM_PCL_VERSION);

  bool teaserFound = false;
#ifdef teaserpp_FOUND
  teaserFound = true;
#endif
  this->Internals->addBoolItem("Teaser enabled", teaserFound);

#ifdef g2o_FOUND
  this->Internals->addBoolItem("G2O enabled", true);
  this->Internals->addItem("G2O vesion", SLAM_G2O_VERSION);
#else
  this->Internals->addBoolItem("G2O enabled", false);
#endif

#ifdef GTSAM_FOUND
  this->Internals->addBoolItem("GTSAM enabled", true);
  this->Internals->addItem("GTSAM vesion", SLAM_GTSAM_VERSION);
#else
  this->Internals->addBoolItem("GTSAM enabled", false);
#endif

#ifdef OpenCV_FOUND
  this->Internals->addBoolItem("OpenCV enabled", true);
  this->Internals->addItem("OpenCV vesion", SLAM_OPENCV_VERSION);
#else
  this->Internals->addBoolItem("OpenCV enabled", false);
#endif

#ifdef OpenMP_FOUND
  this->Internals->addBoolItem("OpenMP enabled", true);
#else
  this->Internals->addBoolItem("OpenMP enabled", false);
#endif

  this->Internals->Ui->SlamInformation->resizeColumnToContents(0);
}

//-----------------------------------------------------------------------------
sqAboutSlamDialog::~sqAboutSlamDialog() = default;

//-----------------------------------------------------------------------------
QString sqAboutSlamDialog::formatToText(QTreeWidget* tree)
{
  QString text;
  QTreeWidgetItemIterator it(tree);
  while (*it)
  {
    text += (*it)->text(0) + ": " + (*it)->text(1) + "\n";
    ++it;
  }
  return text;
}

//-----------------------------------------------------------------------------
QString sqAboutSlamDialog::formatToText()
{
  QString text = tr("Slam Information:\n");
  QTreeWidget* tree = this->Internals->Ui->SlamInformation;
  text += this->formatToText(tree);
  return text;
}

//-----------------------------------------------------------------------------
void sqAboutSlamDialog::saveToFile()
{
  pqFileDialog fileDialog(nullptr, pqCoreUtilities::mainWidget(), tr("Save to File"), QString(),
    tr("Text Files") + " (*.txt);;" + tr("All Files") + " (*)", false);
  fileDialog.setFileMode(pqFileDialog::AnyFile);
  if (fileDialog.exec() != pqFileDialog::Accepted)
  {
    // Canceled
    return;
  }

  QString filename = fileDialog.getSelectedFiles().first();
  QByteArray filename_ba = filename.toUtf8();
  std::ofstream fileStream;
  fileStream.open(filename_ba.data());
  if (fileStream.is_open())
  {
    fileStream << this->formatToText().toStdString();
    fileStream.close();
  }
}

//-----------------------------------------------------------------------------
void sqAboutSlamDialog::copyToClipboard()
{
  QClipboard* clipboard = QGuiApplication::clipboard();
  clipboard->setText(this->formatToText());
}
