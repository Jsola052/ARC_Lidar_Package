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

#ifndef sqAboutSlamDialog_h
#define sqAboutSlamDialog_h

#include <QDialog>
#include <QString>

#include <memory>

class QTreeWidget;

class sqAboutSlamDialog : public QDialog
{
  Q_OBJECT
  typedef QDialog Superclass;

public:
  sqAboutSlamDialog(QWidget* Parent = nullptr);
  ~sqAboutSlamDialog() override;

  /**
   * Format the about dialog content into textual form
   */
  QString formatToText();

public Q_SLOTS:
  /**
   * Saves about dialog formatted output to a file.
   */
  void saveToFile();

  /**
   * Copy about dialog formatted output to the clipboard.
   */
  void copyToClipboard();

private:
  QString formatToText(QTreeWidget* tree);

private:
  struct sqInternals;
  std::unique_ptr<sqInternals> Internals;
};

#endif
