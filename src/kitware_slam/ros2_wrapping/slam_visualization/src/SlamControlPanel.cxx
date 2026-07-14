//==============================================================================
// Copyright 2019-2020 Kitware, Inc., Kitware SAS
// Author: Arthur Bourbousson (Kitware SAS)
// Creation date: 2023-06-16
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

#include "SlamControlPanel.h"

#include <rviz_common/display_context.hpp>

#include <QGridLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QFileDialog>

namespace slam_visualization
{

//----------------------------------------------------------------------------
SlamControlPanel::SlamControlPanel(QWidget* parent)
{
  this->CreateLayout();

  this->visualization_node = std::make_shared<rclcpp::Node>("slam_control_pannel_node");

  this->CommandPublisher = this->visualization_node->create_publisher<lidar_slam::msg::SlamCommand>("slam_command", 1);
  this->SavePcSlamClient = this->visualization_node->create_client<lidar_slam::srv::SavePc>("lidar_slam/save_pc");
  this->SavePcAggrClient = this->visualization_node->create_client<lidar_slam::srv::SavePc>("aggregation/save_pc");
  this->ResetSlamClient  = this->visualization_node->create_client<lidar_slam::srv::Reset>("lidar_slam/reset");
  this->ResetAggrClient  = this->visualization_node->create_client<lidar_slam::srv::Reset>("aggregation/reset");
  this->DetectLoopClient = this->visualization_node->create_client<lidar_slam::srv::DetectLoop>("lidar_slam/detect_loop");
}

//----------------------------------------------------------------------------
void  SlamControlPanel::onInitialize()
{
  // Access to rviz node using the context of the plugin
  auto rviz_ros_node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create parallel callback group to separate our interfaces from other rviz2 interfaces
  this->SlamCallbackGroup = rviz_ros_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions ops;
  ops.callback_group = this->SlamCallbackGroup;

  this->ConfidenceSubscriber = rviz_ros_node->create_subscription<lidar_slam::msg::Confidence>("/slam_confidence", 1,
                               std::bind(&SlamControlPanel::SlamConfidenceCallback, this, std::placeholders::_1), ops);
}

//----------------------------------------------------------------------------
void SlamControlPanel::CreateLayout()
{
  // Reset the SLAM process
  auto resetStateButton = new QPushButton;
  resetStateButton->setText("Reset state");
  resetStateButton->setToolTip("Clear the map, reset the pose to\n"
                               "initial pose and empty logged info.");
  connect(resetStateButton, &QPushButton::clicked, this, &SlamControlPanel::ResetSlamState);

 // Reset trajectory
  auto resetTrajButton = new QPushButton;
  resetTrajButton->setText("Reset trajectory");
  resetTrajButton->setToolTip("Reset the trajectory from a CSV file with header:\n"
                              "frame_id\n"
                              "Time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2\n"
                              "Time being the timestamp,\n"
                              "x,y,z being the position and\n"
                              "xi,yi,zi being an axis of the 3D frame\n"
                              "of the trajectory pose (ith column of the rotation).");
  connect(resetTrajButton, &QPushButton::clicked, this, &SlamControlPanel::ResetSlamTraj);

  // Turn on/off the SLAM process
  auto switchOnOffButton = new QPushButton;
  switchOnOffButton->setText("Switch on/off");
  switchOnOffButton->setToolTip("Disable the SLAM process but keep current state");
  connect(switchOnOffButton, &QPushButton::clicked, this, &SlamControlPanel::SwitchOnOff);

  // Disable the update of the maps, they will remain the same
  auto disableMapUpdateButton = new QPushButton;
  disableMapUpdateButton->setText("Disable map update");
  disableMapUpdateButton->setToolTip("Fix the map to its current state\n"
                                     "new keypoints will not be added.");
  connect(disableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::DisableMapUpdate);

  // Enable the expansion of the maps, keeping the initial one
  auto enableMapExpansionButton = new QPushButton;
  enableMapExpansionButton->setText("Enable map expansion");
  enableMapExpansionButton->setToolTip("Add new keypoints to the SLAM maps as soon as\n"
                                       "they don't replace an initial keypoint (loaded from file)");
  connect(enableMapExpansionButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapExpansion);

  // Enable the complete update of the maps
  auto enableMapUpdateButton = new QPushButton;
  enableMapUpdateButton->setText("Enable map update");
  enableMapUpdateButton->setToolTip("Add all new keypoints to the SLAM maps");
  connect(enableMapUpdateButton, &QPushButton::clicked, this, &SlamControlPanel::EnableMapUpdate);

 // Save trajectory
  auto saveTrajButton = new QPushButton;
  saveTrajButton->setText("Save trajectory");
  saveTrajButton->setToolTip("This will save the trajectory as a CSV type file\n"
                             "with header <index,Time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2>\n"
                             "x,y,z being the position and \n"
                             "xi,yi,zi being an axis of the 3D frame \n"
                             "of the trajectory pose (ith column of the rotation).");
  connect(saveTrajButton, &QPushButton::clicked, this, &SlamControlPanel::SaveTraj);

// Save maps
  auto saveMapsButton = new QPushButton;
  saveMapsButton->setText("Save maps");
  saveMapsButton->setToolTip("Save the maps to the required path.\n"
                             "For example for the input prefix path path/to/map,\n"
                             "one map by keypoint type will be saved as\n"
                             "path/to/map_kptType.pcd.\n"
                             "If the aggregation node has been turned on,\n"
                             "the aggregated map will also\n"
                             "be saved as path/to/map_posixTime.pcd.");
  connect(saveMapsButton, &QPushButton::clicked, this, &SlamControlPanel::SaveMaps);

 // Calibrate with external poses
  auto calibrateButton = new QPushButton;
  calibrateButton->setText("Calibrate");
  calibrateButton->setToolTip("Estimate the calibration between\n"
                              "the frame tracked by the SLAM and\n"
                              "the frame tracked in the input file.\n"
                              "This frame ID must be written in the first line of the input file.\n"
                              "The file must contain the fields\n"
                              "<Time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2>\n"
                              "Time being the timestamp,\n"
                              "x,y,z being the position and \n"
                              "xi,yi,zi being an axis of the 3D frame \n"
                              "of the trajectory pose (ith column of the rotation matrix).");
  connect(calibrateButton, &QPushButton::clicked, this, &SlamControlPanel::Calibrate);

  // Load external poses
  auto loadExtTrajButton = new QPushButton;
  loadExtTrajButton->setText("Load ext trajectory");
  loadExtTrajButton->setToolTip("Load an external trajectory\n"
                                "These poses can be used to de-skew the input scan,\n"
                                "to supply a prior pose to the registration algorithm,\n"
                                "or to add a constraint in the SLAM optimization.\n"
                                "The first line must contain the frame ID of the sensor.\n"
                                "The file must contain the fields\n"
                                "<Time,x,y,z,x0,y0,z0,x1,y1,z1,x2,y2,z2>\n"
                                "Time being the timestamp,\n"
                                "x,y,z being the position and \n"
                                "xi,yi,zi being an axis of the 3D frame \n"
                                "of the trajectory pose (ith column of the rotation matrix).\n"
                                "The frame ID must be written in the first line of the file.\n");
  connect(loadExtTrajButton, &QPushButton::clicked, this, &SlamControlPanel::LoadExtTrajectory);

 // Add loop closure indices from external file
  auto loadLoopIndicesButton = new QPushButton;
  loadLoopIndicesButton->setText("Load loop indices");
  loadLoopIndicesButton->setToolTip("Load loop closure indices\n"
                                    "from an external files and add \n"
                                    "them into LoopDetections vector.\n"
                                    "The file must contain the fields\n"
                                    "<queryIdx, revistedIdx>");
  connect(loadLoopIndicesButton, &QPushButton::clicked, this, &SlamControlPanel::LoadLoopIndices);

 // Detect loop closure
  auto detectLoopButton = new QPushButton;
  detectLoopButton->setText("Detect loop");
  detectLoopButton->setToolTip("Launch loop closure detection\n"
                               "The NEAREST POSE loop detector is used");
  connect(detectLoopButton, &QPushButton::clicked, this, &SlamControlPanel::DetectLoop);

 // Optimize graph
  auto optimizeGraphButton = new QPushButton;
  optimizeGraphButton->setText("Optimize graph");
  optimizeGraphButton->setToolTip("Launch pose graph optimization\n"
                                  "The Logging timeout needs to be\n"
                                  "set appropriately.");
  connect(optimizeGraphButton, &QPushButton::clicked, this, &SlamControlPanel::OptimizeGraph);

  // Create the whole command space
  auto commandLayout = new QVBoxLayout;
  commandLayout->addWidget(resetStateButton);
  commandLayout->addWidget(resetTrajButton);
  commandLayout->addWidget(disableMapUpdateButton);
  commandLayout->addWidget(enableMapExpansionButton);
  commandLayout->addWidget(enableMapUpdateButton);
  commandLayout->addWidget(switchOnOffButton);
  commandLayout->addWidget(saveTrajButton);
  commandLayout->addWidget(saveMapsButton);
  commandLayout->addWidget(calibrateButton);
  commandLayout->addWidget(loadExtTrajButton);
  commandLayout->addWidget(loadLoopIndicesButton);
  commandLayout->addWidget(detectLoopButton);
  commandLayout->addWidget(optimizeGraphButton);

  auto commandBox = new QGroupBox;
  commandBox->setLayout(commandLayout);
  commandBox->setTitle("Commands");

  // Confidence
  auto failureLabel = new QLabel{ "SLAM has failed: " };
  this->FailureValueLabel = new QLabel;
  this->FailureValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  failureLabel->setBuddy(this->FailureValueLabel);

  auto overlapLabel = new QLabel{ "Overlap:" };
  this->OverlapValueLabel = new QLabel;
  this->OverlapValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  overlapLabel->setBuddy(overlapLabel);

  auto complyMotionLimitsLabel = new QLabel{ "Comply motion limits: " };
  this->ComplyMotionLimitsValueLabel = new QLabel;
  this->ComplyMotionLimitsValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  complyMotionLimitsLabel->setBuddy(this->ComplyMotionLimitsValueLabel);

  auto stdPositionErrorValueLabel = new QLabel{ "Position error estimated: " };
  this->StdPositionErrorValueLabel = new QLabel;
  this->StdPositionErrorValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  stdPositionErrorValueLabel->setBuddy(this->ComplyMotionLimitsValueLabel);

  auto computationTimeLabel = new QLabel{ "Frame computation time: " };
  this->ComputationTimeValueLabel = new QLabel;
  this->ComputationTimeValueLabel->setTextFormat(Qt::TextFormat::PlainText);
  computationTimeLabel->setBuddy(this->ComputationTimeValueLabel);

  auto confidenceLayout = new QGridLayout;
  confidenceLayout->addWidget(failureLabel, 1, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->FailureValueLabel, 1, 1, Qt::AlignRight);
  confidenceLayout->addWidget(overlapLabel, 2,0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->OverlapValueLabel, 2, 1, Qt::AlignRight);
  confidenceLayout->addWidget(complyMotionLimitsLabel, 3, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComplyMotionLimitsValueLabel, 3, 1, Qt::AlignRight);
  confidenceLayout->addWidget(stdPositionErrorValueLabel, 4, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->StdPositionErrorValueLabel, 4, 1, Qt::AlignRight);
  confidenceLayout->addWidget(computationTimeLabel, 5, 0, Qt::AlignLeft);
  confidenceLayout->addWidget(this->ComputationTimeValueLabel, 5, 1, Qt::AlignRight);

  auto confidenceBox = new QGroupBox;
  confidenceBox->setTitle("Confidence estimator");
  confidenceBox->setLayout(confidenceLayout);

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(commandBox);
  mainLayout->addWidget(confidenceBox);

  setLayout(mainLayout);
}

//----------------------------------------------------------------------------
void SlamControlPanel::ResetSlamState()
{
  auto req = std::make_shared<lidar_slam::srv::Reset::Request>();

  // Reset SLAM states
  this->ResetSlamClient->async_send_request(req);

  // Reset Aggregation map
  this->ResetAggrClient->async_send_request(req);
}

//----------------------------------------------------------------------------
void SlamControlPanel::ResetSlamTraj()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Trajectory files (*.csv)"));
  this->SendCommand(lidar_slam::msg::SlamCommand::RESET_TRAJECTORY, filePath.toStdString());
}

//----------------------------------------------------------------------------
void SlamControlPanel::DisableMapUpdate()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::DISABLE_SLAM_MAP_UPDATE);
}

//----------------------------------------------------------------------------
void SlamControlPanel::EnableMapExpansion()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_EXPANSION);
}

//----------------------------------------------------------------------------
void SlamControlPanel::EnableMapUpdate()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::ENABLE_SLAM_MAP_UPDATE);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SwitchOnOff()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::SWITCH_ON_OFF);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SaveTraj()
{
  QString filePath = QFileDialog::getSaveFileName(this, "Create a CSV file", "", "Trajectory Files (*.csv)");
  QFileInfo fileInfo(filePath);
  QString extension = fileInfo.suffix();
  if (extension != "csv")
    filePath = fileInfo.path() + "/" + fileInfo.baseName() + ".csv";
  this->SendCommand(lidar_slam::msg::SlamCommand::SAVE_TRAJECTORY, filePath.toStdString());
}

//----------------------------------------------------------------------------
void SlamControlPanel::SaveMaps()
{
  QString filePath = QFileDialog::getSaveFileName(this, "Create a file prefix", "", "Prefix");
  QFileInfo fileInfo(filePath);
  QString extension = fileInfo.suffix();
  if (extension != "")
    filePath = fileInfo.path() + "/" + fileInfo.baseName();

  // Call the save_pc service
  auto srv = std::make_shared<lidar_slam::srv::SavePc::Request>();
  srv->output_prefix_path = filePath.toStdString();
  srv->format = 2;
  srv->filtered = true;
  this->SavePcSlamClient->async_send_request(srv);
  this->SavePcAggrClient->async_send_request(srv);
}

//----------------------------------------------------------------------------
void SlamControlPanel::Calibrate()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Trajectory files (*.csv)"));
  this->SendCommand(lidar_slam::msg::SlamCommand::CALIBRATE_WITH_POSES, filePath.toStdString());
}

//----------------------------------------------------------------------------
void SlamControlPanel::LoadExtTrajectory()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Trajectory files (*.csv)"));
  this->SendCommand(lidar_slam::msg::SlamCommand::LOAD_POSES, filePath.toStdString());
}

//----------------------------------------------------------------------------
void SlamControlPanel::LoadLoopIndices()
{
  QString filePath = QFileDialog::getOpenFileName(this, tr("Open File"), "/home", tr("Loop closure indices file (*.csv)"));
  this->SendCommand(lidar_slam::msg::SlamCommand::LOAD_LOOP_INDICES, filePath.toStdString());
}

//----------------------------------------------------------------------------
void SlamControlPanel::DetectLoop()
{
  auto req = std::make_shared<lidar_slam::srv::DetectLoop::Request>();
  this->DetectLoopClient->async_send_request(req);
}

//----------------------------------------------------------------------------
void SlamControlPanel::OptimizeGraph()
{
  this->SendCommand(lidar_slam::msg::SlamCommand::OPTIMIZE_GRAPH);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SendCommand(
  lidar_slam::msg::SlamCommand::_command_type command, lidar_slam::msg::SlamCommand::_string_arg_type arg)
{
  lidar_slam::msg::SlamCommand msg;
  msg.command = command;
  msg.string_arg = std::move(arg);
  this->CommandPublisher->publish(msg);
}

//----------------------------------------------------------------------------
void SlamControlPanel::SlamConfidenceCallback(const lidar_slam::msg::Confidence &confidence)
{
  QPalette palette1 = this->FailureValueLabel->palette();
  palette1.setColor(this->FailureValueLabel->foregroundRole(),
    confidence.failure ? Qt::red : Qt::black);

  this->FailureValueLabel->setPalette(palette1);
  this->FailureValueLabel->setText(confidence.failure ? "Yes" : "No");

  this->OverlapValueLabel->setText(
    QString::number(static_cast<int>(confidence.overlap * 100)) + '%');

  QPalette palette2 = this->ComplyMotionLimitsValueLabel->palette();
  palette2.setColor(this->ComplyMotionLimitsValueLabel->foregroundRole(),
    confidence.comply_motion_limits ? Qt::black : Qt::red);

  this->ComplyMotionLimitsValueLabel->setPalette(palette2);
  this->ComplyMotionLimitsValueLabel->setText(confidence.comply_motion_limits ? "Yes" : "No");

  this->StdPositionErrorValueLabel->setText(
    QString::number(confidence.std_position_error) + " m");

  this->ComputationTimeValueLabel->setText(
    QString::number(confidence.computation_time * 1000.0) + " ms");
}

} // namespace slam_visualization.

PLUGINLIB_EXPORT_CLASS(slam_visualization::SlamControlPanel, rviz_common::Panel)
