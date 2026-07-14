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

#ifndef SLAM_CONTROL_PANEL_H
#define SLAM_CONTROL_PANEL_H

#include <QLabel>
#include <QPushButton>

#include <pluginlib/class_list_macros.hpp>

#include <lidar_slam/msg/confidence.hpp>
#include <lidar_slam/msg/slam_command.hpp>
#include <lidar_slam/srv/save_pc.hpp>
#include <lidar_slam/srv/reset.hpp>
#include <lidar_slam/srv/detect_loop.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rviz_common/panel.hpp>

namespace slam_visualization
{

class SlamControlPanel :
        public rviz_common::Panel
{

public:
  //----------------------------------------------------------------------------
  /*!
   * @brief     Constructor.
   * @param[in] widget Parent widget.
   */
  // explicit SlamControlPanel(QWidget* parent = nullptr);
  explicit SlamControlPanel(QWidget* parent = nullptr);

  ~SlamControlPanel() override = default;

public Q_SLOTS:

  //----------------------------------------------------------------------------
  /*!
   * @brief Function executed after the rviz2 initializatin of the plugin
            Create subscriber to the confidence estimator of the SLAM
   */
  void onInitialize() override;

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a RESET_SLAM command to the slam node.
   */
  void ResetSlamState();

  //----------------------------------------------------------------------------
  /*!
   * @brief Open a dialog to choose a csv trajectory file
   *        and send a RESET_TRAJECTORY command to the slam node.
   */
  void ResetSlamTraj();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a DISABLE_SLAM_MAP_UPDATE command to the slam node.
   */
  void DisableMapUpdate();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a ENABLE_SLAM_MAP_EXPANSION command to the slam node.
   */
  void EnableMapExpansion();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a ENABLE_SLAM_MAP_UPDATE command to the slam node.
   */
  void EnableMapUpdate();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a SWITCH_ON_OFF command to the slam node.
   */
  void SwitchOnOff();

  //----------------------------------------------------------------------------
  /*!
   * @brief Open a dialog to chose a path to save the trajectory
   *        and send a SAVE_TRAJECTORY command to the slam node.
   */
  void SaveTraj();

  //----------------------------------------------------------------------------
  /*!
   * @brief Open a dialog to chose a path to save the keypoint maps and the aggregation points.
   *        Send a SAVE_TRAJECTORY command to the slam node and a SavePc request to aggregation node.
   */
  void SaveMaps();

  //----------------------------------------------------------------------------
  /*!
   * @brief Open a dialog to choose a csv calibration file
   *        and send a CALIBRATE_WITH_POSES command to the slam node.
   */
  void Calibrate();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a LOAD_POSES command to the slam node.
   */
  void LoadExtTrajectory();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a LOAD_LOOP_INDICES command to the slam node.
   */
  void LoadLoopIndices();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a request to detect loop service.
   */
  void DetectLoop();

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a OPTIMIZE_GRAPH command to the slam node.
   */
  void OptimizeGraph();

private:
  //----------------------------------------------------------------------------
  /*!
   * @brief Create and fill the widget layout.
   */
  void CreateLayout();

  //----------------------------------------------------------------------------
  /*!
   * @brief SLAM confidence callback.
   * @param[in] confidence Confidence values.
   */
  void SlamConfidenceCallback(const lidar_slam::msg::Confidence& confidence);

  //----------------------------------------------------------------------------
  /*!
   * @brief Send a command to the SLAM node.
   * @param[in] command Command type. See the enum in lidar_slam::msg::SlamCommand
   * @param[in] arg Optional command arg.
   */
  void SendCommand(lidar_slam::msg::SlamCommand::_command_type command,
    lidar_slam::msg::SlamCommand::_string_arg_type arg = "");

  //----------------------------------------------------------------------------

  // UI widgets
  QLabel* FailureValueLabel = nullptr;
  QLabel* OverlapValueLabel = nullptr;
  QLabel* ComplyMotionLimitsValueLabel = nullptr;
  QLabel* StdPositionErrorValueLabel = nullptr;
  QLabel* ComputationTimeValueLabel = nullptr;

  // ROS interface
  rclcpp::Node::SharedPtr visualization_node;
  rclcpp::CallbackGroup::SharedPtr SlamCallbackGroup;
  rclcpp::Publisher<lidar_slam::msg::SlamCommand>::SharedPtr CommandPublisher;
  rclcpp::Subscription<lidar_slam::msg::Confidence>::SharedPtr ConfidenceSubscriber;
  rclcpp::Client<lidar_slam::srv::SavePc>::SharedPtr SavePcSlamClient;
  rclcpp::Client<lidar_slam::srv::SavePc>::SharedPtr SavePcAggrClient;
  rclcpp::Client<lidar_slam::srv::Reset>::SharedPtr ResetSlamClient;
  rclcpp::Client<lidar_slam::srv::Reset>::SharedPtr ResetAggrClient;
  rclcpp::Client<lidar_slam::srv::DetectLoop>::SharedPtr DetectLoopClient;
};

} // namespace lidar_visualization.

#endif // SLAM_CONTROL_PANEL_H
