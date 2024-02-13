//
//  Copyright 2022 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef TRAFFIC_LIGHT_MONITOR_PANEL_HPP_
#define TRAFFIC_LIGHT_MONITOR_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QSpinBox>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>

#include <map>
#include <set>

namespace rviz_plugins
{

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_perception_msgs::msg::TrafficSignal;
using autoware_perception_msgs::msg::TrafficSignalArray;
using autoware_perception_msgs::msg::TrafficSignalElement;
class TrafficLightMonitorPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TrafficLightMonitorPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:

protected:
  void onTimer();
  void onInternalTrafficSignals(const TrafficSignalArray::ConstSharedPtr msg);
  void onExternalTrafficSignals(const TrafficSignalArray::ConstSharedPtr msg);
  void onArbiterTrafficSignals(const TrafficSignalArray::ConstSharedPtr msg);
  void onCurrentSignal(const TrafficSignal::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_signals_internal_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_signals_v2i_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr sub_traffic_signals_arbiter_;
  rclcpp::Subscription<TrafficSignal>::SharedPtr sub_current_signal_;

  QTableWidget * camera_table_;
  QTableWidget * v2i_table_;
  QTableWidget * arbiter_table_;
  std::optional<TrafficSignal::_traffic_signal_id_type> current_signal_id_;
};

}  // namespace rviz_plugins

#endif  // TRAFFIC_LIGHT_MONITOR_PANEL_HPP_
