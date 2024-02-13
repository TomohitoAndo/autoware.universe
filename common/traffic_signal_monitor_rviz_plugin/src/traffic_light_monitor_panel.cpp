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

#include "traffic_light_monitor_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rviz_common/display_context.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#undef signals
namespace rviz_plugins
{

namespace util
{
std::map<TrafficSignal::_traffic_signal_id_type, TrafficSignal> create_id_signal_map(
  const TrafficSignalArray & traffic_signal_array)
{
  std::map<TrafficSignal::_traffic_signal_id_type, TrafficSignal> id_signal_map;
  for (const auto & signal : traffic_signal_array.signals) {
    id_signal_map[signal.traffic_signal_id] = signal;
  }

  return id_signal_map;
}

}  // namespace util

TrafficLightMonitorPanel::TrafficLightMonitorPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Camera
  auto * h_layout_1 = new QHBoxLayout;
  auto * v_layout_1 = new QVBoxLayout;
  v_layout_1->addWidget(new QLabel("Camera Traffic Signals"));
  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);
  camera_table_ = new QTableWidget();
  camera_table_->setColumnCount(5);
  camera_table_->setHorizontalHeaderLabels({"ID", "Color", "Shape", "Status", "Confidence"});
  camera_table_->setVerticalHeader(vertical_header);
  camera_table_->setHorizontalHeader(horizontal_header);
  v_layout_1->addWidget(camera_table_);
  h_layout_1->addLayout(v_layout_1);

  // V2I
  auto * v_layout_2 = new QVBoxLayout;
  auto * h_layout_2 = new QHBoxLayout;
  v_layout_2->addLayout(h_layout_1);
  v_layout_2->addWidget(new QLabel("V2I Traffic Signals"));
  auto vertical_header_2 = new QHeaderView(Qt::Vertical);
  vertical_header_2->hide();
  auto horizontal_header_2 = new QHeaderView(Qt::Horizontal);
  horizontal_header_2->setSectionResizeMode(QHeaderView::Stretch);
  v2i_table_ = new QTableWidget();
  v2i_table_->setColumnCount(5);
  v2i_table_->setHorizontalHeaderLabels({"ID", "Color", "Shape", "Status", "Confidence"});
  v2i_table_->setVerticalHeader(vertical_header_2);
  v2i_table_->setHorizontalHeader(horizontal_header_2);
  v_layout_2->addWidget(v2i_table_);
  h_layout_2->addLayout(v_layout_2);

  // Traffic Light Arbiter
  auto * v_layout_3 = new QVBoxLayout;
  v_layout_3->addWidget(new QLabel("Traffic Light Arbiter"));
  auto vertical_header_3 = new QHeaderView(Qt::Vertical);
  vertical_header_3->hide();
  auto horizontal_header_3 = new QHeaderView(Qt::Horizontal);
  horizontal_header_3->setSectionResizeMode(QHeaderView::Stretch);
  arbiter_table_ = new QTableWidget();
  arbiter_table_->setColumnCount(5);
  arbiter_table_->setHorizontalHeaderLabels({"ID", "Color", "Shape", "Status", "Confidence"});
  arbiter_table_->setVerticalHeader(vertical_header_3);
  arbiter_table_->setHorizontalHeader(horizontal_header_3);
  v_layout_3->addWidget(arbiter_table_);

  // メインレイアウトにTraffic Light Arbiterセクションを追加
  auto * main_layout = new QVBoxLayout;
  main_layout->addLayout(h_layout_2);  // 既存のh_layout_2を含む
  main_layout->addLayout(v_layout_3);  // 新しいTraffic Light Arbiterセクション

  setLayout(main_layout);  // メインレイアウトをセット
}

void TrafficLightMonitorPanel::onInitialize()
{
  using std::placeholders::_1;
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_traffic_signals_internal_ = raw_node_->create_subscription<TrafficSignalArray>(
    "/perception/traffic_light_recognition/internal/traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightMonitorPanel::onInternalTrafficSignals, this, _1));

  sub_traffic_signals_v2i_ = raw_node_->create_subscription<TrafficSignalArray>(
    "/v2x/traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightMonitorPanel::onExternalTrafficSignals, this, _1));

  sub_traffic_signals_arbiter_ = raw_node_->create_subscription<TrafficSignalArray>(
    "/perception/traffic_light_recognition/traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightMonitorPanel::onArbiterTrafficSignals, this, _1));

  sub_current_signal_ = raw_node_->create_subscription<TrafficSignal>(
    "/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal",
    rclcpp::QoS(1), std::bind(&TrafficLightMonitorPanel::onCurrentSignal, this, _1));

  pub_timer_ = raw_node_->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(5000)), [&]() { onTimer(); });
}

void TrafficLightMonitorPanel::onInternalTrafficSignals(
  const TrafficSignalArray::ConstSharedPtr msg)
{
  camera_table_->setRowCount(msg->signals.size());

  if (msg->signals.empty()) {
    return;
  }

  const auto id_signal_map = util::create_id_signal_map(*msg);

  auto cell_row = 0;
  for (const auto & id_signal : id_signal_map) {
    const auto & signal = id_signal.second;

    if (signal.elements.empty()) {
      continue;
    }

    auto id_label = new QLabel(QString::number(signal.traffic_signal_id));
    id_label->setAlignment(Qt::AlignCenter);
    if (current_signal_id_ && id_signal.first == current_signal_id_.value()) {
      id_label->setStyleSheet("background-color: #87CEFA;");
    }

    auto color_label = new QLabel();
    color_label->setAlignment(Qt::AlignCenter);

    const auto & light = signal.elements.front();
    switch (light.color) {
      case TrafficSignalElement::RED:
        color_label->setText("RED");
        color_label->setStyleSheet("background-color: #FF0000;");
        break;
      case TrafficSignalElement::AMBER:
        color_label->setText("AMBER");
        color_label->setStyleSheet("background-color: #FFBF00;");
        break;
      case TrafficSignalElement::GREEN:
        color_label->setText("GREEN");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficSignalElement::WHITE:
        color_label->setText("WHITE");
        color_label->setStyleSheet("background-color: #FFFFFF;");
        break;
      case TrafficSignalElement::UNKNOWN:
        color_label->setText("UNKNOWN");
        color_label->setStyleSheet("background-color: #808080;");
        break;
      default:
        break;
    }

    auto shape_label = new QLabel();
    shape_label->setAlignment(Qt::AlignCenter);

    switch (light.shape) {
      case TrafficSignalElement::CIRCLE:
        shape_label->setText("CIRCLE");
        break;
      case TrafficSignalElement::LEFT_ARROW:
        shape_label->setText("LEFT_ARROW");
        break;
      case TrafficSignalElement::RIGHT_ARROW:
        shape_label->setText("RIGHT_ARROW");
        break;
      case TrafficSignalElement::UP_ARROW:
        shape_label->setText("UP_ARROW");
        break;
      case TrafficSignalElement::DOWN_ARROW:
        shape_label->setText("DOWN_ARROW");
        break;
      case TrafficSignalElement::DOWN_LEFT_ARROW:
        shape_label->setText("DOWN_LEFT_ARROW");
        break;
      case TrafficSignalElement::DOWN_RIGHT_ARROW:
        shape_label->setText("DOWN_RIGHT_ARROW");
        break;
      case TrafficSignalElement::UNKNOWN:
        shape_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto status_label = new QLabel();
    status_label->setAlignment(Qt::AlignCenter);

    switch (light.status) {
      case TrafficSignalElement::SOLID_OFF:
        status_label->setText("SOLID_OFF");
        break;
      case TrafficSignalElement::SOLID_ON:
        status_label->setText("SOLID_ON");
        break;
      case TrafficSignalElement::FLASHING:
        status_label->setText("FLASHING");
        break;
      case TrafficSignalElement::UNKNOWN:
        status_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto confidence_label = new QLabel(QString::number(light.confidence));
    confidence_label->setAlignment(Qt::AlignCenter);

    camera_table_->setCellWidget(cell_row, 0, id_label);
    camera_table_->setCellWidget(cell_row, 1, color_label);
    camera_table_->setCellWidget(cell_row, 2, shape_label);
    camera_table_->setCellWidget(cell_row, 3, status_label);
    camera_table_->setCellWidget(cell_row, 4, confidence_label);
    cell_row++;
  }
  camera_table_->update();
}

void TrafficLightMonitorPanel::onExternalTrafficSignals(
  const TrafficSignalArray::ConstSharedPtr msg)
{
  v2i_table_->setRowCount(msg->signals.size());

  if (msg->signals.empty()) {
    return;
  }

  const auto id_signal_map = util::create_id_signal_map(*msg);

  auto cell_row = 0;
  for (const auto & id_signal : id_signal_map) {
    const auto & signal = id_signal.second;

    if (signal.elements.empty()) {
      continue;
    }

    auto id_label = new QLabel(QString::number(signal.traffic_signal_id));
    id_label->setAlignment(Qt::AlignCenter);
    if (current_signal_id_ && id_signal.first == current_signal_id_.value()) {
      id_label->setStyleSheet("background-color: #87CEFA;");
    }

    auto color_label = new QLabel();
    color_label->setAlignment(Qt::AlignCenter);

    const auto & light = signal.elements.front();
    switch (light.color) {
      case TrafficSignalElement::RED:
        color_label->setText("RED");
        color_label->setStyleSheet("background-color: #FF0000;");
        break;
      case TrafficSignalElement::AMBER:
        color_label->setText("AMBER");
        color_label->setStyleSheet("background-color: #FFBF00;");
        break;
      case TrafficSignalElement::GREEN:
        color_label->setText("GREEN");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficSignalElement::WHITE:
        color_label->setText("WHITE");
        color_label->setStyleSheet("background-color: #FFFFFF;");
        break;
      case TrafficSignalElement::UNKNOWN:
        color_label->setText("UNKNOWN");
        color_label->setStyleSheet("background-color: #808080;");
        break;
      default:
        break;
    }

    auto shape_label = new QLabel();
    shape_label->setAlignment(Qt::AlignCenter);

    switch (light.shape) {
      case TrafficSignalElement::CIRCLE:
        shape_label->setText("CIRCLE");
        break;
      case TrafficSignalElement::LEFT_ARROW:
        shape_label->setText("LEFT_ARROW");
        break;
      case TrafficSignalElement::RIGHT_ARROW:
        shape_label->setText("RIGHT_ARROW");
        break;
      case TrafficSignalElement::UP_ARROW:
        shape_label->setText("UP_ARROW");
        break;
      case TrafficSignalElement::DOWN_ARROW:
        shape_label->setText("DOWN_ARROW");
        break;
      case TrafficSignalElement::DOWN_LEFT_ARROW:
        shape_label->setText("DOWN_LEFT_ARROW");
        break;
      case TrafficSignalElement::DOWN_RIGHT_ARROW:
        shape_label->setText("DOWN_RIGHT_ARROW");
        break;
      case TrafficSignalElement::UNKNOWN:
        shape_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto status_label = new QLabel();
    status_label->setAlignment(Qt::AlignCenter);

    switch (light.status) {
      case TrafficSignalElement::SOLID_OFF:
        status_label->setText("SOLID_OFF");
        break;
      case TrafficSignalElement::SOLID_ON:
        status_label->setText("SOLID_ON");
        break;
      case TrafficSignalElement::FLASHING:
        status_label->setText("FLASHING");
        break;
      case TrafficSignalElement::UNKNOWN:
        status_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto confidence_label = new QLabel(QString::number(light.confidence));
    confidence_label->setAlignment(Qt::AlignCenter);

    v2i_table_->setCellWidget(cell_row, 0, id_label);
    v2i_table_->setCellWidget(cell_row, 1, color_label);
    v2i_table_->setCellWidget(cell_row, 2, shape_label);
    v2i_table_->setCellWidget(cell_row, 3, status_label);
    v2i_table_->setCellWidget(cell_row, 4, confidence_label);
    cell_row++;
  }
  v2i_table_->update();
}

void TrafficLightMonitorPanel::onArbiterTrafficSignals(const TrafficSignalArray::ConstSharedPtr msg)
{
  arbiter_table_->setRowCount(msg->signals.size());

  if (msg->signals.empty()) {
    return;
  }

  const auto id_signal_map = util::create_id_signal_map(*msg);

  auto cell_row = 0;
  for (const auto & id_signal : id_signal_map) {
    const auto & signal = id_signal.second;

    if (signal.elements.empty()) {
      continue;
    }

    auto id_label = new QLabel(QString::number(id_signal.first));
    id_label->setAlignment(Qt::AlignCenter);
    if (current_signal_id_ && id_signal.first == current_signal_id_.value()) {
      id_label->setStyleSheet("background-color: #87CEFA;");
    }

    auto color_label = new QLabel();
    color_label->setAlignment(Qt::AlignCenter);

    const auto & light = signal.elements.front();
    switch (light.color) {
      case TrafficSignalElement::RED:
        color_label->setText("RED");
        color_label->setStyleSheet("background-color: #FF0000;");
        break;
      case TrafficSignalElement::AMBER:
        color_label->setText("AMBER");
        color_label->setStyleSheet("background-color: #FFBF00;");
        break;
      case TrafficSignalElement::GREEN:
        color_label->setText("GREEN");
        color_label->setStyleSheet("background-color: #7CFC00;");
        break;
      case TrafficSignalElement::WHITE:
        color_label->setText("WHITE");
        color_label->setStyleSheet("background-color: #FFFFFF;");
        break;
      case TrafficSignalElement::UNKNOWN:
        color_label->setText("UNKNOWN");
        color_label->setStyleSheet("background-color: #808080;");
        break;
      default:
        break;
    }

    auto shape_label = new QLabel();
    shape_label->setAlignment(Qt::AlignCenter);

    switch (light.shape) {
      case TrafficSignalElement::CIRCLE:
        shape_label->setText("CIRCLE");
        break;
      case TrafficSignalElement::LEFT_ARROW:
        shape_label->setText("LEFT_ARROW");
        break;
      case TrafficSignalElement::RIGHT_ARROW:
        shape_label->setText("RIGHT_ARROW");
        break;
      case TrafficSignalElement::UP_ARROW:
        shape_label->setText("UP_ARROW");
        break;
      case TrafficSignalElement::DOWN_ARROW:
        shape_label->setText("DOWN_ARROW");
        break;
      case TrafficSignalElement::DOWN_LEFT_ARROW:
        shape_label->setText("DOWN_LEFT_ARROW");
        break;
      case TrafficSignalElement::DOWN_RIGHT_ARROW:
        shape_label->setText("DOWN_RIGHT_ARROW");
        break;
      case TrafficSignalElement::UNKNOWN:
        shape_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto status_label = new QLabel();
    status_label->setAlignment(Qt::AlignCenter);

    switch (light.status) {
      case TrafficSignalElement::SOLID_OFF:
        status_label->setText("SOLID_OFF");
        break;
      case TrafficSignalElement::SOLID_ON:
        status_label->setText("SOLID_ON");
        break;
      case TrafficSignalElement::FLASHING:
        status_label->setText("FLASHING");
        break;
      case TrafficSignalElement::UNKNOWN:
        status_label->setText("UNKNOWN");
        break;
      default:
        break;
    }

    auto confidence_label = new QLabel(QString::number(light.confidence));
    confidence_label->setAlignment(Qt::AlignCenter);

    arbiter_table_->setCellWidget(cell_row, 0, id_label);
    arbiter_table_->setCellWidget(cell_row, 1, color_label);
    arbiter_table_->setCellWidget(cell_row, 2, shape_label);
    arbiter_table_->setCellWidget(cell_row, 3, status_label);
    arbiter_table_->setCellWidget(cell_row, 4, confidence_label);
    cell_row++;
  }
  arbiter_table_->update();
}

void TrafficLightMonitorPanel::onCurrentSignal(const TrafficSignal::ConstSharedPtr msg)
{
  current_signal_id_ = msg->traffic_signal_id;
}

void TrafficLightMonitorPanel::onTimer()
{
  v2i_table_->setRowCount(0);
  camera_table_->setRowCount(0);
  arbiter_table_->setRowCount(0);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::TrafficLightMonitorPanel, rviz_common::Panel)
