// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENE_MODULE__BUS_STOP__SCENE_HPP_
#define SCENE_MODULE__BUS_STOP__SCENE_HPP_

#include "scene_module/bus_stop/state_machine.hpp"
#include "scene_module/bus_stop/turn_indicator.hpp"
#include "scene_module/scene_module_interface.hpp"

#include <boost/optional.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <lanelet2_extension/regulatory_elements/bus_stop.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal_processing/lowpass_filter_1d.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

#include <deque>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bus_stop
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;  // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, Point2d>;                // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                  // front index, offset
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;
using StateMachine = bus_stop::StateMachine;
using State = StateMachine::State;

class BusStopModule : public SceneModuleInterface
{
public:
  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    std::vector<geometry_msgs::msg::Pose> dead_line_poses;
    geometry_msgs::msg::Pose first_stop_pose;
    std::vector<geometry_msgs::msg::Point> obstacle_points;
    geometry_msgs::msg::Point nearest_point;
  };

  struct PlannerParam
  {
    size_t buffer_size;
    double lpf_gain;
    StateMachine::StateParam state_param;
  };

  struct PointWithDistStamped
  {
    geometry_msgs::msg::Point point;
    double dist;
    rclcpp::Time stamp;
  };

public:
  BusStopModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::BusStop & bus_stop_reg_elem, const PlannerParam & planner_param,
    rclcpp::Node & node, const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  LineString2d getStopLineGeometry2d() const;
  boost::optional<PathIndexWithPose> calcStopPoint(const PathWithLaneId & path) const;
  double calcStopDistance(const PathWithLaneId & path, const PathIndexWithPose & stop_point);
  void updatePointsBuffer(const BusStopModule::PointWithDistStamped & point_with_dist);
  double calcPredictedVelocity();
  bool isRTCActivated(const double stop_distance, const bool safe);

  // Lane id
  int64_t lane_id_;

  // Key Feature
  const lanelet::autoware::BusStop & bus_stop_reg_elem_;

  // For publishing turn indicator
  std::shared_ptr<TurnIndicator> turn_indicator_;

  // For handling states
  std::shared_ptr<StateMachine> state_machine_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // Nearest points buffer
  std::deque<PointWithDistStamped> points_buffer_;
  std::deque<double> velocity_buffer_;
  std::deque<double> velocity_buffer_lpf_;
  std::shared_ptr<LowpassFilter1d> lpf_;
};
}  // namespace bus_stop
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__BUS_STOP__SCENE_HPP_
