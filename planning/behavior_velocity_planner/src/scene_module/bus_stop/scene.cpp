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

#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "utilization/arc_lane_util.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <scene_module/bus_stop/scene.hpp>
#include <utilization/util.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bus_stop
{
namespace bg = boost::geometry;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::calcLongitudinalDeviation;

namespace
{
// calc smallest enclosing circle with average O(N) algorithm
// reference:
// https://erickimphotography.com/blog/wp-content/uploads/2018/09/Computational-Geometry-Algorithms-and-Applications-3rd-Ed.pdf
std::pair<lanelet::BasicPoint2d, double> calcSmallestEnclosingCircle(
  const lanelet::ConstPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checkings.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double A = (p2 - p3).squaredNorm();
    const double B = (p3 - p1).squaredNorm();
    const double C = (p1 - p2).squaredNorm();
    const double S = cross(p2 - p1, p3 - p1);
    if (std::abs(S) < eps) return;
    center = (A * (B + C - A) * p1 + B * (C + A - B) * p2 + C * (A + B - C) * p3) / (4 * S * S);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i].basicPoint2d();
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0].basicPoint2d();
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j].basicPoint2d();
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k].basicPoint2d();
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}

std::vector<geometry_msgs::msg::Point> findPointsWithinPolygons(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const lanelet::ConstPolygons3d & polys)
{
  std::vector<geometry_msgs::msg::Point> obstacle_points;

  for (const auto & poly : polys) {
    const auto poly_2d = lanelet::utils::to2D(poly);
    const auto circle = calcSmallestEnclosingCircle(poly_2d);
    for (const auto p : input_points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist <= circle.second) {
        if (bg::within(Point2d{p.x, p.y}, poly_2d.basicPolygon())) {
          obstacle_points.push_back(planning_utils::toRosPoint(p));
        }
      }
    }
  }

  return obstacle_points;
}

// TODO: use optional?
// find longitudinal nearest point in the behind of the base_pose
std::pair<geometry_msgs::msg::Point, double> findLongitudinalNearestPointBehind(
  const std::vector<geometry_msgs::msg::Point> & input_points,
  const geometry_msgs::msg::Pose & base_pose)
{
  double min_dist = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_point;

  for (const auto & p : input_points) {
    const auto longitudinal_deviation = calcLongitudinalDeviation(base_pose, p);

    // ignore the points ahead of the base_pose
    if (longitudinal_deviation > 0) {
      continue;
    }

    if (-longitudinal_deviation < min_dist) {
      min_dist = -longitudinal_deviation;
      nearest_point = p;
    }
  }

  return {nearest_point, min_dist};
}

// TODO: appropriate update method
void updatePointsBuffer(std::deque<BusStopModule::PointWithDistStamped> & points_buffer)
{
  if (points_buffer.size() > 2) {
    points_buffer.pop_back();
  }
}

double calcPredictedVelocity(const std::deque<BusStopModule::PointWithDistStamped> & points_buffer)
{
  // p1 is older data
  const auto & p1 = points_buffer.back();
  const auto & p2 = points_buffer.front();

  // if dist_diff is positive, the obstacle is approaching
  const double dist_diff = p1.dist - p2.dist;
  const double time_diff = (p2.stamp - p1.stamp).seconds();
  const double vel_mps = dist_diff / time_diff;
  return vel_mps;
}

BusStopModule::PointWithDistStamped createPointWithDist(
  const geometry_msgs::msg::Point & point, const double dist, std::uint64_t stamp_pcl)
{
  BusStopModule::PointWithDistStamped point_with_dist;
  point_with_dist.point = point;
  point_with_dist.dist = dist;
  pcl_conversions::fromPCL(stamp_pcl, point_with_dist.stamp);

  return point_with_dist;
}

std::string toStringState(const StateMachine::State & state)
{
  switch (state) {
    case StateMachine::State::GO:
      return "GO";

    case StateMachine::State::STOP:
      return "STOP";

    case StateMachine::State::READY:
      return "READY";

    default:
      return "UNKNOWN";
  }
}
}  // namespace

BusStopModule::BusStopModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::BusStop & bus_stop_reg_elem, const PlannerParam & planner_param,
  rclcpp::Node & node, const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  bus_stop_reg_elem_(bus_stop_reg_elem),
  planner_param_(planner_param)
{
  turn_indicator_ = std::make_shared<TurnIndicator>(node);
  state_machine_ = std::make_shared<StateMachine>(node, planner_param.state_param);

  //! debug
  // change log level for debugging
  const auto result = rcutils_logging_set_logger_level("debug", RCUTILS_LOG_SEVERITY_DEBUG);
  if (result == RCUTILS_RET_ERROR) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "failed to set logger level.");
  }
}

LineString2d BusStopModule::getStopLineGeometry2d() const
{
  const lanelet::ConstLineString3d stop_line = bus_stop_reg_elem_.stopLine();
  return planning_utils::extendLine(
    stop_line[0], stop_line[1], planner_data_->stop_line_extend_length);
}

bool BusStopModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  // Store original path
  const auto original_path = *path;

  // Reset debug data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // Find obstacles in the bus stop area
  const auto obstacle_points =
    findPointsWithinPolygons(*planner_data_->no_ground_pointcloud, bus_stop_reg_elem_.busStops());

  //! debug
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "obstacle points size: " << obstacle_points.size());

  // ---
  // find longitudinal nearest point behind the ego vehicle
  // use the first path point as base pose
  const auto nearest_point_with_dist =
    findLongitudinalNearestPointBehind(obstacle_points, path->points.at(0).point.pose);
  debug_data_.nearest_point = nearest_point_with_dist.first;

  // update buffer and calculate predicted velocity from nearest obstacle point
  PointWithDistStamped point_with_dist = createPointWithDist(
    nearest_point_with_dist.first, nearest_point_with_dist.second,
    planner_data_->no_ground_pointcloud->header.stamp);
  points_buffer_.emplace_front(point_with_dist);
  updatePointsBuffer(points_buffer_);
  const double predicted_vel_mps = calcPredictedVelocity(points_buffer_);
  // ---

  //! debug
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "longitudinal dist: " << point_with_dist.dist);
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "predicted vel: " << predicted_vel_mps * 3.6 << " [km/h]");

  // Get stop line geometry
  const auto stop_line = getStopLineGeometry2d();

  // Get stop point
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line, lane_id_, 0.0 /* stop margin */,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!stop_point) {
    return true;
  }
  const auto & stop_point_idx = stop_point->first;
  const auto & stop_pose = stop_point->second;
  const size_t stop_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    path->points, stop_pose.position, stop_point_idx);

  state_machine_->updateState({predicted_vel_mps}, *clock_);
  const auto current_state = state_machine_->getCurrentState();
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "current_state: " << toStringState(current_state));

  if (current_state == State::STOP) {
    // Insert stop point
    planning_utils::insertStopPoint(stop_pose.position, stop_line_seg_idx, *path);

    // For virtual wall
    debug_data_.stop_poses.push_back(stop_point->second);

    return true;
  }

  if (current_state == State::READY) {
    // Insert stop point
    planning_utils::insertStopPoint(stop_pose.position, stop_line_seg_idx, *path);

    // For virtual wall
    debug_data_.stop_poses.push_back(stop_point->second);

    // Set Turn Indicator
    turn_indicator_->setTurnSignal(TurnIndicatorsCommand::ENABLE_RIGHT, clock_->now());
    turn_indicator_->publish();

    return true;
  }

  if (current_state == State::GO) {
    return true;
  }

  //! debug
  return true;

  //! RTC
  // TODO: where to place
  // // Get self pose
  // const auto & self_pose = planner_data_->current_pose.pose;
  // const size_t current_seg_idx = findEgoSegmentIndex(path->points);

  // // const auto is_stopped = planner_data_->isVehicleStopped(0.0);
  // const auto stop_dist = calcSignedArcLength(
  //   path->points, self_pose.position, current_seg_idx, stop_pose.position, stop_line_seg_idx);

  // setDistance(stop_dist);
  // // Check state
  // setSafe(canClearStopState());
  // if (isActivated()) {
  //   state_ = State::GO;
  //   last_detection_time_ = {};
  //   return true;
  // }
}

bool BusStopModule::canClearStopState() const
{
  // vehicle can clear stop state if the obstacle has never appeared in bus stop area
  // if (!last_detection_time_) {
  //   return true;
  // }

  // // vehicle can clear stop state if the certain time has passed since the obstacle disappeared
  // const auto elapsed_time = clock_->now() - *last_detection_time_;
  // if (elapsed_time.seconds() >= 2.0) {
  //   return true;
  // }

  // // rollback in simulation mode
  // if (elapsed_time.seconds() < 0.0) {
  //   return true;
  // }

  return false;
}

bool BusStopModule::hasEnoughBrakingDistance(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const
{
  // get vehicle info and compute pass_judge_line_distance
  const auto current_velocity = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  const double pass_judge_line_distance =
    planning_utils::calcJudgeLineDistWithAccLimit(current_velocity, max_acc, delay_response_time);

  return arc_lane_utils::calcSignedDistance(self_pose, line_pose.position) >
         pass_judge_line_distance;
}
}  // namespace bus_stop
}  // namespace behavior_velocity_planner
