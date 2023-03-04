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

  // if this dist_diff is positive, the obstacle is approaching
  const double dist_diff = p1.dist - p2.dist;
  const double time_diff = (p2.stamp - p1.stamp).seconds();
  const double vel_mps = dist_diff / time_diff;
  return vel_mps;
}

}  // namespace

BusStopModule::BusStopModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::BusStop & bus_stop_reg_elem,
  const std::shared_ptr<TurnIndicator> turn_indicator, const PlannerParam & planner_param,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  bus_stop_reg_elem_(bus_stop_reg_elem),
  turn_indicator_(turn_indicator),
  state_(State::GO),
  planner_param_(planner_param)
{
  // begin to put the turn indicator when bus stop module is launched
  put_turn_indicator_time_ = std::make_shared<const rclcpp::Time>(clock_->now());

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
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"), "obstacle points size: " << obstacle_points.size());

  const auto nearest_point_with_dist =
    findLongitudinalNearestPointBehind(obstacle_points, planner_data_->current_pose.pose);
  debug_data_.nearest_point = nearest_point_with_dist.first;

  // update buffer
  PointWithDistStamped point_with_dist;
  point_with_dist.point = nearest_point_with_dist.first;
  point_with_dist.dist = nearest_point_with_dist.second;
  pcl_conversions::fromPCL(
    planner_data_->no_ground_pointcloud->header.stamp, point_with_dist.stamp);
  points_buffer_.emplace_front(point_with_dist);
  updatePointsBuffer(points_buffer_);
  const double predicted_vel_mps = calcPredictedVelocity(points_buffer_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "longitudinal dist: " << point_with_dist.dist);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("debug"), "predicted vel: " << predicted_vel_mps * 3.6);

  //! for debug
  if (predicted_vel_mps < -10) {
    put_turn_indicator_time_ = std::make_shared<rclcpp::Time>(clock_->now());
  }

  // Get stop line geometry
  const auto stop_line = getStopLineGeometry2d();

  // Get stop point
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line, lane_id_, planner_param_.stop_margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!stop_point) {
    return true;
  }

  const auto & stop_point_idx = stop_point->first;
  const auto & stop_pose = stop_point->second;
  const size_t stop_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
    path->points, stop_pose.position, stop_point_idx);

  // TODO: parameter
  const double duration_turn_indicator = 10.0;
  const auto time_from_turn_indicator = clock_->now() - *put_turn_indicator_time_;
  // debug
  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("debug"),
    "time from turn indicator: " << time_from_turn_indicator.seconds());

  // if elapsed time from putting the indicator is less than threshold, keep stopping
  if (time_from_turn_indicator.seconds() < duration_turn_indicator) {
    // Insert stop point
    planning_utils::insertStopPoint(stop_pose.position, stop_line_seg_idx, *path);

    // For virtual wall
    debug_data_.stop_poses.push_back(stop_point->second);

    // Set Turn Indicator
    turn_indicator_->setTurnSignal(TurnIndicatorsCommand::ENABLE_RIGHT, clock_->now());
    turn_indicator_->publish();

    return true;
  }

  // decide if the ego can go from the predicted velocity of the obstacle in the bus stop area
  const double vel_thresh_kmph = 5.0;
  if (predicted_vel_mps > vel_thresh_kmph / 3.6) {
    // Insert stop point
    planning_utils::insertStopPoint(stop_pose.position, stop_line_seg_idx, *path);

    // For virtual wall
    debug_data_.stop_poses.push_back(stop_point->second);

    // Set Turn Indicator
    turn_indicator_->setTurnSignal(TurnIndicatorsCommand::ENABLE_RIGHT, clock_->now());
    turn_indicator_->publish();

    return true;
  }

  // Get self pose
  const auto & self_pose = planner_data_->current_pose.pose;
  const size_t current_seg_idx = findEgoSegmentIndex(path->points);

  // const auto is_stopped = planner_data_->isVehicleStopped(0.0);
  const auto stop_dist = calcSignedArcLength(
    path->points, self_pose.position, current_seg_idx, stop_pose.position, stop_line_seg_idx);

  // TODO: where to place RTC?
  setDistance(stop_dist);
  // Check state
  setSafe(canClearStopState());
  if (isActivated()) {
    state_ = State::GO;
    last_obstacle_found_time_ = {};
    return true;
  }

  // Force ignore objects after dead_line
  // if (planner_param_.use_dead_line) {
  //   // Use '-' for margin because it's the backward distance from stop line
  //   const auto dead_line_point = arc_lane_utils::createTargetPoint(
  //     original_path, stop_line, lane_id_, -planner_param_.dead_line_margin,
  //     planner_data_->vehicle_info_.max_longitudinal_offset_m);

  //   if (dead_line_point) {
  //     const size_t dead_line_point_idx = dead_line_point->first;
  //     const auto & dead_line_pose = dead_line_point->second;

  //     const size_t dead_line_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
  //       path->points, dead_line_pose.position, dead_line_point_idx);

  //     debug_data_.dead_line_poses.push_back(dead_line_pose);

  //     const double dist_from_ego_to_dead_line = calcSignedArcLength(
  //       original_path.points, self_pose.position, current_seg_idx, dead_line_pose.position,
  //       dead_line_seg_idx);
  //     if (dist_from_ego_to_dead_line < 0.0) {
  //       RCLCPP_WARN(logger_, "[detection_area] vehicle is over dead line");
  //       setSafe(true);
  //       return true;
  //     }
  //   }
  // }

  // // Ignore objects detected after stop_line if not in STOP state
  // const double dist_from_ego_to_stop = calcSignedArcLength(
  //   original_path.points, self_pose.position, current_seg_idx, stop_pose.position,
  //   stop_line_seg_idx);
  // if (state_ != State::STOP && dist_from_ego_to_stop < 0.0) {
  //   setSafe(true);
  //   return true;
  // }

  // // Ignore objects if braking distance is not enough
  // if (planner_param_.use_pass_judge_line) {
  //   if (state_ != State::STOP && !hasEnoughBrakingDistance(self_pose, stop_point->second)) {
  //     RCLCPP_WARN_THROTTLE(
  //       logger_, *clock_, std::chrono::milliseconds(1000).count(),
  //       "[detection_area] vehicle is over stop border");
  //     setSafe(true);
  //     return true;
  //   }
  // }

  // // Create StopReason
  // {
  //   StopFactor stop_factor{};
  //   stop_factor.stop_pose = stop_point->second;
  //   stop_factor.stop_factor_points = obstacle_points;
  //   planning_utils::appendStopReason(stop_factor, stop_reason);
  // }

  // // Create legacy StopReason
  // {
  //   const auto insert_idx = stop_point->first + 1;

  //   if (
  //     !first_stop_path_point_index_ ||
  //     static_cast<int>(insert_idx) < first_stop_path_point_index_) {
  //     debug_data_.first_stop_pose = stop_point->second;
  //     first_stop_path_point_index_ = static_cast<int>(insert_idx);
  //   }
  // }

  return true;
}

// std::vector<geometry_msgs::msg::Point> BusStopModule::getObstaclePoints() const
// {
//   std::vector<geometry_msgs::msg::Point> obstacle_points;

//   const auto bus_stops = bus_stop_reg_elem_.busStops();
//   const auto & points = *(planner_data_->no_ground_pointcloud);

//   for (const auto & bus_stop : bus_stops) {
//     const auto poly = lanelet::utils::to2D(bus_stop);
//     const auto circle = calcSmallestEnclosingCircle(poly);
//     for (const auto p : points) {
//       const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
//                                   (circle.first.y() - p.y) * (circle.first.y() - p.y);
//       if (squared_dist <= circle.second) {
//         if (bg::within(Point2d{p.x, p.y}, poly.basicPolygon())) {
//           obstacle_points.push_back(planning_utils::toRosPoint(p));
//           // get all obstacle point becomes high computation cost so skip if any point is found
//           break;
//         }
//       }
//     }
//   }

//   return obstacle_points;
// }

bool BusStopModule::canClearStopState() const
{
  // vehicle can clear stop state if the obstacle has never appeared in bus stop area
  if (!last_obstacle_found_time_) {
    return true;
  }

  // vehicle can clear stop state if the certain time has passed since the obstacle disappeared
  const auto elapsed_time = clock_->now() - *last_obstacle_found_time_;
  if (elapsed_time.seconds() >= planner_param_.state_clear_time) {
    return true;
  }

  // rollback in simulation mode
  if (elapsed_time.seconds() < 0.0) {
    return true;
  }

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
}  // namespace behavior_velocity_planner
