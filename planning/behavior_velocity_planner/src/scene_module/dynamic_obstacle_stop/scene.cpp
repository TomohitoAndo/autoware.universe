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

#include "scene_module/dynamic_obstacle_stop/scene.hpp"

#include "utilization/util.hpp"

#include <tier4_autoware_utils/trajectory/tmp_conversion.hpp>
#include <tier4_autoware_utils/trajectory/trajectory.hpp>

#include <pcl/filters/voxel_grid.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
}  // namespace

DynamicObstacleStopModule::DynamicObstacleStopModule(
  const int64_t module_id, const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), planner_param_(planner_param)
{
}

DynamicObstacleStopModule::DynamicObstacleStopModule(
  const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const std::shared_ptr<motion_velocity_smoother::SmootherBase> & smoother,
  const std::shared_ptr<DynamicObstacleStopDebug> & debug_ptr, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  planner_param_(planner_param),
  smoother_(smoother),
  debug_ptr_(debug_ptr)
{
  if (planner_param.dynamic_obstacle_stop.use_partition_lanelet) {
    const lanelet::LaneletMapConstPtr & ll = planner_data->route_handler_->getLaneletMapPtr();
    planning_utils::getAllPartitionLanelets(ll, partition_lanelets_);
  }
}

void DynamicObstacleStopModule::setPlannerParam(const PlannerParam & planner_param)
{
  planner_param_ = planner_param;
}

bool DynamicObstacleStopModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] tier4_planning_msgs::msg::StopReason * stop_reason)
{
  if (!planner_param_.dynamic_obstacle_stop.enable_dynamic_obstacle_stop) {
    return true;
  }

  if (!planner_data_->current_velocity || !planner_data_->current_accel) {
    return true;
  }

  if (!planner_data_->predicted_objects) {
    return true;
  }

  if (!planner_data_->compare_map_filtered_pointcloud) {
    return true;
  }

  if (!planner_data_->prev_smoothed_trajectory) {
    return true;
  }

  if (!planner_data_->velocity_limit) {
    return true;
  }

  const auto & current_pose = planner_data_->current_pose.pose;

  float current_vel;
  float current_acc;
  if (planner_param_.dynamic_obstacle_stop.use_closest) {
    dynamic_obstacle_stop_utils::findClosestPlannedVelocityAndAcceleration(
      *planner_data_->prev_smoothed_trajectory, current_pose, current_vel, current_acc);
  } else {
    current_vel = planner_data_->current_velocity->twist.linear.x;
    current_acc = planner_data_->current_accel.get();
  }

  // temporary convert to trajectory
  const auto input_traj = dynamic_obstacle_stop_utils::convertPathToTrajectory(*path);

  // extend trajectory to consider obstacles after the goal
  const auto base_trajectory =
    extendTrajectory(input_traj, planner_param_.dynamic_obstacle_stop.extend_distance);

  // smooth the velocity by using smoother
  const auto smoothed_trajectory =
    applySmoother(base_trajectory, current_pose, current_vel, current_acc);
  if (!smoothed_trajectory) {
    RCLCPP_WARN_STREAM(logger_, "failed to apply smoother.");
    return true;
  }

  // trim trajectory ahead of the base_link
  const double trim_distance = planner_param_.dynamic_obstacle_stop.detection_distance;
  const auto trim_trajectory =
    trimTrajectoryFromSelfPose(smoothed_trajectory.get(), current_pose, trim_distance);

  // TODO(Tomohito Ando): make options easier to understand
  std::vector<DynamicObstacle> dynamic_obstacles;
  if (
    planner_param_.dynamic_obstacle_stop.use_objects &&
    planner_param_.dynamic_obstacle_stop.use_predicted_path) {
    dynamic_obstacles = createDynamicObstaclesFromObjects(*planner_data_->predicted_objects);
  } else if (
    planner_param_.dynamic_obstacle_stop.use_objects &&
    !planner_param_.dynamic_obstacle_stop.use_predicted_path) {
    dynamic_obstacles =
      createDynamicObstaclesFromObjects(*planner_data_->predicted_objects, input_traj);

    // debug
    visualizePassingArea(trim_trajectory, current_pose);
  } else {
    const auto voxel_grid_filtered_points =
      applyVoxelGridFilter(planner_data_->compare_map_filtered_pointcloud);
    const auto extracted_points =
      extractObstaclePointsWithRectangle(voxel_grid_filtered_points, current_pose);
    dynamic_obstacles = createDynamicObstaclesFromPoints(extracted_points, input_traj);

    visualizePassingArea(trim_trajectory, current_pose);
  }

  const auto partition_excluded_obstacles =
    excludeObstaclesOutSideOfPartition(dynamic_obstacles, trim_trajectory, current_pose);

  // timer starts
  const auto t1 = std::chrono::system_clock::now();

  const auto dynamic_obstacle = detectCollision(partition_excluded_obstacles, trim_trajectory);

  // timer ends
  const auto t2 = std::chrono::system_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

  if (planner_param_.approaching.enable) {
    insertVelocityWithApproaching(
      dynamic_obstacle, current_pose, current_vel, current_acc, trim_trajectory, *path);
  } else {
    const auto stop_point =
      calcStopPoint(dynamic_obstacle, trim_trajectory, current_pose, current_vel, current_acc);
    insertStopPoint(stop_point, *path);
  }

  // apply max jerk limit if the ego can't stop with specified max jerk and acc
  if (planner_param_.slow_down_limit.enable) {
    applyMaxJerkLimit(trim_trajectory, current_pose, current_vel, current_acc, *path);
  }

  // debug
  {
    if (dynamic_obstacle) {
      const auto lateral_dist = std::abs(tier4_autoware_utils::calcLateralOffset(
                                  trim_trajectory.points, dynamic_obstacle->pose_.position)) -
                                planner_param_.vehicle_param.width / 2.0;
      const auto longitudinal_dist_to_obstacle =
        tier4_autoware_utils::calcSignedArcLength(
          trim_trajectory.points, current_pose.position, dynamic_obstacle->pose_.position) -
        planner_param_.vehicle_param.base_to_front;

      const float dist_to_collision_point = tier4_autoware_utils::calcSignedArcLength(
        trim_trajectory.points, current_pose.position, dynamic_obstacle->nearest_collision_point_);
      const auto dist_to_collision =
        dist_to_collision_point - planner_param_.vehicle_param.base_to_front;

      debug_ptr_->setDebugValues(DebugValues::TYPE::LONGITUDINAL_DIST_COLLISION, dist_to_collision);
      debug_ptr_->setDebugValues(DebugValues::TYPE::LATERAL_DIST, lateral_dist);
      debug_ptr_->setDebugValues(
        DebugValues::TYPE::LONGITUDINAL_DIST_OBSTACLE, longitudinal_dist_to_obstacle);
    } else {
      // max value
      constexpr float max_val = 50.0;
      debug_ptr_->setDebugValues(DebugValues::TYPE::LATERAL_DIST, max_val);
      debug_ptr_->setDebugValues(DebugValues::TYPE::LONGITUDINAL_DIST_COLLISION, max_val);
      debug_ptr_->setDebugValues(DebugValues::TYPE::LONGITUDINAL_DIST_OBSTACLE, max_val);
    }

    if (partition_excluded_obstacles.empty()) {
      debug_ptr_->setAccelReason(AccelReason::NO_OBSTACLE);
    }

    debug_ptr_->setDebugValues(DebugValues::TYPE::NUM_OBSTACLES, dynamic_obstacles.size());
    debug_ptr_->setDebugValues(DebugValues::TYPE::CALCULATION_TIME, elapsed.count() / 1000.0);

    debug_ptr_->publish();

    debug_ptr_->publishDebugTrajectory(*smoothed_trajectory);
  }

  return true;
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::createDynamicObstaclesFromPoints(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud, const Trajectory & trajectory) const
{
  std::vector<DynamicObstacle> dynamic_obstacles{};
  for (const auto & p : obstacle_pointcloud) {
    const auto ros_point = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    DynamicObstacle dynamic_obstacle(planner_param_.dynamic_obstacle);
    dynamic_obstacle.createDynamicObstacle(ros_point, trajectory);
    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::createDynamicObstaclesFromObjects(
  const PredictedObjects & predicted_objects, const Trajectory & trajectory) const
{
  std::vector<DynamicObstacle> dynamic_obstacles{};
  for (const auto & object : predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle(planner_param_.dynamic_obstacle);
    dynamic_obstacle.createDynamicObstacle(object, trajectory);

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::createDynamicObstaclesFromObjects(
  const PredictedObjects & predicted_objects) const
{
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & object : predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle(planner_param_.dynamic_obstacle);
    dynamic_obstacle.createDynamicObstacle(object);

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

pcl::PointCloud<pcl::PointXYZ> DynamicObstacleStopModule::applyVoxelGridFilter(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_points) const
{
  auto no_height_points = *input_points;
  for (auto & p : no_height_points) {
    p.z = 0.0;
  }

  // use boost::makeshared instead of std beacause filter.setInputCloud requires boost shared ptr
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(no_height_points));
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);

  pcl::PointCloud<pcl::PointXYZ> output_points;
  filter.filter(output_points);

  return output_points;
}

pcl::PointCloud<pcl::PointXYZ> DynamicObstacleStopModule::extractObstaclePointsWithRectangle(
  const pcl::PointCloud<pcl::PointXYZ> & input_points,
  const geometry_msgs::msg::Pose & current_pose) const
{
  const auto detection_area_polygon =
    createDetectionAreaPolygon(current_pose, planner_param_.detection_area);

  debug_ptr_->pushDebugPolygons(detection_area_polygon);

  const auto extracted_points = pointsWithinPolygon(detection_area_polygon, input_points);

  return extracted_points;
}

std::vector<geometry_msgs::msg::Point> DynamicObstacleStopModule::createDetectionAreaPolygon(
  const geometry_msgs::msg::Pose & current_pose, const DetectionAreaSize detection_area_size) const
{
  const auto & d = detection_area_size;
  const auto p1 = tier4_autoware_utils::calcOffsetPose(current_pose, d.dist_ahead, d.dist_left, 0);
  const auto p2 =
    tier4_autoware_utils::calcOffsetPose(current_pose, d.dist_ahead, -d.dist_right, 0);
  const auto p3 =
    tier4_autoware_utils::calcOffsetPose(current_pose, -d.dist_behind, -d.dist_right, 0);
  const auto p4 =
    tier4_autoware_utils::calcOffsetPose(current_pose, -d.dist_behind, d.dist_left, 0);

  std::vector<geometry_msgs::msg::Point> detection_area;
  detection_area.emplace_back(p1.position);
  detection_area.emplace_back(p2.position);
  detection_area.emplace_back(p3.position);
  detection_area.emplace_back(p4.position);

  return detection_area;
}

void DynamicObstacleStopModule::visualizePassingArea(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose) const
{
  // TODO(Tomohito Ando): parameter
  constexpr float dist_max = 50.0;
  const auto passing_dist = calcPassingDistanceForEachPoint(
    trajectory, planner_param_.dynamic_obstacle_stop.obstacle_velocity_kph / 3.6, dist_max);

  const auto deceleration_line_idx = calcDecelerationLineIndex(trajectory, current_pose);
  if (!deceleration_line_idx) {
    return;
  }

  const auto passing_lines = calcPassingLines(trajectory, passing_dist);

  const size_t passing_dist_idx = std::min(*deceleration_line_idx, passing_lines.at(0).size() - 1);
  std::stringstream sstream;
  sstream << std::setprecision(4) << passing_dist.at(passing_dist_idx) << "m";
  debug_ptr_->pushDebugTexts(dynamic_obstacle_stop_utils::createDebugText(
    sstream.str(), passing_lines.at(0).at(passing_dist_idx)));
  debug_ptr_->pushDebugTexts(dynamic_obstacle_stop_utils::createDebugText(
    sstream.str(), passing_lines.at(1).at(passing_dist_idx)));
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::LATERAL_PASS_DIST, passing_dist.at(passing_dist_idx));

  const auto detection_area_polygon =
    createDetectionAreaPolygon(passing_lines, *deceleration_line_idx);
  if (!detection_area_polygon) {
    return;
  }

  debug_ptr_->pushDebugLines(*detection_area_polygon);
}

std::vector<float> DynamicObstacleStopModule::calcPassingDistanceForEachPoint(
  const Trajectory & trajectory, const float obstacle_vel_mps, const float dist_max) const
{
  std::vector<float> lateral_passing_dist{};

  if (trajectory.points.size() < 2) {
    return lateral_passing_dist;
  }

  // calculate transition time from base_link for each point
  float transition_time = 0;
  float dist_sum = 0;
  for (size_t i = 0; i < trajectory.points.size(); i++) {
    // assume that transition time to the nearest point is zero
    if (i == 0) {
      lateral_passing_dist.push_back(0.0);
      continue;
    }

    const float ds =
      tier4_autoware_utils::calcDistance2d(trajectory.points.at(i - 1), trajectory.points.at(i));
    dist_sum += ds;
    if (dist_sum > dist_max) {
      return lateral_passing_dist;
    }

    // calculate the distance that obstacles can move until ego reach the trajectory point
    // assume that obstacles move with constant velocity (specified in parameter)
    const float vel_mps = trajectory.points.at(i - 1).longitudinal_velocity_mps;
    const float dt = ds / std::max(vel_mps, 1.0f);
    transition_time += dt;
    lateral_passing_dist.push_back(transition_time * obstacle_vel_mps);
  }

  return lateral_passing_dist;
}

boost::optional<size_t> DynamicObstacleStopModule::calcDecelerationLineIndex(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose) const
{
  if (trajectory.points.empty()) {
    return {};
  }

  // calculate distance needed to stop with jerk and acc constraints
  const float initial_vel = trajectory.points.at(0).longitudinal_velocity_mps;
  const float initial_acc = trajectory.points.at(0).acceleration_mps2;
  const float target_vel = 0.0;
  const float jerk_dec = planner_param_.dynamic_obstacle_stop.stop_start_jerk_dec;
  const float jerk_acc = std::abs(jerk_dec);
  const float planning_dec = jerk_dec < planner_param_.common.normal_min_jerk
                               ? planner_param_.common.limit_min_acc
                               : planner_param_.common.normal_min_acc;
  const auto stop_dist = dynamic_obstacle_stop_utils::calcDecelDistWithJerkAndAccConstraints(
    initial_vel, target_vel, initial_acc, planning_dec, jerk_acc, jerk_dec);

  rclcpp::Clock clock{RCL_ROS_TIME};
  if (!stop_dist) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock, std::chrono::milliseconds(1000).count(), "failed to calculate stop distance");
    return {};
  }

  // calculate the index of stop position of base_link
  const size_t deceleration_line_idx = calcTrajectoryIndexByLength(
    trajectory, current_pose, stop_dist.get() + planner_param_.dynamic_obstacle_stop.stop_margin);

  return deceleration_line_idx;
}

std::vector<std::vector<geometry_msgs::msg::Point>> DynamicObstacleStopModule::calcPassingLines(
  const Trajectory & trajectory, const std::vector<float> & lateral_passing_dist) const
{
  std::vector<geometry_msgs::msg::Point> passing_line_left;
  std::vector<geometry_msgs::msg::Point> passing_line_right;
  for (size_t idx = 0; idx < lateral_passing_dist.size(); idx++) {
    const auto & traj_point = trajectory.points.at(idx).pose;
    const float lateral_offset =
      lateral_passing_dist.at(idx) + planner_param_.vehicle_param.width / 2.0;
    const auto offset_pose_left = tier4_autoware_utils::calcOffsetPose(
      traj_point, planner_param_.vehicle_param.base_to_front, lateral_offset, 0);
    const auto offset_pose_right = tier4_autoware_utils::calcOffsetPose(
      traj_point, planner_param_.vehicle_param.base_to_front, -lateral_offset, 0);

    passing_line_left.push_back(offset_pose_left.position);
    passing_line_right.push_back(offset_pose_right.position);
  }

  std::vector<std::vector<geometry_msgs::msg::Point>> passing_lines;
  passing_lines.push_back(passing_line_left);
  passing_lines.push_back(passing_line_right);

  return passing_lines;
}

// create polygon for passing lines and deceleration line calculated by stopping jerk
// note that this polygon is not closed
boost::optional<std::vector<geometry_msgs::msg::Point>>
DynamicObstacleStopModule::createDetectionAreaPolygon(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & passing_lines,
  const size_t deceleration_line_idx) const
{
  if (passing_lines.size() != 2) {
    return {};
  }

  std::vector<geometry_msgs::msg::Point> detection_area_polygon;
  const auto & line1 = passing_lines.at(0);
  const int poly_corner_idx = std::min(deceleration_line_idx, line1.size() - 1);
  for (int i = 0; i <= poly_corner_idx; i++) {
    const auto & p = line1.at(i);
    detection_area_polygon.push_back(p);
  }

  // push points from the end to create the polygon
  const auto & line2 = passing_lines.at(1);
  for (int i = poly_corner_idx; i >= 0; i--) {
    const auto & p = line2.at(i);
    detection_area_polygon.push_back(p);
  }

  return detection_area_polygon;
}

pcl::PointCloud<pcl::PointXYZ> DynamicObstacleStopModule::pointsWithinPolygon(
  const std::vector<geometry_msgs::msg::Point> & polygon,
  const pcl::PointCloud<pcl::PointXYZ> & candidate_points) const
{
  // convert to boost type
  const tier4_autoware_utils::Polygon2d bg_poly =
    dynamic_obstacle_stop_utils::createBoostPolyFromMsg(polygon);

  // find points in detection area
  pcl::PointCloud<pcl::PointXYZ> within_points;
  for (const auto & p : candidate_points) {
    tier4_autoware_utils::Point2d point(p.x, p.y);

    if (!bg::covered_by(point, bg_poly)) {
      continue;
    }

    within_points.push_back(p);
  }

  return within_points;
}

boost::optional<DynamicObstacle> DynamicObstacleStopModule::detectCollision(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const Trajectory & trajectory) const
{
  if (trajectory.points.size() < 2) {
    RCLCPP_WARN_STREAM(logger_, "trajectory doesn't have enough points.");
    return {};
  }

  // detect collision with obstacles from the nearest trajectory point to the end
  // ignore the travel time from current pose to nearest trajectory point?
  // TODO(Tomohito Ando): parameter
  float travel_time = 0.0;
  float dist_sum = 0.0;
  for (size_t idx = 1; idx < trajectory.points.size(); idx++) {
    const auto & p1 = trajectory.points.at(idx - 1);
    const auto & p2 = trajectory.points.at(idx);
    const float prev_vel = std::max(
      p1.longitudinal_velocity_mps, planner_param_.dynamic_obstacle_stop.min_vel_ego_kmph / 3.6f);
    const float ds = tier4_autoware_utils::calcDistance2d(p1, p2);

    // calculate travel time from nearest point to p2
    travel_time += ds / prev_vel;
    dist_sum += ds;

    // skip collision detection to reduce calculation time
    if (idx != 1 && dist_sum < planner_param_.dynamic_obstacle_stop.detection_span) {
      continue;
    }
    dist_sum = 0.0;

    const auto vehicle_poly = createVehiclePolygon(p2.pose);

    debug_ptr_->pushDebugPolygons(vehicle_poly);
    std::stringstream sstream;
    sstream << std::setprecision(4) << travel_time << "s";
    debug_ptr_->pushDebugTexts(dynamic_obstacle_stop_utils::createDebugText(
      sstream.str(), p2.pose, /* lateral_offset */ 3.0));

    auto obstacles_collision =
      checkCollisionWithObstacles(dynamic_obstacles, vehicle_poly, travel_time);
    if (obstacles_collision.empty()) {
      continue;
    }

    const auto obstacle_selected =
      findNearestCollisionObstacle(trajectory, p2.pose, obstacles_collision);
    if (!obstacle_selected) {
      continue;
    }

    // debug
    {
      std::stringstream sstream;
      sstream << std::setprecision(4) << "ttc: " << std::to_string(travel_time) << "s";
      debug_ptr_->pushDebugTexts(dynamic_obstacle_stop_utils::createDebugText(
        sstream.str(), obstacle_selected->nearest_collision_point_));

      debug_ptr_->pushDebugPoints(obstacle_selected->collision_points_);
      debug_ptr_->pushDebugPoints(obstacle_selected->nearest_collision_point_, PointType::Red);
    }

    return obstacle_selected;
  }

  // no collision detected
  return {};
}

boost::optional<DynamicObstacle> DynamicObstacleStopModule::findNearestCollisionObstacle(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & base_pose,
  std::vector<DynamicObstacle> & dynamic_obstacles) const
{
  // sort obstacles with distance from ego
  std::sort(
    dynamic_obstacles.begin(), dynamic_obstacles.end(),
    [&trajectory, &base_pose](const auto & lhs, const auto & rhs) -> bool {
      const auto dist_lhs = tier4_autoware_utils::calcSignedArcLength(
        trajectory.points, base_pose.position, lhs.pose_.position);
      const auto dist_rhs = tier4_autoware_utils::calcSignedArcLength(
        trajectory.points, base_pose.position, rhs.pose_.position);

      return dist_lhs < dist_rhs;
    });

  // select obstacle to decelerate from the nearest obstacle
  DynamicObstacle obstacle_collision;
  for (const auto & obstacle : dynamic_obstacles) {
    const auto obstacle_same_side_points = dynamic_obstacle_stop_utils::findLateralSameSidePoints(
      obstacle.collision_points_, base_pose, obstacle.pose_.position);

    const auto nearest_collision_point = dynamic_obstacle_stop_utils::findLongitudinalNearestPoint(
      trajectory, base_pose.position, obstacle_same_side_points);

    const auto collision_position_from_ego_front =
      calcCollisionPositionOfVehicleSide(nearest_collision_point, base_pose);

    // if position of collision on ego side is less than passing margin,
    // which is considered to be collision
    // TODO(Tomohito Ando): calculate collision position more precisely
    if (collision_position_from_ego_front < planner_param_.dynamic_obstacle_stop.passing_margin) {
      debug_ptr_->setDebugValues(
        DebugValues::TYPE::COLLISION_POS_FROM_EGO_FRONT, collision_position_from_ego_front);

      obstacle_collision = obstacle;
      obstacle_collision.nearest_collision_point_ = nearest_collision_point;
      return obstacle_collision;
    }

    // the obstacle is considered to be able to pass
    debug_ptr_->setAccelReason(AccelReason::PASS);
  }

  // no collision points
  return {};
}

geometry_msgs::msg::Point DynamicObstacleStopModule::selectCollisionPoint(
  const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & base_pose,
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose) const
{
  const auto obstacle_same_side_points = dynamic_obstacle_stop_utils::findLateralSameSidePoints(
    dynamic_obstacle.collision_points_, base_pose, dynamic_obstacle.pose_.position);
  const auto nearest_collision_point = dynamic_obstacle_stop_utils::findLongitudinalNearestPoint(
    trajectory, current_pose.position, obstacle_same_side_points);

  return nearest_collision_point;
}

// calculate longitudinal offset of collision point from vehicle front
float DynamicObstacleStopModule::calcCollisionPositionOfVehicleSide(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & base_pose) const
{
  const auto vehicle_front_pose = tier4_autoware_utils::calcOffsetPose(
    base_pose, planner_param_.vehicle_param.base_to_front, 0, 0);
  const auto longitudinal_offset_from_front =
    std::abs(tier4_autoware_utils::calcLongitudinalDeviation(vehicle_front_pose, point));

  return longitudinal_offset_from_front;
}

/**
 *    p4               p1
 *    +----------------+
 *    |  ↑-> (base)   |
 *    +----------------+
 *    p3               p2
 */
std::vector<geometry_msgs::msg::Point> DynamicObstacleStopModule::createVehiclePolygon(
  const geometry_msgs::msg::Pose & base_pose) const
{
  const float base_to_rear = planner_param_.vehicle_param.base_to_rear;
  const float base_to_front = planner_param_.vehicle_param.base_to_front;
  const float half_width = planner_param_.vehicle_param.width / 2.0;

  const auto p1 = tier4_autoware_utils::calcOffsetPose(base_pose, base_to_front, half_width, 0.0);
  const auto p2 = tier4_autoware_utils::calcOffsetPose(base_pose, base_to_front, -half_width, 0.0);
  const auto p3 = tier4_autoware_utils::calcOffsetPose(base_pose, -base_to_rear, -half_width, 0.0);
  const auto p4 = tier4_autoware_utils::calcOffsetPose(base_pose, -base_to_rear, half_width, 0.0);

  std::vector<geometry_msgs::msg::Point> vehicle_poly;
  vehicle_poly.push_back(p1.position);
  vehicle_poly.push_back(p2.position);
  vehicle_poly.push_back(p3.position);
  vehicle_poly.push_back(p4.position);

  return vehicle_poly;
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::checkCollisionWithObstacles(
  const std::vector<DynamicObstacle> & dynamic_obstacles,
  std::vector<geometry_msgs::msg::Point> poly, const float travel_time) const
{
  const auto bg_poly_vehicle = dynamic_obstacle_stop_utils::createBoostPolyFromMsg(poly);

  // check collision for each objects
  std::vector<DynamicObstacle> obstacles_collision;
  for (const auto & obstacle : dynamic_obstacles) {
    // get classification that has highest probability
    const auto classification =
      dynamic_obstacle_stop_utils::getHighestProbLabel(obstacle.classification_);

    // detect only pedestrian and bicycle
    if (
      classification != ObjectClassification::PEDESTRIAN &&
      classification != ObjectClassification::BICYCLE) {
      continue;
    }

    // calculate predicted obstacle pose for min velocity and max veloicty
    const auto predicted_obstacle_pose_min_vel =
      calcPredictedObstaclePose(obstacle.predicted_paths_, travel_time, obstacle.min_velocity_mps_);
    const auto predicted_obstacle_pose_max_vel =
      calcPredictedObstaclePose(obstacle.predicted_paths_, travel_time, obstacle.max_velocity_mps_);
    if (!predicted_obstacle_pose_min_vel || !predicted_obstacle_pose_max_vel) {
      continue;
    }
    const PoseWithRange pose_with_range = {
      *predicted_obstacle_pose_min_vel, *predicted_obstacle_pose_max_vel};

    std::vector<geometry_msgs::msg::Point> collision_points;
    const bool collision_detected =
      checkCollisionWithShape(bg_poly_vehicle, pose_with_range, obstacle.shape_, collision_points);

    if (!collision_detected) {
      continue;
    }

    DynamicObstacle obstacle_collision = obstacle;
    obstacle_collision.collision_points_ = collision_points;
    obstacles_collision.emplace_back(obstacle_collision);
  }

  return obstacles_collision;
}

// calculate the predicted pose of the obstacle on the predicted path with given travel time
// assume that the obstacle moves with constant velocity
boost::optional<geometry_msgs::msg::Pose> DynamicObstacleStopModule::calcPredictedObstaclePose(
  const std::vector<DynamicObstacle::PredictedPath> & predicted_paths, const float travel_time,
  const float velocity_mps) const
{
  // use the path that has highest confidence for now
  const auto predicted_path =
    dynamic_obstacle_stop_utils::getHighestConfidencePath(predicted_paths);

  if (predicted_path.size() < 2) {
    RCLCPP_WARN_STREAM(logger_, "predicted path doesn't have enough points");
    return {};
  }

  if (
    travel_time < std::numeric_limits<float>::epsilon() ||
    velocity_mps < std::numeric_limits<float>::epsilon()) {
    return predicted_path.at(0);
  }

  // calculate predicted pose
  float time_sum = 0.0;
  for (size_t i = 1; i < predicted_path.size(); i++) {
    const auto & p1 = predicted_path.at(i - 1);
    const auto & p2 = predicted_path.at(i);

    const float ds = tier4_autoware_utils::calcDistance2d(p1, p2);
    const float dt = ds / velocity_mps;

    // apply linear interpolation between the predicted path points
    if (time_sum + dt > travel_time) {
      const float time_remaining = travel_time - time_sum;
      const float ratio = time_remaining / dt;
      return dynamic_obstacle_stop_utils::lerpByPose(p1, p2, ratio);
    }

    time_sum += dt;
  }

  // reach the end of the predicted path
  return predicted_path.back();
}

bool DynamicObstacleStopModule::checkCollisionWithShape(
  const tier4_autoware_utils::Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
  const Shape & shape, std::vector<geometry_msgs::msg::Point> & collision_points) const
{
  bool collision_detected = false;
  switch (shape.type) {
    case Shape::CYLINDER:
      collision_detected = checkCollisionWithCylinder(
        vehicle_polygon, pose_with_range, shape.dimensions.x / 2.0, collision_points);
      break;

    case Shape::BOUNDING_BOX:
      collision_detected = checkCollisionWithBoundingBox(
        vehicle_polygon, pose_with_range, shape.dimensions, collision_points);
      break;

    case Shape::POLYGON:
      collision_detected = checkCollisionWithPolygon();
      break;

    default:
      break;
  }

  return collision_detected;
}

bool DynamicObstacleStopModule::checkCollisionWithCylinder(
  const tier4_autoware_utils::Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
  const float radius, std::vector<geometry_msgs::msg::Point> & collision_points) const
{
  // create bounding box for min and max velocity point
  const auto bounding_box_for_points =
    createBoundingBoxForRangedPoints(pose_with_range, radius, radius);
  const auto bg_bounding_box_for_points =
    dynamic_obstacle_stop_utils::createBoostPolyFromMsg(bounding_box_for_points);

  // debug
  debug_ptr_->pushDebugPolygons(bounding_box_for_points);

  // check collision with 2d polygon
  std::vector<tier4_autoware_utils::Point2d> collision_points_bg;
  bg::intersection(vehicle_polygon, bg_bounding_box_for_points, collision_points_bg);

  // no collision detected
  if (collision_points_bg.empty()) {
    return false;
  }

  for (const auto & p : collision_points_bg) {
    const auto p_msg =
      tier4_autoware_utils::createPoint(p.x(), p.y(), pose_with_range.pose_min.position.z);
    collision_points.emplace_back(p_msg);
  }

  return true;
}

// create 2D bounding box for two points
// Box is better to reduce calculation cost?
std::vector<geometry_msgs::msg::Point> DynamicObstacleStopModule::createBoundingBoxForRangedPoints(
  const PoseWithRange & pose_with_range, const float x_offset, const float y_offset) const
{
  const auto dist_p1_p2 =
    tier4_autoware_utils::calcDistance2d(pose_with_range.pose_min, pose_with_range.pose_max);

  geometry_msgs::msg::Pose p_min_to_p_max;
  const auto azimuth_angle = tier4_autoware_utils::calcAzimuthAngle(
    pose_with_range.pose_min.position, pose_with_range.pose_max.position);
  p_min_to_p_max.position = pose_with_range.pose_min.position;
  p_min_to_p_max.orientation = tier4_autoware_utils::createQuaternionFromYaw(azimuth_angle);

  std::vector<geometry_msgs::msg::Point> poly;
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, dist_p1_p2 + x_offset, y_offset, 0.0)
      .position);
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, dist_p1_p2 + x_offset, -y_offset, 0.0)
      .position);
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, -x_offset, -y_offset, 0.0).position);
  poly.emplace_back(
    tier4_autoware_utils::calcOffsetPose(p_min_to_p_max, -x_offset, y_offset, 0.0).position);

  return poly;
}

bool DynamicObstacleStopModule::checkCollisionWithBoundingBox(
  const tier4_autoware_utils::Polygon2d & vehicle_polygon, const PoseWithRange pose_with_range,
  const geometry_msgs::msg::Vector3 & dimension,
  std::vector<geometry_msgs::msg::Point> & collision_points) const
{
  // create bounding box for min and max velocity point
  const auto bounding_box =
    createBoundingBoxForRangedPoints(pose_with_range, dimension.x / 2.0, dimension.y / 2.0);
  const auto bg_bounding_box = dynamic_obstacle_stop_utils::createBoostPolyFromMsg(bounding_box);

  // debug
  debug_ptr_->pushDebugPolygons(bounding_box);

  // check collision with 2d polygon
  std::vector<tier4_autoware_utils::Point2d> collision_points_bg;
  bg::intersection(vehicle_polygon, bg_bounding_box, collision_points_bg);

  // no collision detected
  if (collision_points_bg.empty()) {
    return false;
  }

  for (const auto & p : collision_points_bg) {
    const auto p_msg =
      tier4_autoware_utils::createPoint(p.x(), p.y(), pose_with_range.pose_min.position.z);
    collision_points.emplace_back(p_msg);
  }

  return true;
}

bool DynamicObstacleStopModule::checkCollisionWithPolygon() const
{
  RCLCPP_WARN_STREAM(logger_, "detection for POLYGON type is not implemented yet.");

  return false;
}

TrajectoryPoint DynamicObstacleStopModule::createExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point) const
{
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose =
    tier4_autoware_utils::calcOffsetPose(goal_point.pose, extend_distance, 0.0, 0.0);
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

Trajectory DynamicObstacleStopModule::extendTrajectory(
  const Trajectory & input, const double extend_distance) const
{
  Trajectory output = input;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output;
  }

  const auto goal_point = input.points.back();
  const double interpolation_distance = 0.1;

  double extend_sum = interpolation_distance;
  while (extend_sum <= (extend_distance - interpolation_distance)) {
    const auto extend_trajectory_point = createExtendTrajectoryPoint(extend_sum, goal_point);
    output.points.push_back(extend_trajectory_point);
    extend_sum += interpolation_distance;
  }
  const auto extend_trajectory_point = createExtendTrajectoryPoint(extend_distance, goal_point);
  output.points.push_back(extend_trajectory_point);

  return output;
}

// trim trajectory from self_pose to trim_distance
Trajectory DynamicObstacleStopModule::trimTrajectoryFromSelfPose(
  const Trajectory & input, const geometry_msgs::msg::Pose & self_pose,
  const double trim_distance) const
{
  // findNearestSegmentIndex finds the index behind of the specified point
  // so add 1 to the index to get the nearest point ahead of the ego
  const size_t nearest_idx =
    tier4_autoware_utils::findNearestSegmentIndex(input.points, self_pose.position) + 1;

  Trajectory output{};
  double dist_sum = 0;
  for (size_t i = nearest_idx; i < input.points.size(); ++i) {
    output.points.push_back(input.points.at(i));

    if (i != nearest_idx) {
      dist_sum += tier4_autoware_utils::calcDistance2d(input.points.at(i - 1), input.points.at(i));
    }

    if (dist_sum > trim_distance) {
      break;
    }
  }
  output.header = input.header;

  return output;
}

// todo: move to utils
size_t DynamicObstacleStopModule::calcTrajectoryIndexByLength(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
  const double target_length) const
{
  const size_t nearest_index =
    tier4_autoware_utils::findNearestIndex(trajectory.points, current_pose.position);
  if (target_length < 0) {
    return nearest_index;
  }

  for (size_t i = nearest_index; i < trajectory.points.size(); i++) {
    double length_sum =
      tier4_autoware_utils::calcSignedArcLength(trajectory.points, current_pose.position, i);
    if (length_sum > target_length) {
      return i;
    }
  }

  // reach the end of the trajectory, so return the last index
  return trajectory.points.size() - 1;
}

// todo: move to utils
size_t DynamicObstacleStopModule::calcTrajectoryIndexByLengthReverse(
  const Trajectory & trajectory, const geometry_msgs::msg::Point & src_point,
  const float target_length) const
{
  const auto nearest_seg_idx =
    tier4_autoware_utils::findNearestSegmentIndex(trajectory.points, src_point);
  if (nearest_seg_idx == 0) {
    return 0;
  }

  for (size_t i = nearest_seg_idx; i > 0; i--) {
    const auto length_sum =
      std::abs(tier4_autoware_utils::calcSignedArcLength(trajectory.points, src_point, i));
    if (length_sum > target_length) {
      return i + 1;
    }
  }

  // reach the beginning of the path
  return 0;
}

boost::optional<geometry_msgs::msg::Pose> DynamicObstacleStopModule::calcStopPoint(
  const boost::optional<DynamicObstacle> & dynamic_obstacle, const Trajectory & trajectory,
  const geometry_msgs::msg::Pose & current_pose, const float current_vel,
  const float current_acc) const
{
  // no obstacles
  if (!dynamic_obstacle) {
    return {};
  }

  // calculate distance to collision with the obstacle
  float dist_to_collision;
  const float dist_to_collision_point = tier4_autoware_utils::calcSignedArcLength(
    trajectory.points, current_pose.position, dynamic_obstacle->nearest_collision_point_);
  dist_to_collision = dist_to_collision_point - planner_param_.vehicle_param.base_to_front;

  // calculate distance needed to stop with jerk and acc constraints
  const float target_vel = 0.0;
  const float jerk_dec = planner_param_.dynamic_obstacle_stop.stop_start_jerk_dec;
  const float jerk_acc = std::abs(jerk_dec);
  const float planning_dec = jerk_dec < planner_param_.common.normal_min_jerk
                               ? planner_param_.common.limit_min_acc
                               : planner_param_.common.normal_min_acc;
  auto stop_dist = dynamic_obstacle_stop_utils::calcDecelDistWithJerkAndAccConstraints(
    current_vel, target_vel, current_acc, planning_dec, jerk_acc, jerk_dec);
  if (!stop_dist) {
    RCLCPP_WARN_STREAM(logger_, "failed to calculate stop distance.");

    // force to insert zero velocity
    stop_dist = boost::make_optional<double>(dist_to_collision);
  }

  // debug
  {
    debug_ptr_->setDebugValues(DebugValues::TYPE::STOP_DISTANCE, *stop_dist);

    const auto vehicle_stop_idx = calcTrajectoryIndexByLength(
      trajectory, current_pose, stop_dist.get() + planner_param_.vehicle_param.base_to_front);
    const auto & p = trajectory.points.at(vehicle_stop_idx).pose;
    debug_ptr_->pushDebugPoints(p.position, PointType::Yellow);
  }

  // vehicle have to decelerate if there is not enough distance with stop_start_jerk_dec
  const bool deceleration_needed =
    *stop_dist > dist_to_collision - planner_param_.dynamic_obstacle_stop.stop_margin;
  // avoid acceleration when ego is decelerating
  // TODO(Tomohito Ando): replace with more appropriate method
  constexpr float epsilon = 1.0e-2;
  constexpr float stopping_vel_mps = 2.5 / 3.6;
  const bool is_stopping = current_vel < stopping_vel_mps && current_acc < epsilon;
  if (!deceleration_needed && !is_stopping) {
    debug_ptr_->setAccelReason(AccelReason::LOW_JERK);
    return {};
  }

  // calculate index of stop point
  const float base_to_collision_point =
    planner_param_.dynamic_obstacle_stop.stop_margin + planner_param_.vehicle_param.base_to_front;
  const size_t stop_index = calcTrajectoryIndexByLengthReverse(
    trajectory, dynamic_obstacle->nearest_collision_point_, base_to_collision_point);
  const auto & stop_point = trajectory.points.at(stop_index).pose;

  // debug
  debug_ptr_->setAccelReason(AccelReason::STOP);
  debug_ptr_->pushStopPose(stop_point);

  return stop_point;
}

void DynamicObstacleStopModule::insertStopPoint(
  const boost::optional<geometry_msgs::msg::Pose> stop_point,
  autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  // no stop point
  if (!stop_point) {
    RCLCPP_DEBUG_STREAM(logger_, "already has same point");
    return;
  }

  // find nearest point index behind the stop point
  const auto nearest_seg_idx =
    dynamic_obstacle_stop_utils::findNearestSegmentIndex(path.points, stop_point->position);
  auto insert_idx = nearest_seg_idx + 1;

  // to PathPointWithLaneId
  autoware_auto_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(nearest_seg_idx);
  stop_point_with_lane_id.point.pose = *stop_point;

  planning_utils::insertVelocity(path, stop_point_with_lane_id, 0.0, insert_idx);
}

void DynamicObstacleStopModule::insertVelocityWithApproaching(
  const boost::optional<DynamicObstacle> & dynamic_obstacle,
  const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
  const Trajectory & trajectory, autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  // no obstacles
  if (!dynamic_obstacle) {
    state_ = State::GO;
    return;
  }

  const auto longitudinal_offset_to_collision_point =
    tier4_autoware_utils::calcSignedArcLength(
      trajectory.points, current_pose.position, dynamic_obstacle->nearest_collision_point_) -
    planner_param_.vehicle_param.base_to_front;
  // enough distance to the obstacle
  if (
    longitudinal_offset_to_collision_point >
    planner_param_.dynamic_obstacle_stop.stop_margin + planner_param_.approaching.dist_thresh) {
    state_ = State::GO;
  }

  switch (state_) {
    case State::GO: {
      if (
        planner_data_->current_velocity->twist.linear.x < planner_param_.approaching.stop_thresh) {
        RCLCPP_DEBUG_STREAM(logger_, "transit to STOP state.");
        stop_time_ = clock_->now();
        state_ = State::STOP;
      }

      const auto stop_point =
        calcStopPoint(dynamic_obstacle, trajectory, current_pose, current_vel, current_acc);
      insertStopPoint(stop_point, path);
      break;
    }

    case State::STOP: {
      RCLCPP_DEBUG_STREAM(logger_, "STOP state");
      const auto elapsed_time = (clock_->now() - stop_time_).seconds();
      state_ =
        elapsed_time > planner_param_.approaching.stop_time_thresh ? State::APPROACH : State::STOP;
      RCLCPP_DEBUG_STREAM(logger_, "elapsed time: " << elapsed_time);

      const auto stop_point =
        calcStopPoint(dynamic_obstacle, trajectory, current_pose, current_vel, current_acc);
      insertStopPoint(stop_point, path);
      break;
    }

    case State::APPROACH: {
      RCLCPP_DEBUG_STREAM(logger_, "APPROACH state");
      insertApproachingVelocity(
        *dynamic_obstacle, current_pose, planner_param_.approaching.limit_vel_kmph / 3.6,
        planner_param_.approaching.margin, trajectory, path);
      debug_ptr_->setAccelReason(AccelReason::STOP);
      break;
    }

    default: {
      RCLCPP_WARN_STREAM(logger_, "invalid state");
      break;
    }
  }
}

void DynamicObstacleStopModule::insertApproachingVelocity(
  const DynamicObstacle & dynamic_obstacle, const geometry_msgs::msg::Pose & current_pose,
  const float approaching_vel, const float approach_margin, const Trajectory & trajectory,
  autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  // isnert slow down velocity from nearest segment point
  const auto nearest_seg_idx =
    dynamic_obstacle_stop_utils::findNearestSegmentIndex(path.points, current_pose.position);
  dynamic_obstacle_stop_utils::insertPathVelocityFromIndexLimited(
    nearest_seg_idx, approaching_vel, path.points);

  // debug
  debug_ptr_->pushDebugPoints(
    path.points.at(nearest_seg_idx).point.pose.position, PointType::Yellow);

  // calculate stop point to insert 0 velocity
  const float base_to_collision_point =
    approach_margin + planner_param_.vehicle_param.base_to_front;
  const auto stop_idx = calcTrajectoryIndexByLengthReverse(
    trajectory, dynamic_obstacle.nearest_collision_point_, base_to_collision_point);
  const auto & stop_point = trajectory.points.at(stop_idx).pose;

  // debug
  debug_ptr_->pushStopPose(stop_point);

  const auto nearest_seg_idx_stop =
    dynamic_obstacle_stop_utils::findNearestSegmentIndex(path.points, stop_point.position);
  auto insert_idx_stop = nearest_seg_idx_stop + 1;

  // to PathPointWithLaneId
  // use lane id of point behind inserted point
  autoware_auto_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(nearest_seg_idx_stop);
  stop_point_with_lane_id.point.pose = stop_point;

  planning_utils::insertVelocity(path, stop_point_with_lane_id, 0.0, insert_idx_stop);
}

void DynamicObstacleStopModule::addPointsBehindBase(
  const Trajectory & input_traj, const geometry_msgs::msg::Pose & current_pose,
  Trajectory & output_traj) const
{
  const auto base_nearest_idx =
    tier4_autoware_utils::findNearestIndex(input_traj.points, current_pose.position);

  output_traj.points.insert(
    output_traj.points.begin(), input_traj.points.begin(),
    input_traj.points.begin() + base_nearest_idx);
}

Trajectory DynamicObstacleStopModule::toTrajectoryMsg(
  const TrajectoryPoints & points, const std_msgs::msg::Header & header) const
{
  auto trajectory = tier4_autoware_utils::convertToTrajectory(points);
  trajectory.header = header;
  return trajectory;
}

boost::optional<Trajectory> DynamicObstacleStopModule::applySmoother(
  const Trajectory & input_trajectory, const geometry_msgs::msg::Pose & current_pose,
  const double initial_vel, const double initial_acc) const
{
  // TODO(Tomohito Ando): apply external velocity limit to be consistent with smoother
  // apply max veloicty to do the same calculation as smoother
  Trajectory velocity_limited_trajectory = input_trajectory;
  dynamic_obstacle_stop_utils::applyMaximumVelocityLimit(
    planner_data_->velocity_limit->max_velocity, velocity_limited_trajectory);

  // smooth the velocity by using smoother
  const auto smoother_input =
    tier4_autoware_utils::convertToTrajectoryPointArray(velocity_limited_trajectory);
  TrajectoryPoints smoothed_trajectory_points{};
  if (!smoothVelocity(
        smoother_input, current_pose, initial_vel, initial_acc, smoothed_trajectory_points)) {
    return {};
  }

  const auto smoothed_trajectory =
    toTrajectoryMsg(smoothed_trajectory_points, velocity_limited_trajectory.header);

  return smoothed_trajectory;
}

bool DynamicObstacleStopModule::smoothVelocity(
  const TrajectoryPoints & input, const geometry_msgs::msg::Pose & current_pose,
  const double initial_vel, const double initial_acc, TrajectoryPoints & traj_smoothed) const
{
  // Lateral acceleration limit
  const auto traj_lateral_acc_filtered = smoother_->applyLateralAccelerationFilter(input);
  if (!traj_lateral_acc_filtered) {
    return false;
  }

  // Resample trajectory with ego-velocity based interval distance
  // TODO(Tomohito Ando): parameter
  constexpr double delta_yaw_threshold = 1.0472;
  const auto traj_pre_resampled_closest = tier4_autoware_utils::findNearestIndex(
    *traj_lateral_acc_filtered, current_pose, std::numeric_limits<double>::max(),
    delta_yaw_threshold);
  if (!traj_pre_resampled_closest) {
    RCLCPP_WARN(logger_, "Cannot find closest waypoint for resampled trajectory");
    return false;
  }

  auto traj_resampled = smoother_->resampleTrajectory(
    *traj_lateral_acc_filtered, planner_data_->current_velocity->twist.linear.x,
    *traj_pre_resampled_closest);
  if (!traj_resampled) {
    RCLCPP_WARN(logger_, "Fail to do resampling before the optimization");
    return false;
  }

  // Set 0[m/s] in the terminal point
  if (!traj_resampled->empty()) {
    traj_resampled->back().longitudinal_velocity_mps = 0.0;
  }

  const auto traj_resampled_closest = tier4_autoware_utils::findNearestIndex(
    *traj_resampled, current_pose, std::numeric_limits<double>::max(), delta_yaw_threshold);
  if (!traj_resampled_closest) {
    RCLCPP_WARN(logger_, "Cannot find closest waypoint for resampled trajectory");
    return false;
  }

  // // Calculate initial motion for smoothing
  // double initial_vel{};
  // double initial_acc{};
  // InitializeType type{};
  // std::tie(initial_vel, initial_acc, type) =
  //   calcInitialMotion(*traj_resampled, *traj_resampled_closest, prev_output_);

  // Clip trajectory from closest point
  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(), traj_resampled->begin() + *traj_resampled_closest, traj_resampled->end());

  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother_->apply(initial_vel, initial_acc, clipped, traj_smoothed, debug_trajectories)) {
    RCLCPP_WARN(logger_, "Fail to solve optimization.");
  }

  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled->begin(),
    traj_resampled->begin() + *traj_resampled_closest);

  // // Set 0 velocity after input-stop-point
  // overwriteStopPoint(*traj_resampled, traj_smoothed);

  // For the endpoint of the trajectory
  if (!traj_smoothed.empty()) {
    traj_smoothed.back().longitudinal_velocity_mps = 0.0;
  }

  // // Max velocity filter for safety
  // trajectory_utils::applyMaximumVelocityLimit(
  //   *traj_resampled_closest, traj_smoothed.size(), node_param_.max_velocity, traj_smoothed);

  // // Insert behind velocity for output's consistency
  // insertBehindVelocity(*traj_resampled_closest, type, traj_smoothed);

  return true;
}

void DynamicObstacleStopModule::applyMaxJerkLimit(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & current_pose,
  const float current_vel, const float current_acc,
  autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto stop_point_idx = dynamic_obstacle_stop_utils::findFirstStopPointIdx(path.points);
  if (!stop_point_idx) {
    return;
  }

  const auto stop_point = path.points.at(stop_point_idx.get()).point.pose.position;
  const auto dist_to_stop_point =
    tier4_autoware_utils::calcSignedArcLength(trajectory.points, current_pose.position, stop_point);

  // calculate desired velocity with limited jerk
  const auto jerk_limited_vel = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    planner_param_.slow_down_limit.max_jerk, planner_param_.slow_down_limit.max_acc, current_acc,
    current_vel, dist_to_stop_point);

  // overwrite velocity with limited velocity
  dynamic_obstacle_stop_utils::insertPathVelocityFromIndex(
    stop_point_idx.get(), jerk_limited_vel, path.points);
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::excludeObstaclesOutSideOfPartition(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const Trajectory & trajectory,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!planner_param_.dynamic_obstacle_stop.use_partition_lanelet || partition_lanelets_.empty()) {
    return dynamic_obstacles;
  }

  // extract partitions within detection distance
  BasicPolygons2d close_partitions;
  planning_utils::extractClosePartition(
    current_pose.position, partition_lanelets_, close_partitions,
    planner_param_.dynamic_obstacle_stop.detection_distance);

  // decimate trajectory to reduce calculation time
  constexpr float decimate_step = 1.0;
  const auto decimate_traj =
    dynamic_obstacle_stop_utils::decimateTrajectory(trajectory, decimate_step);

  // exclude obstacles outside of partition
  std::vector<DynamicObstacle> extracted_obstacles = dynamic_obstacles;
  for (const auto & partition : close_partitions) {
    extracted_obstacles = dynamic_obstacle_stop_utils::excludeObstaclesOutSideOfLine(
      extracted_obstacles, decimate_traj, partition);
  }

  return extracted_obstacles;
}

}  // namespace behavior_velocity_planner
