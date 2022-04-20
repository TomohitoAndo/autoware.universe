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

#include "scene_module/dynamic_obstacle_stop/path_utils.hpp"
#include "scene_module/dynamic_obstacle_stop/utils.hpp"
#include "utilization/trajectory_utils.hpp"
#include "utilization/util.hpp"

#include <pcl/filters/voxel_grid.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

DynamicObstacleStopModule::DynamicObstacleStopModule(
  const int64_t module_id, const std::shared_ptr<const PlannerData> & planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const std::shared_ptr<DynamicObstacleStopDebug> & debug_ptr, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  planner_param_(planner_param),
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
  // set planner data
  const auto current_vel = planner_data_->current_velocity->twist.linear.x;
  const auto current_acc = planner_data_->current_accel.get();
  const auto & current_pose = planner_data_->current_pose.pose;

  // smooth velocity of the path to calcute time to collision accurately
  PathWithLaneId smoothed_path;
  if (!smoothPath(*path, smoothed_path, planner_data_)) {
    return true;
  }

  // extend path to consider obstacles after the goal
  const auto extended_smoothed_path = dynamic_obstacle_stop_utils::extendPath(
    smoothed_path, planner_param_.vehicle_param.base_to_front);

  // trim path ahead of the base_link to make calculation easier
  const double trim_distance = planner_param_.dynamic_obstacle_stop.detection_distance;
  const auto trim_smoothed_path = dynamic_obstacle_stop_utils::trimPathFromSelfPose(
    extended_smoothed_path, current_pose, trim_distance);

  // create abstracted dynamic obstacles from objects or points
  const auto dynamic_obstacles = createDynamicObstacles(
    *planner_data_->predicted_objects, planner_data_->compare_map_filtered_pointcloud,
    trim_smoothed_path, current_pose, planner_param_.dynamic_obstacle_stop.detection_method);
  if (!dynamic_obstacles) {
    return true;
  }
  debug_ptr_->setDebugValues(DebugValues::TYPE::NUM_OBSTACLES, dynamic_obstacles->size());

  // extract obstacles using lanelet information
  const auto partition_excluded_obstacles =
    excludeObstaclesOutSideOfPartition(dynamic_obstacles.get(), trim_smoothed_path, current_pose);

  // timer starts
  const auto t1 = std::chrono::system_clock::now();

  // detect collision with dynamic obstacles using velocity planning of ego
  const auto dynamic_obstacle = detectCollision(partition_excluded_obstacles, trim_smoothed_path);

  // timer ends
  const auto t2 = std::chrono::system_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  debug_ptr_->setDebugValues(DebugValues::TYPE::CALCULATION_TIME, elapsed.count() / 1000.0);

  // insert stop point for the detected obstacle
  if (planner_param_.approaching.enable) {
    // approach the obstacle with slow velocity after stopping
    insertVelocityWithApproaching(
      dynamic_obstacle, current_pose, current_vel, current_acc, trim_smoothed_path, *path);
  } else {
    const auto stop_point =
      calcStopPoint(dynamic_obstacle, trim_smoothed_path, current_pose, current_vel, current_acc);
    insertStopPoint(stop_point, *path);
  }

  // apply max jerk limit if the ego can't stop with specified max jerk and acc
  if (planner_param_.slow_down_limit.enable) {
    applyMaxJerkLimit(current_pose, current_vel, current_acc, *path);
  }

  publishDebugValue(
    trim_smoothed_path, partition_excluded_obstacles, dynamic_obstacle, current_pose);

  return true;
}

// create dynamic obstacles for each method
boost::optional<std::vector<DynamicObstacle>> DynamicObstacleStopModule::createDynamicObstacles(
  const PredictedObjects & predicted_objects,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & points, const PathWithLaneId & path,
  const geometry_msgs::msg::Pose & current_pose, const std::string & detection_method) const
{
  using dynamic_obstacle_stop_utils::DetectionMethod;
  const auto detection_method_enum = dynamic_obstacle_stop_utils::toEnum(detection_method);

  switch (detection_method_enum) {
    // create dynamic obstacles from objects
    case DetectionMethod::Object: {
      return createDynamicObstaclesFromObjects(predicted_objects);
    }

    // create dynamic obstacles from objects
    // but overwrite the predicted path to run straight to the path for ego
    case DetectionMethod::ObjectWithoutPath: {
      visualizeDetectionArea(path);
      return createDynamicObstaclesFromObjects(predicted_objects, path);
    }

    // create dynamic obstacles from points
    // predicted path that runs straight to the path for ego is created
    case DetectionMethod::Points: {
      visualizeDetectionArea(path);
      const auto voxel_grid_filtered_points = applyVoxelGridFilter(points);
      // todo: extract points with triangle shaped detection area
      const auto extracted_points =
        extractObstaclePointsWithRectangle(voxel_grid_filtered_points, current_pose);
      return createDynamicObstaclesFromPoints(extracted_points, path);
    }

    default: {
      RCLCPP_WARN_STREAM(logger_, "detection method is invalid.");
      return {};
    }
  }
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::createDynamicObstaclesFromPoints(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud, const PathWithLaneId & path) const
{
  std::vector<DynamicObstacle> dynamic_obstacles{};
  for (const auto & p : obstacle_pointcloud) {
    const auto ros_point = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    DynamicObstacle dynamic_obstacle(planner_param_.dynamic_obstacle);
    dynamic_obstacle.createDynamicObstacle(ros_point, path);
    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::createDynamicObstaclesFromObjects(
  const PredictedObjects & predicted_objects, const PathWithLaneId & path) const
{
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & object : predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle(planner_param_.dynamic_obstacle);
    dynamic_obstacle.createDynamicObstacle(object, path);

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

void DynamicObstacleStopModule::visualizeDetectionArea(const PathWithLaneId & smoothed_path) const
{
  // calculate distance needed to stop with jerk and acc constraints
  const float initial_vel = planner_data_->current_velocity->twist.linear.x;
  const float initial_acc = planner_data_->current_accel.get();
  const float target_vel = 0.0;
  const float jerk_dec = planner_param_.dynamic_obstacle_stop.stop_start_jerk_dec;
  const float jerk_acc = std::abs(jerk_dec);
  const float planning_dec = jerk_dec < planner_param_.common.normal_min_jerk
                               ? planner_param_.common.limit_min_acc
                               : planner_param_.common.normal_min_acc;
  auto stop_dist = dynamic_obstacle_stop_utils::calcDecelDistWithJerkAndAccConstraints(
    initial_vel, target_vel, initial_acc, planning_dec, jerk_acc, jerk_dec);

  if (!stop_dist) {
    return;
  }

  DetectionRange da_range;
  const float obstacle_vel_mps = planner_param_.dynamic_obstacle.max_vel_kmph / 3.6;
  const float predict_time_max =
    planner_param_.dynamic_obstacle.time_step * planner_param_.dynamic_obstacle.path_size;
  da_range.interval = planner_param_.dynamic_obstacle_stop.detection_distance;
  da_range.min_longitudinal_distance = planner_param_.vehicle_param.base_to_front;
  da_range.max_longitudinal_distance = *stop_dist + planner_param_.vehicle_param.base_to_front +
                                       planner_param_.dynamic_obstacle_stop.stop_margin;
  da_range.min_lateral_distance = planner_param_.vehicle_param.width / 2.0;
  da_range.max_lateral_distance = obstacle_vel_mps * predict_time_max;
  Polygons2d detection_area_poly;
  planning_utils::createDetectionAreaPolygons(
    detection_area_poly, smoothed_path, da_range,
    planner_param_.dynamic_obstacle.max_vel_kmph / 3.6);

  for (const auto & poly : detection_area_poly) {
    debug_ptr_->pushDetectionAreaPolygons(poly);
  }
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
  const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path) const
{
  if (path.points.size() < 2) {
    RCLCPP_WARN_STREAM(logger_, "path doesn't have enough points.");
    return {};
  }

  // detect collision with obstacles from the nearest path point to the end
  // ignore the travel time from current pose to nearest path point?
  float travel_time = 0.0;
  float dist_sum = 0.0;
  for (size_t idx = 1; idx < path.points.size(); idx++) {
    const auto & p1 = path.points.at(idx - 1).point;
    const auto & p2 = path.points.at(idx).point;
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

    // debug
    {
      debug_ptr_->pushDebugPolygons(vehicle_poly);
      std::stringstream sstream;
      sstream << std::setprecision(4) << travel_time << "s";
      debug_ptr_->pushDebugTexts(sstream.str(), p2.pose, /* lateral_offset */ 3.0);
    }

    auto obstacles_collision =
      checkCollisionWithObstacles(dynamic_obstacles, vehicle_poly, travel_time);
    if (obstacles_collision.empty()) {
      continue;
    }

    const auto obstacle_selected = findNearestCollisionObstacle(path, p2.pose, obstacles_collision);
    if (!obstacle_selected) {
      continue;
    }

    // debug
    {
      std::stringstream sstream;
      sstream << std::setprecision(4) << "ttc: " << std::to_string(travel_time) << "s";
      debug_ptr_->pushDebugTexts(sstream.str(), obstacle_selected->nearest_collision_point);
      debug_ptr_->pushDebugPoints(obstacle_selected->collision_points);
      debug_ptr_->pushDebugPoints(obstacle_selected->nearest_collision_point, PointType::Red);
    }

    return obstacle_selected;
  }

  // no collision detected
  return {};
}

boost::optional<DynamicObstacle> DynamicObstacleStopModule::findNearestCollisionObstacle(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & base_pose,
  std::vector<DynamicObstacle> & dynamic_obstacles) const
{
  // sort obstacles with distance from ego
  std::sort(
    dynamic_obstacles.begin(), dynamic_obstacles.end(),
    [&path, &base_pose](const auto & lhs, const auto & rhs) -> bool {
      const auto dist_lhs = tier4_autoware_utils::calcSignedArcLength(
        path.points, base_pose.position, lhs.pose.position);
      const auto dist_rhs = tier4_autoware_utils::calcSignedArcLength(
        path.points, base_pose.position, rhs.pose.position);

      return dist_lhs < dist_rhs;
    });

  // select obstacle to decelerate from the nearest obstacle
  DynamicObstacle obstacle_collision;
  for (const auto & obstacle : dynamic_obstacles) {
    const auto obstacle_same_side_points = dynamic_obstacle_stop_utils::findLateralSameSidePoints(
      obstacle.collision_points, base_pose, obstacle.pose.position);

    const auto nearest_collision_point = dynamic_obstacle_stop_utils::findLongitudinalNearestPoint(
      path.points, base_pose.position, obstacle_same_side_points);

    const auto collision_position_from_ego_front =
      calcCollisionPositionOfVehicleSide(nearest_collision_point, base_pose);

    // if position of collision on ego side is less than passing margin,
    // which is considered to be collision
    // TODO(Tomohito Ando): calculate collision position more precisely
    if (collision_position_from_ego_front < planner_param_.dynamic_obstacle_stop.passing_margin) {
      debug_ptr_->setDebugValues(
        DebugValues::TYPE::COLLISION_POS_FROM_EGO_FRONT, collision_position_from_ego_front);

      obstacle_collision = obstacle;
      obstacle_collision.nearest_collision_point = nearest_collision_point;
      return obstacle_collision;
    }

    // the obstacle is considered to be able to pass
    debug_ptr_->setAccelReason(DynamicObstacleStopDebug::AccelReason::PASS);
  }

  // no collision points
  return {};
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
      dynamic_obstacle_stop_utils::getHighestProbLabel(obstacle.classifications);

    // detect only pedestrian and bicycle
    if (
      classification != ObjectClassification::PEDESTRIAN &&
      classification != ObjectClassification::BICYCLE) {
      continue;
    }

    // calculate predicted obstacle pose for min velocity and max veloicty
    const auto predicted_obstacle_pose_min_vel =
      calcPredictedObstaclePose(obstacle.predicted_paths, travel_time, obstacle.min_velocity_mps);
    const auto predicted_obstacle_pose_max_vel =
      calcPredictedObstaclePose(obstacle.predicted_paths, travel_time, obstacle.max_velocity_mps);
    if (!predicted_obstacle_pose_min_vel || !predicted_obstacle_pose_max_vel) {
      continue;
    }
    const PoseWithRange pose_with_range = {
      *predicted_obstacle_pose_min_vel, *predicted_obstacle_pose_max_vel};

    std::vector<geometry_msgs::msg::Point> collision_points;
    const bool collision_detected =
      checkCollisionWithShape(bg_poly_vehicle, pose_with_range, obstacle.shape, collision_points);

    if (!collision_detected) {
      continue;
    }

    DynamicObstacle obstacle_collision = obstacle;
    obstacle_collision.collision_points = collision_points;
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

boost::optional<geometry_msgs::msg::Pose> DynamicObstacleStopModule::calcStopPoint(
  const boost::optional<DynamicObstacle> & dynamic_obstacle, const PathWithLaneId & path,
  const geometry_msgs::msg::Pose & current_pose, const float current_vel,
  const float current_acc) const
{
  // no obstacles
  if (!dynamic_obstacle) {
    return {};
  }

  // calculate distance to collision with the obstacle
  const float dist_to_collision_point = tier4_autoware_utils::calcSignedArcLength(
    path.points, current_pose.position, dynamic_obstacle->nearest_collision_point);
  const float dist_to_collision =
    dist_to_collision_point - planner_param_.vehicle_param.base_to_front;

  // insert the stop point without considering the distance from the obstacle
  // smoother will calculate appropriate jerk for deceleration
  if (!planner_param_.dynamic_obstacle_stop.specify_decel_jerk) {
    // calculate index of stop point
    const float base_to_collision_point =
      planner_param_.dynamic_obstacle_stop.stop_margin + planner_param_.vehicle_param.base_to_front;
    const size_t stop_index = dynamic_obstacle_stop_utils::calcIndexByLengthReverse(
      path.points, dynamic_obstacle->nearest_collision_point, base_to_collision_point);
    const auto & stop_point = path.points.at(stop_index).point.pose;

    // debug
    debug_ptr_->setAccelReason(DynamicObstacleStopDebug::AccelReason::STOP);
    debug_ptr_->pushStopPose(stop_point);

    return stop_point;
  }

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
    const float base_to_obstacle =
      planner_param_.vehicle_param.base_to_front + planner_param_.dynamic_obstacle_stop.stop_margin;
    const auto vehicle_stop_idx = dynamic_obstacle_stop_utils::calcIndexByLength(
      path.points, current_pose, stop_dist.get() + base_to_obstacle);
    const auto & p = path.points.at(vehicle_stop_idx).point.pose.position;
    debug_ptr_->pushDebugPoints(p, PointType::Yellow);
    debug_ptr_->setDebugValues(DebugValues::TYPE::STOP_DISTANCE, *stop_dist);
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
    debug_ptr_->setAccelReason(DynamicObstacleStopDebug::AccelReason::LOW_JERK);
    return {};
  }

  // calculate index of stop point
  const float base_to_collision_point =
    planner_param_.dynamic_obstacle_stop.stop_margin + planner_param_.vehicle_param.base_to_front;
  const size_t stop_index = dynamic_obstacle_stop_utils::calcIndexByLengthReverse(
    path.points, dynamic_obstacle->nearest_collision_point, base_to_collision_point);
  const auto & stop_point = path.points.at(stop_index).point.pose;

  // debug
  debug_ptr_->setAccelReason(DynamicObstacleStopDebug::AccelReason::STOP);
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
    tier4_autoware_utils::findNearestSegmentIndex(path.points, stop_point->position);
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
  const PathWithLaneId & resampled_path, PathWithLaneId & output_path)
{
  // no obstacles
  if (!dynamic_obstacle) {
    state_ = State::GO;
    return;
  }

  const auto longitudinal_offset_to_collision_point =
    tier4_autoware_utils::calcSignedArcLength(
      resampled_path.points, current_pose.position, dynamic_obstacle->nearest_collision_point) -
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
        calcStopPoint(dynamic_obstacle, resampled_path, current_pose, current_vel, current_acc);
      insertStopPoint(stop_point, output_path);
      break;
    }

    case State::STOP: {
      RCLCPP_DEBUG_STREAM(logger_, "STOP state");
      const auto elapsed_time = (clock_->now() - stop_time_).seconds();
      state_ =
        elapsed_time > planner_param_.approaching.stop_time_thresh ? State::APPROACH : State::STOP;
      RCLCPP_DEBUG_STREAM(logger_, "elapsed time: " << elapsed_time);

      const auto stop_point =
        calcStopPoint(dynamic_obstacle, resampled_path, current_pose, current_vel, current_acc);
      insertStopPoint(stop_point, output_path);
      break;
    }

    case State::APPROACH: {
      RCLCPP_DEBUG_STREAM(logger_, "APPROACH state");
      insertApproachingVelocity(
        *dynamic_obstacle, current_pose, planner_param_.approaching.limit_vel_kmph / 3.6,
        planner_param_.approaching.margin, resampled_path, output_path);
      debug_ptr_->setAccelReason(DynamicObstacleStopDebug::AccelReason::STOP);
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
  const float approaching_vel, const float approach_margin, const PathWithLaneId & resampled_path,
  PathWithLaneId & output_path)
{
  // isnert slow down velocity from nearest segment point
  const auto nearest_seg_idx =
    tier4_autoware_utils::findNearestSegmentIndex(output_path.points, current_pose.position);
  dynamic_obstacle_stop_utils::insertPathVelocityFromIndexLimited(
    nearest_seg_idx, approaching_vel, output_path.points);

  // debug
  debug_ptr_->pushDebugPoints(
    output_path.points.at(nearest_seg_idx).point.pose.position, PointType::Yellow);

  // calculate stop point to insert 0 velocity
  const float base_to_collision_point =
    approach_margin + planner_param_.vehicle_param.base_to_front;
  const auto stop_idx = dynamic_obstacle_stop_utils::calcIndexByLengthReverse(
    resampled_path.points, dynamic_obstacle.nearest_collision_point, base_to_collision_point);
  const auto & stop_point = resampled_path.points.at(stop_idx).point.pose;

  // debug
  debug_ptr_->pushStopPose(stop_point);

  const auto nearest_seg_idx_stop =
    tier4_autoware_utils::findNearestSegmentIndex(output_path.points, stop_point.position);
  auto insert_idx_stop = nearest_seg_idx_stop + 1;

  // to PathPointWithLaneId
  // use lane id of point behind inserted point
  autoware_auto_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = output_path.points.at(nearest_seg_idx_stop);
  stop_point_with_lane_id.point.pose = stop_point;

  planning_utils::insertVelocity(output_path, stop_point_with_lane_id, 0.0, insert_idx_stop);
}

void DynamicObstacleStopModule::applyMaxJerkLimit(
  const geometry_msgs::msg::Pose & current_pose, const float current_vel, const float current_acc,
  PathWithLaneId & path) const
{
  const auto stop_point_idx = dynamic_obstacle_stop_utils::findFirstStopPointIdx(path.points);
  if (!stop_point_idx) {
    return;
  }

  const auto stop_point = path.points.at(stop_point_idx.get()).point.pose.position;
  const auto dist_to_stop_point =
    tier4_autoware_utils::calcSignedArcLength(path.points, current_pose.position, stop_point);

  // calculate desired velocity with limited jerk
  const auto jerk_limited_vel = planning_utils::calcDecelerationVelocityFromDistanceToTarget(
    planner_param_.slow_down_limit.max_jerk, planner_param_.slow_down_limit.max_acc, current_acc,
    current_vel, dist_to_stop_point);

  // overwrite velocity with limited velocity
  dynamic_obstacle_stop_utils::insertPathVelocityFromIndex(
    stop_point_idx.get(), jerk_limited_vel, path.points);
}

std::vector<DynamicObstacle> DynamicObstacleStopModule::excludeObstaclesOutSideOfPartition(
  const std::vector<DynamicObstacle> & dynamic_obstacles, const PathWithLaneId & path,
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
  const auto decimate_path_points =
    dynamic_obstacle_stop_utils::decimatePathPoints(path.points, decimate_step);

  // exclude obstacles outside of partition
  std::vector<DynamicObstacle> extracted_obstacles = dynamic_obstacles;
  for (const auto & partition : close_partitions) {
    extracted_obstacles = dynamic_obstacle_stop_utils::excludeObstaclesOutSideOfLine(
      extracted_obstacles, decimate_path_points, partition);
  }

  return extracted_obstacles;
}

void DynamicObstacleStopModule::publishDebugValue(
  const PathWithLaneId & path, const std::vector<DynamicObstacle> extracted_obstacles,
  const boost::optional<DynamicObstacle> & dynamic_obstacle,
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (dynamic_obstacle) {
    const auto lateral_dist = std::abs(tier4_autoware_utils::calcLateralOffset(
                                path.points, dynamic_obstacle->pose.position)) -
                              planner_param_.vehicle_param.width / 2.0;
    const auto longitudinal_dist_to_obstacle =
      tier4_autoware_utils::calcSignedArcLength(
        path.points, current_pose.position, dynamic_obstacle->pose.position) -
      planner_param_.vehicle_param.base_to_front;

    const float dist_to_collision_point = tier4_autoware_utils::calcSignedArcLength(
      path.points, current_pose.position, dynamic_obstacle->nearest_collision_point);
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

  if (extracted_obstacles.empty()) {
    debug_ptr_->setAccelReason(DynamicObstacleStopDebug::AccelReason::NO_OBSTACLE);
  }

  debug_ptr_->publishDebugValue();
}

}  // namespace behavior_velocity_planner
