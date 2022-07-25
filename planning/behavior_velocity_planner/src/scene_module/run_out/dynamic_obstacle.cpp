// Copyright 2022 TIER IV, Inc.
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

#include "scene_module/run_out/dynamic_obstacle.hpp"

#include <pcl/filters/voxel_grid.h>

namespace behavior_velocity_planner
{
namespace
{
// create quaternion facing to the nearest trajectory point
geometry_msgs::msg::Quaternion createQuaternionFacingToTrajectory(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point)
{
  const auto nearest_idx = tier4_autoware_utils::findNearestIndex(path_points, point);
  const auto & nearest_pose = path_points.at(nearest_idx).point.pose;

  const auto longitudinal_offset =
    tier4_autoware_utils::calcLongitudinalDeviation(nearest_pose, point);
  const auto vertical_point =
    tier4_autoware_utils::calcOffsetPose(nearest_pose, longitudinal_offset, 0, 0).position;
  const auto azimuth_angle = tier4_autoware_utils::calcAzimuthAngle(point, vertical_point);

  return tier4_autoware_utils::createQuaternionFromYaw(azimuth_angle);
}

// create predicted path assuming that obstacles move with constant velocity
std::vector<geometry_msgs::msg::Pose> createPredictedPath(
  const geometry_msgs::msg::Pose & initial_pose, const float time_step,
  const float max_velocity_mps, const float max_prediction_time)
{
  const size_t path_size = max_prediction_time / time_step;
  std::vector<geometry_msgs::msg::Pose> path_points;
  for (size_t i = 0; i < path_size; i++) {
    const float travel_dist = max_velocity_mps * time_step * i;
    const auto predicted_pose =
      tier4_autoware_utils::calcOffsetPose(initial_pose, travel_dist, 0, 0);
    path_points.emplace_back(predicted_pose);
  }

  return path_points;
}

pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_points)
{
  auto no_height_points = *input_points;
  for (auto & p : no_height_points) {
    p.z = 0.0;
  }

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(no_height_points));
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);

  pcl::PointCloud<pcl::PointXYZ> output_points;
  filter.filter(output_points);

  return output_points;
}

pcl::PointCloud<pcl::PointXYZ> extractObstaclePointsWithinPolygon(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const Polygons2d & polys)
{
  namespace bg = boost::geometry;

  if (polys.empty()) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("run_out"), "detection area polygon is empty. return empty points.");

    const pcl::PointCloud<pcl::PointXYZ> empty_points;
    return empty_points;
  }

  pcl::PointCloud<pcl::PointXYZ> output_points;
  for (const auto & poly : polys) {
    const auto bounding_box = bg::return_envelope<tier4_autoware_utils::Box2d>(poly);
    for (const auto & p : input_points) {
      Point2d point(p.x, p.y);

      // filter with bounding box to reduce calculation time
      if (!bg::covered_by(point, bounding_box)) {
        continue;
      }

      if (!bg::covered_by(point, poly)) {
        continue;
      }

      output_points.push_back(p);
    }
  }

  return output_points;
}

// NOTE: path_points must have points with constant interval
std::vector<pcl::PointCloud<pcl::PointXYZ>> groupPointsWithLongitudinalDistance(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const PathPointsWithLaneId & path_points,
  const double interval)
{
  // assign index with interval
  // example:
  //   longitudinal distance = 0.35, interval = 0.1 -> index is 3
  std::vector<pcl::PointCloud<pcl::PointXYZ>> points_with_index;
  // const auto path_length = tier4_autoware_utils::calcArcLength(path_points);
  // const size_t index_size = std::floor(path_length / interval) + 1;
  // points_with_index.resize(index_size);
  points_with_index.resize(path_points.size());

  const auto front_path_point = path_points.front().point.pose.position;
  for (const auto & p : input_points.points) {
    const auto arc_length = tier4_autoware_utils::calcSignedArcLength(
      path_points, front_path_point, tier4_autoware_utils::createPoint(p.x, p.y, p.z));
    if (arc_length < 0) {
      continue;
    }

    const size_t index = std::floor(arc_length / interval);
    if (index >= points_with_index.size()) {
      continue;
    }

    points_with_index.at(index).points.push_back(p);
  }

  return points_with_index;
}

pcl::PointXYZ calculateLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const geometry_msgs::msg::Pose & base_pose)
{
  const auto lateral_nearest_point = std::min_element(
    input_points.points.begin(), input_points.points.end(), [&](const auto & p1, const auto & p2) {
      const auto lateral_deviation_p1 = tier4_autoware_utils::calcLateralDeviation(
        base_pose, tier4_autoware_utils::createPoint(p1.x, p1.y, p1.x));
      const auto lateral_deviation_p2 = tier4_autoware_utils::calcLateralDeviation(
        base_pose, tier4_autoware_utils::createPoint(p2.x, p2.y, p2.x));
      return lateral_deviation_p1 < lateral_deviation_p2;
    });

  return *lateral_nearest_point;
}

pcl::PointCloud<pcl::PointXYZ> selectLateralNearestPoints(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>> & points_with_index,
  const PathPointsWithLaneId & path_points)
{
  pcl::PointCloud<pcl::PointXYZ> lateral_nearest_points;
  for (size_t idx = 0; idx < points_with_index.size(); idx++) {
    lateral_nearest_points.push_back(
      calculateLateralNearestPoint(points_with_index.at(idx), path_points.at(idx).point.pose));
  }

  return lateral_nearest_points;
}

pcl::PointCloud<pcl::PointXYZ> extractLateralNearestPoints(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const PathPointsWithLaneId & path_points,
  const double interval)
{
  // interpolate path points with interval
  // TODO: implement
  const auto interpolated_path_points = path_points;

  // divide points into groups according to longitudinal distance
  const auto points_with_index =
    groupPointsWithLongitudinalDistance(input_points, interpolated_path_points, interval);

  // select the lateral nearest point for each group
  const auto lateral_nearest_points =
    selectLateralNearestPoints(points_with_index, interpolated_path_points);

  return lateral_nearest_points;
}
}  // namespace

DynamicObstacleCreatorForObject::DynamicObstacleCreatorForObject(rclcpp::Node & node)
: DynamicObstacleCreator(node)
{
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForObject::createDynamicObstacles()
{
  // create dynamic obstacles from predicted objects
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & predicted_object : dynamic_obstacle_data_.predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle;
    dynamic_obstacle.pose = predicted_object.kinematics.initial_pose_with_covariance.pose;

    // TODO(Tomohito Ando): calculate velocity from covariance of predicted_object
    dynamic_obstacle.min_velocity_mps = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);
    dynamic_obstacle.classifications = predicted_object.classification;
    dynamic_obstacle.shape = predicted_object.shape;

    // get predicted paths of predicted_objects
    for (const auto & path : predicted_object.kinematics.predicted_paths) {
      PredictedPath predicted_path;
      predicted_path.confidence = path.confidence;
      predicted_path.path.resize(path.path.size());
      std::copy(path.path.cbegin(), path.path.cend(), predicted_path.path.begin());

      dynamic_obstacle.predicted_paths.emplace_back(predicted_path);
    }

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

DynamicObstacleCreatorForObjectWithoutPath::DynamicObstacleCreatorForObjectWithoutPath(
  rclcpp::Node & node)
: DynamicObstacleCreator(node)
{
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForObjectWithoutPath::createDynamicObstacles()
{
  std::vector<DynamicObstacle> dynamic_obstacles;

  for (const auto & predicted_object : dynamic_obstacle_data_.predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle;
    dynamic_obstacle.pose.position =
      predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    dynamic_obstacle.pose.orientation = createQuaternionFacingToTrajectory(
      dynamic_obstacle_data_.path.points, dynamic_obstacle.pose.position);

    dynamic_obstacle.min_velocity_mps = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);
    dynamic_obstacle.classifications = predicted_object.classification;
    dynamic_obstacle.shape = predicted_object.shape;

    // replace predicted path with path that runs straight to lane
    PredictedPath predicted_path;
    predicted_path.path = createPredictedPath(
      dynamic_obstacle.pose, param_.time_step, dynamic_obstacle.max_velocity_mps,
      param_.max_prediction_time);
    predicted_path.confidence = 1.0;
    dynamic_obstacle.predicted_paths.emplace_back(predicted_path);

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

DynamicObstacleCreatorForPoints::DynamicObstacleCreatorForPoints(rclcpp::Node & node)
: DynamicObstacleCreator(node), tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  sub_compare_map_filtered_pointcloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/compare_map_filtered_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud, this, _1));
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForPoints::createDynamicObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & point : dynamic_obstacle_data_.compare_map_filtered_pointcloud) {
    DynamicObstacle dynamic_obstacle;

    // create pose facing the direction of the lane
    dynamic_obstacle.pose.position = tier4_autoware_utils::createPoint(point.x, point.y, point.z);
    dynamic_obstacle.pose.orientation = createQuaternionFacingToTrajectory(
      dynamic_obstacle_data_.path.points, dynamic_obstacle.pose.position);

    dynamic_obstacle.min_velocity_mps = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);

    // create classification of points as pedestrian
    ObjectClassification classification;
    classification.label = ObjectClassification::PEDESTRIAN;
    classification.probability = 1.0;
    dynamic_obstacle.classifications.emplace_back(classification);

    // create shape of points as cylinder
    dynamic_obstacle.shape.type = Shape::CYLINDER;
    dynamic_obstacle.shape.dimensions.x = param_.diameter;
    dynamic_obstacle.shape.dimensions.y = param_.diameter;
    dynamic_obstacle.shape.dimensions.z = param_.height;

    // create predicted path of points
    PredictedPath predicted_path;
    predicted_path.path = createPredictedPath(
      dynamic_obstacle.pose, param_.time_step, dynamic_obstacle.max_velocity_mps,
      param_.max_prediction_time);
    predicted_path.confidence = 1.0;
    dynamic_obstacle.predicted_paths.emplace_back(predicted_path);

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

void DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_obstacle_data_.compare_map_filtered_pointcloud.clear();
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(node_.get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(pc, *pc_transformed, affine);

  // apply voxel grid filter to reduce calculation cost
  const auto voxel_grid_filtered_points = applyVoxelGridFilter(pc_transformed);

  // filter obstacle points within detection area polygon
  const auto & d = dynamic_obstacle_data_;
  const auto detection_area_filtered_points =
    extractObstaclePointsWithinPolygon(voxel_grid_filtered_points, d.detection_area_polygon);

  // filter points that have lateral nearest distance
  // TODO(Tomohito Ando): change the name of `compare_map_filtered_pointcloud`
  std::lock_guard<std::mutex> lock(mutex_);
  dynamic_obstacle_data_.compare_map_filtered_pointcloud =
    extractLateralNearestPoints(detection_area_filtered_points, d.path.points, 0.1);
}
}  // namespace behavior_velocity_planner
