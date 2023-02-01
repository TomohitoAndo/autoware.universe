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
  const auto nearest_idx = motion_utils::findNearestIndex(path_points, point);
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

pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(
  const sensor_msgs::msg::PointCloud2 & input_points)
{
  if (input_points.data.empty()) {
    return pcl::PointCloud<pcl::PointXYZ>();
  }

  pcl::PointCloud<pcl::PointXYZ> input_points_pcl;
  pcl::fromROSMsg(input_points, input_points_pcl);

  auto no_height_points = input_points_pcl;
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

bool isAheadOf(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Pose & base_pose)
{
  const auto longitudinal_deviation =
    tier4_autoware_utils::calcLongitudinalDeviation(base_pose, target_point);
  const bool is_ahead = longitudinal_deviation > 0;
  return is_ahead;
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

// group points with its nearest segment of path points
std::vector<pcl::PointCloud<pcl::PointXYZ>> groupPointsWithNearestSegmentIndex(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const PathPointsWithLaneId & path_points)
{
  // assign nearest segment index to each point
  std::vector<pcl::PointCloud<pcl::PointXYZ>> points_with_index;
  points_with_index.resize(path_points.size());

  for (const auto & p : input_points.points) {
    const auto ros_point = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    const size_t nearest_seg_idx = motion_utils::findNearestSegmentIndex(path_points, ros_point);

    // if the point is ahead of end of the path, index should be path.size() - 1
    if (
      nearest_seg_idx == path_points.size() - 2 &&
      isAheadOf(ros_point, path_points.back().point.pose)) {
      points_with_index.back().push_back(p);
      continue;
    }

    points_with_index.at(nearest_seg_idx).push_back(p);
  }

  return points_with_index;
}

// calculate lateral nearest point from base_pose
pcl::PointXYZ calculateLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const geometry_msgs::msg::Pose & base_pose)
{
  const auto lateral_nearest_point = std::min_element(
    input_points.points.begin(), input_points.points.end(), [&](const auto & p1, const auto & p2) {
      const auto lateral_deviation_p1 = std::abs(tier4_autoware_utils::calcLateralDeviation(
        base_pose, tier4_autoware_utils::createPoint(p1.x, p1.y, 0)));
      const auto lateral_deviation_p2 = std::abs(tier4_autoware_utils::calcLateralDeviation(
        base_pose, tier4_autoware_utils::createPoint(p2.x, p2.y, 0)));

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
    if (points_with_index.at(idx).points.empty()) {
      continue;
    }

    lateral_nearest_points.push_back(
      calculateLateralNearestPoint(points_with_index.at(idx), path_points.at(idx).point.pose));
  }

  return lateral_nearest_points;
}

// extract lateral nearest points for nearest segment of the path
// path is interpolated with given interval
pcl::PointCloud<pcl::PointXYZ> extractLateralNearestPoints(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const PathWithLaneId & path,
  const float interval)
{
  // interpolate path points with given interval
  PathWithLaneId interpolated_path;
  if (!splineInterpolate(
        path, interval, interpolated_path, rclcpp::get_logger("dynamic_obstacle_creator"))) {
    return input_points;
  }

  // divide points into groups according to nearest segment index
  const auto points_with_index =
    groupPointsWithNearestSegmentIndex(input_points, interpolated_path.points);

  // select the lateral nearest point for each group
  const auto lateral_nearest_points =
    selectLateralNearestPoints(points_with_index, interpolated_path.points);

  return lateral_nearest_points;
}
}  // namespace

DynamicObstacleCreatorForObject::DynamicObstacleCreatorForObject(
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr)
: DynamicObstacleCreator(node, debug_ptr)
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
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr)
: DynamicObstacleCreator(node, debug_ptr)
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

DynamicObstacleCreatorForPoints::DynamicObstacleCreatorForPoints(
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr)
: DynamicObstacleCreator(node, debug_ptr), tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  sub_compare_map_filtered_pointcloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/compare_map_filtered_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud, this, _1));

  // Subscribe the input using message filter
  const size_t max_queue_size = 10;
  sub_compare_map_filtered_pointcloud_sync_.subscribe(
    &node, "~/input/compare_map_filtered_pointcloud",
    rclcpp::SensorDataQoS().keep_last(10).get_rmw_qos_profile());
  sub_vector_map_inside_area_filtered_pointcloud_sync_.subscribe(
    &node, "~/input/vector_map_inside_area_filtered_pointcloud",
    rclcpp::SensorDataQoS().keep_last(10).get_rmw_qos_profile());

  // sync subscribers with ExactTime Sync Policy
  exact_time_synchronizer_ = std::make_unique<ExactTimeSynchronizer>(max_queue_size);
  exact_time_synchronizer_->connectInput(
    sub_compare_map_filtered_pointcloud_sync_,
    sub_vector_map_inside_area_filtered_pointcloud_sync_);
  exact_time_synchronizer_->registerCallback(
    &DynamicObstacleCreatorForPoints::onSynchronizedPointCloud, this);

  // debug publisher
  pub_filtered_compare_map_points_ = node.create_publisher<PointCloud2>(
    "compare_map_points_run_out_filtered", rclcpp::SensorDataQoS().keep_last(1));
  pub_filtered_vector_map_points_ = node.create_publisher<PointCloud2>(
    "vector_map_points_run_out_filtered", rclcpp::SensorDataQoS().keep_last(1));
  pub_concat_points_ =
    node.create_publisher<PointCloud2>("concat_points", rclcpp::SensorDataQoS().keep_last(1));
  pub_concat_no_overlap_points_ = node.create_publisher<PointCloud2>(
    "concat_no_overlap_points", rclcpp::SensorDataQoS().keep_last(1));
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForPoints::createDynamicObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & point : obstacle_points_map_filtered_) {
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
    obstacle_points_map_filtered_.clear();
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

  // these variables are written in another callback
  mutex_.lock();
  const auto detection_area_polygon = dynamic_obstacle_data_.detection_area_high_jerk;
  const auto path = dynamic_obstacle_data_.path;
  mutex_.unlock();

  // filter obstacle points within detection area polygon
  const auto detection_area_filtered_points =
    extractObstaclePointsWithinPolygon(voxel_grid_filtered_points, detection_area_polygon);

  // filter points that have lateral nearest distance
  const auto lateral_nearest_points =
    extractLateralNearestPoints(detection_area_filtered_points, path, param_.points_interval);

  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_points_map_filtered_ = lateral_nearest_points;
}

void DynamicObstacleCreatorForPoints::onSynchronizedPointCloud(
  const PointCloud2::ConstSharedPtr compare_map_filtered_points,
  const PointCloud2::ConstSharedPtr vector_map_filtered_points)
{
  if (compare_map_filtered_points->data.empty() && vector_map_filtered_points->data.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_points_map_filtered_.clear();
    return;
  }

  // transform pointcloud
  // geometry_msgs::msg::TransformStamped transform;
  // try {
  //   transform = tf_buffer_.lookupTransform(
  //     "map", compare_map_filtered_points->header.frame_id,
  //     compare_map_filtered_points->header.stamp);
  // } catch (tf2::TransformException & e) {
  //   RCLCPP_WARN(node_.get_logger(), "no transform found: %s", e.what());
  //   return;
  // }
  // Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();

  // TODO: function
  Eigen_::Affine3f transform_matrix = getTransformMatrix(
    tf_buffer_, compare_map_filtered_points->header.frame_id,
    compare_map_filtered_points->header.stamp);

  pcl::PointCloud<pcl::PointXYZ> pc_compare_map;
  pcl::fromROSMsg(*compare_map_filtered_points, pc_compare_map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_compare_map_transformed(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(pc_compare_map, *pc_compare_map_transformed, affine);

  pcl::PointCloud<pcl::PointXYZ> pc_vector_map;
  pcl::fromROSMsg(*vector_map_filtered_points, pc_vector_map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_vector_map_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  // vector map filtered points may be empty if the obstacle is in the vector map area
  if (!pc_vector_map.points.empty()) {
    pcl::transformPointCloud(pc_vector_map, *pc_vector_map_transformed, affine);
  }

  // apply voxel grid filter to reduce calculation cost
  const auto voxel_grid_filtered_compare_map_points =
    applyVoxelGridFilter(pc_compare_map_transformed);
  const auto voxel_grid_filtered_vector_map_points =
    applyVoxelGridFilter(pc_vector_map_transformed);

  // these variables are written in another callback
  mutex_.lock();
  const auto detection_area_high_jerk = dynamic_obstacle_data_.detection_area_high_jerk;
  const auto detection_area_low_jerk = dynamic_obstacle_data_.detection_area_low_jerk;
  const auto path = dynamic_obstacle_data_.path;
  mutex_.unlock();

  // filter obstacle points within detection area polygon
  const auto detection_area_filtered_compare_map_points = extractObstaclePointsWithinPolygon(
    voxel_grid_filtered_compare_map_points, detection_area_high_jerk);
  const auto detection_area_filtered_vector_map_points = extractObstaclePointsWithinPolygon(
    voxel_grid_filtered_vector_map_points, detection_area_low_jerk);

  // publish points for debug
  // TODO: function
  PointCloud2 detection_area_filtered_compare_map_points_ros;
  PointCloud2 detection_area_filtered_vector_map_points_ros;
  pcl::toROSMsg(
    detection_area_filtered_compare_map_points, detection_area_filtered_compare_map_points_ros);
  pcl::toROSMsg(
    detection_area_filtered_vector_map_points, detection_area_filtered_vector_map_points_ros);
  detection_area_filtered_compare_map_points_ros.header = compare_map_filtered_points->header;
  detection_area_filtered_vector_map_points_ros.header = vector_map_filtered_points->header;
  detection_area_filtered_compare_map_points_ros.header.frame_id = "map";
  detection_area_filtered_vector_map_points_ros.header.frame_id = "map";
  pub_filtered_compare_map_points_->publish(detection_area_filtered_compare_map_points_ros);
  pub_filtered_vector_map_points_->publish(detection_area_filtered_vector_map_points_ros);

  PointCloud2 concat_points;
  pcl::concatenatePointCloud(
    detection_area_filtered_compare_map_points_ros, detection_area_filtered_vector_map_points_ros,
    concat_points);
  concat_points.header = detection_area_filtered_compare_map_points_ros.header;

  // remove overlap points
  const auto concat_points_no_overlap = applyVoxelGridFilter(concat_points);
  PointCloud2 concat_points_no_overlap_ros;
  pcl::toROSMsg(concat_points_no_overlap, concat_points_no_overlap_ros);

  pub_concat_no_overlap_points_->publish(concat_points_no_overlap_ros);

  // filter points that have lateral nearest distance
  const auto lateral_nearest_points =
    extractLateralNearestPoints(concat_points_no_overlap, path, param_.points_interval);

  //   std::lock_guard<std::mutex> lock(mutex_);
  obstacle_points_map_filtered_ = lateral_nearest_points;
}
}  // namespace behavior_velocity_planner
