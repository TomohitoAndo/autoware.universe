// Copyright 2021 Tier IV, Inc.
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

#include "lateral_nearest_map_point_publisher.hpp"

namespace
{
using lateral_nearest_map_point_publisher::BasicPolygons2d;
using lateral_nearest_map_point_publisher::TrajectoryPoints;
using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::Point2d;

pcl::PointCloud<pcl::PointXYZ> extractPointsInCircle(
  const pcl::PointCloud<pcl::PointXYZ> & points,
  lateral_nearest_map_point_publisher::Circle & circle)
{
  pcl::PointCloud<pcl::PointXYZ> points_within_circle;
  for (const auto & p : points) {
    const double squared_dist = std::pow(circle.x - p.x, 2) + std::pow(circle.y - p.y, 2);
    // point is outside the circle
    if (squared_dist > std::pow(circle.radius, 2)) {
      continue;
    }

    points_within_circle.emplace_back(p);
  }

  return points_within_circle;
}

pcl::PointCloud<pcl::PointXYZ> extractLateralPoints(
  const pcl::PointCloud<pcl::PointXYZ> & points, const geometry_msgs::msg::Pose & pose,
  const double offset_behind, const double offset_ahead)
{
  pcl::PointCloud<pcl::PointXYZ> lateral_points;
  for (const auto & p : points) {
    const auto longitudinal_offset = tier4_autoware_utils::calcLongitudinalDeviation(
      pose, tier4_autoware_utils::createPoint(p.x, p.y, p.z));
    if (longitudinal_offset > offset_ahead || longitudinal_offset < -offset_behind) {
      continue;
    }

    lateral_points.emplace_back(p);
  }

  return lateral_points;
}

pcl::PointCloud<pcl::PointXYZ> extractNoGroundPoints(
  const pcl::PointCloud<pcl::PointXYZ> & points, const double height_base,
  const double height_threshold_min, const double height_threshold_max)
{
  pcl::PointCloud<pcl::PointXYZ> height_filtered_points;
  for (const auto & p : points) {
    if (p.z < height_base + height_threshold_min || p.z > height_base + height_threshold_max) {
      continue;
    }

    height_filtered_points.emplace_back(p);
  }

  return height_filtered_points;
}

lateral_nearest_map_point_publisher::LateralNearestPoints extractLateralNearestPoints(
  const pcl::PointCloud<pcl::PointXYZ> & points, const geometry_msgs::msg::Pose & pose,
  const double vehicle_half_width)
{
  lateral_nearest_map_point_publisher::LateralNearestPoints lateral_nearest_points;
  lateral_nearest_points.left.distance = std::numeric_limits<double>::max();
  lateral_nearest_points.right.distance = std::numeric_limits<double>::max();
  for (const auto & p : points) {
    const auto p_ros = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    const auto lateral_deviation = tier4_autoware_utils::calcLateralDeviation(pose, p_ros);

    if (lateral_deviation < 0) {
      if (-lateral_deviation < lateral_nearest_points.right.distance) {
        lateral_nearest_points.right.distance = -lateral_deviation - vehicle_half_width;
        lateral_nearest_points.right.point = p_ros;
      }
    } else {  // (lateral_deviation > 0)
      if (lateral_deviation < lateral_nearest_points.left.distance) {
        lateral_nearest_points.left.distance = lateral_deviation - vehicle_half_width;
        lateral_nearest_points.left.point = p_ros;
      }
    }
  }

  return lateral_nearest_points;
}

// lanelet
void getAllPartitionLanelets(const lanelet::LaneletMapConstPtr ll, BasicPolygons2d & polys)
{
  const lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(ll);
  for (const auto & partition : partitions) {
    lanelet::BasicLineString2d line;
    for (const auto & p : partition) {
      line.emplace_back(lanelet::BasicPoint2d{p.x(), p.y()});
    }
    // correct line to calculate distance in accurate
    boost::geometry::correct(line);
    polys.emplace_back(lanelet::BasicPolygon2d(line));
  }
}

void extractClosePartition(
  const geometry_msgs::msg::Point position, const BasicPolygons2d & all_partitions,
  BasicPolygons2d & close_partition, const double distance_thresh = 30.0)
{
  close_partition.clear();
  for (const auto & p : all_partitions) {
    if (boost::geometry::distance(Point2d(position.x, position.y), p) < distance_thresh) {
      close_partition.emplace_back(p);
    }
  }
  return;
}

LineString2d createLineString2d(const lanelet::BasicPolygon2d & poly)
{
  LineString2d line_string;
  for (const auto & p : poly) {
    Point2d bg_point{p.x(), p.y()};
    line_string.push_back(bg_point);
  }

  return line_string;
}

template <class T>
pcl::PointCloud<pcl::PointXYZ> excludeObstaclesOutSideOfLine(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const T & path_points,
  const lanelet::BasicPolygon2d & partition)
{
  pcl::PointCloud<pcl::PointXYZ> extracted_points;
  for (const auto & p : input_points) {
    const auto ros_point = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    const auto obstacle_nearest_idx = motion_utils::findNearestIndex(path_points, ros_point);
    const auto & obstacle_nearest_path_point = path_points.at(obstacle_nearest_idx).pose.position;

    // create linestring from traj point to obstacle
    const LineString2d path_point_to_obstacle{
      {obstacle_nearest_path_point.x, obstacle_nearest_path_point.y}, {ros_point.x, ros_point.y}};

    // create linestring for partition
    const LineString2d partition_bg = createLineString2d(partition);

    // ignore obstacle outside of partition
    if (boost::geometry::intersects(path_point_to_obstacle, partition_bg)) {
      continue;
    }
    extracted_points.emplace_back(p);
  }

  return extracted_points;
}

template <class T>
pcl::PointCloud<pcl::PointXYZ> excludeObstaclesOutSideOfPartition(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const T & path_points,
  const BasicPolygons2d & partition_lanelets, const geometry_msgs::msg::Pose & current_pose)
{
  if (partition_lanelets.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "partition lanelet is empty.");
    return input_points;
  }

  // extract partitions within detection distance
  BasicPolygons2d close_partitions;
  extractClosePartition(current_pose.position, partition_lanelets, close_partitions);

  // decimate trajectory to reduce calculation time
  // constexpr float decimate_step = 1.0;
  // const auto decimate_path_points = run_out_utils::decimatePathPoints(path.points,
  // decimate_step);

  // exclude obstacles outside of partition
  pcl::PointCloud<pcl::PointXYZ> extracted_points = input_points;
  for (const auto & partition : close_partitions) {
    extracted_points = excludeObstaclesOutSideOfLine(extracted_points, path_points, partition);
  }

  return extracted_points;
}
}  // namespace

namespace lateral_nearest_map_point_publisher
{
LateralNearestMapPointPublisher::LateralNearestMapPointPublisher(
  const rclcpp::NodeOptions & node_options)
: Node("lateral_nearest_map_point_publisher", node_options)
{
  using std::placeholders::_1;

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  const auto & v = vehicle_info;
  vehicle_.base_to_front = v.wheel_base_m + v.front_overhang_m;
  vehicle_.half_width = v.vehicle_width_m / 2.0;

  RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "base to front: " << vehicle_.base_to_front);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "half width: " << vehicle_.half_width);

  sub_map_pointcloud_ = this->create_subscription<PointCloud2>(
    "/map/pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&LateralNearestMapPointPublisher::onMapPointCloud, this, _1));

  sub_lanelet_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "/map/vector_map", rclcpp::QoS(10).transient_local(),
    std::bind(&LateralNearestMapPointPublisher::onLaneletMap, this, _1));

  sub_odometry_ = this->create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    std::bind(&LateralNearestMapPointPublisher::onOdometry, this, _1));

  sub_trajectory_ = this->create_subscription<Trajectory>(
    "/planning/scenario_planning/lane_driving/trajectory", 1,
    std::bind(&LateralNearestMapPointPublisher::onTrajectory, this, _1));

  pub_ranged_pointcloud_ = this->create_publisher<PointCloud2>(
    "~/pub/lateral_nearest_map_points/ranged_points", rclcpp::SensorDataQoS().keep_last(1));

  pub_filtered_pointcloud_ = this->create_publisher<PointCloud2>(
    "~/pub/lateral_nearest_map_points/filtered_points", rclcpp::SensorDataQoS().keep_last(1));

  pub_left_nearest_point_ =
    this->create_publisher<PointStamped>("~/pub/left/lateral_nearest_point", 1);

  pub_left_nearest_distance_ = this->create_publisher<Float32Stamped>("~/pub/left/distance", 1);

  pub_right_nearest_point_ =
    this->create_publisher<PointStamped>("~/pub/right/lateral_nearest_point", 1);

  pub_right_nearest_distance_ = this->create_publisher<Float32Stamped>("~/pub/right/distance", 1);
}

void LateralNearestMapPointPublisher::onMapPointCloud(const PointCloud2::ConstSharedPtr msg)
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "map subscribed");
  map_pointcloud_ = std::make_unique<PointCloud2>(*msg);
}

void LateralNearestMapPointPublisher::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = std::make_unique<Odometry>(*msg);
}

void LateralNearestMapPointPublisher::onLaneletMap(const HADMapBin::ConstSharedPtr msg)
{
  // Load map
  route_handler_ = std::make_shared<route_handler::RouteHandler>(*msg);

  const lanelet::LaneletMapConstPtr & ll = route_handler_->getLaneletMapPtr();
  getAllPartitionLanelets(ll, partition_lanelets_);
}

void LateralNearestMapPointPublisher::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  if (!map_pointcloud_ || !odometry_) {
    RCLCPP_WARN_STREAM(get_logger(), "map or odometry is not subscribed.");
    return;
  }

  if (!route_handler_) {
    RCLCPP_WARN_STREAM(get_logger(), "Waiting for the initialization of route_handler");
    return;
  }
  if (!route_handler_->isMapMsgReady()) {
    RCLCPP_WARN_STREAM(get_logger(), "Waiting for the initialization of map");
    return;
  }

  // timer start
  stop_watch.tic();

  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::fromROSMsg(*map_pointcloud_, map_pcl);

  const auto nearest_idx =
    motion_utils::findNearestIndex(msg->points, odometry_->pose.pose.position);
  const auto & nearest_path_point = msg->points.at(nearest_idx);

  // extract Points around the vehicle to reduce calculation time
  // TODO: parameter
  const double circle_radius = 7.0;
  const auto & nearest_path_position = nearest_path_point.pose.position;
  Circle circle = {nearest_path_position.x, nearest_path_position.y, circle_radius};
  const auto points_around_vehicle = extractPointsInCircle(map_pcl, circle);

  // TODO: parameter
  // filter points with its height
  const double height_threshold_min = 0.5;
  const double height_threshold_max = 2.5;
  const auto height_filtered_points = extractNoGroundPoints(
    points_around_vehicle, nearest_path_position.z, height_threshold_min, height_threshold_max);

  // extract points around the nearest path point to calculate lateral distance precisely
  // TODO: parameter
  const double longitudinal_range = 0.1;
  const auto lateral_points_around_vehicle = extractLateralPoints(
    height_filtered_points, nearest_path_point.pose, longitudinal_range, longitudinal_range);

  // extract obstacles using lanelet information
  const auto partition_excluded_points = excludeObstaclesOutSideOfPartition(
    lateral_points_around_vehicle, msg->points, partition_lanelets_, odometry_->pose.pose);

  const LateralNearestPoints lateral_nearest_points = extractLateralNearestPoints(
    partition_excluded_points, nearest_path_point.pose, vehicle_.half_width);

  // publish lateral nearest points
  const auto & lnp = lateral_nearest_points;
  {
    Float32Stamped distance;
    distance.data = lnp.left.distance < 50 ? lnp.left.distance : 50;
    distance.stamp = this->now();
    pub_left_nearest_distance_->publish(distance);

    PointStamped point_stamped;
    point_stamped.point = lnp.left.point;
    point_stamped.header.stamp = this->now();
    point_stamped.header.frame_id = "map";
    pub_left_nearest_point_->publish(point_stamped);
  }

  {
    Float32Stamped distance;
    distance.data = lnp.right.distance < 50 ? lnp.right.distance : 50;
    distance.stamp = this->now();
    pub_right_nearest_distance_->publish(distance);

    PointStamped point_stamped;
    point_stamped.point = lnp.right.point;
    point_stamped.header.stamp = this->now();
    point_stamped.header.frame_id = "map";
    pub_right_nearest_point_->publish(point_stamped);
  }

  // publish pointcloud for debug
  PointCloud2 ranged_points_ros;
  pcl::toROSMsg(height_filtered_points, ranged_points_ros);
  ranged_points_ros.header.frame_id = map_pointcloud_->header.frame_id;
  ranged_points_ros.header.stamp = map_pointcloud_->header.stamp;
  pub_ranged_pointcloud_->publish(ranged_points_ros);

  PointCloud2 filtered_points_ros;
  pcl::toROSMsg(partition_excluded_points, filtered_points_ros);
  filtered_points_ros.header.frame_id = map_pointcloud_->header.frame_id;
  filtered_points_ros.header.stamp = map_pointcloud_->header.stamp;
  pub_filtered_pointcloud_->publish(filtered_points_ros);

  // timer end
  const auto time = stop_watch.toc();
  RCLCPP_WARN_STREAM(get_logger(), "time: " << time);
}

}  // namespace lateral_nearest_map_point_publisher
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  lateral_nearest_map_point_publisher::LateralNearestMapPointPublisher)
