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
  const double height_threshold)
{
  pcl::PointCloud<pcl::PointXYZ> height_filtered_points;
  for (const auto & p : points) {
    if (p.z < height_base + height_threshold) {
      continue;
    }

    height_filtered_points.emplace_back(p);
  }

  return height_filtered_points;
}

lateral_nearest_map_point_publisher::LateralNearestPoints extractLateralNearestPoints(
  const pcl::PointCloud<pcl::PointXYZ> & points, const geometry_msgs::msg::Pose & pose)
{
  lateral_nearest_map_point_publisher::LateralNearestPoints lateral_nearest_points;
  lateral_nearest_points.left.distance = std::numeric_limits<double>::max();
  lateral_nearest_points.right.distance = std::numeric_limits<double>::max();
  for (const auto & p : points) {
    const auto p_ros = tier4_autoware_utils::createPoint(p.x, p.y, p.z);
    const auto lateral_deviation = tier4_autoware_utils::calcLateralDeviation(pose, p_ros);

    if (lateral_deviation < 0) {
      if (-lateral_deviation < lateral_nearest_points.right.distance) {
        lateral_nearest_points.right.distance = -lateral_deviation;
        lateral_nearest_points.right.point = p_ros;
      }
    } else {  // (lateral_deviation > 0)
      if (lateral_deviation < lateral_nearest_points.left.distance) {
        lateral_nearest_points.left.distance = lateral_deviation;
        lateral_nearest_points.left.point = p_ros;
      }
    }
  }

  return lateral_nearest_points;
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

  /* Parameters */
  // yaw_threshold_to_search_closest_ =
  //   declare_parameter("yaw_threshold_to_search_closest", M_PI / 4.0);

  // pub_lateral_error_ =
  //   create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/lateral_error", 1);

  sub_map_pointcloud_ = this->create_subscription<PointCloud2>(
    "/map/pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&LateralNearestMapPointPublisher::onMapPointCloud, this, _1));

  sub_odometry_ = this->create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    std::bind(&LateralNearestMapPointPublisher::onOdometry, this, _1));

  sub_trajectory_ = this->create_subscription<Trajectory>(
    "/planning/scenario_planning/lane_driving/trajectory", 1,
    std::bind(&LateralNearestMapPointPublisher::onTrajectory, this, _1));

  pub_pointcloud_ = this->create_publisher<PointCloud2>(
    "~/pub/lateral_nearest_map_points", rclcpp::SensorDataQoS().keep_last(1));

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

  // if (!map_pointcloud_) {
  //   RCLCPP_WARN_STREAM(get_logger(), "map pointcloud is not subscribed.");
  //   return;
  // }

  // // timer start
  // stop_watch.tic();

  // pcl::PointCloud<pcl::PointXYZ> map_pcl;
  // pcl::fromROSMsg(*map_pointcloud_, map_pcl);

  // // const auto & current_pose = odometry_->pose.pose;
  // // const auto vehicle_front_pose =
  // //   tier4_autoware_utils::calcOffsetPose(current_pose, vehicle_.base_to_front, 0, 0);

  // // extract Points around the vehicle to reduce calculation time
  // // TODO: parameter
  // const double circle_radius = 7.0;
  // const auto & odometry_position = odometry_->pose.pose.position;
  // Circle circle = {odometry_position.x, odometry_position.y, circle_radius};
  // const auto points_around_vehicle = extractPointsInCircle(map_pcl, circle);

  // // extract points around the odometry to calculate lateral distance precisely
  // // TODO: parameter
  // const double longitudinal_range = 0.1;
  // const auto lateral_points_around_odometry = extractLateralPoints(
  //   points_around_vehicle, odometry_->pose.pose, longitudinal_range, longitudinal_range);

  // const double height_threshold = 0.5;
  // const auto height_filtered_points =
  //   extractNoGroundPoints(lateral_points_around_odometry, odometry_position.z, height_threshold);

  // // publish pointcloud
  // PointCloud2 filtered_points_ros;
  // pcl::toROSMsg(height_filtered_points, filtered_points_ros);
  // filtered_points_ros.header.frame_id = map_pointcloud_->header.frame_id;
  // filtered_points_ros.header.stamp = map_pointcloud_->header.stamp;
  // pub_pointcloud_->publish(filtered_points_ros);

  // // timer end
  // const auto time = stop_watch.toc();
  // RCLCPP_WARN_STREAM(get_logger(), "time: " << time);
}

void LateralNearestMapPointPublisher::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  if (!map_pointcloud_ || !odometry_) {
    RCLCPP_WARN_STREAM(get_logger(), "map or odometry is not subscribed.");
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

  // extract points around the nearest path point to calculate lateral distance precisely
  // TODO: parameter
  const double longitudinal_range = 0.1;
  const auto lateral_points_around_vehicle = extractLateralPoints(
    points_around_vehicle, nearest_path_point.pose, longitudinal_range, longitudinal_range);

  // TODO: parameter
  const double height_threshold = 0.5;
  const auto height_filtered_points =
    extractNoGroundPoints(lateral_points_around_vehicle, nearest_path_position.z, height_threshold);

  const LateralNearestPoints lateral_nearest_points =
    extractLateralNearestPoints(height_filtered_points, nearest_path_point.pose);

  // publish lateral nearest points
  if (lateral_nearest_points.left.distance < 100000) {
    Float32Stamped distance;
    distance.data = lateral_nearest_points.left.distance;
    distance.stamp = this->now();
    pub_left_nearest_distance_->publish(distance);

    PointStamped point_stamped;
    point_stamped.point = lateral_nearest_points.left.point;
    point_stamped.header.stamp = this->now();
    point_stamped.header.frame_id = "map";
    pub_left_nearest_point_->publish(point_stamped);
  }

  if (lateral_nearest_points.right.distance < 100000) {
    Float32Stamped distance;
    distance.data = lateral_nearest_points.right.distance;
    distance.stamp = this->now();
    pub_right_nearest_distance_->publish(distance);

    PointStamped point_stamped;
    point_stamped.point = lateral_nearest_points.right.point;
    point_stamped.header.stamp = this->now();
    point_stamped.header.frame_id = "map";
    pub_right_nearest_point_->publish(point_stamped);
  }

  // publish pointcloud
  PointCloud2 filtered_points_ros;
  pcl::toROSMsg(points_around_vehicle, filtered_points_ros);
  filtered_points_ros.header.frame_id = map_pointcloud_->header.frame_id;
  filtered_points_ros.header.stamp = map_pointcloud_->header.stamp;
  pub_pointcloud_->publish(filtered_points_ros);

  // timer end
  const auto time = stop_watch.toc();
  RCLCPP_WARN_STREAM(get_logger(), "time: " << time);
}

}  // namespace lateral_nearest_map_point_publisher
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  lateral_nearest_map_point_publisher::LateralNearestMapPointPublisher)
