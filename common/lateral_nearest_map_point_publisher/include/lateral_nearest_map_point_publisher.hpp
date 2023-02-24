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

#ifndef LATERAL_NEAREST_POINT_PUBLISHER_
#define LATERAL_NEAREST_POINT_PUBLISHER_

// #define EIGEN_MPL2_ONLY

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/geometry/pose_deviation.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <memory>
#include <optional>

namespace lateral_nearest_map_point_publisher
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tier4_debug_msgs::msg::Float32Stamped;
using vehicle_info_util::VehicleInfo;
using geometry_msgs::msg::PointStamped;

struct Circle
{
  double x;
  double y;
  double radius;
};

struct Vehicle
{
  double base_to_front;
  double half_width;
};

struct PointWithDist
{
  geometry_msgs::msg::Point point;
  double distance;
};

struct LateralNearestPoints
{
  PointWithDist left;
  PointWithDist right;
};

class LateralNearestMapPointPublisher : public rclcpp::Node
{
public:
  explicit LateralNearestMapPointPublisher(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_pointcloud_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_left_nearest_point_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_left_nearest_distance_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_right_nearest_point_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_right_nearest_distance_;

  void onMapPointCloud(const PointCloud2::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onTrajectory(const Trajectory::ConstSharedPtr msg);

  // variables
  std::unique_ptr<PointCloud2> map_pointcloud_{nullptr};
  std::unique_ptr<Odometry> odometry_{nullptr};
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  Vehicle vehicle_;
};

}  // namespace lateral_nearest_map_point_publisher
#endif  // LATERAL_NEAREST_POINT_PUBLISHER_
