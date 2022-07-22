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

#ifndef POINTCLOUD_PREPROCESSOR__NO_DETECTION_AREA_FILTER__LANELET2_NO_DETECTION_AREA_FILTER_HPP_
#define POINTCLOUD_PREPROCESSOR__NO_DETECTION_AREA_FILTER__LANELET2_NO_DETECTION_AREA_FILTER_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::Point2d;

namespace pointcloud_preprocessor
{
class NoDetectionAreaFilterComponent : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

public:
  explicit NoDetectionAreaFilterComponent(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr filtered_pointcloud_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr debug_pub_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstPolygons3d no_detection_area_lanelets_;

  void pointcloudCallback(const PointCloud2ConstPtr msg);

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  bool transformPointCloud(
    const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
    PointCloud2 * out_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZ> filterPointCloud(
    const lanelet::ConstPolygons3d & no_detection_area_lanelets,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__NO_DETECTION_AREA_FILTER__LANELET2_NO_DETECTION_AREA_FILTER_HPP_
