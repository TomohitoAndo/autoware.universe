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

#include "pointcloud_preprocessor/no_detection_area_filter/no_detection_area_filter.hpp"

#include "pointcloud_preprocessor/filter.hpp"

#include <pcl_ros/transforms.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
bool pointWithinLanelets(const Point2d & point, const lanelet::ConstPolygons3d & lanelets)
{
  for (const auto & lanelet : lanelets) {
    if (boost::geometry::within(point, lanelet::utils::to2D(lanelet).basicPolygon())) {
      return true;
    }
  }

  return false;
}

tier4_autoware_utils::Box2d calcBoundingBox(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_cloud)
{
  MultiPoint2d candidate_points;
  for (const auto & p : input_cloud->points) {
    candidate_points.emplace_back(p.x, p.y);
  }

  // const auto envelope_box =
  //   boost::geometry::return_envelope<tier4_autoware_utils::Box2d>(candidate_points);

  // return envelope_box;
  return boost::geometry::return_envelope<tier4_autoware_utils::Box2d>(candidate_points);
}

lanelet::ConstPolygons3d calcIntersectedPolygons(
  const tier4_autoware_utils::Box2d & bounding_box, const lanelet::ConstPolygons3d & polygons)
{
  lanelet::ConstPolygons3d intersected_polygons;
  for (const auto & polygon : polygons) {
    if (boost::geometry::intersects(bounding_box, lanelet::utils::to2D(polygon).basicPolygon())) {
      intersected_polygons.push_back(polygon);
    }
  }
  return intersected_polygons;
}

}  // anonymous namespace

namespace pointcloud_preprocessor
{
NoDetectionAreaFilterComponent::NoDetectionAreaFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Node("NoDetectionAreaFilter", node_options)
{
  using std::placeholders::_1;

  // Set publisher
  {
    filtered_pointcloud_pub_ =
      this->create_publisher<PointCloud2>("output", rclcpp::SensorDataQoS());

    debug_pub_ =
      this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/processing_time", 1);
    debug_pub2_ =
      this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/points_size", 1);
  }

  // Set subscriber
  {
    map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&NoDetectionAreaFilterComponent::mapCallback, this, _1));
    pointcloud_sub_ = this->create_subscription<PointCloud2>(
      "input/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&NoDetectionAreaFilterComponent::pointcloudCallback, this, _1));
  }

  // Set tf
  {
    rclcpp::Clock::SharedPtr ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_clock);
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
}

bool NoDetectionAreaFilterComponent::transformPointCloud(
  const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
  PointCloud2 * out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }

  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

pcl::PointCloud<pcl::PointXYZ> NoDetectionAreaFilterComponent::filterPointCloud(
  const lanelet::ConstPolygons3d & no_detection_area_lanelets,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filtered_cloud.header = cloud->header;

  for (const auto & p : cloud->points) {
    if (pointWithinLanelets(Point2d(p.x, p.y), no_detection_area_lanelets)) {
      continue;
    }

    filtered_cloud.points.push_back(p);
  }

  return filtered_cloud;
}

void NoDetectionAreaFilterComponent::pointcloudCallback(const PointCloud2ConstPtr cloud_msg)
{
  // debug
  // timer starts
  const auto t1 = std::chrono::system_clock::now();

  if (!lanelet_map_ptr_ || cloud_msg->data.empty()) {
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>(*cloud_msg);
    filtered_pointcloud_pub_->publish(std::move(output));
    return;
  }

  // transform pointcloud to map frame
  PointCloud2Ptr input_transformed_cloud_ptr(new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud("map", cloud_msg, input_transformed_cloud_ptr.get())) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "Failed transform from "
        << "map"
        << " to " << cloud_msg->header.frame_id);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*input_transformed_cloud_ptr, *cloud);

  // calculate bounding box of points
  const auto bounding_box = calcBoundingBox(cloud);

  // get intersected lanelets
  const auto intersected_lanelets =
    calcIntersectedPolygons(bounding_box, no_detection_area_lanelets_);

  // filter pointcloud by lanelet
  const auto filtered_cloud = filterPointCloud(intersected_lanelets, cloud);

  // transform pointcloud to input frame
  PointCloud2Ptr output_cloud_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(filtered_cloud, *output_cloud_ptr);
  PointCloud2Ptr output_transformed_cloud_ptr(new sensor_msgs::msg::PointCloud2);
  if (!transformPointCloud(
        cloud_msg->header.frame_id, output_cloud_ptr, output_transformed_cloud_ptr.get())) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(10000).count(),
      "Failed transform from " << cloud_msg->header.frame_id << " to "
                               << output_cloud_ptr->header.frame_id);
    return;
  }
  auto output = std::make_unique<sensor_msgs::msg::PointCloud2>(*output_transformed_cloud_ptr);
  filtered_pointcloud_pub_->publish(std::move(output));

  // debug
  // timer ends
  const auto t2 = std::chrono::system_clock::now();
  const auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
  tier4_debug_msgs::msg::Float32Stamped debug_msg;
  debug_msg.stamp = this->now();
  debug_msg.data = elapsed_time.count() / 1000.0;
  debug_pub_->publish(debug_msg);

  tier4_debug_msgs::msg::Float32Stamped debug_msg2;
  debug_msg2.stamp = cloud_msg->header.stamp;
  debug_msg2.data = cloud->size();
  debug_pub2_->publish(debug_msg2);
}

void NoDetectionAreaFilterComponent::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr_);
  no_detection_area_lanelets_ = lanelet::utils::query::getAllNoDetectionArea(lanelet_map_ptr_);
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::NoDetectionAreaFilterComponent)
