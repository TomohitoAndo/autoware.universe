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

#include <motion_utils/motion_utils.hpp>
#include <scene_module/bus_stop/scene.hpp>
#include <utilization/debug.hpp>
#include <utilization/util.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <vector>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

namespace
{
using DebugData = BusStopModule::DebugData;

lanelet::BasicPoint3d getCentroidPoint(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

geometry_msgs::msg::Point toMsg(const lanelet::BasicPoint3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

visualization_msgs::msg::MarkerArray createCorrespondenceMarkerArray(
  const lanelet::autoware::BusStop & bus_stop_reg_elem, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  const lanelet::ConstLineString3d stop_line = bus_stop_reg_elem.stopLine();
  const auto stop_line_center_point =
    (stop_line.front().basicPoint() + stop_line.back().basicPoint()) / 2;

  // ID
  {
    auto marker = createDefaultMarker(
      "map", now, "bus_stop_id", bus_stop_reg_elem.id(),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & bus_stop : bus_stop_reg_elem.busStops()) {
      const auto poly = bus_stop.basicPolygon();

      marker.pose.position = toMsg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(bus_stop_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = createDefaultMarker(
      "map", now, "bus_stop_polygon", bus_stop_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & bus_stop : bus_stop_reg_elem.busStops()) {
      const auto poly = bus_stop.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(toMsg(p_front));
        marker.points.push_back(toMsg(p_back));
      }
    }

    msg.markers.push_back(marker);
  }

  // Polygon to StopLine
  {
    auto marker = createDefaultMarker(
      "map", now, "bus_stop_correspondence", bus_stop_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & bus_stop : bus_stop_reg_elem.busStops()) {
      const auto poly = bus_stop.basicPolygon();
      const auto centroid_point = getCentroidPoint(poly);
      for (size_t i = 0; i < poly.size(); ++i) {
        marker.points.push_back(toMsg(centroid_point));
        marker.points.push_back(toMsg(stop_line_center_point));
      }
    }

    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

visualization_msgs::msg::MarkerArray BusStopModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker;
  const rclcpp::Time now = clock_->now();

  if (!debug_data_.stop_poses.empty()) {
    appendMarkerArray(createCorrespondenceMarkerArray(bus_stop_reg_elem_, now), &debug_marker, now);

    appendMarkerArray(
      debug::createPointsMarkerArray(
        debug_data_.obstacle_points, "obstalces", module_id_, now, 0.6, 0.6, 0.6, 1.0, 0.0, 0.0),
      &debug_marker, now);

    auto nearest_obstacle_marker = createDefaultMarker(
      "map", now, "nearest_obstacle", module_id_, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.6, 0.6, 0.6), createMarkerColor(1.0, 0, 0, 0.999));
    nearest_obstacle_marker.pose.position = debug_data_.nearest_point;
    nearest_obstacle_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    debug_marker.markers.emplace_back(nearest_obstacle_marker);
  }

  return debug_marker;
}

visualization_msgs::msg::MarkerArray BusStopModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const rclcpp::Time now = clock_->now();

  auto id = getModuleId();
  for (const auto & p : debug_data_.stop_poses) {
    const auto p_front =
      tier4_autoware_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(p_front, "bus_stop", now, id++), &wall_marker, now);
  }

  for (const auto & p : debug_data_.dead_line_poses) {
    const auto p_front =
      tier4_autoware_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    appendMarkerArray(
      motion_utils::createDeadLineVirtualWallMarker(p_front, "bus_stop", now, id++), &wall_marker,
      now);
  }

  return wall_marker;
}

}  // namespace behavior_velocity_planner
