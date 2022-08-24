// Copyright 2022 Tier IV, Inc.
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

pcl::PointCloud<pcl::PointXYZ> removePointsWithinPolygons(
  const lanelet::ConstPolygons3d & polygons, const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filtered_cloud.header = cloud->header;

  for (const auto & p : cloud->points) {
    if (pointWithinLanelets(Point2d(p.x, p.y), polygons)) {
      continue;
    }

    filtered_cloud.points.push_back(p);
  }

  return filtered_cloud;
}

// std::vector<PointCgal> polygon_geometry_to_cgal(
//   const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in)
// {
//   std::vector<PointCgal> polyline_polygon;
//   if (polygon_in->points.size() < 3) {
//     throw std::length_error("Polygon vertex count should be larger than 2.");
//   }
//   const auto & vertices_in = polygon_in->points;
//   polyline_polygon.resize(vertices_in.size());
//   std::transform(
//     polygon_in->points.begin(), polygon_in->points.end(), polyline_polygon.begin(),
//     [](const geometry_msgs::msg::Point32 & p_in) { return PointCgal(p_in.x, p_in.y); });
//   return polyline_polygon;
// }

std::vector<PointCgal> polygon_lanelet_to_cgal(const lanelet::BasicPolygon2d & polygon_in)
{
  std::vector<PointCgal> polyline_polygon;
  if (polygon_in.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in;
  polyline_polygon.resize(vertices_in.size());
  std::transform(
    polygon_in.cbegin(), polygon_in.cend(), polyline_polygon.begin(),
    [](const Eigen::Matrix<double, 2, 1> & p_in) { return PointCgal(p_in.x(), p_in.y()); });
  return polyline_polygon;
}

// sensor_msgs::msg::PointCloud2 remove_polygon_cgal_from_cloud(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_in_ptr,
//   const std::vector<PointCgal> & polyline_polygon)
// {
//   sensor_msgs::msg::PointCloud2 output;
//   pcl::PointCloud<pcl::PointXYZ> pcl_output;

//   sensor_msgs::msg::PointCloud2 transformed_cluster = *cloud_in_ptr;

//   for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
//        iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
//        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
//     if (
//       CGAL::bounded_side_2(
//         polyline_polygon.begin(), polyline_polygon.end(), PointCgal(*iter_x, *iter_y), K()) ==
//       CGAL::ON_UNBOUNDED_SIDE) {
//       pcl::PointXYZ p;
//       p.x = *iter_x;
//       p.y = *iter_y;
//       p.z = *iter_z;
//       pcl_output.emplace_back(p);
//     }
//   }
//   pcl::toROSMsg(pcl_output, output);
//   output.header = cloud_in_ptr->header;
//   return output;
// }

pcl::PointCloud<pcl::PointXYZ> remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const std::vector<PointCgal> & polyline_polygon)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_output;

  for (const auto & p : cloud_in) {
    if (
      CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(), PointCgal(p.x, p.y), K()) ==
      CGAL::ON_UNBOUNDED_SIDE) {
      pcl_output.emplace_back(p);
    }
  }

  return pcl_output;
}

// sensor_msgs::msg::PointCloud2 removePointsWithinPolygons(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_in_ptr,
//   const lanelet::ConstPolygons3d & polygons)
// {
//   auto filtered_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(*cloud_in_ptr);

//   for (const auto & polygon : polygons) {
//     const auto lanelet_poly = lanelet::utils::to2D(polygon).basicPolygon();
//     const auto cgal_poly = polygon_lanelet_to_cgal(lanelet_poly);
//     filtered_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(
//       remove_polygon_cgal_from_cloud(filtered_cloud_ptr, cgal_poly));
//   }

//   return *filtered_cloud_ptr;
// }

pcl::PointCloud<pcl::PointXYZ> removePointsWithinCgalPolygons(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, const lanelet::ConstPolygons3d & polygons)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud = *cloud_in;

  for (const auto & polygon : polygons) {
    const auto lanelet_poly = lanelet::utils::to2D(polygon).basicPolygon();
    const auto cgal_poly = polygon_lanelet_to_cgal(lanelet_poly);
    filtered_cloud = remove_polygon_cgal_from_cloud(filtered_cloud, cgal_poly);
  }

  return filtered_cloud;
}

}  // anonymous namespace

namespace pointcloud_preprocessor
{
NoDetectionAreaFilterComponent::NoDetectionAreaFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Filter("NoDetectionAreaFilter", node_options)
{
  polygon_type_ =
    static_cast<std::string>(declare_parameter("polygon_type", "no_obstacle_segmentation_area"));

  debug_value_publisher_ = std::make_unique<DebugValuePublisher>(*this);

  using std::placeholders::_1;
  // Set subscriber
  map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NoDetectionAreaFilterComponent::mapCallback, this, _1));
}

void NoDetectionAreaFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  if (no_detection_area_lanelets_.empty()) {
    output = *input;
    return;
  }

  // convert to PCL message
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*input, *pc_input);

  // calculate bounding box of points
  const auto bounding_box = calcBoundingBox(pc_input);

  // use only intersected lanelets to reduce calculation cost
  const auto intersected_lanelets =
    calcIntersectedPolygons(bounding_box, no_detection_area_lanelets_);

  // timer starts
  const auto t1_bg = std::chrono::system_clock::now();

  // filter pointcloud by lanelet
  // const auto filtered_pc = removePointsWithinPolygons(intersected_lanelets, pc_input);
  const auto filtered_pc = removePointsWithinPolygons(no_detection_area_lanelets_, pc_input);

  // timer ends
  const auto t2_bg = std::chrono::system_clock::now();
  const auto elapsed_bg = std::chrono::duration_cast<std::chrono::microseconds>(t2_bg - t1_bg);

  // timer starts
  const auto t1_cgal = std::chrono::system_clock::now();

  // CGAL points remover
  // output = removePointsWithinPolygons(input, intersected_lanelets);
  // const auto cgal_filtered_pc = removePointsWithinCgalPolygons(pc_input, intersected_lanelets);
  const auto cgal_filtered_pc =
    removePointsWithinCgalPolygons(pc_input, no_detection_area_lanelets_);

  // timer ends
  const auto t2_cgal = std::chrono::system_clock::now();
  const auto elapsed_cgal =
    std::chrono::duration_cast<std::chrono::microseconds>(t2_cgal - t1_cgal);

  RCLCPP_WARN_STREAM(
    rclcpp::get_logger("debug"), "polygon size: " << no_detection_area_lanelets_.size());
  RCLCPP_WARN_STREAM(
    rclcpp::get_logger("debug"),
    "bg, cgal = " << elapsed_bg.count() / 1000.0 << ", " << elapsed_cgal.count() / 1000.0);

  debug_value_publisher_->setDebugValues(
    DebugValues::TYPE::CALCULATION_TIME_BG, elapsed_bg.count() / 1000.0);
  debug_value_publisher_->setDebugValues(
    DebugValues::TYPE::CALCULATION_TIME_CGAL, elapsed_cgal.count() / 1000.0);
  debug_value_publisher_->setDebugValues(DebugValues::TYPE::SIZE_POINTS, pc_input->size());
  debug_value_publisher_->publishDebugValue();

  // convert to ROS message
  pcl::toROSMsg(cgal_filtered_pc, output);
  output.header = input->header;
}

void NoDetectionAreaFilterComponent::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr map_msg)
{
  tf_input_frame_ = map_msg->header.frame_id;

  const auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_ptr);
  no_detection_area_lanelets_ =
    lanelet::utils::query::getAllPolygonsByType(lanelet_map_ptr, polygon_type_);
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::NoDetectionAreaFilterComponent)
