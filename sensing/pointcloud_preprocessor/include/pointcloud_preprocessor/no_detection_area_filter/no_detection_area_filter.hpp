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

#ifndef POINTCLOUD_PREPROCESSOR__NO_DETECTION_AREA_FILTER__NO_DETECTION_AREA_FILTER_HPP_
#define POINTCLOUD_PREPROCESSOR__NO_DETECTION_AREA_FILTER__NO_DETECTION_AREA_FILTER_HPP_

#include "pointcloud_preprocessor/filter.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <lanelet2_core/geometry/Polygon.h>

// debug
#include "pointcloud_preprocessor/no_detection_area_filter/debug.hpp"

#include <memory>
#include <string>
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointCgal = K::Point_2;

using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::Point2d;

namespace pointcloud_preprocessor
{
class NoDetectionAreaFilterComponent : public pointcloud_preprocessor::Filter
{
private:
  void filter(
    const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
    PointCloud2 & output) override;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  lanelet::ConstPolygons3d no_detection_area_lanelets_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  // parameter
  std::string polygon_type_;

  // debug
  std::unique_ptr<DebugValuePublisher> debug_value_publisher_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit NoDetectionAreaFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__NO_DETECTION_AREA_FILTER__NO_DETECTION_AREA_FILTER_HPP_
