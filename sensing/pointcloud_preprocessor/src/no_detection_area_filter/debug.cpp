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

#include "no_detection_area_filter/debug.hpp"

namespace pointcloud_preprocessor
{
namespace
{
}  // namespace

DebugValuePublisher::DebugValuePublisher(rclcpp::Node & node) : node_(node)
{
  pub_debug_values_ = node.create_publisher<Float32MultiArrayStamped>("~/debug/debug_values", 1);
}

void DebugValuePublisher::publishDebugValue()
{
  // publish debug values
  tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = node_.now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_values_->publish(debug_msg);
}
}  // namespace pointcloud_preprocessor
