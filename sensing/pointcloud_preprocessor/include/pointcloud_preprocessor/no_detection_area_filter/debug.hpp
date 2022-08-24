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
#ifndef LANELET_AREA_FILTER__DEBUG_HPP_
#define LANELET_AREA_FILTER__DEBUG_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>

#include <array>
#include <string>
#include <vector>

namespace pointcloud_preprocessor
{
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

class DebugValues
{
public:
  enum class TYPE {
    CALCULATION_TIME_BG = 0,
    CALCULATION_TIME_CGAL = 1,
    SIZE_POINTS = 2,
    SIZE,  // this is the number of enum elements
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  int getValuesIdx(const TYPE type) const { return static_cast<int>(type); }
  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<float, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE type, const float val) { values_.at(static_cast<int>(type)) = val; }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const int type, const float val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<float, static_cast<int>(TYPE::SIZE)> values_;
};

class DebugValuePublisher
{
public:
  // explicit DebugValuePublisher(rclcpp::Node & node);
  explicit DebugValuePublisher(rclcpp::Node & node) : node_(node)
  {
    pub_debug_values_ = node.create_publisher<Float32MultiArrayStamped>("~/debug/debug_values", 1);
  }
  ~DebugValuePublisher() {}

  void setDebugValues(const DebugValues::TYPE type, const double val)
  {
    debug_values_.setValues(type, val);
  }

  void publishDebugValue()
  {
    // publish debug values
    tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
    debug_msg.stamp = node_.now();
    for (const auto & v : debug_values_.getValues()) {
      debug_msg.data.push_back(v);
    }
    pub_debug_values_->publish(debug_msg);
  }

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_values_;
  DebugValues debug_values_;
};

}  // namespace pointcloud_preprocessor

#endif  // LANELET_AREA_FILTER__DEBUG_HPP_
