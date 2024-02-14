// Copyright 2024 The Autoware Contributors
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

#include "traffic_light_arbiter/signal_match_validator.hpp"

namespace util
{
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficSignalArray;
using TrafficSignal = autoware_perception_msgs::msg::TrafficSignal;
using Element = autoware_perception_msgs::msg::TrafficSignalElement;

std::optional<TrafficSignal> find_signal_by_id(
  const TrafficSignalArray & signals, int64_t signal_id)
{
  auto it = std::find_if(
    signals.signals.begin(), signals.signals.end(),
    [signal_id](const TrafficSignal & signal) { return signal.traffic_signal_id == signal_id; });
  if (it != signals.signals.end()) {
    return *it;
  } else {
    return std::nullopt;
  }
}

Element create_element(
  const Element::_color_type & color, const Element::_shape_type & shape,
  const Element::_status_type & status, const Element::_confidence_type & confidence)
{
  Element signal_element;
  signal_element.color = color;
  signal_element.shape = shape;
  signal_element.status = status;
  signal_element.confidence = confidence;

  return signal_element;
}

// Create unknown elements for each shape
std::vector<Element> create_unknown_elements(
  const std::vector<Element> & elements1, const std::vector<Element> & elements2)
{
  std::unordered_set<Element::_shape_type> shape_set;
  for (const auto & element : elements1) {
    shape_set.emplace(element.shape);
  }
  for (const auto & element : elements2) {
    shape_set.emplace(element.shape);
  }

  std::vector<Element> unknown_elements;
  for (const auto & shape : shape_set) {
    // Confidence doesn't matter because this is the unknown signal
    unknown_elements.emplace_back(
      util::create_element(Element::UNKNOWN, shape, Element::UNKNOWN, /* confidence */ 1.0));
  }

  return unknown_elements;
}

TrafficSignal create_unknown_signal(const TrafficSignal & signal)
{
  TrafficSignal unknown_signal;
  unknown_signal.traffic_signal_id = signal.traffic_signal_id;
  for (const auto & element : signal.elements) {
    // Confidence doesn't matter because this is the unknown signal
    const auto unknown_element =
      util::create_element(Element::UNKNOWN, element.shape, Element::UNKNOWN, /* confidence */ 1.0);
    unknown_signal.elements.emplace_back(unknown_element);
  }

  return unknown_signal;
}

TrafficSignal create_unknown_signal(const TrafficSignal & signal1, const TrafficSignal & signal2)
{
  TrafficSignal unknown_signal;

  // Assume that the both ids are same
  unknown_signal.traffic_signal_id = signal1.traffic_signal_id;

  const auto unknown_elements = util::create_unknown_elements(signal1.elements, signal2.elements);
  for (const auto & element : unknown_elements) {
    unknown_signal.elements.emplace_back(element);
  }

  return unknown_signal;
}

bool are_all_elements_equivalent(
  const std::vector<Element> & signal1, const std::vector<Element> & signal2)
{
  // Check if the vectors have the same size
  if (signal1.size() != signal2.size()) {
    return false;
  }

  // Create copies of the vectors
  std::vector<Element> sorted_signal1 = signal1;
  std::vector<Element> sorted_signal2 = signal2;

  // Sort based on the shape to ensure that they are same order
  auto compare_by_shape = [](const Element & a, const Element & b) { return a.shape < b.shape; };
  std::sort(sorted_signal1.begin(), sorted_signal1.end(), compare_by_shape);
  std::sort(sorted_signal2.begin(), sorted_signal2.end(), compare_by_shape);

  // Compare the sorted vectors and return true if they have all the same elements
  return std::equal(
    sorted_signal1.begin(), sorted_signal1.end(), sorted_signal2.begin(), sorted_signal2.end(),
    [](const Element & a, const Element & b) { return a.color == b.color && a.shape == b.shape; });
}

std::unordered_set<lanelet::Id> create_signal_id_set(
  const std::vector<TrafficSignal> & signals1, const std::vector<TrafficSignal> & signals2)
{
  std::unordered_set<lanelet::Id> signal_id_set;
  for (const auto & signal : signals1) {
    signal_id_set.emplace(signal.traffic_signal_id);
  }
  for (const auto & signal : signals2) {
    signal_id_set.emplace(signal.traffic_signal_id);
  }

  return signal_id_set;
}

TrafficSignal get_highest_confidence_signal(
  const std::optional<TrafficSignal> & perception_signal,
  const std::optional<TrafficSignal> & external_signal, const bool external_priority)
{
  // If the either of the signal doesn't exist, return the signal that exists
  if (!perception_signal) {
    return *external_signal;
  }
  if (!external_signal) {
    return *perception_signal;
  }

  // If the external_priority is true, use the external results
  if (external_priority) {
    return *external_signal;
  }

  // Create map using shape as key
  using Key = Element::_shape_type;
  std::map<Key, std::vector<Element>> shape_element_map;
  for (const auto & element : perception_signal->elements) {
    shape_element_map[element.shape].emplace_back(element);
  }
  for (const auto & element : external_signal->elements) {
    shape_element_map[element.shape].emplace_back(element);
  }

  TrafficSignal highest_confidence_signal;

  // Assume that the both ids are same
  highest_confidence_signal.traffic_signal_id = perception_signal->traffic_signal_id;

  // Find the highest confidence element and push it
  for (const auto & shape_and_elements : shape_element_map) {
    const auto & elements = shape_and_elements.second;
    const auto highest_confidence_element = std::max_element(
      elements.begin(), elements.end(),
      [](const Element & a, const Element & b) { return a.confidence < b.confidence; });
    highest_confidence_signal.elements.emplace_back(*highest_confidence_element);
  }

  return highest_confidence_signal;
}

using Time = builtin_interfaces::msg::Time;
Time get_newer_stamp(const Time & stamp1, const Time & stamp2)
{
  if (stamp1.sec > stamp2.sec || (stamp1.sec == stamp2.sec && stamp1.nanosec > stamp2.nanosec)) {
    return stamp1;
  } else {
    return stamp2;
  }
}

}  // namespace util

autoware_perception_msgs::msg::TrafficSignalArray SignalMatchValidator::validateSignals(
  const TrafficSignalArray & perception_signals, const TrafficSignalArray & external_signals)
{
  TrafficSignalArray validated_signals;

  // Set newer stamp
  validated_signals.stamp = util::get_newer_stamp(perception_signals.stamp, external_signals.stamp);

  // Create the unique set of the received id,
  // then compare the signal element for each received signal id
  const auto received_signal_id_set =
    util::create_signal_id_set(perception_signals.signals, external_signals.signals);

  for (const auto & signal_id : received_signal_id_set) {
    const auto perception_result = util::find_signal_by_id(perception_signals, signal_id);
    const auto external_result = util::find_signal_by_id(external_signals, signal_id);

    // Both results doesn't exist
    if (!perception_result && !external_result) {
      continue;
    }

    // We don't validate the pedestrian signals
    // TODO(TomohitoAndo): Validate pedestrian signals
    if (isPedestrianSignal(signal_id)) {
      validated_signals.signals.emplace_back(util::get_highest_confidence_signal(
        perception_result, external_result, external_priority_));

      continue;
    }

    // If either of the signal is not received, treat as unknown signal
    if (!perception_result && external_result) {
      const auto unknown_signal = util::create_unknown_signal(*external_result);
      validated_signals.signals.emplace_back(unknown_signal);
      continue;
    }
    if (!external_result && perception_result) {
      const auto unknown_signal = util::create_unknown_signal(*perception_result);
      validated_signals.signals.emplace_back(unknown_signal);
      continue;
    }

    // Check if they have the same elements
    if (!util::are_all_elements_equivalent(
          perception_result->elements, external_result->elements)) {
      // RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "Not the same signal");

      const auto unknown_signal = util::create_unknown_signal(*perception_result, *external_result);
      validated_signals.signals.emplace_back(unknown_signal);
      continue;
    }

    // Both results are same, then insert the received color
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "Both results are same");
    validated_signals.signals.emplace_back(*perception_result);
  }

  return validated_signals;
}

void SignalMatchValidator::setPedestrianSignals(
  const std::vector<TrafficLightConstPtr> & pedestrian_signals)
{
  for (const auto & signal : pedestrian_signals) {
    map_pedestrian_signal_regulatory_elements_set_.emplace(signal->id());
  }
}

void SignalMatchValidator::setExternalPriority(const bool external_priority)
{
  external_priority_ = external_priority;
}

bool SignalMatchValidator::isPedestrianSignal(const lanelet::Id & signal_id)
{
  return map_pedestrian_signal_regulatory_elements_set_.find(signal_id) !=
         map_pedestrian_signal_regulatory_elements_set_.end();
}
