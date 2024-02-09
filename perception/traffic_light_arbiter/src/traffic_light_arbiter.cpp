// Copyright 2023 The Autoware Contributors
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

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/time.hpp>
#include <traffic_light_arbiter/traffic_light_arbiter.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lanelet
{

using TrafficLightConstPtr = std::shared_ptr<const TrafficLight>;
using TrafficLightConstPtr = lanelet::TrafficLightConstPtr;

std::vector<TrafficLightConstPtr> filter_traffic_signals(const LaneletMapConstPtr map)
{
  std::vector<TrafficLightConstPtr> signals;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto signal = std::dynamic_pointer_cast<const TrafficLight>(element);
    if (signal) {
      signals.push_back(signal);
    }
  }

  namespace query = lanelet::utils::query;
  lanelet::ConstLanelets all_lanelets = query::laneletLayer(map);
  std::vector<TrafficLightConstPtr> all_lanelet_traffic_lights = query::trafficLights(all_lanelets);

  for (const auto & light_reg_elem : all_lanelet_traffic_lights) {
    const auto light_id = light_reg_elem->trafficLights().front().id();
    RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), light_reg_elem->id());
    const auto traffic_light_line_string = map->lineStringLayer.get(light_id);
    const auto subtype =
      traffic_light_line_string.attributeOr(lanelet::AttributeName::Subtype, "none");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "subtype: " << subtype);
  }

  // RCLCPP_WARN_STREAM(
  //   rclcpp::get_logger("debug"), "--------------- lanelet traffic light ---------------");

  // std::vector<lanelet::TrafficLightConstPtr> all_traffic_lights =
  //   query::trafficLights(all_lanelets);
  // for (const auto & light : all_traffic_lights) {
  //   RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), light->id());
  // }

  return signals;
}

// std::vector<TrafficLightConstPtr> filter_pedestrian_signals(const LaneletMapConstPtr map)
// {
//   namespace query = lanelet::utils::query;

//   const auto all_lanelets = query::laneletLayer(map);
//   const auto crosswalks = query::crosswalkLanelets(all_lanelets);
//   std::vector<TrafficLightConstPtr> signals;
//   for (const auto & crosswalk : crosswalks) {
//     const auto traffic_light_reg_elems =
//       crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();
//     for (const auto & reg_elem : traffic_light_reg_elems) {
//       signals.emplace_back(reg_elem);
//     }
//   }

//   return signals;
// }

std::vector<TrafficLightConstPtr> filter_pedestrian_signals(const LaneletMapConstPtr map)
{
  namespace query = lanelet::utils::query;

  const auto all_lanelets = query::laneletLayer(map);
  const auto crosswalks = query::crosswalkLanelets(all_lanelets);
  std::vector<TrafficLightConstPtr> signals;

  for (const auto & crosswalk : crosswalks) {
    const auto traffic_light_reg_elems =
      crosswalk.regulatoryElementsAs<const lanelet::TrafficLight>();
    std::transform(
      traffic_light_reg_elems.begin(), traffic_light_reg_elems.end(), std::back_inserter(signals),
      [](const auto & elem) { return elem; });
  }

  return signals;
}

}  // namespace lanelet

namespace util
{
using Element = autoware_perception_msgs::msg::TrafficSignalElement;
using TrafficSignal = autoware_perception_msgs::msg::TrafficSignal;

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
    unknown_elements.emplace_back(
      util::create_element(Element::UNKNOWN, shape, Element::SOLID_ON, 1.0));
  }

  return unknown_elements;
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
    [](const Element & a, const Element & b) {
      return a.color == b.color && a.shape == b.shape && a.status == b.status;
    });
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

}  // namespace util

TrafficLightArbiter::TrafficLightArbiter(const rclcpp::NodeOptions & options)
: Node("traffic_light_arbiter", options)
{
  external_time_tolerance_ = this->declare_parameter<double>("external_time_tolerance", 5.0);
  perception_time_tolerance_ = this->declare_parameter<double>("perception_time_tolerance", 1.0);
  external_priority_ = this->declare_parameter<bool>("external_priority", false);

  map_sub_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightArbiter::onMap, this, std::placeholders::_1));

  perception_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/perception_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::onPerceptionMsg, this, std::placeholders::_1));

  external_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/external_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::onExternalMsg, this, std::placeholders::_1));

  pub_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));
}

void TrafficLightArbiter::onMap(const LaneletMapBin::ConstSharedPtr msg)
{
  const auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map);

  const auto signals = lanelet::filter_traffic_signals(map);
  map_regulatory_elements_set_ = std::make_unique<std::unordered_set<lanelet::Id>>();

  for (const auto & signal : signals) {
    map_regulatory_elements_set_->emplace(signal->id());
  }

  // Filter only pedestrian signals to distinguish them in compare function
  const auto pedestrian_signals = lanelet::filter_pedestrian_signals(map);
  map_pedestrian_signal_regulatory_elements_set_ =
    std::make_unique<std::unordered_set<lanelet::Id>>();

  for (const auto & signal : pedestrian_signals) {
    map_pedestrian_signal_regulatory_elements_set_->emplace(signal->id());
  }
}

void TrafficLightArbiter::onPerceptionMsg(const TrafficSignalArray::ConstSharedPtr msg)
{
  latest_perception_msg_ = *msg;

  // TODO: maybe this is not needed anymore
  // if (
  //   (rclcpp::Time(msg->stamp) - rclcpp::Time(latest_external_msg_.stamp)).seconds() >
  //   external_time_tolerance_) {
  //   latest_external_msg_.signals.clear();
  // }

  arbitrateAndPublish(msg->stamp);
}

void TrafficLightArbiter::onExternalMsg(const TrafficSignalArray::ConstSharedPtr msg)
{
  latest_external_msg_ = *msg;

  // if (
  //   (rclcpp::Time(msg->stamp) - rclcpp::Time(latest_perception_msg_.stamp)).seconds() >
  //   perception_time_tolerance_) {
  //   latest_perception_msg_.signals.clear();
  // }

  arbitrateAndPublish(msg->stamp);
}

void TrafficLightArbiter::arbitrateAndPublish(const builtin_interfaces::msg::Time & stamp)
{
  using ElementAndPriority = std::pair<Element, bool>;
  std::unordered_map<lanelet::Id, std::vector<ElementAndPriority>> regulatory_element_signals_map;

  if (map_regulatory_elements_set_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received traffic signal messages before a map");
    return;
  }

  TrafficSignalArray output_signals_msg;
  output_signals_msg.stamp = stamp;

  if (map_regulatory_elements_set_->empty()) {
    pub_->publish(output_signals_msg);
    return;
  }

  auto add_signal_function = [&](const auto & signal, bool priority) {
    const auto id = signal.traffic_signal_id;
    if (!map_regulatory_elements_set_->count(id)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Received a traffic signal not present in the current map (%lu)", id);
      return;
    }

    auto & elements_and_priority = regulatory_element_signals_map[id];
    for (const auto & element : signal.elements) {
      elements_and_priority.emplace_back(element, priority);
    }
  };

  auto compare_and_add_signal_function = [&](
                                           const TrafficSignalArray & perception_signals,
                                           const TrafficSignalArray & external_signals) {
    // Compare the signal element for each received signal id
    const auto received_signal_id_set =
      util::create_signal_id_set(perception_signals.signals, external_signals.signals);

    for (const auto & signal_id : received_signal_id_set) {
      const auto find_signal_by_id = [&signal_id](const TrafficSignalArray & signals) {
        return std::find_if(
          signals.signals.begin(), signals.signals.end(),
          [&signal_id](const TrafficSignal & signal) {
            return signal.traffic_signal_id == signal_id;
          });
      };

      const auto perception_result = find_signal_by_id(perception_signals);
      const auto external_result = find_signal_by_id(external_signals);

      auto & elements_and_priority = regulatory_element_signals_map[signal_id];
      const bool perception_result_exists = perception_result != perception_signals.signals.end();
      const bool external_result_exists = external_result != external_signals.signals.end();

      // Ignore pedestrian lights
      if (
        map_pedestrian_signal_regulatory_elements_set_->find(signal_id) !=
        map_pedestrian_signal_regulatory_elements_set_->end()) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "Ignoring pedestrian light: " << signal_id);
        if (perception_result_exists) {
          add_signal_function(*perception_result, false);
        }
        if (external_result_exists) {
          add_signal_function(*external_result, true);
        }
        continue;
      }

      auto insert_unknown_elements = [&](const auto & result) {
        std::vector<Element> unknown_elements;
        for (const auto & element : result->elements) {
          unknown_elements.emplace_back(
            util::create_element(Element::UNKNOWN, element.shape, Element::SOLID_ON, 1.0));
        }
        std::transform(
          unknown_elements.begin(), unknown_elements.end(),
          std::back_inserter(elements_and_priority),
          [](const auto & elem) { return std::make_pair(elem, false); });
      };

      // If either of the signal is not received, treat as unknown signal
      if (!perception_result_exists || !external_result_exists) {
        if (perception_result_exists) {
          insert_unknown_elements(perception_result);
        } else if (external_result_exists) {
          insert_unknown_elements(external_result);
        }
        continue;
      }

      // Check if they have the same elements
      if (!util::are_all_elements_equivalent(
            perception_result->elements, external_result->elements)) {
        // Insert unknown signal if not same
        const auto unknown_elements =
          util::create_unknown_elements(perception_result->elements, external_result->elements);
        std::transform(
          unknown_elements.begin(), unknown_elements.end(),
          std::back_inserter(elements_and_priority),
          [](const auto & elem) { return std::make_pair(elem, false); });
      } else {
        // Both results are same, so insert the received color
        for (const auto & element : perception_result->elements) {
          elements_and_priority.emplace_back(element, false);
        }
      }
    }
  };

  // TODO: parameter
  const bool compare_perception_and_external_result = true;
  if (compare_perception_and_external_result) {
    compare_and_add_signal_function(latest_perception_msg_, latest_external_msg_);
  } else {
    for (const auto & signal : latest_perception_msg_.signals) {
      add_signal_function(signal, false);
    }

    for (const auto & signal : latest_external_msg_.signals) {
      add_signal_function(signal, external_priority_);
    }
  }

  const auto get_highest_confidence_elements =
    [](const std::vector<ElementAndPriority> & elements_and_priority_vector) {
      using Key = Element::_shape_type;
      std::map<Key, ElementAndPriority> highest_score_element_and_priority_map;
      std::vector<Element> highest_score_elements_vector;

      for (const auto & elements_and_priority : elements_and_priority_vector) {
        const auto & element = elements_and_priority.first;
        const auto & element_priority = elements_and_priority.second;
        const auto key = element.shape;
        auto [iter, success] =
          highest_score_element_and_priority_map.try_emplace(key, elements_and_priority);
        const auto & iter_element = iter->second.first;
        const auto & iter_priority = iter->second.second;

        if (
          !success &&
          (iter_element.confidence < element.confidence || iter_priority < element_priority)) {
          iter->second = elements_and_priority;
        }
      }

      for (const auto & [k, v] : highest_score_element_and_priority_map) {
        highest_score_elements_vector.emplace_back(v.first);
      }

      return highest_score_elements_vector;
    };

  output_signals_msg.signals.reserve(regulatory_element_signals_map.size());

  for (const auto & [regulatory_element_id, elements] : regulatory_element_signals_map) {
    TrafficSignal signal_msg;
    signal_msg.traffic_signal_id = regulatory_element_id;
    signal_msg.elements = get_highest_confidence_elements(elements);
    output_signals_msg.signals.emplace_back(signal_msg);
  }

  pub_->publish(output_signals_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightArbiter)
