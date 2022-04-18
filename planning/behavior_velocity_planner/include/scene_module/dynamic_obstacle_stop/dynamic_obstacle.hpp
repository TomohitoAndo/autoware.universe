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

#ifndef DYNAMIC_OBSTACLE_HPP
#define DYNAMIC_OBSTACLE_HPP

#include "utilization/util.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <geometry_msgs/msg/twist.h>

#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using PathPointsWithLaneId = std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>;

struct DynamicObstacleParam
{
  float min_vel_kmph{0.0};
  float max_vel_kmph{5.0};

  // parameter to convert points to dynamic obstacle
  float diameter{0.1};  // [m]
  float height{2.0};    // [m]
  size_t path_size{20};
  float time_step{0.5};  // [sec]
};

struct PoseWithRange
{
  geometry_msgs::msg::Pose pose_min;
  geometry_msgs::msg::Pose pose_max;
};

class DynamicObstacle
{
public:
  // since we use the minimum and maximum velocity,
  // define the PredictedPath without time_step
  struct PredictedPath
  {
    std::vector<geometry_msgs::msg::Pose> path;
    float confidence;
  };

  DynamicObstacle();
  explicit DynamicObstacle(const DynamicObstacleParam & param);
  void createDynamicObstacle(
    const geometry_msgs::msg::Point & point, const PathWithLaneId & trajectory);
  void createDynamicObstacle(const autoware_auto_perception_msgs::msg::PredictedObject & object);
  void createDynamicObstacle(
    const autoware_auto_perception_msgs::msg::PredictedObject & object,
    const PathWithLaneId & trajectory);

  //  obstacle information
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Point> collision_points;
  geometry_msgs::msg::Point nearest_collision_point;
  float min_velocity_mps;
  float max_velocity_mps;
  std::vector<ObjectClassification> classifications;
  Shape shape;
  std::vector<PredictedPath> predicted_paths;

private:
  std::vector<geometry_msgs::msg::Pose> createPredictedPath(
    const geometry_msgs::msg::Pose & initial_pose, const float time_step,
    const float max_velocity_mps, const size_t path_size) const;

  // params
  DynamicObstacleParam param_;
};
}  // namespace behavior_velocity_planner

#endif  // DYNAMIC_OBSTACLE_HPP
