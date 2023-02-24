cmake_minimum_required(VERSION 3.14)
project(lateral_nearest_map_point_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_add_library(lateral_nearest_map_point_publisher SHARED
  src/lateral_nearest_map_point_publisher.cpp
)

# find_package(PCL REQUIRED COMPONENTS common filters)
target_include_directories(lateral_nearest_map_point_publisher
  SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

rclcpp_components_register_node(lateral_nearest_map_point_publisher
  PLUGIN "lateral_nearest_map_point_publisher::LateralNearestMapPointPublisher"
  EXECUTABLE lateral_nearest_map_point_publisher_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
