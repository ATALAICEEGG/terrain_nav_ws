cmake_minimum_required(VERSION 3.8)
project(terrain_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(terrain_planner_node
  src/terrain_planner_node.cpp
)

ament_target_dependencies(terrain_planner_node
  rclcpp
  grid_map_msgs
  grid_map_ros
  grid_map_core
  nav_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS terrain_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
