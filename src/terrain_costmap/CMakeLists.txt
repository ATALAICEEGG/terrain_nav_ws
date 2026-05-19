cmake_minimum_required(VERSION 3.8)
project(terrain_costmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)

# Create executable
add_executable(terrain_costmap_node
  src/terrain_costmap_node.cpp
)

ament_target_dependencies(terrain_costmap_node
  rclcpp
  std_msgs
  grid_map_msgs
  grid_map_ros
)

# Install targets
install(TARGETS terrain_costmap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
