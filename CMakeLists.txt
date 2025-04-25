cmake_minimum_required(VERSION 3.8)
project(pcl_filter_project)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Libreria
add_library(pointcloud_filter src/pointcloud_filter.cpp)
ament_target_dependencies(
  pointcloud_filter
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  PCL
)

# Nodo
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
ament_target_dependencies(pointcloud_filter_node rclcpp sensor_msgs std_msgs pcl_conversions)

target_link_libraries(pointcloud_filter_node
  pointcloud_filter
)

# Installazioni
install(TARGETS
  pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
