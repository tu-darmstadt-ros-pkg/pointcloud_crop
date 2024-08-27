cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_crop)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_crop_node src/pointcloud_crop_node.cpp)

target_link_libraries(pointcloud_crop_node
  ${catkin_LIBRARIES}
)
