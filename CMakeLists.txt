cmake_minimum_required(VERSION 2.8.3)
project(export_bag)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  sensor_msgs
  cv_bridge
  pcl_conversions
  flex_sync
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

find_package(OpenCV 3.3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(export_bag src/export_bag_node.cpp src/export_bag.cpp)
add_dependencies(export_bag ${catkin_EXPORTED_TARGETS})
target_link_libraries(export_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)
