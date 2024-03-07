cmake_minimum_required(VERSION 3.0.2)
project(viz_ros1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES viz_ros1
 CATKIN_DEPENDS  geometry_msgs nav_msgs roscpp rospy std_msgs visualization_msgs 
)

add_executable(test_rviz_node src/test_rviz.cc)
target_link_libraries(test_rviz_node  ${catkin_LIBRARIES})
