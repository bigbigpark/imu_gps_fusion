cmake_minimum_required(VERSION 3.0.2)
project(pose_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package (GeographicLib REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_fusion
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs visualization_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(main src/main.cpp src/pose_fusion.cpp src/gps_converter.cpp)
target_link_libraries(main
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  Eigen3::Eigen
)

add_executable(eval src/eval.cpp)
target_link_libraries(eval
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)