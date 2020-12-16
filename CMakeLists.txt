cmake_minimum_required(VERSION 2.8.3)
project(lidar_imu_calib)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-o3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  tf
  rosbag
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_imu_calib
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN3_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(calib src/lib/calibExRLidar2Imu.cpp)
target_link_libraries(calib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp)
add_executable(calib_exR_lidar2imu_node src/calib_exR_lidar2imu_node.cpp)
target_link_libraries(calib_exR_lidar2imu_node calib)
