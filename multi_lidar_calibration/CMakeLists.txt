cmake_minimum_required(VERSION 3.0.2)
project(multi_lidar_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver
  nav_msgs
  ndt_omp
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED QUIET)
find_package(Eigen3 3.3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES multi_lidar_calibration
 CATKIN_DEPENDS geometry_msgs livox_ros_driver nav_msgs ndt_omp pcl_conversions pcl_ros rosbag roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(calibration src/calibration_node.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES})


