cmake_minimum_required(VERSION 2.8.3)
project(apriltags_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs 
  DEPENDS
    eigen3
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(AprilTags . /opt/local/include)

add_executable(apriltags_detector src/apriltagdetector.cpp)
target_link_libraries(apriltags_detector ${catkin_LIBRARIES})
add_dependencies(apriltags_detector apriltagdetector)


