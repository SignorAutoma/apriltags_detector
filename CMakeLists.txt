cmake_minimum_required(VERSION 2.8.3)
project(apriltags_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

##list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation

)

add_message_files(
   FILES
   AprilTag.msg
   AprilTagList.msg
)


generate_messages(
  DEPENDENCIES
  sensor_msgs#   std_msgs
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs 
    message_runtime
  DEPENDS
    eigen3
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include /usr/include/eigen3)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(
	apriltags_detector src/apriltagdetector.cpp 
	src/AprilTags/Edge.cc
	src/AprilTags/FloatImage.cc
	src/AprilTags/GLine2D.cc
	src/AprilTags/GLineSegment2D.cc
	src/AprilTags/Gaussian.cc
	src/AprilTags/GrayModel.cc
	src/AprilTags/Homography33.cc
	src/AprilTags/MathUtil.cc
	src/AprilTags/Quad.cc
	src/AprilTags/Segment.cc
	src/AprilTags/TagDetection.cc
	src/AprilTags/TagDetector.cc
	src/AprilTags/TagFamily.cc
	src/AprilTags/UnionFindSimple.cc
)
target_link_libraries(apriltags_detector ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

#~ file(GLOB SOURCE_FILES "src/AprilTags/*.cc")
#~ include_directories(AprilTags . /opt/local/include)
#~ add_library(apriltags ${SOURCE_FILES})

#~ include_directories(include)
#~ include_directories(${catkin_INCLUDE_DIRS})
#~ include_directories(${Eigen_INCLUDE_DIRS})
#~ include_directories(AprilTags . /opt/local/include)

#~ add_executable(apriltags_detector src/apriltagdetector.cpp)
#~ target_link_libraries(apriltags_detector ${catkin_LIBRARIES})
#~ add_dependencies(apriltags_detector apriltagdetector)


