cmake_minimum_required(VERSION 3.0.2)
project(dataset_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
find_package(
  Boost 
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs 
  cv_bridge 
  image_transport
  OpenCV
  livox_ros_driver
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS     
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

include_directories(/usr/local/include)
include_directories(/usr/local/include/opencv)
include_directories(/usr/local/include/opencv2)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(/usr/local/lib)
set(ROS_LINK_FLAGS "-lopencv_core -lopencv_highgui -lopencv_legacy -lopencv_imgproc -lopencv_features2d")

 
add_executable(aviaFormatConvert src/aviaFormatConvert.cpp)
target_link_libraries(aviaFormatConvert ${catkin_LIBRARIES})

add_executable(horizonFormatConvert src/horizonFormatConvert.cpp)
target_link_libraries(horizonFormatConvert ${catkin_LIBRARIES})
 
add_executable(lidarsExtrinsicComp src/lidars_extrinsic_comp.cpp)
target_link_libraries(lidarsExtrinsicComp ${catkin_LIBRARIES})
 
 