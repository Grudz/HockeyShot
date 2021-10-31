cmake_minimum_required(VERSION 3.0.2)
project(hockey_shot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_msgs
  pcl_conversions
  ugv_course_libs
  geometry_msgs
  std_msgs
  visualization_msgs
  sensor_msgs 
  dynamic_reconfigure
  avs_lecture_msgs
  image_transport
  image_geometry
  cv_bridge
  pcl_ros
  rospy
)

# This may be needed for rviz overlay
#find_package(Qt4 REQUIRED)
#include(${QT_USE_FILE})
#add_definitions(-DQT_NO_KEYWORDS)

# System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)   

generate_dynamic_reconfigure_options(
  cfg/HockeyShot.cfg
)

# Declare catkin package
catkin_package(
  DEPENDS OpenCV
)

# Include core ROS library directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Compile an executable node
add_executable(hockey_shot 
  src/hockey_shot.cpp
  src/main.cpp
)
add_dependencies(hockey_shot
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(hockey_shot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

