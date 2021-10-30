cmake_minimum_required(VERSION 2.8.3)
project(hockey_shot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ugv_course_libs
  geometry_msgs
  std_msgs
  visualization_msgs
  sensor_msgs 
)


# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(hockey_shot 
  src/hockey_shot.cpp
  src/main.cpp)
  
target_link_libraries(hockey_shot ${catkin_LIBRARIES})
