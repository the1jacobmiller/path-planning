cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  geometry_msgs
  dbw_mkz_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planner
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/motion_planner.cpp
# )

add_executable(path_planner_node src/path_planner.cpp)
target_link_libraries(path_planner_node ${catkin_LIBRARIES})
add_dependencies(path_planner_node ${catkin_EXPORTED_TARGETS})

add_executable(extract_lane_and_speed_data_node src/extract_lane_and_speed_data.cpp)
target_link_libraries(extract_lane_and_speed_data_node ${catkin_LIBRARIES})
add_dependencies(extract_lane_and_speed_data_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
