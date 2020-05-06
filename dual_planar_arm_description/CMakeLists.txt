cmake_minimum_required(VERSION 2.8.3)
project(dual_planar_arm_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
)

## System dependencies are found with CMake's conventions

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dual_planar_arm_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(parser src/parser.cpp)


target_link_libraries(parser ${catkin_LIBRARIES})

