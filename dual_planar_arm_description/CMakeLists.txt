cmake_minimum_required(VERSION 2.8.3)
project(dual_planar_arm_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(parser src/parser.cpp)

target_link_libraries(parser ${catkin_LIBRARIES})