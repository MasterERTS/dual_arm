cmake_minimum_required(VERSION 2.8.3)
project(dual_planar_arm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
roscpp
sensor_msgs
dual_planar_arm_msgs
tf
)

catkin_package(
#  INCLUDE_DIRS include
LIBRARIES dual_planar_arm
CATKIN_DEPENDS roscpp sensor_msgs dual_planar_arm_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/dual_planar_arm.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/dual_planar_arm_node.cpp)
add_executable(simulator src/simulator.cpp)
add_executable(move_left_arm src/move_left_arm.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_executable(tf_listener src/tf_listener.cpp)
add_executable(right_arm_IK_service src/right_arm_IK_service.cpp)
add_executable(track_left_arm src/track_left_arm.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simulator ${catkin_EXPORTED_TARGETS})
add_dependencies(tf_broadcaster ${catkin_EXPORTED_TARGETS})
add_dependencies(tf_listener ${catkin_EXPORTED_TARGETS})
add_dependencies(right_arm_IK_service ${catkin_EXPORTED_TARGETS})
add_dependencies(move_left_arm ${catkin_EXPORTED_TARGETS})
add_dependencies(track_left_arm ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(simulator
  ${catkin_LIBRARIES}
)
target_link_libraries(move_left_arm
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_broadcaster 
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_listener
  ${catkin_LIBRARIES}
)
target_link_libraries(right_arm_IK_service 
  ${catkin_LIBRARIES}
)
target_link_libraries(track_left_arm 
  ${catkin_LIBRARIES}
)

