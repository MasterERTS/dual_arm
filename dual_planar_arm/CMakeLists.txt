cmake_minimum_required(VERSION 2.8.3)
project(dual_planar_arm)

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

include_directories(
include ${catkin_INCLUDE_DIRS}
)

add_executable(simulator src/simulator.cpp)
add_executable(move_left_arm src/move_left_arm.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_executable(right_arm_IK_service src/right_arm_IK_service.cpp)
add_executable(track_left_arm src/track_left_arm.cpp)

add_dependencies(simulator ${catkin_EXPORTED_TARGETS})
add_dependencies(tf_broadcaster ${catkin_EXPORTED_TARGETS})
add_dependencies(right_arm_IK_service ${catkin_EXPORTED_TARGETS})
add_dependencies(move_left_arm ${catkin_EXPORTED_TARGETS})
add_dependencies(track_left_arm ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulator
  ${catkin_LIBRARIES}
)
target_link_libraries(move_left_arm
  ${catkin_LIBRARIES}
)
target_link_libraries(tf_broadcaster 
  ${catkin_LIBRARIES}
)
target_link_libraries(right_arm_IK_service 
  ${catkin_LIBRARIES}
)
target_link_libraries(track_left_arm 
  ${catkin_LIBRARIES}
)