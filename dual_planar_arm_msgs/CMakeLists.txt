cmake_minimum_required(VERSION 2.8.3)
project(dual_planar_arm_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy sensor_msgs geometry_msgs message_generation
)

add_service_files(
   FILES
   DualArmIK.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs geometry_msgs
)

catkin_package(

CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)