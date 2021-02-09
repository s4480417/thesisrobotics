cmake_minimum_required(VERSION 2.8.3)
project(usr_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  OpenPosePosition.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

