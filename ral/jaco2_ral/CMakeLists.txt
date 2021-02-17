cmake_minimum_required(VERSION 2.8.3)
project(jaco2_ral)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kinova_driver
  kinova_msgs
  std_msgs
  geometry_msgs
  roscpp
  rospy
  jaco2_driver
  util
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  kinova_driver
  kinova_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${jaco2_driver_INCLUDE_DIRS}
  ${util_INCLUDE_DIRS}
)


add_executable( jaco2_ral_joints src/jaco2_ral_joints.cpp )

target_link_libraries( jaco2_ral_joints ${catkin_LIBRARIES} ${jaco2_driver_LIBS} ${util_LIBS} )

install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/jaco2_ral/joints_demo.py
  nodes/jaco2_ral/pose_demo.py
  jaco2_ral_joints_demo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/jaco2_left_joints.launch
  launch/jaco2_right_joints.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)