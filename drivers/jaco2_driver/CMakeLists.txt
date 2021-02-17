cmake_minimum_required(VERSION 2.8.3)
project(jaco2_driver)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kinova_driver
  kinova_msgs
  std_msgs
  geometry_msgs
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
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
)

add_library( ${PROJECT_NAME} src/jaco2_driver.cpp )

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  launch/kinova_robot.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/jaco2_driver/fingers_action_client.py
  nodes/jaco2_driver/joints_action_client.py
  nodes/jaco2_driver/pose_action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
