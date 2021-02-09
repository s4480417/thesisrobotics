cmake_minimum_required(VERSION 2.8.3)
project(viz_ral)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    include
    $ENV{HOME}/freenect2/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

find_file(FREENECT_LIB libfreenect2.so $ENV{HOME}/freenect2/lib)
add_executable( viz_ral_main src/viz_ral_main.cpp )

target_link_libraries( viz_ral_main ${FREENECT_LIB} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${kin_driver_LIBS} )
