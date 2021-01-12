cmake_minimum_required(VERSION 2.8.3)
project(kinect_driver)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
)

find_package( OpenCV REQUIRED )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    /home/valencimm/freenect2/include
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
)

find_file(FREENECT_LIB libfreenect2.so /home/valencimm/freenect2/lib)

add_executable( kinect_main src/kinect_main.cpp )

target_link_libraries( kinect_main ${FREENECT_LIB} ${catkin_LIBRARIES} ${OpenCV_LIBS} )