cmake_minimum_required(VERSION 2.8.3)
project(fast_klt_vo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
   
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})


add_executable(fast_klt_main src/fast_klt_main.cpp)
target_link_libraries(fast_klt_main ${catkin_LIBRARIES})
target_link_libraries(fast_klt_main ${OpenCV_LIBRARIES})
