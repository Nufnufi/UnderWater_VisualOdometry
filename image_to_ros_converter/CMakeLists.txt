cmake_minimum_required(VERSION 2.8.3)
project(image_to_ros_converter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})

add_executable(image_converter_ros src/image_converter_ros.cpp)
target_link_libraries(image_converter_ros ${catkin_LIBRARIES})
target_link_libraries(image_converter_ros ${OpenCV_LIBRARIES})
