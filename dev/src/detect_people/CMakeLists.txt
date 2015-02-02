cmake_minimum_required(VERSION 2.8.3)
project(detect_people)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
)
find_package(PCL 1.7 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  .
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(ground_based_rgbd_people_detector
  MACOSX_BUNDLE
  main_ground_based_people_detection.cpp
)

target_link_libraries(ground_based_rgbd_people_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

