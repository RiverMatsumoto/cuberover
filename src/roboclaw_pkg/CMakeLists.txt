cmake_minimum_required(VERSION 3.0.2)
project(roboclaw_pkg)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  joint_state_controller
  hardware_interface
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
)

include_directories(include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(roboclaw_test
  src/test_roboclaw.cpp
  src/roboclaw.cpp
  src/simple_uart.c
)
target_link_libraries(roboclaw_test
  ${catkin_LIBRARIES}
)
link_directories(${catkin_LIBRARY_DIRS})