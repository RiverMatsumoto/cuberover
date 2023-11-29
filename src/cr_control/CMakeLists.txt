cmake_minimum_required(VERSION 3.0.2)
project(cr_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  controller_manager_msgs
  joint_state_controller
  urdf
  hardware_interface
  tf
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  wheel_data.msg
  trial_sample.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  vicon_bridge
  cr_control
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES cr_control
   CATKIN_DEPENDS 
   roscpp 
   controller_manager 
   hardware_interface
   message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(
  test_imu
    src/test_imu.cpp
	  src/BNO055.cpp
    include/cr_control/I2C/src/I2C.cpp
)
set_target_properties(test_imu PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(
  test_imu 
    ${catkin_LIBRARIES}
)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(
  wheel_hardware_interface 
    src/main.cpp
    src/wheel_hardware_interface.cpp 
    src/roboclaw.cpp
    src/BNO055.cpp
    include/cr_control/I2C/src/I2C.cpp
)
target_link_libraries(
  wheel_hardware_interface 
    ${catkin_LIBRARIES}
)
set_target_properties(wheel_hardware_interface PROPERTIES LINKER_LANGUAGE CXX)
link_directories(${catkin_LIBRARY_DIRS})
add_dependencies(wheel_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
