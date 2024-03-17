cmake_minimum_required(VERSION 3.0.2)
project(cr_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
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

find_package(JetsonGPIO)

add_message_files(
  FILES
  wheel_data.msg
  trial_sample.msg
)

add_service_files(
  FILES
  LightState.srv
  ServoState.srv
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
   CATKIN_DEPENDS 
   roscpp 
   controller_manager 
   hardware_interface
   message_runtime
)

###########
## Build ##
###########
include_directories(include 
  ${catkin_INCLUDE_DIRS}
  include/cr_control/I2C/src
)

add_executable(test_imu
  src/test_imu.cpp
  src/BNO055.cpp
  include/cr_control/I2C/src/I2C.cpp
)
set_target_properties(test_imu PROPERTIES 
    LINKER_LANGUAGE CXX
)
target_link_libraries(test_imu ${catkin_LIBRARIES})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(wheel_hardware_interface 
  src/main.cpp
  src/wheel_hardware_interface.cpp 
  src/linear_actuator.cpp
  src/roboclaw.cpp
  src/BNO055.cpp
  include/cr_control/I2C/src/I2C.cpp
)
target_link_libraries(wheel_hardware_interface ${catkin_LIBRARIES})
set_target_properties(wheel_hardware_interface PROPERTIES 
    LINKER_LANGUAGE CXX
)
link_directories(${catkin_LIBRARY_DIRS})
add_dependencies(wheel_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(light_node
  src/light.cpp
)
target_link_libraries(light_node 
  ${catkin_LIBRARIES}
  JetsonGPIO::JetsonGPIO
)
link_directories(${catkin_LIBRARY_DIRS})
add_dependencies(light_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(servo_node
  src/servo.cpp
)
target_link_libraries(servo_node
  ${catkin_LIBRARIES}
  JetsonGPIO::JetsonGPIO
)
link_directories(${catkin_LIBRARY_DIRS})
add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS})