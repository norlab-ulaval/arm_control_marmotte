cmake_minimum_required(VERSION 3.0.2)
project(arm_control_marmotte)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kortex_driver
  roscpp
  rospy
  dynamic_reconfigure
  message_generation
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/speed_ratios.cfg
)

add_service_files(
  FILES
  sendRobotToPosition.srv
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS kortex_driver roscpp rospy dynamic_reconfigure
)

include_directories(
  srv
  cfg
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/arm_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )