cmake_minimum_required(VERSION 3.0.2)
project(ardusub_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArdusubStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs mavros_msgs tf message_runtime
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  ardusub_ros_node.py
  ardusub_controller.py
  control_ardusub.py
  udp_connect.py
  connect_main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
