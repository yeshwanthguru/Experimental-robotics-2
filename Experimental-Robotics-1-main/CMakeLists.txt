cmake_minimum_required(VERSION 3.0.2)
project(expo_assignment_1)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Point.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetPose.srv
  GetPose.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Control.action
  Plan.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation message_runtime rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
install(PROGRAMS
  scripts/robot_state_machine.py
  scripts/robot_brain.py
  scripts/robot_brain2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)

