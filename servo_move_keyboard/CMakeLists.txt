cmake_minimum_required(VERSION 3.0.2)
project(servo_move_keyboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  i2cpwm_board
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MotionCmd.srv
  PoseCmd.srv 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy i2cpwm_board
)

catkin_install_python(PROGRAMS 
  scripts/servoMoveKeyboard.py
  scripts/servoConfigTest.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)