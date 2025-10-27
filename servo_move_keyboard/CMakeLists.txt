cmake_minimum_required(VERSION 3.0.2)
project(servo_move_keyboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  i2cpwm_board
)

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