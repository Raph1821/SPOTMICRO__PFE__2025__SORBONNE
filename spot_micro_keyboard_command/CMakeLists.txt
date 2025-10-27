cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_keyboard_command)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  std_srvs
  spot_micro_motion_cmd
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs std_srvs spot_micro_motion_cmd
)

catkin_install_python(PROGRAMS scripts/spotMicroKeyboardMove.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)