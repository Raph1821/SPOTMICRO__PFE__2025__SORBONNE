cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_pybullet)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  i2cpwm_board
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/spot_micro_pybullet_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY Parts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Parts
)

install(DIRECTORY STL/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/STL
)