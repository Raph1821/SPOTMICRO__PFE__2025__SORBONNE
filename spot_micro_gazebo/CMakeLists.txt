cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  sensor_msgs
  geometry_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/servo_to_gazebo_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
