cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_rviz)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  urdf
  xacro
  joint_state_publisher
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp urdf xacro joint_state_publisher
)

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)