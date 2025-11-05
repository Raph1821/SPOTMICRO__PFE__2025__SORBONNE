cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_rviz)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  urdf
  xacro
  joint_state_publisher
  std_msgs
  geometry_msgs
  message_generation
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
  CATKIN_DEPENDS rospy roscpp urdf xacro joint_state_publisher std_msgs geometry_msgs message_runtime
)

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)