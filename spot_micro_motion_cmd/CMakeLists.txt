cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_motion_cmd)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  message_generation
  tf2
  tf2_ros
  tf2_geometry_msgs
  i2cpwm_board
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs std_srvs message_runtime tf2 tf2_ros tf2_geometry_msgs i2cpwm_board
  DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  libs/spot_micro_kinematics_cpp
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node 
  src/spot_micro_motion_cmd_node.cpp
  src/spot_micro_motion_cmd.cpp
  src/utils.cpp
  src/smfsm/spot_micro_idle.cpp
  src/smfsm/spot_micro_stand.cpp
  src/smfsm/spot_micro_state.cpp
  src/smfsm/spot_micro_transition_idle.cpp
  src/smfsm/spot_micro_transition_stand.cpp
  src/smfsm/spot_micro_walk.cpp
  libs/spot_micro_kinematics_cpp/spot_micro_kinematics.cpp
  libs/spot_micro_kinematics_cpp/spot_micro_leg.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and config files)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)