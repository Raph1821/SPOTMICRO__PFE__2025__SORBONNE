cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_autonomous_slam)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf2
  tf2_ros
  costmap_2d
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    rospy 
    roscpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    actionlib 
    actionlib_msgs 
    tf2 
    tf2_ros 
    costmap_2d
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/map_saver.py
  scripts/odom_publisher.py
  scripts/simple_explorer.py
  scripts/path_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

