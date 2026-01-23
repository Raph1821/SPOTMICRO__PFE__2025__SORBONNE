cmake_minimum_required(VERSION 3.0.2)
project(spot_micro_rl)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/spot_rl_train_ars.py
  scripts/spot_rl_train_sac.py
  scripts/spot_rl_train_td3.py
  scripts/spot_rl_train_sac_parallel.py
  scripts/spot_rl_train_td3_parallel.py
  scripts/spot_rl_eval.py
  scripts/spot_rl_deploy.py
  scripts/gmbc_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
