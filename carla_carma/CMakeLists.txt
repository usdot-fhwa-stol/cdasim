cmake_minimum_required(VERSION 2.8.3)
project(carla_carma)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  carla_msgs
  autoware_msgs
  cav_msgs
  roslaunch
  rosgraph_msgs
)

catkin_python_setup()

roslaunch_add_file_check(launch)

catkin_package()

install(PROGRAMS
  src/carla_carma/CARMA_client_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
