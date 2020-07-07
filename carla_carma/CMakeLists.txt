cmake_minimum_required(VERSION 2.8.3)
project(carla_carma)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  carla_msgs
  roslaunch
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    ExternalObject.msg
    ExternalObjectList.msg
    CarlaEgoVehicleControl.msg
    CarmaInit.msg
    VehicleStatus.msg
    CarlaEnabled.msg
    RobotEnabled.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
    DEPENDENCIES
    std_msgs
    carla_msgs
)

roslaunch_add_file_check(launch)

catkin_package()

install(PROGRAMS
  src/carla_carma/CARMA_client_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
