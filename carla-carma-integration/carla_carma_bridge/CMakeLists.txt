cmake_minimum_required(VERSION 2.8.3)
project(carla_carma_bridge)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  rospy
  cav_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

