# Copyright (c) 2021 Intel Corporation
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

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
    src/carla_carma_bridge/carla_to_carma_driver_status
    src/carla_carma_bridge/carla_to_carma_robot_status
    src/carla_carma_bridge/carla_to_carma_vehicle_status
    src/carla_carma_bridge/carla_to_carma_localization
    src/carla_carma_bridge/carma_to_carla_ackermann_cmd
    src/carla_carma_bridge/carla_to_carma_external_objects
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
