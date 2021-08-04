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
    src/carla_carma_bridge/carla_to_autoware_localization
    src/carla_carma_bridge/carla_to_autoware_vehicle_status
    src/carla_carma_bridge/odometry_to_posestamped
    src/carla_carma_bridge/vehiclecmd_to_ackermanndrive
    src/carla_carma_bridge/carla_to_carma_external_objects
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

