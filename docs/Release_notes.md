# CARMA Simulation Release Notes

## February 10th, 2023 - Version 1.1.0
### <ins>CARMA Simulation 1.1.0 </ins>
The CARMA XiL co-simulation tool is upgraded to support ROS2 versions of the CARMA Platform (4.2.0).

Enhancements in this release:
- Updated CARMA-CARLA Ros bridge to support ROS2 versions of the CARMA Platform (4.2.0).

Known Issues in this release:
- CARMA Platform issues were observed during simulation testing, please refer to CARMA Platform issues [#2034](https://github.com/usdot-fhwa-stol/carma-platform/issues/2034) and [#2035](https://github.com/usdot-fhwa-stol/carma-platform/issues/2035)

## October 4th, 2022 - Version 1.0.1

### <ins>CARMA Simulation 1.0.1</ins>
carma-simulation release version 1.0.1 is a hotfix to correct an artifacts problem in the build environment for 1.0.0. Also, added the dockerization feature for Docker image build process.

#### Features:
- Added carma-simulation dockerization feature which updates Docker configuration to be similar to how the rest of the CARMA product ecosystem utilizes Docker for builds and configuration management.

#### Fixes:
- Resolve error while processing the POMs during the mvn clean
- Resolve mosaic.sh file not show up in the zip file after mvn build complete

# Full list of PRs included in this release:
* Feature/dockerization update by @kjrush in https://github.com/usdot-fhwa-stol/carma-simulation/pull/83
* Fix/simulation_scenario by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/84
* Fix/docker sumo version by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/85
* Feature/dockerization upload scenario by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/86
* hotfix/build_environment by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/87

## August 10th, 2022 - Version 1.0.0
Release version 1.0.0 of CARMA Simulation products. Initital release intended
to deliver minimum-viable-proudct integration of several major simulation
components: CARMA Platform, MOSAIC, SUMO, CARLA.

### <ins>CARMA Simulation 1.0.0</ins>
Initial full release. Minimum viable product version of CARMA Simulation to act as a base for additional feature development and to allow prospective users to begin integration and early use of tool.

#### Features:
- Incorporate Eclipse MOSAIC Co-simulation framework including MOSAIC Runtime
  Infrastructure (RTI), SUMO, and NS-3.
- Integrate CARLA with MOSAIC RTI.
  - Vehicle synchronization between CARLA and SUMO
  - Traffic signal synchronization between CARLA and SUMO
  - Simulation map synchronization between CARLA and SUMO
- Dockerized setup of MOSAIC and simulators including CARLA
- Configure data output and logging to record data for verifying synchronization
- Integrate new Turner-Fairbank Highway Research Center simulation map with SUMO
  and CARLA
- Develop initial build and deployment documentation for CARMA Simulation 1.0.0

#### Fixes:
- Resolve MOSAIC RTI priority queue conflicts which caused simulation deadlock

### <ins>CARMA-CARLA Integration 1.0.0</ins>
Initial full release. Minimum viable product version of CARMA-CARLA Integration packages to act as a base for additional feature development and to allow prospective users to begin integration and early use of tool.

#### Features:
- Support for registration of CARMA Platform vehicles with CARLA ego-vehicle
  roles
- Integration of CARMA Platform controller plugins with CARLA control inputs
  via Ackermann Control node
- Integration of CARMA Platform localization with "ground truth" position of
  vehicle in CARLA
- Integration of CARMA Platform perception with "ground truth" object list of
  neighboring vehicles and obstacles
- Dockerized setup of CARMA Platform integration nodes

#### Fixes:
- Resolve data rate consistency issues with CARMA-CARLA bridge



# Full list of PRs included in this release:
* Feature/ci by @jtbaird in https://github.com/usdot-fhwa-stol/carma-simulation/pull/2
* Feature/carla ackermann control wrapper by @fangzhou1227 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/1
* add launch files for carma-simulation by @fangzhou1227 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/3
* Make CI use dev images by @msmcconnell in https://github.com/usdot-fhwa-stol/carma-simulation/pull/4
* Fix file not found error during cmake_install by @fangzhou1227 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/5
* Updated package.xml files by @jsun6065 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/6
* added release flag by @jtbaird in https://github.com/usdot-fhwa-stol/carma-simulation/pull/7
* Fix/camke install error by @fangzhou1227 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/9
* remove java by @MishkaMN in https://github.com/usdot-fhwa-stol/carma-simulation/pull/10
* Release/wanderer by @jtbaird in https://github.com/usdot-fhwa-stol/carma-simulation/pull/11
* Feature/building with colcon by @jsun6065 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/8
* Feature/sync 3.4.3 by @msmcconnell in https://github.com/usdot-fhwa-stol/carma-simulation/pull/12
* Fix build image .sh for dev and candidate images by @msmcconnell in https://github.com/usdot-fhwa-stol/carma-simulation/pull/13
* Feature/repo cleanup for xil by @kjrush in https://github.com/usdot-fhwa-stol/carma-simulation/pull/15
* upload script of creating corresponding CARLA virtual vehicle by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/18
* Create carma_to_carla_ackermann_cmd by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/20
* Feature/mosaic carla sumo ambassadors by @kjrush in https://github.com/usdot-fhwa-stol/carma-simulation/pull/17
* Feature/carla carma agent by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/22
* Create carla_to_carma_localization by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/21
* Fix the path errors in the run.sh. by @DFCui in https://github.com/usdot-fhwa-stol/carma-simulation/pull/36
* Fix CARLA CARMA bridge scripts by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/33
* Fix/carla carma agent remove autoware by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/38
* carla carma integration docker by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/37
* CARLA-CARMA integration tool user instruction README.md by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/34
* Create carla_to_carma_external_objects by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/23
* Create carla_to_carma_twist by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/19
* Fix/carla carma integration build and launch issue by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/40
* Update/co simulation tool docker README by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/41
* Correct wrong image name by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/42
* Adding method to calculate velocity twist for external object function. by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/48
* Update/driver status and robot status by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/45
* Fix/carla sumo traffic light sync by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/53
* Added environment setting in carla-carma-integration README.md file by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/52
* Fix/incontinuous data publishing by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/51
* Fix/soruce name not matching by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/56
* Fix/driver manager issue by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/58
* carla_mosaic_update by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/70
* Update/carla mosaic bridge by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/71
* update co-simulation docker and instruction by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/72
* Feature/merge 1.0 to master by @chengyuan0124 in https://github.com/usdot-fhwa-stol/carma-simulation/pull/74

## New Contributors
* @jtbaird made their first contribution in https://github.com/usdot-fhwa-stol/carma-simulation/pull/2
* @fangzhou1227 made their first contribution in https://github.com/usdot-fhwa-stol/carma-simulation/pull/1
* @msmcconnell made their first contribution in https://github.com/usdot-fhwa-stol/carma-simulation/pull/4
* @jsun6065 made their first contribution in https://github.com/usdot-fhwa-stol/carma-simulation/pull/6
* @MishkaMN made their first contribution in https://github.com/usdot-fhwa-stol/carma-simulation/pull/10
* @DFCui made their first contribution in https://github.com/usdot-fhwa-stol/carma-simulation/pull/36

**Full Changelog**: https://github.com/usdot-fhwa-stol/carma-simulation/commits/carma-simulation-1.0.0
