# Changelog Eclipse MOSAIC 20.0 (October 2020

[M+] Moved main code to new public repository github-com/eclipse-mosaic
[M+] Changed license to EPL 2.0
[M+] Revised and refactored all public code.
[M+] Significantly improved and extended the documentation, including new tutorials
[M-] Replaced dependencies which are incompatible with EPL.
[M+] Major overhaul of configuration files, e.g.
     * vsimrti/vsimrti_config.xml -> scenario_config.json
     * etc/defaults.xml -> etc/runtime.json
[A+] Mapping configuration has been extended with new features (e.g. typeDistributions, parameter variations).
[A+] New API for traffic light applications
[C+] SNS supports most important Geo-Routing features for ad-hoc multihop communication
[T+] Now supports SUMO 1.7.0

# Changelog VSimRTI 19.1 (October 2019)

[C+] The OMNeT++ federate has been migrated to OMNeT++ 5.5 and INET 4. The federate needs to be rebuild completely.
[S+] The option "--db2vsimrti" has been extended and creates now further default configuration files.
[A+] The collection of example applications has been extended.
[A+] The navigation API for VSimRTI applications has been improved with new methods.
[M+] Configuration of vehicle types now supports the emergencyDecel parameter.
[V+] The visualizers can now be configured to visualize messages within a specific time period only.
[V+] The websocket visualizer now centers the viewport in the browser automatically to the first simulated vehicle.
[S-] The intersection detection in scenario-convert has been fixed.
[V-] Several bugfixes and changes int the command line interface of VSimRTI and scenario-convert.
[V-] The bundled LuST tutorial has been fixed to work again with VSimRTI.
[V-] The performance GUI has been removed.
[V+] Now supports SUMO 1.3.0


# Changelog VSimRTI 19.0 (April 2019)

[V+] Added support for Java 11 and OpenJDK on all operating systems.
[V+] A new statistics visualizer collects and aggregates values from vehicles during the simulation. Find more details in the user manual.
[V+] The web socket visualizer now uses the OpenLayers API instead of Google API.
[V+] The configuration of vehicle spawners in the mapping configuration has been improved (e.g. depart speed, lane selection, scaling traffic).
[A+] A model for Infrastructure to Vehicle Information (IVI) messages has been introduced.
[A-] A bug has been fixed which led to false configuration of ad-hoc modules in application.
[S+] The import of SUMO net and route files into scenario database has been improved.
[S+] The new option "--db2vsimrti" has been added, which generates a simple simulation scenario from a database file.
[V-] The Barnim tutorial scenario has been updated.
[V-] Various performance improvements have been made.
[V+] Now supports SUMO 1.1.0 and 1.2.0

# Changelog VSimRTI 18.1 (October 2018)

[A+] A new simulation entity has been added to simulate Traffic Management Center applications.
[A+] It's now possible to parametrize applications directly in the mapping configuration.
[C+] The Simple Network Simulator (SNS) now supports simple multi hopping.
[C+] Cell2 now models handovers when vehicles move to another region.
[C+] The OMNeT++ federate has been updated to support OMNeT++ 5.3 and INET 3.6
[C+] The NS-3 federate has been updated to support ns-3 3.28
[V+] Improved printing of exceptions to the console output.
[V+] A new 3D visualization tool is now available in the commercial license of VSimRTI.
[S-] The export of roundabouts for SUMO net files has been fixed.
[T+] A vehicle class configured in the mapping configuration is now translated to a suitable SUMO vClass.
[T+] Now supports and requires SUMO 1.0.x

# Changelog VSimRTI 18.0 (April 2018)

[T+] The integration of pre-existing SUMO scenarios has been improved. However, minor limitations exist.
[S+] It is now possible to create a scenario database from any given SUMO network file.
[S-] The scenario database scheme has been changed slightly over the previous releases. Old databases are now unsupported. See conversion guide for details.
[V+] The Luxembourg SUMO Traffic (LuST) scenario [1] has been integrated into VSimRTI . See user manual for details.
[S-] Roundabouts from OSM data is now imported resulting in correct right of way behavior at roundabouts.
[C+] Various improvements regarding Mobile Edge Computing (MEC) in the Cell2 simulator.
[V+] The GeoTools library has been upgraded to its latest version.
[V+] VSimRTI supports Java 9 and 10 on Linux based systems. On Windows, Java 8 is still required.
[V-] Various performance improvements in the core of VSimRTI have been made.
[T+] Now supports SUMO 0.32.0

[1] https://github.com/lcodeca/LuSTScenario

# Changelog VSimRTI 17.1 (October 2017)

[V-] Fixed a bug which would not preserve the order of events in rare cases.
[A+] Ids for messages are now unique per unit, not globally.
[A+] Introduced a simplified API for sending V2X messages from applications.
[C+] It's now possible to define regions as polygons for the Cell2 simulator.
[C+] The Cell2 simulator respects the maxmimum bandwidth of each vehicle.
[C+] The OMneT++ and NS-3 federates now require protobuf3.
[C+] The NS-3 federate experienced a major code cleanup.
[B-] Fixed a bug in the calculation of air drag in the battery simulator.
[T-] The SUMO TraCI Client code has been reimplemented and is now more robust.
[T+] Now supports SUMO 0.31.0

# Changelog VSimRTI 17.0 (April 2017)

[V+] VSimRTI supports and requires Java Runtime Environment (or JDK) Version 8.
[V+] Now supports elevation data for nodes.
[V+] The file visualizer optionally compresses its output file.
[A+] Applications are provided with more information about the road the vehicle is driving on. See conversion guide for details.
[C+] The OMNeT++ and ns-3 federate can now be executed inside a Docker container. See user manual for more details.
[C-] The configuration of the Cell2 simulator has been revised. See conversion guide for details.
[S+] Added the option --srtm2db for importing elevation data provided by ASC files (experimental feature).
[T+] The slope of a vehicle is read out from SUMO via TraCI (requires SUMO > 0.27.0)
[T-] Fixed a bug in SUMO ambassador where the vehicle signals have been read out incorrectly.
[V-] Support of JiST/SWANS has been removed due to technical reasons.
[A-] Fixed a bug in the application simulator where applications received events before they had been set up.
[A-] Removed unused parameter BehaviorDataStruct from application API. See conversion guide for details.
[T+] Now support SUMO 0.29.0

# Changelog VSimRTI 0.16.2 (October 2016)

[V+] Added a new tutorial to the User Documentation regarding mapping of traffic lights.
[V+] VSimRTI now provides a global random number generator whose seed can be set in vsimrti_config.xml
[A+] Reworked the Application API for configuring the AdHoc and Cell modules. Please read the Conversion Guide.
[T+] Improved performance of SUMO coupling.
[T+] Configure SUMO specific parameters for the vehicle type, such as emissionClass or carFollowModel 
[T+] Fuel consumption is now read out from vehicles.
[T-] Bugfix in SUMO ambassador regarding Change Speed.
[A-] Fixed a bug in the application simulator which did not properly simulate traffic light applications.
[V-] Fixed a bug which occurred when the distance sensor was activated and the web visualizer was used at the same time.
[S+] Added the option --db2shp to scenario-convert which provides a conversion of the database to shapefile format.
[T+] Now support SUMO 0.27.1

# Changelog VSimRTI 0.16.1 (June 2016)

[A+] Application API allows to change vehicle parameters, such as minimum gap or maximum speed, during the simulation
[C+] Revised installer scripts for network simulators OMNeT++ and ns-3
[C-] Bugfix in coupling of network simulators OMNet++ and ns-3
[T+] SUMO coupling now provides distance sensor information for vehicles (opt-in) and the longitudal acceleration
[T+] Now support SUMO 0.26.0

# Changelog VSimRTI 0.16.0 (April 2016)

[V+] Improved User Documentation with new detailed Tutorials of Tiergarten and Barnim, and extended API-Config-Doxygen
[A-] Application API Refactoring to allow more flexible selection of implemented functionalities (CommunicationApplication, ElectricVehicleApplication, ...)
[A+] Introduction of new Central Navigation Component to Application to provide advanced navigation functionalities for the applications (e.g. calculate new routes with own cost functions)
[A+] New interactions with traffic simulator (mainly SUMO) for ChangeSpeed
[C-] Improved stability of coupling of network simulators OMNeT++ and ns-3 (based on and needs Google Protocol Buffers)
[C+] Now support latest ns-3.25
[C+] Introduced new built-in Simple Network Simulator (SNS), supporting various ad hoc communication modes, for quick and easy usage without further installations
[S-] Bugfix in scenario-convert regarding SUMO rou-file import in mode --sumo2db
[T-] Revised SUMO interfaces for compatibility with SUMO 0.25.0 (regarding headings)
[T+] Support visualization and control of simulations directly from SUMO-GUI

Please note, this version only delivers correct results (regarding headings) with SUMO 0.25.0 or higher

# Changelog VSimRTI 0.15.1 (December 2015)

[V+] A new CLI argument (--scenario) is available for vsimrti.bat/sh which lets the user pass the name of scenario instead of the full path to vsimrti_config.xml
[A+] Support new addressing handling of tcp-like message Acknowledgment (for cellular communication)
[A+] Enable/disable WLANmodule for ad hoc communication from applications
[A+] Support configurable IP Address Resolver
[B+] New configuration for the Battery Ambassador
[C+] OMNeT++: support OMNeT++ 4.6 and INET 3.0 now
[C-] Improved minor bugs in Cellular Simulator
[C+] Support new schemes for geographic addressing over cellular (include MBMS/eMBMS feature)
[N+] Major improvements in navigation simulator, which now uses GraphHopper for route calculation (e.g. routing now considers turn costs and turn restrictions).
[S+] Several improvements in scenario-convert (now supports start with json-config).
[T-] Fixed a bug which let SUMO crash when the navigation simulator calculated an invalid route.
[T+] Now support SUMO 0.25.0


# Changelog VSimRTI 0.15.0 (September 2015)

[INTERNAL] new versioning system: every module of the aggregate has now the same version

[V-] Several minor bugfixes for the general framework
[A+] Slight improvements of interfaces of VSimRTI_App
[B+] Extended modeling in the Battery simulator (improved handling and switching of discharging/charging)
[C+] New generation of the VSimRTI_Cell cellular simulator with additional features (separated up/downlink, improved region definition with kml-visualization)
[C-] Stability improvements for ns-3 and OMNeT++ (now support OMNeT++ 4.4.1)
[T+] Enhanced SUMO integration regarding navigation interactions (ChangeRoute, ChangeTarget), now support SUMO 0.24.0
Please note, this version only works with sumo v0.21.0 or higher


# Changelog VSimRTI 0.14.0

[V+] General performance improvements
[V+] Improved SimRunner configuration
[A+] Completely Revised Application simulator with new features and improved performance
[C+] Additional Features for cellular simulator VSimRTI_Cell
[C-] Stability Improvements for ns-3 and OMNeT++
Please note, this version only works with sumo v0.21.0 or higher

Changelog (Features and Bugfixes) Legend:
[M] MOSAIC [V] VSimRTI [A] Application simulator [B] Battery simulator [C] Communication simulator [E] Environment simulator [N] Navigation component [S] Scenario-convert [T] Traffic simulator [+/-] new Feature/Bugfix