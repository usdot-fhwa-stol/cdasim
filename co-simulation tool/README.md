

# Carla-Sumo-Mosaic Co-Simulation Tool

This co-simulation tool is used to run [CARLA simulator](http://carla.org/) and [SUMO simulator](https://www.eclipse.org/sumo/) with [Eclipse MOSAIC framework](https://github.com/eclipse/mosaic). The co-simulation tool can simulate both co-simulation scenarios and original Eclipse MOSAIC scenarios on both Windows systems and Linux systems.  

## Installation of Co-simulation Tool

**a.**   **Executable bundle file**

Download the CARLA-SUMO-MOSAIC-1.0.zip bundle and extract it to an arbitrary path. 

**b.**   **Installation from source code**

Download the co-simulation tool code and refer to the Eclipse-MOSAIC build instruction in this [link]( https://github.com/eclipse/mosaic) .

***Note:*** **The source code is in the folder of co-simulation tool rather then the root folder.** To build it, users should first go to the folder of co-simulation tool and then build it.

The installation path is referenced as <cosimulation-root>. After the co-simulation tool is installed from source code, the zipped bridge folder needs to be extracted to <cosimulation-root>.

## **Folder Content**

The folder structure of co-simulation tool is described in the following.

```
└─ <cosimulation-root>
  ├─ bridge ................ Directory with bridge file for connecting CARLA ambassador and CARLA simulator.
  |  ├─ bridge.bat ......... An example of bridge script for a scenario on Windows systems.
  |  ├─ bridge.sh .......... An example of bridge Script for a scenario on  Linux systems.      
  ├─ etc
  |  ├─ hosts.json ......... Configuration of the execution host, e.g. temporary directory.
  |  ├─ logback.xml ........ Configuration of log files and levels.
  |  └─ runtime.json ....... Configuration of all Ambassadors and Federates coupled with the MOSAIC
  ├─ lib ................... Directory with all Java compiled libraries required for MOSAIC.
  ├─ logs .................. Directory with log files.
  ├─ scenarios ............. Directory containing all simulation scenarios.
  ├─ tools ................. Additional tools, like the HTML Visualizer.
  ├─ CONTRIBUTING.md 
  ├─ LICENSE 
  ├─ mosaic.bat ............ Start script for Windows systems.
  └─ mosaic.sh ............. Start script for GNU/Linux systems.
```

##  **Prerequisites**

For a successful running of the co-simulation tool, the following software must be installed:

-  CARLA simulator 0.9.10
-  JAVA runtime environment (JRE) - JAVA 8 or 11 
-  Python 3.7 - The default python version must be python 3.7 and the package `lxml==4.5.0` needs to be installed using `pip install lxml==4.5.0`. 
-  SUMO 1.8.0 or new - The environment variable **`SUMO_HOME`** **must be configured to define the installation directory of SUMO**.

Moreover, a high performance computer is needed to run the co-simulation tool. The configuration of a recommended system is in the following.

- Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD ryzen 7 / AMD ryzen 9
- +16 GB RAM memory
- NVIDIA RTX 2070 / NVIDIA RTX 2080 / NVIDIA RTX 3070, NVIDIA RTX 3080

## **Co-Simulation Scenario**

To run the co-simulation tool, you need to prepare a co-simulation scenario. The following file structure shows the setup of a typical co-simulation scenario.

```
└─ <scenarioName>
 ├─ application
   | └─ <scenarioName>.db................ Scenario database file
 ├─ carla
 | └─ carla_config.json ................. CARLA configuration file
 | └─ bridge.sh/bridge.bat .............. Bridge file
 ├─ mapping
 | └─ mapping_config.json ............... Mapping configuration file
 ├─ sumo
 | └─ <scenarioName>.net.xml ............ SUMO network file
 | └─ <scenarioName>.rou.xml ............ SUMO route file
 | └─ <scenarioName>.sumocfg ............ SUMO configuration file
 └─ scenario_config.json ................ Basic configuration of the simulation scenario
```

Here the scenario configuration file , the CARLA configuration file and bridge file are discussed. For other configuration files and scenario creation, refer to the documentation of Eclipse MOSAIC [[link](https://www.eclipse.org/mosaic/docs/)]. Moreover, the  CARLA 3D maps corresponding to the scenarios must be generated for co-simulation scenarios.

**a.**   **Scenario configuration file**

The scenario_config.json is the main configuration file of a co-simulation scenario. A co-simulation scenario file looks like the following example.

```json
{
  "simulation": {
   "id": "town04",
   "duration": "200s",
   "randomSeed": 212323853,
   "projection": {
      "centerCoordinates": {
        	"latitude": 52.63,
        	"longitude": 13.56
      },
      "cartesianOffset": {
       		"x": -563984.16,
      	 	"y": -5933566.87
     }
    },
   "network": {
    	"netMask": "255.255.0.0",
    	"vehicleNet": "10.1.0.0",
    	"rsuNet": "10.2.0.0",
    	"tlNet": "10.3.0.0",
   	 	"csNet": "10.4.0.0",
    	"serverNet": "10.5.0.0",
    	"tmcNet": "10.6.0.0"
    }
  },
  "federates": {
  		"application": true,
  		"cell": false,
  		"environment": false,
  		"sns": false,
  		"ns3": false,
  		"omnetpp": false,
  		"output": true,
  		"carla": true,
  		"sumo": true
  }
}
```

The federate tags show which simulators are  used in the simulation. For the co-simulation scenarios, CARLA and SUMO federate must be enabled. If the CARLA federate is disabled, the simulation of SUMO and MOSAIC will be run.

**b.**   **Carla configuration file**

The CARLA ambassador can be configured with the carla configuration file. The specific path is <scenarioName>/carla/carla_config.json.  The example configuration files of CARLA ambassador on Linux systems and on Windows systems are shown in the following, respectively.

**Linux**

```json
{
    "updateInterval": 100,
	"carlaUE4Path": "/CARLA_0.9.10/",
	"bridgePath": "./scenarios/Town04_10/carla; bridge.sh",
	"carlaConnectionPort": 8913
}
```

**Windows**

```json
{
    "updateInterval": 100,
	"carlaUE4Path": "D:/CARLA_0.9.10/",
	"bridgePath": "./scenarios/Town04_10/carla; bridge.bat",
	"carlaConnectionPort": 8913
}
```

The following table descripts the parameters in the CARLA configuration file.

| Parameters          | **Type** | **Description**                                              | **Required** |
| ------------------- | -------- | ------------------------------------------------------------ | ------------ |
| updateInterval      | number   | The Interval after which the CARLA is updated.  The default unit is ms. Define the size  of one simulation step in CARLA. The updateInterval of Carla and Sumo  ambassador must be the same. | Yes          |
| carlaUE4Path        | string   | Path to CarlaUE4 executable file. If the environment  variable **`CARLA_HOME`** is configured to define  the installation directory of CarlaUE4 executable file, this parameter can be removed from  the configuration file. | No           |
| bridgePath          | string   | This parameter includes two information separated by the  semicolon. The first substring is the path of bridge .sh or .bat. The second  substring is the file name of bridge .sh or .bat. The extension of the bridge file is needed. | Yes          |
| carlaConnectionPort | number   | The bridge server port . It must be the same as  the bridge server port number in the bridge .sh or .bat file.  If the entry is not used, the default port number is 8913. | No           |

**c.**  **Bridge file**

The bridge file (bridge.sh or bridge.bat) is used to run the bridge to connect CARLA simulator and CARLA ambassador, load maps to CARLA simulator, pass messages between CARLA ambassador and CARLA simulator.  The TraCI protocol and Python API of CARLA simulator are used. The examples of  bridge files on Linux systems and on Windows systems are shown in the following, respectively.

**Linux**

```bash
#!/bin/bash

#x-terminal-emulator
cd ../../../bridge

python3 carla_mosaic_bridge.py --bridge-server-port 8913 --map Town04 net/Town04.net.xml
```

 **Windows**

```powershell
@ECHO OFF
cd ../../../bridge
python carla_mosaic_bridge.py --bridge-server-port 8913 --map Town04 net/Town04.net.xml
```

***Note:*** The default python version should be 3.7. Check what command is used to run the python script on the local machine or in a docker container. Some systems may use ***python*** to run the python scripts, others may use ***python3*** or ***python3.7*** to run the python scripts. 

The arguments in the above python commands are shown in the following.

**`--bridge-server-port 8913`:** The port number that the bridge server is listening to. It must be the same as the carlaConnectionPort in the carla configuration file. If not specified, the default port number 8913 will be used for the bridge server.

**`--map Tow04`**: the map loaded in the CARLA simulator. If not specified, the default map of CARLA simulator will be used.

**`net/Town04.net.xml`**: the net file of the CARLA map or SUMO, the net file is located in the net folder under the bridge folder. Since SUMO and CARLA simulator use different reference systems,  the SUMO net offset calculated from the net file is used to calculate the transform of SUMO actors in CARLA simulator and the transform of CARLA actors in SUMO. If not specified, the net file will be generated using the CARLA map.

## **Run a Co-simulation Scenario**

To run a co-simulation scenario via Command Line Interface, call the start script with the following command line arguments. For more CLI options, please refer to Eclipse MOSAIC documentation. After CARLA loads map and SUMO GUI pops out, click the play button in SUMO GUI to start running co-simulation. 

**GNU/Linux**

```
./mosaic.sh -s <scenario-name>
./mosaic.sh -c ./scenarios/<scenario_name>/scenario_config.json
```

**Windows**

```
mosaic.bat -s <scenario-name>
mosaic.bat -c .\scenarios\<scenario_name>\scenario_config.json
```

**Example**

The following call starts the co-simulation scenario “Tow04” on a Windows Machine.

```
mosaic.bat -s Town04
```

 ![co-simulation demo](https://github.com/usdot-fhwa-stol/carma-simulation/blob/feature/mosaic-carla-sumo-ambassadors/co-simulation%20tool/co-simulation%20demo.gif)

## **Check the Log Files**

Log files are generated for the used ambassadors for each simulation run. They are stored in the folder `<cosimulation-root>/logs/log-<timestamp>-scenarioname`. Each simulation run has a  new log file folder generated. The folder structure of log file folder looks like the following. The log file Carla.log is about CARLA ambassador. 

```
└─ log-<timestamp>-secarioname
  ├─ Application.log ................ Information about the application ambassador
  ├─ Carla.log ...................... CARLA ambssador log     
  ├─ Cell.log ....................... Cellular network log
  ├─ Communication.log ........ ..... (Ad hoc) network simulation ambassador log
  ├─ Environment.log ................ Logs of the environmental eventserver
  ├─ Mapping.log .................... Mapping configuration logs
  ├─ MOSAIC.log ..................... General information, e.g. startup sequence information
  ├─ Navigation.log ................. Detailed logs about navigation component in the application ambassador
  ├─ output.csv ..................... Recorded data of the integrated File Output Generator
  ├─ RunTimeEvents.csv .............. Logs of run time events
  ├─ Traffic.log .................... Traffic simulation log (SUMO)
```

##  Troubleshooting

If the maps of CARLA for the scenarios is the same as the default map of CARLA simulator, the argument about map in the bridge file is recommended to be removed. Loading a map in CARLA simulator takes time and sometimes may break the connection of CARLA client. The users have to restart the co-simulation to solve the connection issue.

Sometimes, the CARLA client can not connect to CARLA simulator. Check whether there is a CARLA process running in the background or not. If so, terminate the CARLA process.

 

 