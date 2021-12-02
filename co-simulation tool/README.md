

# Carla-Sumo-Mosaic Co-Simulation Tool

This user guide intends to provide users instructions on how to install the prerequisite software, configure the system environment variable, and run a Co-simulation tool developed for supporting the cooperative driving automation research on **Linux systems Ubuntu 16.04** and **Windows 10 systems**. This co-simulation tool is developed based on the open-source concept. Specifically, it is based on [Eclipse MOSAIC framework](https://www.eclipse.org/mosaic/) and consists of an open-source traffic simulator, namely [SUMO](https://www.eclipse.org/sumo/), an autonomous driving simulator, namely [CARLA](http://carla.org/), an open-source network simulator, namely [ns-3](https://www.nsnam.org/). The co-simulation tool can simulate V2X communication among vehicles and infrastructure using the ns-3 network simulator.

##  **Prerequisites**

For a successful running of the co-simulation tool, the following software must be installed or updated:

-  CARLA simulator 0.9.10
-  JAVA 8 or 11 - Recommend using the [Adopt OpenJDK](https://adoptium.net/?variant=openjdk8). The environment variable **JAVA_HOME** needs be configured to the location of JDK.
-  Python 3.7 - If the users use different version of Python other than 3.7, the users need to copy the CARLA library .egg file to the folder bridge/ PythonAPI /carla/dist. In addition, the package `lxml==4.5.0` needs to be installed. 
-  SUMO 1.8.0 - The environment variable **`SUMO_HOME`** **must be configured to define the installation directory of SUMO**.
-  ns-3 simulator
-  TraCI library has to be updated.

Moreover, a high performance computer is needed to run the co-simulation tool. The configuration of a recommended system is in the following.

- **CPU**: Intel i7 gen 9th - 11th / Intel i9 gen 9th - 11th / AMD ryzen 7 / AMD ryzen 9
- **RAM memory:** minimum 16GB 
- **Graphics card:** NVIDIA RTX 2070 / RTX 2080 / RTX 3070 / RTX 3080

## Installation of Prerequisites

### 1. Installation of ns-3 3.28 

#### a. Linux system

#####   Installation of dependencies for ns-3

-  Install gcc, g++, python 3.6 and other dependencies

```
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update && sudo apt-get install gcc-7 g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 20 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update && sudo apt install python3.6
sudo apt-get update && sudo apt-get install unzip tar python3.6-dev pkg-config sqlite3 autoconf automake libtool curl make libxml2 libsqlite3-dev libxml2-dev
```

- Install protobuff

```
curl -OL "https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.3.0.tar.gz"
tar  xvf  v3.3.0.tar.gz
cd protobuf-3.3.0
./autogen.sh
 ./configure --prefix=/usr
make
make check
sudo make install
```

##### Installation of ns-3 federate

First go to the root folder of co-simulation tool and then go to **/bin/fed/ns3**, find **ns3_installer.sh** file.

- Run the following command to make ns3_installer.sh executable.  

```
chmod a+x ns3_installer.sh
```

- Open ns3_installer.sh, add the following command before **log "Build ns3-federate"** in function **build_ns3()**.

```
sudo cp -ar ns-3.28/build/ns3 /usr/include/
```

- Run the following command to install ns-3 federate.

```
./ns3_installer.sh
```

- After installation, open run.sh in **/bin/fed/ns3** and change the relative path to the absolute path in     LD_LIBRARY_PATH.

- Set environment variable **NS3_HOME** to the root folder of run.sh.

  ```
  export NS3_HOME=path_to_run.sh
  ```

#### b. Windows system

Since ns-3 is primarily developed on and for GNU/Linux platforms, users can execute ns-3 within a docker container.

- Install Docker ≥ 1.13 on windows systems by following the [link](https://docs.docker.com/desktop/windows/install/)

- Switch to the location of the ns-3 Dockerfile in **/bin/fed/ns3** and run the following command on command line. A docker image named ns3-federate will be generated.

  ```
  docker build -t ns3-federate . 
  ```

- Open  etc/runtime.json, enter the name of the ns-3 docker in the ns3-section into the property dockerImage:

  ```
  "federates": [
     ...
     {
        "id": "ns3",
        "dockerImage": "ns3-federate",
        ...
     },
     ...
  ]
  ```

### 2. Installation of Adopt OpenJDK

#### a.  Linux system

- Install JAVA 11 using the following commands

  ```
  wget -qO - https://adoptopenjdk.jfrog.io/adoptopenjdk/api/gpg/key/public | sudo apt-key add -
  sudo add-apt-repository --yes https://adoptopenjdk.jfrog.io/adoptopenjdk/deb/
  sudo apt update && sudo apt install adoptopenjdk-11-hotspot
  ```

- Set **JAVA_HOME** to the root folder of Java JDK

  ```
  export JAVA_HOME=path_to_java JDK
  ```

#### b.  Windows system

- Download prebuilt OpenJDK from this [link](https://adoptium.net/?variant=openjdk11).
- Open the package to install it. Follow the instructions and select  **Set JAVA_HOME variable**.

### 3. Installation of SUMO-1.8.0

#### a. Linux system

- Install the required tools and libraries

  ```
  sudo apt-get install cmake libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev
  ```

- Install SUMO-1.8.0

  ```
  curl -OL https://github.com/eclipse/sumo/archive/refs/tags/v1_8_0.tar.gz"
  tar  xvf  v1_8_0.tar.gz
  cd sumo-1_8_0
  mkdir build/cmake-build && cd build/cmake-build
  cmake ../..
  make -j$(nproc)
  sudo make install
  ```

- Set environment variable SUMO_HOME and add installation path to the PATH

  ```
  export SUMO_HOME=/usr/local/share/sumo
  export PATH=/usr/local/share/sumo/bin:$PATH
  export PATH=/usr/local/share/sumo/tools:$PATH
  ```

#### b. Windows system

- Download the binaries from this [link](https://sourceforge.net/projects/sumo/files/sumo/version 1.8.0/)

- Open the installer and follow the instruction to install it. Select **set SUMO_HOME.**

- And add the SUMO bin folder and tool folder to the Path environment variable.

  ```
  C:\Program Files (x86)\Eclipse\Sumo\bin\
  C:\Program Files (x86)\Eclipse\Sumo\tools\
  ```

### 4. Update TraCI library

#### a. Linux system

Go to **bin/fed/ns3 folder, copy constants.py, connection.py, and main.py** to traci folder of SUMO using following commands.

```
sudo \cp constants.py /usr/local/share/sumo/tools/traci
sudo \cp connection.py /usr/local/share/sumo/tools/traci
sudo \cp main.py /usr/local/share/sumo/tools/traci
```

#### b. Windows system

Go to **bin/fed/ns3** folder of executable co-simulation tool, copy **constants.py, connection.py, and main.py** to traci folder of SUMO (**C:\Program Files (x86)\Eclipse\Sumo\tools\traci**).

### 5. Installation of CARLA 0.9.10 

#### a. Linux system

   *Since there is no CARLA executable binary on Ubuntu 16.04.* please refer to CARLA documentation to build the executable binary of CARLA 0.9.10.

#### b. Windows system

- Download the binaries from this [link](https://github.com/carla-simulator/carla/releases/tag/0.9.10).
- Extract compressed files and a folder named carla-0.9.10 will be generated.

### 6. Installation of python 3.7 and lxml

#### a. Linux system

- Install python 3.7

  ```
  sudo add-apt-repository ppa:deadsnakes/ppa
  sudo apt update && sudo apt install python3.7
  ```

- Install lxml

  ```
  sudo apt install python3-pip
  python3.7 -m pip install pip
  pip3.7 install lxml==4.5.0
  ```

#### b. Windows system

- Download the executable installer from  [link](https://www.python.org/downloads/release/python-379/).
- Open the installer and follow the instruction to install it.  Select Add Python 3.7 to PATH and customize installation.

- Install lxml==4.5.0 by using the following command in Windows terminal.

  ```
  pip install lxml==4.5.0
  ```

## Installation of Co-simulation Tool

### **1.**   **Executable Bundle File**

Download the co-simulation tool bundle file and extract it to an arbitrary path. 

### **2.**   **Installation from Source Code**

For a successfully build the co-simulation tool, Apache Maven needs to be installed. If Maven 3.1.x or higher has not been installed, please use the following steps to install it. 

#### a. Installation of Maven

**Linux system**

- Download the binary distribution archive **apache-maven-3.8.3** from this [link](https://maven.apache.org/download.cgi). 

- Installation extract distribution archive in any directory

  ```
  tar xzvf apache-maven-3.8.3-bin.tar.gz
  ```

- Add the **bin** directory of the created directory **apache-maven-3.8.3** to the **PATH** environment variable.

  ```
  export PATH=path_to_apache_maven-3.8.3/apache-maven-3.8.3/bin:$PATH
  ```

  
- Verify the installation by running **mvn -v** in a new terminal. 

**Windows system**

- Download the binary distribution archive **apache-maven-3.8.3** from this [link](https://dlcdn.apache.org/maven/maven-3/3.8.3/binaries/apache-maven-3.8.3-bin.zip).
- Extract compressed files and a folder named apache-maven-3.8.3 will be generated.
- Add the bin directory of the created directory apache-maven-3.8.3 to the PATH environment variable.
- Verify the installation by running **mvn -v** in a new terminal. 

#### b. Build Co-simulation Tool

- Download zip file of Co-simulation tool or clone the code from this [link](https://github.com/usdot-fhwa-stol/carma-simulation/tree/feature/mosaic-carla-sumo-ambassadors).


  ```
  git clone https://github.com/usdot-fhwa-stol/carma-simulation.git
  ```

- If you use git clone the code, you need to use the following command to go to the co-simulation tool branch.

  ```
  git checkout feature/mosaic-carla-sumo-ambassadors
  ```

- Go to co-simulation folder and build co-simulation tool using the following command

  ```
  cd 'co-simulation tool'
  mvn clean install
  ```

- The above mvn command executes all tests as well. In order to skip test execution, run the following command:

  ```
  mvn clean install -DskipTests
  ```

  

- After building, go to bundle\target directory, a zip file of co-simulation bundle "**carla-sumo-mosaic-21.2.zip**" can be found.

- Extract the bundle to an arbitrary path. The installation folder is referenced as `<cosimulation-root>`. 

## **Folder Content**

The folder structure of  `<cosimulation-root>` is described in the following.


```
└─ <cosimulation-root>
  ├─ bridge ................ Directory with bridge file for connecting CARLA ambassador and CARLA simulator.
  |  ├─ bridge.bat ......... An example of bridge script for a scenario on Windows systems.
  |  ├─ bridge.sh .......... An example of bridge Script for a scenario on Linux systems.      
  ├─ etc
  |  ├─ hosts.json ......... Configuration of the execution host, e.g., temporary directory.
  |  ├─ logback.xml ........ Configuration of log files and levels.
  |  └─ runtime.json ....... Configuration of all Ambassadors and Federates coupled with the MOSAIC.
  ├─ lib ................... Directory with all Java compiled libraries required for MOSAIC.
  ├─ scenarios ............. Directory containing all simulation scenarios.
  ├─ tools ................. Additional tools, like the HTML Visualizer.
  ├─ CONTRIBUTING.md 
  ├─ LICENSE 
  ├─ mosaic.bat ............ Start script for Windows systems.
  └─ mosaic.sh ............. Start script for GNU/Linux systems.
  
```

## **Co-Simulation Scenario**

To run the co-simulation tool, you need to prepare a co-simulation scenario. The following file structure shows the setup of a typical co-simulation scenario.


```
└─ <scenarioName>
 ├─ application
   └─ <scenarioName>.db................ Scenario database file
   └─xx.jar............................ Application jar file
 ├─ carla
 | └─ carla_config.json ................. CARLA configuration file
 | └─ bridge.sh/bridge.bat .............. Bridge file
 ├─ carma
 | └─ carma_config.json ................. CARMA configuration file
 ├─ infrastructure
 | └─ infrastructure_config.json .......... Infrastructure configuration file
 ├─ mapping
 | └─ mapping_config.json ............... Mapping configuration file
 ├─ ns3
 | └─ ns3_config.json ...................... ns-3 configuration file
 | └─ ns3_federate_config.xml .............. ns-3 federate configuration file
 ├─ sumo
 | └─ <scenarioName>.net.xml ............ SUMO network file
 | └─ <scenarioName>.rou.xml ............ SUMO route file
 | └─ <scenarioName>.sumocfg ............ SUMO configuration file
 └─ scenario_config.json ................ Basic configuration of the simulation scenario

```


Here the scenario configuration file, the CARLA configuration file, CARMA configuration file, Infrastructure configuration file, and bridge file are introduced in detail below. For other configuration files and scenario creation, refer to the documentation of Eclipse MOSAIC [[link](https://www.eclipse.org/mosaic/docs/)]. Moreover, the CARLA 3D maps corresponding to the scenarios must be generated for co-simulation scenarios.

### **1.**   **Scenario configuration file**

The **scenario_config.json** is the main configuration file of a co-simulation scenario. A co-simulation scenario file looks like the following example.



```json
{
  "simulation": {
   "id": "co-simulation",
   "duration": "400s",
   "randomSeed": 212323853,
   "projection": {
      "centerCoordinates": {
        	"latitude": 52.63,
        	"longitude": 13.56
      },
      "cartesianOffset": {
       	"x": -395635.35,
      	"y": -5826456.24
     }
    },
   "network": {
    		"netMask": "255.255.0.0",
    		"vehicleNet": "10.1.0.0",
    		"rsuNet": "10.2.0.0",
    		"tlNet": "10.3.0.0",
   	 	    "csNet": "10.4.0.0",
    		"serverNet": "10.5.0.0",
    		"tmcNet": "10.6.0.0",
            "carmaVehicleNet": "10.7.0.0",
	        "carlaVehicleNet": "10.8.0.0"
    }
  },
  "federates": {
  		"application": true,
  		"cell": false,
  		"environment": false,
  		"sns": false,
  		"ns3": true,
  		"omnetpp": false,
  		"output": false,
  		"carla": true,
        "carma": true,
  		"sumo": true,
        "infrastructure": true
  }
}
```

The federate tags show which simulators are  used in the simulation. For the co-simulation scenarios, CARLA and SUMO federate must be enabled. If the CARLA federate is disabled, the simulation of SUMO and MOSAIC will be run.

### **2.**   **Carla configuration file**

The CARLA ambassador can be configured with the carla configuration file. The specific path is **<scenarioName>/carla/carla_config.json**.  The example configuration files of CARLA ambassador on Linux systems and on Windows systems are shown in the following, respectively.

**Linux**

```json
{
    "updateInterval": 100,
	"carlaUE4Path": "/CARLA_0.9.10/",
	"bridgePath": "./scenarios/co-simulation/carla; bridge.sh",
	"carlaConnectionPort": 8913
}
```

**Windows**

```json
{
    "updateInterval": 100,
	"carlaUE4Path": "D:/CARLA_0.9.10/",
	"bridgePath": "./scenarios/co-simulation/carla; bridge.bat",
	"carlaConnectionPort": 8913
}
```

The following table descripts the parameters in the CARLA configuration file.

| Parameters          | **Type** | **Description**                                              | **Required** |
| ------------------- | -------- | ------------------------------------------------------------ | ------------ |
| updateInterval      | number   | The Interval after which the CARLA is updated.  The default unit is ms. Define the size  of one simulation step in CARLA. The updateInterval of Carla and Sumo  ambassador must be the same.  **Note**: the minimal value of updateInterval is 100ms and the default setting of updateInterval being predefined in the java code is 1000ms for both carla and sumo configuration. | Yes          |
| carlaUE4Path        | string   | Path to CarlaUE4 executable file (Windows systems: CarlaUE4.exe; Linux systems: CarlaUE4.sh). If the environment  variable **`CARLA_HOME`** is configured to define  the installation directory of CarlaUE4 executable file, this parameter can be removed from  the configuration file. | No           |
| bridgePath          | string   | This parameter includes two information separated by the  semicolon. The first substring is the path of bridge .sh or .bat. The second  substring is the file name of bridge .sh or .bat. The extension of the bridge file is needed. | Yes          |
| carlaConnectionPort | number   | The bridge server port . It must be the same as  the bridge server port number in the bridge .sh or .bat file.  If the entry is not used, the default port number is 8913. | No           |

### **3.**  **Bridge file**

The bridge file (bridge.sh or bridge.bat) is used to run the bridge to connect CARLA simulator and CARLA ambassador, load maps to CARLA simulator, pass messages between CARLA ambassador and CARLA simulator. The examples of  bridge files on Linux systems and on Windows systems are shown in the following, respectively.

**Linux system**

```bash
#!/bin/bash

cd ../../../bridge

x-terminal-emulator python3.7 carla_mosaic_bridge.py --bridge-server-port 8913 --map Town04 net/Town04.net.xml --step-length 0.1
```

 **Windows system**

```powershell
@ECHO OFF
cd ../../../bridge
python carla_mosaic_bridge.py --bridge-server-port 8913 --map Town04 net/Town04.net.xml --step-length 0.1
```

The arguments in the above python commands are shown in the following.

**`--bridge-server-port 8913`:** The port number that the bridge server is listening to. It must be the same as the carlaConnectionPort in the carla configuration file. If not specified, the default port number 8913 will be used for the bridge server.

**`--map Tow04`**: the map loaded in the CARLA simulator. If not specified, the default map of CARLA simulator will be used.

**`net/Town04.net.xml`**: the net file of the CARLA map or SUMO, the net file is located in the net folder under the bridge folder. Since SUMO and CARLA simulator use different reference systems,  the SUMO net offset calculated from the net file is used to calculate the transform of SUMO actors in CARLA simulator and the transform of CARLA actors in SUMO. If not specified, the net file will be generated using the CARLA map.

**--step-length**: the simulation step length, which should be the same as the updateInterval of CARLA.

### 4. Carma Configuration File

The CARMA ambassador can be configured with the CARMA configuration file. The specific path is **<scenarioName>/carma/carma_config.json**. An example configuration files of CARMA ambassador is shown in the following.

```
{
     "updateInterval": 100,
    "carmaVehicles":[
{   
    "routeID": "0",
    "lane": 1,
    "position": 0,
    "departSpeed": 0,
    "vehicleType": "vehicle.chevrolet.impala",
    "applications":["org.eclipse.mosaic.app.tutorial.VehicleCommunicationApp"],
    "geoPosition":  {
        "latitude": 52.579272059028646, 
        "longitude": 13.467165499469328
    },
    "projectedPosition":
    {
        "x": 501.62, 
        "y": 116.95
    },

    "heading": 24.204351784500364,
    "slope": 0.0
}],
"senderCarmaVehicleId":"carma_0"
}
```

The following table describes the parameters in the CARMA configuration file.

| **Parameters**       | **Type**      | **Description**                                              |
| -------------------- | ------------- | ------------------------------------------------------------ |
| updateInterval       | number        | The Interval after which the CARMA is updated. The default unit is ms. |
| carmaVehicles        | list          | A list of CARMA vehicles controlled by CARMA platforms.      |
| senderCarmaVehicleID | string        | The id of virtual CARMA vehicle which transmit the received  messages from its corresponding real CARMA vehicle. |
| routeID              | number        | The route ID on which the vehicle will be spawned.           |
| lane                 | number        | The lane on which the vehicle will be spawned.               |
| position             | number        | Position within the route where the vehicle should be spawned. |
| departSpeed          | number        | The speed at which the vehicle is supposed to depart.        |
| vehicleType          | string        | The vehicle type                                             |
| application          | list          | the applications to be used for this vehicle.                |
| geoPosition          | geopoint      | The geo position at which the vehicle is currently  located. |
| projectedPosition    | CatesianPoint | The projected position at which currently the vehicle is  located. |
| heading              | number        | Direction/Heading of the vehicle in degrees from north  clockwise. |
| slope                | number        | The slope of vehicle in degrees                              |

### 5. Infrastructure Configuration File

The Infrastructure ambassador can be configured with the Infrastructure configuration file. The specific path is **<scenarioName>/infrastructure/infrastructure_config.json**. An example configuration files of Infrastructure ambassador is shown in the following.

```
{
    "updateInterval": 1000,
    "senderRSUId": "rsu_0"
}
```

The following table describes the parameters in the Infrastructure configuration file.

| **Parameters** | **Type** | **Description**                                              |
| -------------- | -------- | ------------------------------------------------------------ |
| updateInterval | number   | The Interval after which the infrastructure is  updated. The default unit is ms. |
| senderRSUId    | string   | The id of virtual RSU which transmits received messages  from corresponding real RSU. |

### 6. Mapping Configuration File

The Mapping ambassador can be configured with the mapping configuration file. The specific path is **<scenarioName>/mapping/mapping_config.json**. Please refer to this [link](https://www.eclipse.org/mosaic/docs/simulators/application_mapping/) for more information about the configuration file.

### 7. NS-3 Configuration File

The ns-3 ambassador and ns-3 federate can be configured with the configuration files. The specific path is **<scenarioName>/ns3/**. Please refer to this [link](https://www.eclipse.org/mosaic/docs/simulators/network_simulator_ns3/#configuration) for more information about the configuration file. 

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

The following call starts the co-simulation scenario “Co-simulation” on a Linux Machine.

```
./mosaic.sh -s Co-simulation -v -w 0
```

The following call starts the co-simulation scenario “Tow04” on a Windows Machine.

```
mosaic.bat -s Co-simulation -v -w 0
```

 ![co-simulation demo](https://github.com/usdot-fhwa-stol/carma-simulation/blob/feature/mosaic-carla-sumo-ambassadors/co-simulation%20tool/co-simulation%20demo.gif)

## **Check the Log Files**

Log files are generated for the used ambassadors for each simulation run. They are stored in the folder `<cosimulation-root>/logs/log-<timestamp>-scenarioname`. Each simulation run has a  new log file folder generated. The folder structure of log file folder looks like the following. The log file Carla.log is about CARLA ambassador. 

```
└─ log-<timestamp>-secarioname
  Apps . . . . . . . . . . . . . . . . the folder for logs of applications
  ├─ Application.log .......... Information about the application ambassador
  ├─ Carla.log ...................... CARLA ambassador log  
  ├─ Carma.log ...................... CARMA ambassador log        
  ├─ Cell.log ....................... Cellular network log
  ├─ Communication.log ......... (Ad hoc) network simulation ambassador log
  ├─ CommunicationDetails.log ........ (Ad hoc) network simulation detail log
  ├─ Environment.log ................ Logs of the environmental eventserver
  ├─ Infrastructure.log ................ Infrastructure ambassador log
  ├─ Mapping.log .................... Mapping configuration logs
  ├─ MOSAIC.log ...... General information, e.g. startup sequence information
  ├─ Navigation.log ......... Detailed logs about navigation component in the         application ambassador
  ├─ output.csv ....... Recorded data of the integrated File Output Generator
  ├─ RunTimeEvents.csv .............. Logs of run time events
  ├─ Traffic.log .................... Traffic simulation log (SUMO)

```

##  Troubleshooting

If the maps of CARLA for the scenarios is the same as the default map of CARLA simulator, the argument about map in the bridge file is recommended to be removed. Loading a map in CARLA simulator takes time and sometimes may break the connection of CARLA client. The users need restart the co-simulation to solve the connection issue.

Sometimes, the CARLA client may not be connected to CARLA simulator. Check whether there is a CARLA process running in the background or not. If so, terminate the CARLA process.

During the simulation, change the view of spectator may cause the CARLA simulator freeze. Users should adjust the view of spectator before starting the co-simulation.

 

 