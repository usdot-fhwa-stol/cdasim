#  Running CARLA-SUMO-MOSAIC Co-Simulation Tool with NS-3 Integration in Docker 

This is the tutorial to run CARLA-SUMO-MOSAIC co-simulation tool in a docker container. It is recommended to run the co-simulation tool on a **high-performance** computer.

## Test Platform

- Operating System: Ubuntu 20.04.2 LTS 
- CPU: Intel(R) Core(TM) i7-9700K CPU @ 3.60 GHz
- RAM: 16 GB
- Graphics: NVIDIA GeForce RTX 2070 Super 8 GB

## Prerequisites

- NVIDIA GPU with at least 8 GB
- NVIDIA drivers > 361.93
- Free Disk Space > 30 GB
- NVIDIA Container Toolkit
- Docker

If you do not have Docker installed, please refer to this [link](https://docs.docker.com/engine/install/ubuntu/) for installation.

If you do not have NVIDIA Container Toolkit installed, please refer to this [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) for installation.

## Build Docker Image of Co-simulation Tool

#### Preparation for building docker image

- Put the docker file (Dockerfile), the folder of CARLA executable file (CARLA_0.9.10), and the folder of `ns-3-integration`in the same folder.
- Change the CARLA path in the CARLA configuration file of scenarios to the location of the CARLA executable file.
- The .sh files (CarlaUE4.sh and bridge.sh) must be given permission.

#### Build docker image

##### Step 1:  Run the following command to build the docker image

```
docker build - < Dockerfile -t ns-3-integration
```

##### Step 2: Copy CARLA and Co-Simulation tool with NS-3 to the docker image and commit them

Download the executable bundle file `carla-sumo-mosaic-21.2.zip` from GitHub and extract the zip file, the folder named `carla-sumo-mosaic-21.2` will be generated and rename the folder as `ns-3-integration`.

Copy `CARLA_0.9.10` and `ns-3-integration` folder into this container.

```
docker cp CARLA_0.9.10 <container-id>:./
docker cp ns-3-integration <container-id>:./
docker container commit <container-id> ns-3-integration
```

### Run Docker Image

```
docker run --rm -it --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 --user=carla_sumo_mosaic ns-3-integration
```

### Modify files in Docker Container

#### Step 1: Modify `ns3_installer.sh`

In Docker container, type:

```
cd ns-3-integration/bin/fed/ns3
```

Then modify `ns3_installer.sh` by:

```
vim ns3_installer.sh
```

(If you see a message `vim: command not found`, please use `sudo apt-get install vim` to install vim as text editor)

In the `build_ns3()` function, after `CXXFLAGS="-Wno-error" python3.6 ./build.py --disable-netanim`, add the following command 

```
sudo cp -ar ns-3.28/build/ns3 /usr/include/
```

#### Step 2: Build NS-3

In directory `ns-3-integration/bin/fed/ns3`, type:

```
./ns3_installer.sh
```

Once all 1947 files are compiled and a successful message shows up, it means NS-3 is successful installed.

#### Step 3: Add NS-3 as environment variable

```
export NS3_HOME='/ns-3-integration/bin/fed/ns3'
```

Use `printenv` to check if NS-3 is successful added to environment variable.

#### Step 4: Modify `run.sh`

In directory `ns-3-integration/bin/fed/ns3`, type:

```
vim run.sh
```

Change line 11 to:

```
LD_LIBRARY_PATH=/ns-3-integration/fed/bin/ns3/ns-allinone-3.28/ns-3.28/build /ns-3-integration/fed/bin/ns3/ns-allinone-3.28/ns-3.28/build/scratch/mosaic_starter --port=$port --cmdPort=$cmdport --configFile=scratch/ns3_federate_config.xml
```

Now, you can test if NS-3 works.

In root directory, type:

```
cd ns-3-integration
```

Use example `Tiergarten` to test:

```
./mosaic.sh -s Tiergarten
```

#### Step 5: Modify traci library (For Co-Simulation)

In root directory, type:

```
cd /usr/share/sumo/tools/traci
```

Then in the directory `/usr/share/sumo/tools/traci`, modify the following three files:

1. `constants.py`

   ```
   sudo nano constants.py
   ```

   Add these to `constants.py`:

   ```python
   # command: get V2X message
   CMD_GET_V2X = 0x0d
   
   # command: set V2X message
   CMD_SET_V2X = 0x2f
   ```

2. `main.py`

   ```
   sudo nano main.py
   ```

   Add these methods to `main.py`:

   ```python
   def getV2xMessage():
       if "" not in _connections:
           raise FatalTraCIError("Not connected.")
       return _connections[""].getV2xMessage()
   
   def setV2xMessage(message):
       if "" not in _connections:
           raise FatalTraCIError("Not connected.")
       return _connections[""].setV2xMessage(message)
   ```

3. `connection.py`

   ```
   sudo nano connection.py
   ```

   Add the following commands:

   ```python
   def getV2xMessage(self):
       command = tc.CMD_GET_V2X
       result = self._sendCmd(command,None,None)
       result.readLength()
       response = result.read("!B")[0]
   
       if response != command:
           raise FatalTraCIError("Received answer %s for command %s." % (response, command))
       return result.readStringList()
   
   def setV2xMessage(self, message):
       self._sendCmd(tc.CMD_SET_V2X, None, None, "s", message)
   ```

#### Step 6: Modify `carla_config.json`

Please modify this file to:

```
{
    "updateInterval": 100,
    "carlaUE4Path": "./CARLA_0.9.10/",
    "bridgePath": "./scenarios/Co-simulation/carla; bridge.sh",
    "carlaConnectionPort": 8913
}
```

### Run Co-simulation

Once everything is ready, under directory `/ns-3-integration`, simply type:

```
./mosaic.sh -s Co-simulation
```

After the map in CARLA is loaded, you can start co-simulation with ns-3 in SUMO-GUI.
