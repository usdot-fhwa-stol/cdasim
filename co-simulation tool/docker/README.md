#  Running CARLA_SUMO_MOSAIC Co-Simulation Tool in Docker 

This is the tutorial to run CARLA_SUMO_MOSAIC co-simulation tool in a docker container. It is recommended to run the co-simulation tool on a **high-performance** computer.

## Test platform

- Operating System: Ubuntu 20.04.2 LTS
- CPU: Intel(R) Core(TM) i7-9700K CPU @ 3.60GHz
- RAM: 16GB
- Graphics: NVIDIA GeForce RTX 2070 Super 8GB

### Prerequisite

- NVIDIA GPU with at least 8GB
- NVIDIA drivers > 361.93
- Free Disk Space > 30GB
- NVIDIA Container Toolkit
- Docker

If you do not install Docker, please see this [link](https://docs.docker.com/engine/install/ubuntu/).

To install NVIDIA Container Toolkit, please see this [link](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

### Build Docker Image of Co-simulation Tool

#### Preparation for building docker image

- Put the docker file (Dockerfile), the folder of CARLA executable file (CARLA_0.9.10), the folder of Co-simulation tool bundle (Carla_Sumo_Mosaic) in the same folder.
- Change the CARLA path in the CARLA configuration file of scenarios to the location of CARLA executable file
- The .sh files (CarlaUE4.sh and bridge.sh) must be given permission

#### Build docker image

##### step 1:  Run the following command to build the docker image

```
$ docker build - < Dockerfile -t carla-sumo-mosaic
```

##### step 2: copy CARLA and Co-Simulation tool to the docker image and commit them

Copy `CARLA_0.9.10` and `Carla_Sumo_Mosaic` folder into this container.

```
$ docker cp CARLA_0.9.10 <container-id>:./
$ docker cp Carla_Sumo_Mosaic <container-id>:./
$ docker container commit <container-id> carla-sumo-mosaic
```

### Run Image

```
$ docker run   --rm   -it   --gpus all   -v /tmp/.X11-unix:/tmp/.X11-unix   -e DISPLAY=$DISPLAY   -e QT_X11_NO_MITSHM=1 --user=carla_sumo_mosaic carla-sumo-mosaic
```

### Run Co-simulation 

In the docker container, first go to the `Carla_Sumo_Mosaic` folder and then use the following command to run the co-simulation. There are two co-simulation scenarios. One has 10 vehicles, and the other has 200 vehicles. After the map in the CARLA is loaded, start simulation in the SUMO GUI.

```
$ cd Carla_Sumo_Mosaic
$ ./mosaic -s Town04_10
#Run co-simulation with 10 vehicles
$ ./mosaic -s Town04_200
#Run co-simulation with 200 vehicles. (Need high performance hardware)
```

### Troubleshooting

If the CARLA window does not load properly, exit the container and re-enter will solve this problem. Before running each simulation, check whether CARLA is running in the background or not. If it is running, kill the process of CARLA. 
