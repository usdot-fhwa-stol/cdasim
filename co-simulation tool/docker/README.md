#  Running CARLA-SUMO-MOSAIC Co-Simulation Tool in Docker 

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

- Put the docker file (Dockerfile), the folder of CARLA executable file (CARLA_0.9.10), and the folder of Co-simulation tool bundle (Carla_Sumo_Mosaic) in the same folder.
- Change the CARLA path in the CARLA configuration file of scenarios to the location of the CARLA executable file.
- The .sh files (CarlaUE4.sh and bridge.sh) must be given permission.

#### Build docker image

##### Step 1:  Run the following command to build the docker image

```
$ docker build - < Dockerfile -t carla-sumo-mosaic
```

##### Step 2: Copy CARLA and Co-Simulation tool to the docker image and commit them

Copy `CARLA_0.9.10` and `Carla_Sumo_Mosaic` folder into this container.

```
$ docker cp CARLA_0.9.10 <container-id>:./
$ docker cp Carla_Sumo_Mosaic <container-id>:./
$ docker container commit <container-id> carla-sumo-mosaic
```

### Run Docker Image

```
$ docker run --rm -it --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 --user=carla_sumo_mosaic carla-sumo-mosaic
```

### Run Co-simulation 

In the docker container, first go to the `Carla_Sumo_Mosaic` folder and then use the following command to run the co-simulation. There are two co-simulation scenarios. One has 10 vehicles, and the other has 200 vehicles. After the map in the CARLA is loaded, start simulation in the SUMO GUI.

#### Example 1: Run co-simulation with 10 vehicles

```
$ cd Carla_Sumo_Mosaic
$ ./mosaic -s Town04_10
```

#### Example 2: Run co-simulation with 200 vehicles. (Need high performance hardware)

```
$ cd Carla_Sumo_Mosaic
$ ./mosaic -s Town04_200
```

### Troubleshooting

If the CARLA window does not load properly, exit the container and re-enter will solve this problem. Before running each simulation, check whether CARLA is running in the background or not. If it is running, first terminate the CARLA process and then run the co-simulation.
