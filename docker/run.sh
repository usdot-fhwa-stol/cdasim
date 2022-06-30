#!/bin/bash

RUNTIME=""
IMAGE="carma-xil-cosimulation"
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        -v|--version)
            COMPONENT_VERSION_STRING="$2"
            shift
            shift
            ;;
    esac
done
echo "##### Running usdotfhwastol/carma-xil-cosimulation:$COMPONENT_VERSION_STRING docker container #####"
docker run \
       --rm -it\
       --gpus all\
       --net=host\
       -v /tmp/.X11-unix:/tmp/.X11-unix\
       -e DISPLAY=$DISPLAY\
       -e QT_X11_NO_MITSHM=1\
       --user=carla_sumo_mosaic usdotfhwastol/carma-xil-cosimulation:$COMPONENT_VERSION_STRING
