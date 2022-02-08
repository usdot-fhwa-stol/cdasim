#!/bin/bash

RUNTIME=""
DOCKER_VERSION=$(docker version --format '{{.Client.Version}}' | cut --delimiter=. --fields=1,2)
if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi
docker run \
       -it --rm\
       --name carla_carma_integration \
       --net=host \
       usdotfhwastol/carla-carma-integration:latest
