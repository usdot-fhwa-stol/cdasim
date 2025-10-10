#!/bin/bash
# CarlaUE4.sh (or CarlaExecutor.sh, linked as CarlaUE4.sh)

# Retrieve the Docker network name of the current host's container
NETWORK=$(docker inspect --format '{{range $key, $value := .NetworkSettings.Networks}}{{$key}}{{end}}' $(hostname))

# Check if NETWORK is empty
if [ -z "$NETWORK" ]; then
  echo "Warning: Could not determine Docker network. Falling back to 'host' network."
  NETWORK="host"
fi

# Run the Docker command with the dynamically determined network
docker run --privileged --gpus all --net="$NETWORK" --env=DISPLAY=$DISPLAY \
  --env=NVIDIA_VISIBLE_DEVICES=all --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --env=SDL_VIDEODRIVER=x11 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro" \
  carlasim/carla:0.10.0 bash CarlaUnreal.sh -vulkan -nosound "$@"