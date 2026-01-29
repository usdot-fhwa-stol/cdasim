#!/bin/bash
# CarlaUE4.sh (or CarlaExecutor.sh, linked as CarlaUE4.sh)

KEYWORD="sim"

# Get all network names the current container is attached to
NETWORKS=$(docker inspect \
  --format '{{range $key, $value := .NetworkSettings.Networks}}{{$key}} {{end}}' \
  "$(hostname)")

# Filter networks containing the keyword
MATCHING_NETWORKS=$(echo "$NETWORKS" | tr ' ' '\n' | grep "$KEYWORD")

MATCH_COUNT=$(echo "$MATCHING_NETWORKS" | grep -c .)

if [ "$MATCH_COUNT" -eq 1 ]; then
  NETWORK="$MATCHING_NETWORKS"
  echo "Using Docker network: $NETWORK"
elif [ "$MATCH_COUNT" -gt 1 ]; then
  echo "Error: Multiple Docker networks match keyword '$KEYWORD':"
  echo "$MATCHING_NETWORKS"
  echo "Please refine the keyword or hardcode the network."
  exit 1
else
  echo "Warning: No Docker network matching '$KEYWORD' found. Falling back to 'host'."
  NETWORK="host"
fi

# Run CARLA
docker run --privileged --rm --gpus all \
  --name carla-server \
  --net="$NETWORK" \
  --env=DISPLAY=$DISPLAY \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  --env=SDL_VIDEODRIVER=x11 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro" \
  carlasim/carla:0.10.0 \
  bash CarlaUnreal.sh -vulkan -nosound --ros2 "$@"

