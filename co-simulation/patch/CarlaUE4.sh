#!/bin/bash
# CarlaUE4.sh (or CarlaExecutor.sh, linked as CarlaUE4.sh)

KEYWORD="sim"
CARLA_RENDER_OFFSCREEN="${CARLA_RENDER_OFFSCREEN:-true}"
CARLA_EXECUTABLE="/home/carla/CarlaUnreal/Binaries/Linux/CarlaUnreal-Linux-Shipping"
CARLA_LAUNCH_ARGS=(-vulkan -nosound --ros2)

case "${CARLA_RENDER_OFFSCREEN,,}" in
  true|1|yes)
    CARLA_LAUNCH_ARGS+=(-RenderOffScreen)
    ;;
  false|0|no)
    ;;
  *)
    echo "Error: CARLA_RENDER_OFFSCREEN must be true or false, got '$CARLA_RENDER_OFFSCREEN'."
    exit 1
    ;;
esac

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

# Run the shipping binary directly instead of through the image's wrapper.
# For `bash -c`, `carla-launcher` becomes $0 and CARLA_EXECUTABLE becomes $1.
# `shift` removes the executable path from the positional arguments, and
# `chmod +x` ensures the CARLA binary is executable.
# `exec` replaces the shell with Unreal so `docker stop` sends SIGTERM
# directly to CARLA for correctly start and stop simulator. 
# "$@" forwards the launch arguments assembled above together with 
# any additional arguments supplied by CDASim.

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
  bash -c 'executable="$1"; shift; chmod +x "$executable"; exec "$executable" CarlaUnreal "$@"' \
  carla-launcher "$CARLA_EXECUTABLE" "${CARLA_LAUNCH_ARGS[@]}" "$@"
