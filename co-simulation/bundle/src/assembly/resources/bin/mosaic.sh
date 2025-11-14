#!/bin/bash
set -e
set -x # echo commands

# set maximum JVM memory
javaMemorySizeXmx="2g"
# javaRemoteDebugging="-agentlib:jdwp=transport=dt_socket,server=y,suspend=y,address=10000"

# Get scenario name from args (default Town04) and forward args to Java
SCENARIO="Town04"
FORWARD_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -s|--scenario)
      SCENARIO="${2##*/}"; FORWARD_ARGS+=("$1" "$2"); shift 2;;
    *) FORWARD_ARGS+=("$1"); shift;;
  esac
done

# Set logging directory / pass to java (used by logback.xml)
export LOG_DIR="/opt/carma-simulation/logs/log-$(date +%Y%m%d-%H%M%S)-${SCENARIO}"
mkdir -p "$LOG_DIR"
export JAVA_TOOL_OPTIONS="-DlogDirectory=$LOG_DIR"

# mosaic
dir_mosaic=./lib/mosaic
tmp=$(ls "${dir_mosaic}" | grep jar)
mosaic=${dir_mosaic}/${tmp//[^A-Za-z0-9\-\.]/:${dir_mosaic}/}

# third-party
dir_libs=./lib/third-party
tmp=$(ls "${dir_libs}" | grep jar)
libs=${dir_libs}/${tmp//[^A-Za-z0-9\-\.]/:${dir_libs}/}

# check if mosaic/third-party jars exist
if [[ -z "${mosaic}" || -z "${libs}" ]]; then
  echo "ERROR: Could not build classpath. Check ./lib/mosaic and ./lib/third-party for jars."
  exit 1
fi

echo "CLASSPATH=.:./etc:${mosaic}:${libs}"
echo "Using LOG_DIR=$LOG_DIR"

# Run
exec java -Xmx"${javaMemorySizeXmx}" \
  ${javaRemoteDebugging:+${javaRemoteDebugging}} \
  -Dlogback.configurationFile=/opt/carma-simulation/etc/logback.xml \
  -Dlogback.debug=true \
  -cp ".:/opt/carma-simulation/etc:${mosaic}:${libs}" \
  org.eclipse.mosaic.starter.MosaicStarter "${FORWARD_ARGS[@]}"