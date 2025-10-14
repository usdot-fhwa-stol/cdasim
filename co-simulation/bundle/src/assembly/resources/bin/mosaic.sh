#!/bin/bash
set -e
set -x # echo commands

# set maximum JVM memory
javaMemorySizeXmx="2g"

# uncomment to activate remote debugging
# javaRemoteDebugging="-agentlib:jdwp=transport=dt_socket,server=y,suspend=y,address=10000"

# mosaic
dir_mosaic=./lib/mosaic
tmp=`ls ${dir_mosaic} | grep jar`
mosaic=${dir_mosaic}/${tmp//[^A-Za-z0-9\-\.]/:${dir_mosaic}/}

# third-party
dir_libs=./lib/third-party
tmp=`ls ${dir_libs} | grep jar`
libs=${dir_libs}/${tmp//[^A-Za-z0-9\-\.]/:${dir_libs}/}

# check if mosaic/third-party jars exist
if [[ -z "${mosaic}" || -z "${libs}" ]]; then
  echo "ERROR: Could not build classpath. Check ./lib/mosaic and ./lib/third-party for jars."
  exit 1
fi

echo "CLASSPATH=.:./etc:${mosaic}:${libs}"

# create and run command
exec java -Xmx"${javaMemorySizeXmx}" \
  ${javaRemoteDebugging:+${javaRemoteDebugging}} \
  -Dlogback.configurationFile=/opt/carma-simulation/etc/logback.xml \
  -Dlogback.debug=true \
  -cp ".:/opt/carma-simulation/etc:${mosaic}:${libs}" \
  org.eclipse.mosaic.starter.MosaicStarter "$@"