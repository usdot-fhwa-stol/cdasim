#!/bin/bash

port=$1
cmdport=$2

if [[ -z $cmdport ]]; then
  cmdport=0
fi

cd ns3
LD_LIBRARY_PATH=/opt/carma-simulation/bin/fed/ns3/ns-allinone-3.28/ns-3.28/build /opt/carma-simulation/bin/fed/ns3/ns-allinone-3.28/ns-3.28/build/scratch/mosaic_starter --port=$port --cmdPort=$cmdport --configFile=scratch/ns3_federate_config.xml

