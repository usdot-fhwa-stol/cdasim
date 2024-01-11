#!/bin/bash

#  Copyright (C) 2018-2024 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.
set -e
source /home/carma/.base-image/env.sh
# Build co-simulation tool
cd /home/carma/src/co-simulation
mvn clean install -DskipTests
cd bundle/target
sudo mkdir -p /opt/carma-simulation
sudo chown -R carma:carma /opt/carma-simulation
unzip carla-sumo-mosaic-22.1-SNAPSHOT.zip -d /opt/carma-simulation
sudo chmod +x /opt/carma-simulation/mosaic.sh
sudo mkdir /opt/carma-simulation/scenarios/tmp_scenario
cp bundle-22.1-SNAPSHOT.jar /opt/carma-simulation

# Deploy scenario files
cd /home/carma/src/co-simulation
unzip sample_scenario.zip -d /opt/carma-simulation/scenarios

# Install NS-3 (has to currently be installed during build step since it's src is included in build zip file)
cd "/opt/carma-simulation/bin/fed/ns3/"
chmod +x ns3_installer.sh
set -x
./ns3_installer.sh -q

sudo cp /home/carma/src/co-simulation/patch/run.sh /opt/carma-simulation/bin/fed/ns3

echo "Build complete!!!"
