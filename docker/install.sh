#!/bin/bash

#  Copyright (C) 2018-2020 LEIDOS.
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

# Install software-proprties-common to be able to setup PPA repos
sudo apt-get update
sudo apt-get install -y software-properties-common

sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get update

# Download apt dependencies
sudo apt-get install -y --allow-unauthenticated gcc-7 g++-7 python3.6 unzip tar python3.6-dev \
  pkg-config sqlite3 autoconf libtool curl make libxml2 libsqlite3-dev \
  libxml2-dev cmake libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev \
  libgl2ps-dev python3.7 python3-pip automake openjdk-8-jdk ant python3.7-dev \
  python3.7-distutils x11-xserver-utils dconf-editor dbus-x11 libglvnd0 libgl1 \
  libglx0 libegl1 libxext6 libx11-6 python3-dev \
  build-essential pkg-config lbzip2 libprotobuf-dev protobuf-compiler patch rsync \
  wget vim nano xterm default-jdk libprotobuf-dev
sudo rm -rf /var/lib/apt/lists/*
export JAVA_HOME="/usr/lib/jvm/java-11-openjdk-amd64"

#update-alternatives --set python /usr/bin/python3.7
sudo apt-get clean
sudo rm -rf /var/cache/oracle-jdk8-installer
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 20 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
sudo update-alternatives --set python /usr/bin/python3.7

cd /home/carma/src

# Install Protobuf - OPTIONAL
#
# Pulled in via apt-get instead of compiled
#
#wget "https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.3.0.tar.gz"
#tar xvf v3.3.0.tar.gz
#cd protobuf-3.3.0
#./autogen.sh
#./configure --prefix=/usr

#make
#make check
#sudo make install

# Install NS-3
#cd "/home/carma/src/co-simulation tool/bundle/src/assembly/resources/fed/ns3/"
# Added sudo cp -ar ns-3.28/build/ns /usr/include before "Build ns3-federate" in ns3_installer.sh
#chmod +x ns3_installer.sh
#set -x
#./ns3_installer.sh -q
#TODO: Add expore NS3_HOME=path_to_run.sh to /bin/fed/ns3/run.sh

# Install SUMO-1.12.0
cd /home/carma/src/
wget "https://github.com/eclipse/sumo/archive/refs/tags/v1_12_0.tar.gz"
sudo mkdir -p /opt/sumo
sudo chown -R carma:carma /opt/sumo
tar xvf v1_12_0.tar.gz -C /opt/sumo
sudo cp co-simulation/traci_update/constants.py /opt/sumo/sumo-1_12_0/tools/traci
sudo cp co-simulation/traci_update/connection.py /opt/sumo/sumo-1_12_0/tools/traci
sudo cp co-simulation/traci_update/main.py /opt/sumo/sumo-1_12_0/tools/traci
cd /opt/sumo/sumo-1_12_0
mkdir -p build/cmake-build && cd build/cmake-build
cmake ../..
make -j$(nproc)
sudo make install

# Install python3.7 and lxml
python3.7 -m pip install pip
python3.7 -m pip install lxml==4.9.2

# Install CARLA
cd /home/carma/src/
wget "https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.tar.gz"
if [[ ! -f '/home/carma/src/CARLA_0.9.10.tar.gz' ]]; then
    echo "!!! CARLA not present in the installation directy, please download CARLA_0.9.10.tar.gz into the work directory and rebuild. !!!"
    exit -1
fi

sudo mkdir -p /opt/carla
sudo chown -R carma:carma /opt/carla
tar xzvf CARLA_0.9.10.tar.gz -C /opt/carla

# Installation of Co-Simulation Tool
wget "https://archive.apache.org/dist/maven/maven-3/3.8.3/binaries/apache-maven-3.8.3-bin.tar.gz"
sudo mkdir -p /opt/maven
sudo chown -R carma:carma /opt/maven
tar xzvf apache-maven-3.8.3-bin.tar.gz -C /opt/maven
export PATH=/opt/maven/apache-maven-3.8.3/bin/:$PATH

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

echo "Build complete!!!"
