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
sudo add-apt-repository ppa:sumo/stable
sudo apt-get update

sudo apt-get install -y --allow-unauthenticated gcc-7 g++-7 python3.6 unzip tar python3.6-dev \
  pkg-config sqlite3 autoconf libtool curl make libxml2 libsqlite3-dev \
  libxml2-dev cmake libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev \
  libgl2ps-dev python3.7 python3-pip automake openjdk-8-jdk ant python3.7-dev \
  python3.7-distutils x11-xserver-utils dconf-editor dbus-x11 libglvnd0 libgl1 \
  libglx0 libegl1 libxext6 libx11-6 python3-dev \
  build-essential pkg-config lbzip2 libprotobuf-dev protobuf-compiler patch rsync \
  wget vim nano xterm default-jdk sumo sumo-tools sumo-doc 
sudo rm -rf /var/lib/apt/lists/*

#update-alternatives --set python /usr/bin/python3.7
sudo apt-get clean 
sudo rm -rf /var/cache/oracle-jdk8-installer
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 20 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
sudo update-alternatives --set python /usr/bin/python3.7

cd /home/carma/src

# Install Protobuf
wget "https://github.com/protocolbuffers/protobuf/archive/refs/tags/v3.3.0.tar.gz"
tar xvf v3.3.0.tar.gz
cd protobuf-3.3.0
./autogen.sh
./configure --prefix=/usr

make
make check
sudo make install

# Install NS-3
#cd "/home/carma/src/co-simulation tool/bundle/src/assembly/resources/fed/ns3/"
# Added sudo cp -ar ns-3.28/build/ns /usr/include before "Build ns3-federate" in ns3_installer.sh
#chmod +x ns3_installer.sh
#set -x
#./ns3_installer.sh -q
#TODO: Add expore NS3_HOME=path_to_run.sh to /bin/fed/ns3/run.sh

# Install SUMO-1.8.0
cd /home/carma/src/
wget "https://github.com/eclipse/sumo/archive/refs/tags/v1_8_0.tar.gz"
tar xvf v1_8_0.tar.gz
cd sumo-1_8_0
mkdir build/cmake-build && cd build/cmake-build
cmake ../..
make -j$(nproc)
sudo make install

# Update TraCI library
sudo cp "/home/carma/src/co-simulation tool/bundle/src/assembly/resources/fed/ns3/constants.py" /usr/local/share/sumo/tools/traci
sudo cp "/home/carma/src/co-simulation tool/bundle/src/assembly/resources/fed/ns3/connection.py" /usr/local/share/sumo/tools/traci
sudo cp "/home/carma/src/co-simulation tool/bundle/src/assembly/resources/fed/ns3/main.py" /usr/local/share/sumo/tools/traci

# Install python3.7 and lxml
python3.7 -m pip install pip
pip3.7 install lxml=4.5.0


# Install CARLA
# TODO: Figure out how to optimize this, as it's a 3.7GB download
# cd /home/carma/src
# wget "https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.tar.gz"
# tar xzvf CARLA_0.9.10.tar.gz

# Installation of Co-Simulation Tool
wget "https://dlcdn.apache.org/maven/maven-3/3.8.3/binaries/apache-maven-3.8.3-bin.tar.gz"
tar xzvf apache-maven-3.8.3-bin.tar.gz
# export PATH=path_to_apache_maven_3.8.3:$PATH

# Build co-simulation tool
cd "../../co-simulation tool"
mvn clean install -DskipTests

cd bundle/target
cp carla-sumo-mosaic-21.2.zip
mkdir -p /opt/carma/simulation
unzip carla-sumo-mosaic-21.2.zip -d /opt/carma-simulation
echo "Build complete!!!"