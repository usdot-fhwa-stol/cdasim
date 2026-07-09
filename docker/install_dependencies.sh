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
#!/bin/bash

set -e

echo "=== Updating APT and installing base tools ==="
sudo apt-get update
sudo apt-get install -y \
  software-properties-common \
  apt-transport-https \
  ca-certificates \
  curl \
  wget \
  unzip \
  tar \
  pkg-config \
  sqlite3 \
  autoconf \
  libtool \
  make \
  build-essential \
  lbzip2 \
  rsync \
  x11-xserver-utils \
  dconf-editor \
  dbus-x11 \
  vim nano xterm

# ---------------------------------------------------------------
#  Fix Docker: Install modern static Docker CLI (API >= 1.44)
# ---------------------------------------------------------------
echo "=== Installing modern static Docker CLI ==="

# Ensure no old docker-ce or docker-ce-cli is present
sudo apt-get remove -y docker-ce docker-ce-cli || true

# Install Docker CLI (static build)
cd /tmp
curl -fsSL https://download.docker.com/linux/static/stable/x86_64/docker-27.2.0.tgz -o docker.tgz
tar xzvf docker.tgz
sudo cp docker/* /usr/bin/
rm -rf docker docker.tgz

echo "Docker CLI installed:"
docker version || true

# ---------------------------------------------------------------
#  GCC 7, Python 3.7, Java 11, and required PPAs
# ---------------------------------------------------------------
echo "=== Adding PPAs for GCC and Python 3.7 ==="
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get update

echo "=== Installing GCC7, Python 3.6/3.7, Java 11, and build deps ==="
sudo apt-get install -y --allow-unauthenticated \
  gcc-7 g++-7 \
  python3.6 python3.6-dev \
  python3.7 python3.7-dev python3.7-distutils python3-pip \
  openjdk-11-jdk ant \
  libxml2 libxml2-dev libsqlite3-dev \
  libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev \
  libgl2ps-dev libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
  protobuf-compiler libprotobuf-dev patch cmake automake

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 20 --slave /usr/bin/g++ g++ /usr/bin/g++-7
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
sudo update-alternatives --set python /usr/bin/python3.7

# ---------------------------------------------------------------
#  Build and Install SUMO 1.15.0
# ---------------------------------------------------------------
echo "=== Building and installing SUMO 1.15.0 ==="

cd /home/carma/src/
wget -q "https://github.com/eclipse/sumo/archive/refs/tags/v1_15_0.tar.gz"
sudo mkdir -p /opt/sumo
sudo chown -R carma:carma /opt/sumo
tar xvf v1_15_0.tar.gz -C /opt/sumo

cd /opt/sumo/sumo-1_15_0
mkdir -p build/cmake-build && cd build/cmake-build
cmake ../..
make -j$(nproc)
sudo make install

# ---------------------------------------------------------------
#  Python packages
# ---------------------------------------------------------------
echo "=== Installing Python lxml ==="
python3.7 -m pip install --upgrade pip
python3.7 -m pip install lxml==4.5.0

# Install CARLA
CARLA_TAR="CARLA_0.9.10.tar.gz"
cd /home/carma/src/
if [[ ! -f "$CARLA_TAR" ]]; then
    echo "!!! $CARLA_TAR not present in the installation directory, downloading automatically instead. This could take a long time, consider downloading the file manually and placing it in the installation directory. !!!"
    wget -q "https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.10.tar.gz"
fi

sudo mkdir -p /opt/carla
sudo chown -R carma:carma /opt/carla
tar xzvf "$CARLA_TAR" -C /opt/carla
# Adding configuration file to fix error output from CARLA (https://github.com/carla-simulator/carla/issues/2820)
echo $'pcm.!default {\n  type plug\n  slave.pcm \"null\"\n}' | sudo tee /etc/asound.conf


# ---------------------------------------------------------------
#  Install Maven
# ---------------------------------------------------------------
echo "=== Installing Maven 3.8.3 ==="
cd /tmp
wget -q "https://archive.apache.org/dist/maven/maven-3/3.8.3/binaries/apache-maven-3.8.3-bin.tar.gz"
tar xzvf apache-maven-3.8.3-bin.tar.gz -C /opt/
sudo chown -R carma:carma /opt/apache-maven-3.8.3/
rm apache-maven-3.8.3-bin.tar.gz

echo "=================================================="
echo " Install Dependencies Complete!!!"
echo "=================================================="