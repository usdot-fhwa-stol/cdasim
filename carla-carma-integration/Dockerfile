# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
FROM ubuntu:16.04
FROM ros:kinetic
WORKDIR /home

# CARLA PythonAPI
RUN mkdir ./PythonAPI
ADD https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.10-py2.7-linux-x86_64.egg ./PythonAPI

RUN apt-get update && apt-get install -y \
		git \
		curl \
		wget \
		nano \
		libpng16-16 \
		libsdl2-2.0 \
		software-properties-common

# Update gcc and g++
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install -y \
		gcc-6 \
		g++-6

# CARLA ROS Bridge
RUN git clone -b '0.9.10.1' --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# CARLA-CARMA integration tool copy from local
COPY . ./carla-carma-integration

# CARLA-CARMA integration tool necessary package and msgs
RUN apt-get update && apt-get install -y --no-install-recommends \
    python-pip \
    python-wheel \
    ros-kinetic-ackermann-msgs \
    ros-kinetic-derived-object-msgs \
    ros-kinetic-jsk-recognition-msgs \
		ros-kinetic-rqt \
		ros-kinetic-rviz \
		ros-kinetic-pcl-conversions \
		ros-kinetic-pcl-ros \
		ros-kinetic-cv-bridge

# Upgrade CMake to 3.13
RUN wget https://cmake.org/files/v3.13/cmake-3.13.0-Linux-x86_64.tar.gz
RUN tar -xzvf cmake-3.13.0-Linux-x86_64.tar.gz
RUN mv cmake-3.13.0-Linux-x86_64 /opt/cmake-3.13.0
RUN ln -sf /opt/cmake-3.13.0/bin/* /usr/bin/
RUN rm cmake-3.13.0-Linux-x86_64.tar.gz

# Catkin make for both ros-bridge and carla-carma-integration
RUN mkdir -p carla_carma_ws/src
RUN cd carla_carma_ws/src \
    && ln -s ../../ros-bridge \
    && ln -s ../../carla-carma-integration \
    && cd .. \
    && /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make'

RUN pip install simple-pid

CMD ["/bin/bash"]
