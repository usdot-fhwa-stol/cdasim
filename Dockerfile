#  Copyright (C) 2018-2021 LEIDOS.
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

FROM ubuntu:18.04

MAINTAINER Leidos, Inc.
LABEL Description="Dockerised Simulation of Carla_Sumo_Mosaic"

ENV SUMO_HOME /usr/share/sumo
ENV SUMO_USER carma


ENV CARLA_HOME /CARLA_0.9.10/
RUN export CARLA_HOME

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-simulation"
LABEL org.label-schema.description="XIL Simulation environment for evaluation and testing of the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-simulation/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64/
RUN export JAVA_HOME

#RUN apt-get -y update

#RUN apt-get install x11-xserver-utils dconf-editor dbus-x11 -y

#RUN apt-get update && apt-get upgrade -y && apt-get clean


#RUN curl -s https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
#    python get-pip.py --force-reinstall && \
#    rm get-pip.py

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN adduser $SUMO_USER --disabled-password

# Enable sudo
RUN apt-get update && apt-get install -y sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN adduser $SUMO_USER sudo --disabled-password

COPY --chown=carma:carma . /home/carma/src
USER carma
WORKDIR /home/carma/src
RUN docker/install.sh

COPY co-simulation/traci_update/connection.py /usr/share/sumo/tools/traci/
COPY co-simulation/traci_update/constants.py /usr/share/sumo/tools/traci/
COPY co-simulation/traci_update/main.py /usr/share/sumo/tools/traci/

#ENTRYPOINT [ "/home/carma/src/docker/entrypoint.sh" ]
#CMD [ "cd /opt/carma-simulation && ./mosaic.sh -s Town04_20" ]

