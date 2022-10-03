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

ARG VERSION
ARG VCS_REF
ARG BUILD_DATE

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

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN adduser $SUMO_USER --disabled-password

# Enable sudo
RUN apt-get update && apt-get install -y sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN adduser $SUMO_USER sudo --disabled-password

COPY --chown=carma:carma . /home/carma/src


# install dependencies
RUN apt update && \
      apt install -y python3-pip \
      gnupg \
      libjpeg-dev \
      libtiff5-dev \
      libomp-dev \
      fontconfig

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub

# fix ALSA errors
RUN echo pcm.!default { type plug slave.pcm "null" } >> /etc/asound.conf

# install NICE DCV (for RoboMaker)
RUN apt update -y && apt upgrade -y && apt install -y wget pgp
RUN wget https://d1uj6qtbmh3dt5.cloudfront.net/NICE-GPG-KEY
RUN gpg --import NICE-GPG-KEY
RUN wget https://d1uj6qtbmh3dt5.cloudfront.net/2021.1/Servers/nice-dcv-2021.1-10598-ubuntu1804-x86_64.tgz
RUN tar -xvzf nice-dcv-2021.1-10598-ubuntu1804-x86_64.tgz
RUN apt update && apt install -y ./nice-dcv-2021.1-10598-ubuntu1804-x86_64/nice-dcv-gl_2021.1.937-1_amd64.ubuntu1804.deb \
                                 ./nice-dcv-2021.1-10598-ubuntu1804-x86_64/nice-dcv-gltest_2021.1.275-1_amd64.ubuntu1804.deb

# install opengl 
RUN apt update && apt install -y libglfw3 libglfw3-dev

USER carma
WORKDIR /home/carma/src
COPY --chown=carma:carma docker/env.sh /home/carma/.base-image/
RUN docker/install.sh

ENTRYPOINT [ "/home/carma/src/docker/entrypoint.sh" ]
CMD [ "mosaic.sh", "-s", "HelloWorld" ]
