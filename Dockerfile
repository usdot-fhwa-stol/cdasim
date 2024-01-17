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
LABEL org.label-schema.name="cdasim"
LABEL org.label-schema.description="XIL Simulation environment for evaluation and testing of the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/cdasim/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}


# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN adduser $SUMO_USER --disabled-password

# Enable sudo
RUN sed -i 's|http://archive.ubuntu.com|http://us.archive.ubuntu.com|g' /etc/apt/sources.list
RUN apt-get update && apt-get install -y sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN adduser $SUMO_USER sudo --disabled-password
COPY docker/ /home/carma/src/docker

RUN /home/carma/src/docker/install_dependencies.sh

COPY --chown=carma:carma . /home/carma/src
USER carma
WORKDIR /home/carma/src
COPY --chown=carma:carma docker/env.sh /home/carma/.base-image/
RUN docker/build.sh

ENTRYPOINT [ "/home/carma/src/docker/entrypoint.sh" ]
CMD [ "mosaic.sh", "-s", "Tiergarten" ]
