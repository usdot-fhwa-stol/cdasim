# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM ubuntu:18.04

ARG CARLA_VERSION=0.9.10

RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends --yes --quiet \
        libpng16-16 \
        libtiff5 \
        libjpeg8 \
        build-essential \
        wget \
        git \
        python3.7 \
        python3.7-dev \
        python3-pip \
        libxerces-c-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN wget -qO- "https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_$CARLA_VERSION.tar.gz" \
        | tar -xz PythonAPI/carla \
    && mkdir -p /app \
    && mv PythonAPI/carla /app \
    && rm -rf *

COPY . .
RUN ./install_carma_scenario_runner --prefix /app $CARLA_VERSION
WORKDIR /app/scenario_runner
RUN wget https://raw.githubusercontent.com/usdot-fhwa-stol/carma-carla-integration/develop/carma-carla-integration/carla-health-poll/src/wait_for_carla.py
ENV PYTHONPATH "/app/carla/agents:/app/carla:/app/carla/dist/carla-$CARLA_VERSION-py3.7-linux-x86_64.egg"

ENTRYPOINT ["python3", "scenario_runner.py"]
