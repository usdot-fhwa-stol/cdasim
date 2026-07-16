 #   Copyright (C) 2025 LEIDOS.
 #
 #   Licensed under the Apache License, Version 2.0 (the "License"); you may not
 #   use this file except in compliance with the License. You may obtain a copy of
 #   the License at
 #
 #   http://www.apache.org/licenses/LICENSE-2.0
 #
 #   Unless required by applicable law or agreed to in writing, software
 #   distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 #   WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 #   License for the specific language governing permissions and limitations under
 #   the License.
 
FROM ubuntu:22.04

MAINTAINER Leidos, Inc.
LABEL Description="Dockerised Xml-rpc server for CDASIM"

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

COPY docker/ /home/carma/src/docker
COPY PythonAPI/ /home/carma/src/PythonAPI
COPY carla_v0.10_xmlrpc_server.py /home/carma/src

RUN chmod +x /home/carma/src/docker/install.sh
RUN /home/carma/src/docker/install.sh

WORKDIR /home/carma/src