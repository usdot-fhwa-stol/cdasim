#  Copyright (C) 2023 LEIDOS.
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

FROM ubuntu:20.04

# Set environment variables for non-interactive installation
ARG EVC_TOKEN="NULL"
ENV evc_token=${EVC_TOKEN}
ENV SUMO_HOME /usr/share/sumo
RUN apt-get update && apt-get install -y sudo

# Create a group with GID 1000
RUN groupadd -g 1000 carma

# Create a user with UID 1000 and GID 1000 for running the browser
RUN useradd -m -u 1000 -g 1000 carma
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Set the user to be used for the following commands
WORKDIR /home/carma
COPY --chown=carma:carma /docker ./docker
COPY --chown=carma:carma /src ./src
COPY --chown=carma:carma /test ./test

RUN docker/install.sh evc_token
USER carma
# Set the working directory for the browser user
WORKDIR /home/carma/src

# Start the virtual framebuffer (Xvfb) and xterm
CMD x-terminal-emulator -e python3.8 evc_sumo_bridge.py --asc3app-path "../evcfile/asc3app-application.zip"