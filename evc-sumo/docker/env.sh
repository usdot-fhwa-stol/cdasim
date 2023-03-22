#!/bin/bash

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

# Set environment variables here.

export NS3_HOME="/opt/ns-3/"
export JAVA_HOME="/usr/lib/jvm/java-11-jdk-amd64"
export SUMO_HOME="/opt/sumo/sumo-1_12_0"
export CARLA_HOME="/opt/carla"
export CARMA_SIMULATION_HOME="/opt/carma-simulation"
export PATH="/usr/local/share/sumo/bin:/usr/local/share/sumo/tools:$PATH"
export PATH="/opt/maven/bin:$PATH"
export PATH="/opt/carma-simulation:$PATH"
