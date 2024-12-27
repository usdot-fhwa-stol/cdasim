#!/bin/bash

#x-terminal-emulator
cd /opt/carma-simulation/bridge

x-terminal-emulator -e python3.7 carla_mosaic_bridge.py --bridge-server-port 8913 -m Town05 /opt/carma-simulation/scenarios/Town05/sumo/Town05.net.xml --step-length 0.1 --tls-manager EVC