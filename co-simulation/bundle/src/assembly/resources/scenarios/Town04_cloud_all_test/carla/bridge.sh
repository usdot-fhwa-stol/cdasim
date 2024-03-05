#!/bin/bash

#x-terminal-emulator
cd /opt/carma-simulation/bridge

x-terminal-emulator -e python3.7 carla_mosaic_bridge.py --bridge-server-port 8913 -m Town04 /opt/carma-simulation/scenarios/Town04_cloud_all_test/sumo/Town04.net.xml --step-length 0.1 --tls-manager EVC
