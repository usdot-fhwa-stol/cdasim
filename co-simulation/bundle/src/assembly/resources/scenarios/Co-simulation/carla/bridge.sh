#!/bin/bash

cd ../../../bridge
#x-terminal-emulator -e
x-terminal-emulator -e python3.7 carla_mosaic_bridge.py --bridge-server-port 8913 --map Town04 net/Town04.net.xml


