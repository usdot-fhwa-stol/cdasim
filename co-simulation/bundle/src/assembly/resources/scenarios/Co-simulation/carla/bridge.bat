@ECHO OFF
cd ../../../bridge
python carla_mosaic_bridge.py --bridge-server-port 8913 --map Town04 net/Town04.net.xml --step-length 0.1
