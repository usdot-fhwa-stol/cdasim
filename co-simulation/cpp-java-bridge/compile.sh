#!/bin/bash
# prerequisites
# 1. assumes carla is cloned into the cpp-java-bridge folder
# 2. carla/LibCarla/.../TrafficManagerServer.h line 51 needed to be changed to "} catch(std::out_of_range const&) {"

# 1. Compile Java
javac -h . example/com/MyBridgeClass.java

# 2.a Compile cpp
cd cpp

make build

# needed following to run successfully TODO
# export LD_LIBRARY_PATH=/workspaces/carma/carma-simulation/co-simulation/cpp-java-bridge/cpp/libcarla-install/lib:$LD_LIBRARY_PATH 
cd ..

# 3. Run Java
java -Djava.library.path=./cpp example.com.MyBridgeClass