#!/bin/bash

# 2. Compile Java
javac -h . example/com/MyBridgeClass.java

# 3. Run Java
java -Djava.library.path=./cpp/lib example.com.MyBridgeClass