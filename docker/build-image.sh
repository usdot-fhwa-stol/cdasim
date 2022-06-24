#!/bin/sh
cd ..
docker build -t usdotfhwastol/carma-xil-cosimulation:1.0.0-beta -f Dockerfile . "$@"
