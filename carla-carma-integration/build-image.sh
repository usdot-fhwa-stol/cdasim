#!/bin/sh
docker build -t carla-carma-integration -f Dockerfile . "$@"
