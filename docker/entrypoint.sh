#!/bin/bash

if [[ -z "$@" ]]; then
    source ~/.base-image/env.sh; cd /opt/carma-simulation; exec "bash"
else
    source ~/.base-image/env.sh; cd /opt/carma-simulation; exec "$@"
fi

