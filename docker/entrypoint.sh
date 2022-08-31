#!/bin/bash

if [[ -z "$@" ]]; then
    source ~/.base-image/env.sh; exec "bash"
else
    source ~/.base-image/env.sh; exec "$@"
fi

