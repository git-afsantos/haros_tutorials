#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

docker run -it \
    --tmpfs /tmp \
    -p 8080:8080 \
    --mount type=bind,source="$SCRIPT_DIR"/../src,target=/root/ws/src/haros_tutorials,readonly \
    fictibot

