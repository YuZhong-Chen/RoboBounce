#!/bin/bash -e

# Get the directory of this script.
# Reference: https://stackoverflow.com/q/59895
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)"
cd "$SCRIPT_DIR"

# Reference:
# - https://hub.docker.com/r/universalrobots/ursim_cb3

# Build the image
if ! docker image ls | grep -q ursim; then
    docker build -t ursim:latest .
fi

# Create the network
if ! docker network ls | grep -q ursim_net; then
    docker network create --subnet=192.168.56.0/24 ursim_net
fi

# Run the container
docker run --rm -it \
    --net ursim_net \
    --ip 192.168.56.101 \
    -p 5900:5900 \
    -p 6080:6080 \
    -e ROBOT_MODEL=UR5 \
    ursim:latest
