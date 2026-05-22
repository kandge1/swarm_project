#!/usr/bin/env bash
xhost +local:docker
docker run --name isaac-sim --entrypoint bash \
  --runtime=nvidia --gpus all \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network=host --rm \
  -it nvcr.io/nvidia/isaac-sim:5.1.0 \
  -c "./isaac-sim.sh"
