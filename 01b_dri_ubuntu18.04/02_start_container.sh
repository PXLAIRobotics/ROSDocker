#!/bin/bash

xhost +local:docker

# --device=/dev/video0:/dev/video0
# For non root usage:
# RUN sudo usermod -a -G video developer

docker run --privileged -it --rm \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    -v `pwd`/../Projects:/home/user/Projects \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    -e "TERM=xterm-256color" \
    --cap-add SYS_ADMIN --device /dev/fuse \
    pxl_air_base_ubuntu18.04:latest \
    bash
