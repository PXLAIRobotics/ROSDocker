#!/bin/bash

if ! command -v glxinfo &> /dev/null
then
    echo "glxinfo command  not found! Execute \'sudo apt install mesa-utils\' to install it."
    exit
fi

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`


if [ $vendor == "NVIDIA" ]; then
    docker run -it --rm \
        --name base_nvidia_image \
        --hostname base_nvidia_image \
        --device /dev/snd \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v `pwd`/../Projects:/home/user/Projects \
        -env="XAUTHORITY=$XAUTH" \
        --gpus all \
        pxl_air_base_ubuntu18.04:latest \
        bash
else
    docker run --privileged -it --rm \
        --name base_opengl_image \
        --hostname base_opengl_image \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        -v `pwd`/../Projects:/home/user/Projects \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        pxl_air_base_ubuntu18.04:latest \
        bash
fi
