#!/bin/bash

xhost +local:docker

# --device=/dev/video0:/dev/video0
# For non root usage:
# RUN sudo usermod -a -G video developer

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`

if [ $vendor == "NVIDIA" ]; then
    docker run -it \
        --device /dev/snd \
        --rm \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -v `pwd`/../ExampleCode:/home/user/ExampleCode \
        -env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --runtime=nvidia \
        --device=/dev/video0:/dev/video0 \
        pxl_air_opencv:latest \
        bash
else
    docker run --privileged -it --rm \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        -v `pwd`/../ExampleCode:/home/user/ExampleCode \
        --device=/dev/dri:/dev/dri \
        --env="DISPLAY=$DISPLAY" \
        -e "TERM=xterm-256color" \
        --cap-add SYS_ADMIN --device /dev/fuse \
        pxl_air_opencv:latest \
        bash
fi
