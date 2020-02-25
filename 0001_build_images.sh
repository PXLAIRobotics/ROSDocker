#!/bin/bash

vendor=`glxinfo | grep vendor | grep OpenGL | awk '{ print $4 }'`


if [ $vendor == "NVIDIA" ]; then
    (cd ./01a_nvidia_ubuntu18.04; ./01_build_image.sh)
else
    (cd ./01b_dri_ubuntu18.04; ./01_build_image.sh)
fi

(cd ./02_opencv; ./01_build_image.sh)
(cd ./03_ros_full_desktop; ./01_build_image.sh)
