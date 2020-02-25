FROM pxl_air_base_ubuntu18.04:latest

USER root

RUN set -x \
        && apt-get update \
        && apt-get upgrade -y \
        && apt-get install -y gcc clang g++ cmake \
        && apt-get install -y ffmpeg \
        && apt-get install -y python3-tk \
        && apt-get install -y python3-opencv \
        && apt-get install -y libopencv-dev	       \
        && apt-get install -y libopencv-highgui3.2     \
        && apt-get install -y libopencv-imgproc-dev    \
        && apt-get install -y libopencv-photo3.2       \
        && apt-get install -y libopencv-stitching-dev  \
        && apt-get install -y libopencv-video-dev      \
        && apt-get install -y libopencv-viz3.2	       \
        && apt-get install -y libopencv-contrib3.2     \
        && apt-get install -y libopencv-features2d3.2  \
        && apt-get install -y libopencv-highgui-dev    \
        && apt-get install -y libopencv-ml3.2	       \
        && apt-get install -y libopencv-photo-dev      \
        && apt-get install -y libopencv-superres3.2    \
        && apt-get install -y libopencv-videoio3.2     \
        && apt-get install -y libopencv-viz-dev	       \
        && apt-get install -y libopencv-apps0d	       \
        && apt-get install -y libopencv-contrib-dev    \
        && apt-get install -y libopencv-features2d-dev \
        && apt-get install -y libopencv-imgcodecs3.2   \
        && apt-get install -y libopencv-ml-dev	       \
        && apt-get install -y libopencv-shape3.2       \
        && apt-get install -y libopencv-superres-dev   \
        && apt-get install -y libopencv-videoio-dev    \
        && apt-get install -y libopencv-apps-dev       \
        && apt-get install -y libopencv-core3.2	       \
        && apt-get install -y libopencv-flann3.2       \
        && apt-get install -y libopencv-imgcodecs-dev  \
        && apt-get install -y libopencv-objdetect3.2   \
        && apt-get install -y libopencv-shape-dev      \
        && apt-get install -y libopencv-ts-dev	       \
        && apt-get install -y libopencv-videostab3.2   \
        && apt-get install -y libopencv-calib3d3.2     \
        && apt-get install -y libopencv-core-dev       \
        && apt-get install -y libopencv-flann-dev      \
        && apt-get install -y libopencv-imgproc3.2     \
        && apt-get install -y libopencv-objdetect-dev  \
        && apt-get install -y libopencv-stitching3.2   \
        && apt-get install -y libopencv-video3.2       \
        && apt-get install -y libopencv-videostab-dev  \
        && apt-get autoremove -y \
        && rm -rf /var/lib/apt/lists/* 

USER user
WORKDIR /home/user
RUN sudo usermod -a -G video user

STOPSIGNAL SIGTERM

ENTRYPOINT ["/scripts/init_commands.sh"]
CMD sudo service ssh start && /bin/bash
