FROM pxl_air_base_ubuntu18.04:latest

USER root

RUN apt-get update \
        && apt-get upgrade -y \
        && apt-get install -y lsb-release bash-completion \
        && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
        && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \ 
        && apt-get update \
        && apt-get install -y ros-melodic-desktop-full \
        && apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential \
        && apt-get install -y ros-melodic-turtlebot3* \
        && apt-get install -y ros-melodic-gmapping \
        && apt-get install -y python-rosdep \
        && apt-get install -y python-pip \
        && rosdep init \
        && apt-get autoremove -y \
        && rm -rf /var/lib/apt/lists/* 
USER user
WORKDIR /home/user

RUN rosdep update \
        && echo "source /opt/ros/melodic/setup.bash" >> /home/user/.bashrc

RUN mkdir -p /home/user/Projects/turtlebot3_extra_ws/src 
WORKDIR /home/user/Projects/turtlebot3_extra_ws/src
COPY --chown=user:user ./m-explore.tar.gz /home/user/Projects/turtlebot3_extra_ws/src/m-explore.tar.gz
RUN tar xzvf ./m-explore.tar.gz && rm ./m-explore.tar.gz
COPY --chown=user:user ./turtlebot3_mas_teleop.tar.gz /home/user/Projects/turtlebot3_extra_ws/src/turtlebot3_mas_teleop.tar.gz
RUN tar xzvf ./turtlebot3_mas_teleop.tar.gz && rm ./turtlebot3_mas_teleop.tar.gz
WORKDIR /home/user/Projects/turtlebot3_extra_ws/
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/user/Projects/turtlebot3_extra_ws; catkin_make'
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/user/Projects/turtlebot3_extra_ws; catkin_make'
RUN echo "source /home/user/Projects/turtlebot3_extra_ws/devel/setup.bash --extend" >> /home/user/.bashrc

WORKDIR /home/user
RUN mkdir -p Projects/catkin_ws/src 
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/user/Projects/catkin_ws; catkin_make'
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/user/Projects/catkin_ws; catkin_make'

RUN echo "source /home/user/Projects/catkin_ws/devel/setup.bash --extend" >> /home/user/.bashrc

STOPSIGNAL SIGTERM

ENTRYPOINT ["/scripts/init_commands.sh"]
CMD sudo service ssh start && /bin/bash
