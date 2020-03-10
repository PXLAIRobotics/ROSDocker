#!/bin/bash

# Checking if host is aleady part of a swarm
swarm_status=$(docker info  2> /dev/null | grep -E "^[[:space:]]*Swarm:[[:space:]]*.?.?active" \
    | awk -F":" '{ print $2 }' | tr -d [:blank:]) 

if [ "$swarm_status" = "inactive" ]; then
    echo "[STOPPING] THIS HOST ISN'T PART OF A SWARM?!?!"
    exit -1
fi

# It the container running?
if ! docker container inspect ros_full_desktop > /dev/null 2>&1; then
    echo "[STOPPING] CONTAINER ISN'T RUNNING?!?!"
    exit -1
fi

IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ros_full_desktop)
> ./Commands/bin/set_swarm_settings.bash
echo "export ROS_HOSTNAME=$IP" >> ./Commands/bin/set_swarm_settings.bash 
echo "export ROS_IP=$IP" >> ./Commands/bin/set_swarm_settings.bash


# Is it the master or not?
if  docker swarm ca > /dev/null 2>&1; then
    # Master
    echo "export ROS_MASTER_URI=http://localhost:11311" >> ./Commands/bin/set_swarm_settings.bash
else
    # Worker
    read -p "IP on overlay network of the master container: " master_ip
    echo "export ROS_MASTER_URI=http://$master_ip:11311" >> ./Commands/bin/set_swarm_settings.bash
fi
