#!/bin/bash

# Checking if host is aleady part of a swarm
swarm_status=$(docker info  2> /dev/null | grep -E "^[[:space:]]*Swarm:[[:space:]]*.?.?active" \
    | awk -F":" '{ print $2 }' | tr -d [:blank:]) 

if [ "$swarm_status" = "active" ]; then
    echo "[STOPPING] THIS HOST IS ALREADY PART OF A SWARM?!?!"
    exit -1
fi

# Make a Docker swarm and make this host its master.
if ! docker swarm init > docker_swarm_master.log; then
    echo "[STOPPING] COULDN'T CREATE A SWARM!"
    exit -1
else
    echo "SWARM SUCCESSFULLY CREATED."
fi

# Create an attachable overlay network
if ! docker network create -d overlay --attachable my_ros_overlay_network > docker_swarm_network.log; then
    echo "[STOPPING] COULDN'T CREATE AN OVERLAY NETWORK!"
    exit -1
else
    echo "OVERLAY NETWORK SUCCESSFULLY CREATED."
fi

# Show the output (This will include the join command.)
echo "SHOWING JOIN COMMAND:"
cat ./docker_swarm_master.log
