#!/bin/bash

# This script stops and cleans up the running docker containers for the G1 simulation.
# It stops:
# - g1_sim_controller: The container running the robot control stack.
# - g1_sim_isaaclab: The container running the physics simulator.

CONTAINERS=("g1_sim_controller" "g1_sim_isaaclab")

for CONTAINER in "${CONTAINERS[@]}"; do
    if [ "$(docker ps -aq -f name=${CONTAINER})" ]; then
        echo "Stopping and removing container ${CONTAINER}..."
        docker stop ${CONTAINER} >/dev/null 2>&1
        docker rm ${CONTAINER} >/dev/null 2>&1
        echo "Container ${CONTAINER} stopped."
    else
        echo "Container ${CONTAINER} is not running."
    fi
done
