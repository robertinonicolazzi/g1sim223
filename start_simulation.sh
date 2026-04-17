#!/bin/bash

# This script starts the full G1 simulation stack.
# It launches two docker containers:
# 1. g1_sim_isaaclab: Runs the Isaac Lab physics simulator and rendering suite.
# 2. g1_sim_controller: Runs the robot control code and communicates with the simulator.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting g1_sim_isaaclab..."
"${SCRIPT_DIR}/run_simulation_bg.sh"

echo "Starting g1_sim_controller..."
"${SCRIPT_DIR}/g1_sim_controller/run.sh"
