#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting g1_sim_isaaclab..."
"${SCRIPT_DIR}/run_simulation_bg.sh"

echo "Starting g1_sim_controller..."
"${SCRIPT_DIR}/g1_sim_controller/run.sh"
