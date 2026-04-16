#!/usr/bin/env bash
# ─── save_map_local.sh ────────────────────────────────────────────────────────
# Saves the current SLAM map from a running slam_toolbox inside the g1_slam
# Docker container running LOCALLY on this machine, then copies the map
# files from the container to a local directory.
#
# This script is meant to be run directly on the computer that has the
# Docker container running (no SSH required).
#
# Usage:
#   ./save_map_local.sh [MAP_NAME]
#
# Example:
#   ./save_map_local.sh                   # → saves as "map_YYYYMMDD_HHMMSS"
#   ./save_map_local.sh my_office_map     # → saves as "my_office_map"
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── Configuration ─────────────────────────────────────────────────────────────
DOCKER_CONTAINER="g1_sim_controller"

# Path inside the Docker container where the map is temporarily saved
CONTAINER_MAP_DIR="/tmp/maps"

# Local directory on this machine where maps are copied to
LOCAL_MAP_DIR="$(cd "$(dirname "$0")" && pwd)/g1_sim_controller/maps"

# ── Map name ──────────────────────────────────────────────────────────────────
MAP_NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║           SLAM Map Save (Local Docker) Script               ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Map name:   ${MAP_NAME}"
echo "║  Container:  ${DOCKER_CONTAINER}"
echo "║  Local dir:  ${LOCAL_MAP_DIR}"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ── Pre-check: ensure the Docker container is running ─────────────────────────
if ! docker inspect --format='{{.State.Running}}' "${DOCKER_CONTAINER}" 2>/dev/null | grep -q "true"; then
    echo "❌ Docker container '${DOCKER_CONTAINER}' is not running."
    echo "   Available containers:"
    docker ps --format "   • {{.Names}} ({{.Image}})" 2>/dev/null || echo "   --- could not list containers ---"
    exit 1
fi

# ── Step 1: Save the serialized pose graph inside the Docker container ────────
echo "▶ [1/4] Saving serialized pose graph (slam_toolbox) inside container..."
docker exec "${DOCKER_CONTAINER}" bash -c "\
    mkdir -p ${CONTAINER_MAP_DIR} && \
    source /opt/ros/humble/setup.bash && \
    source /ros_ws/install/setup.bash && \
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        \"{filename: '${CONTAINER_MAP_DIR}/${MAP_NAME}'}\" \
"

echo "   ✔ Serialized pose graph saved."

# ── Step 2: Save OccupancyGrid as PGM+YAML using map_saver_cli ───────────────
echo "▶ [2/4] Saving OccupancyGrid (.pgm + .yaml) inside container..."
docker exec "${DOCKER_CONTAINER}" bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /ros_ws/install/setup.bash && \
    ros2 run nav2_map_server map_saver_cli \
        -f ${CONTAINER_MAP_DIR}/${MAP_NAME} \
        --ros-args -p save_map_timeout:=2000.0 \
"

if docker exec "${DOCKER_CONTAINER}" [ -f "${CONTAINER_MAP_DIR}/${MAP_NAME}.pgm" ]; then
    echo "   ✔ OccupancyGrid .pgm and .yaml saved."
else
    echo "   ⚠ OccupancyGrid save may have failed (no .pgm found). Continuing..."
fi

# ── Step 3: Copy map files from Docker container to local machine ─────────────
echo "▶ [3/4] Copying map files from container to local directory..."
mkdir -p "${LOCAL_MAP_DIR}"
docker cp "${DOCKER_CONTAINER}:${CONTAINER_MAP_DIR}/." "${LOCAL_MAP_DIR}/"
echo "   ✔ Map files copied to: ${LOCAL_MAP_DIR}"

# ── Step 4: List saved files locally ──────────────────────────────────────────
echo "▶ [4/4] Local files:"
ls -lh "${LOCAL_MAP_DIR}/${MAP_NAME}"* 2>/dev/null || echo "   --- no files matched - check above for errors ---"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " ✅  Map saved successfully!"
echo ""
echo " Local files:"
ls -lh "${LOCAL_MAP_DIR}/${MAP_NAME}"* 2>/dev/null || echo "   --- no files matched ---"
echo ""
echo " Files expected:"
echo "   • ${MAP_NAME}.posegraph   - serialized pose graph for localization"
echo "   • ${MAP_NAME}.data        - pose graph data"
echo "   • ${MAP_NAME}.pgm         - occupancy grid image (if map_saver ran)"
echo "   • ${MAP_NAME}.yaml        - occupancy grid metadata (if map_saver ran)"
echo "═══════════════════════════════════════════════════════════════"
