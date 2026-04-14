#!/usr/bin/env bash
# ─── save_map.sh ──────────────────────────────────────────────────────────────
# Saves the current SLAM map from a running slam_toolbox inside the g1_slam
# Docker container on the G1 robot (accessed via SSH), then copies the map
# files back to the host computer.
#
# Usage:
#   ./save_map.sh [MAP_NAME]
#
# Example:
#   ./save_map.sh                   # → saves as "map_YYYYMMDD_HHMMSS"
#   ./save_map.sh my_office_map     # → saves as "my_office_map"
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

# ── Configuration ─────────────────────────────────────────────────────────────
SSH_USER="rc"
SSH_HOST="35.232.79.9"
SSH_PORT="22"
SSH_KEY="$(cd "$(dirname "$0")" && pwd)/robotic_crew_rsa_gcp"
DOCKER_CONTAINER="g1_sim_controller"

# Directory on the remote (SSH) machine where maps are stored
REMOTE_MAP_DIR="/home/rc/maps"

# Path inside the Docker container where the map is temporarily saved
# (must be a path that exists or can be created inside the container)
CONTAINER_MAP_DIR="/tmp/maps"

# Local directory on the host computer where maps are copied to
LOCAL_MAP_DIR="$(cd "$(dirname "$0")" && pwd)/g1_sim_controller/maps"

# ── Map name ──────────────────────────────────────────────────────────────────
MAP_NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              SLAM Map Save & Download Script                ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Map name:  ${MAP_NAME}"
echo "║  Remote:    ${SSH_USER}@${SSH_HOST}:${SSH_PORT}"
echo "║  Container: ${DOCKER_CONTAINER}"
echo "║  Remote dir: ${REMOTE_MAP_DIR}"
echo "║  Local dir:  ${LOCAL_MAP_DIR}"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Helper: run a command on the remote machine via SSH
ssh_cmd() {
    ssh -o StrictHostKeyChecking=no -i "${SSH_KEY}" \
        -p "${SSH_PORT}" "${SSH_USER}@${SSH_HOST}" "$@"
}

# Helper: copy files from the remote machine using scp
scp_from() {
    scp -o StrictHostKeyChecking=no -i "${SSH_KEY}" \
        -P "${SSH_PORT}" "$1" "$2"
}

# ── Step 1: Save the serialized pose graph inside the Docker container ────────
echo "▶ [1/5] Saving serialized pose graph (slam_toolbox) inside container..."
ssh_cmd "docker exec ${DOCKER_CONTAINER} bash -c '\
    mkdir -p ${CONTAINER_MAP_DIR} && \
    source /opt/ros/humble/setup.bash && \
    source /ros_ws/install/setup.bash && \
    ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        \"{filename: \"'\"'\"'${CONTAINER_MAP_DIR}/${MAP_NAME}'\"'\"'\"}\" \
'"

echo "   ✔ Serialized pose graph saved."

# ── Step 2: Save OccupancyGrid as PGM+YAML using map_saver_cli ───────────────
echo "▶ [2/5] Saving OccupancyGrid (.pgm + .yaml) inside container..."
ssh_cmd "docker exec ${DOCKER_CONTAINER} bash -c '\
    source /opt/ros/humble/setup.bash && \
    source /ros_ws/install/setup.bash && \
    ros2 run nav2_map_server map_saver_cli \
        -f ${CONTAINER_MAP_DIR}/${MAP_NAME} \
        --ros-args -p save_map_timeout:=2000.0 \
' "

if ssh_cmd "docker exec ${DOCKER_CONTAINER} [ -f ${CONTAINER_MAP_DIR}/${MAP_NAME}.pgm ]"; then
    echo "   ✔ OccupancyGrid .pgm and .yaml saved."
else
    echo "   ❌ OccupancyGrid save failed map_saver_cli error."
fi

# ── Step 3: Copy map files from Docker container to the remote host ───────────
echo "▶ [3/5] Copying map files from container to remote host..."
ssh_cmd "\
    mkdir -p ${REMOTE_MAP_DIR} && \
    docker cp ${DOCKER_CONTAINER}:${CONTAINER_MAP_DIR}/. ${REMOTE_MAP_DIR}/ \
"
echo "   ✔ Map files copied to remote host: ${REMOTE_MAP_DIR}"

# ── Step 4: List saved files on the remote ────────────────────────────────────
echo "▶ [4/5] Files on the remote machine:"
ssh_cmd "ls -lh ${REMOTE_MAP_DIR}/${MAP_NAME}*" 2>/dev/null || \
ssh_cmd "ls -lh ${REMOTE_MAP_DIR}/" 2>/dev/null || echo "   --- could not list remote files ---"

# ── Step 5: Copy map files from the remote host to the local machine ──────────
echo "▶ [5/5] Downloading map files to local machine..."
mkdir -p "${LOCAL_MAP_DIR}"

scp_from "${SSH_USER}@${SSH_HOST}:${REMOTE_MAP_DIR}/${MAP_NAME}*" "${LOCAL_MAP_DIR}/"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " ✅  Map saved successfully!"
echo ""
echo " Local files:"
ls -lh "${LOCAL_MAP_DIR}/${MAP_NAME}"* 2>/dev/null || echo "   --- no files matched - check above for errors ---"
echo ""
echo " Files expected:"
echo "   • ${MAP_NAME}.posegraph   - serialized pose graph for localization"
echo "   • ${MAP_NAME}.data        - pose graph data"
echo "   • ${MAP_NAME}.pgm         - occupancy grid image if map_saver ran"
echo "   • ${MAP_NAME}.yaml        - occupancy grid metadata if map_saver ran"
echo "═══════════════════════════════════════════════════════════════"
