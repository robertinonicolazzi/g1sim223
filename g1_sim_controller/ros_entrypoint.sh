#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash"

rm -rf /ros_ws/build
rm -rf /ros_ws/install

colcon build --symlink-install
source "/ros_ws/install/setup.bash"

# SLAM_MODE: mapping (default), localization, none
SLAM_MODE="${SLAM_MODE:-localization}"

case "$SLAM_MODE" in
  localization)
    LAUNCH_ARGS="mapping:=false localization:=true"
    ;;
  none)
    LAUNCH_ARGS="mapping:=false localization:=false"
    ;;
  *)
    LAUNCH_ARGS="mapping:=true localization:=false"
    ;;
esac

exec ros2 launch g1_sim_controller bringup.launch.py $LAUNCH_ARGS
