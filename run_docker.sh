#!/bin/bash

docker run --gpus all -it --rm --network host --ipc host \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics,display \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /etc/vulkan/icd.d:/etc/vulkan/icd.d:ro \
  -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro \
  -v ./assets:/home/code/unitree_sim_isaaclab/assets \
  -v ./sim_main.py:/home/code/unitree_sim_isaaclab/sim_main.py \
  -v ./run_simulation.sh:/home/code/unitree_sim_isaaclab/run_simulation.sh \
  -v ./send_commands_keyboard.py:/home/code/unitree_sim_isaaclab/send_commands_keyboard.py \
  -v ./dds:/home/code/unitree_sim_isaaclab/dds \
  -v ./action_provider:/home/code/unitree_sim_isaaclab/action_provider \
  -v ./tasks:/home/code/unitree_sim_isaaclab/tasks \
  inorbit_g1_sim:latest /bin/bash
