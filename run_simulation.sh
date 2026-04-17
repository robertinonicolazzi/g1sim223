#!/bin/bash

TELEIMAGER_LOG_LEVEL=WARNING python sim_main.py \
    --task Isaac-Move-Cylinder-G129-Dex1-Wholebody \
    --enable_cameras \
    --robot_type g129 \
    --headless \
    --livestream 2 \
    --enable_inspire_dds \
    --camera_include "front_camera" \
    --render_interval 4
