#!/bin/bash

docker exec -it g1_sim_controller bash -c "source /opt/ros/humble/setup.bash && ros2 run rviz2 rviz2 -d /ros_ws/rviz/pointcloud.rviz"