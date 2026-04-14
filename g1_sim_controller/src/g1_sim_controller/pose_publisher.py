#!/usr/bin/env python3
"""
pose_publisher.py
=================
Publishes the robot pose as nav_msgs/Odometry by reading the simulation
robot state from shared memory (isaac_robot_state).

Publications:
  /inorbit/odom_pose  (nav_msgs/Odometry, frame_id: odom, child: base_link)
"""

import time
import threading
import math
import json
import numpy as np
from typing import Optional
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class PosePublisher(Node):

    def __init__(self):
        super().__init__("pose_publisher")

        # ── Robot state tracking ────────────────────────────────────
        self._position = np.array([0.0, 0.0, 0.0])
        self._prev_position = np.array([0.0, 0.0, 0.0])
        self._orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self._linear_vel = np.array([0.0, 0.0, 0.0])
        self._angular_vel = np.array([0.0, 0.0, 0.0])
        self._last_update_time = time.time()
        self._state_lock = threading.Lock()
        self._first_reading = True

        # ── Shared memory (simulation robot state) ──────────────────
        self._robot_state_shm: Optional[shared_memory.SharedMemory] = None
        self._shm_lock = threading.Lock()
        self._shm_connection_attempts = 0
        self._try_connect_shared_memory()

        # ── ROS 2 publisher ─────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(Odometry, "/inorbit/odom_pose", qos)
        self._timer = self.create_timer(0.02, self._publish)  # 50 Hz

        self.get_logger().info(
            "pose_publisher: publishing robot pose on /inorbit/odom_pose (shared memory)"
        )

    # ── Shared memory helpers ───────────────────────────────────────

    def _try_connect_shared_memory(self) -> bool:
        if self._robot_state_shm is not None:
            return True
        try:
            self._robot_state_shm = shared_memory.SharedMemory(name="isaac_robot_state")
            self.get_logger().info("Connected to shared memory: isaac_robot_state")
            return True
        except FileNotFoundError:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts == 1:
                self.get_logger().warn(
                    "Shared memory 'isaac_robot_state' not found – will retry."
                )
            return False
        except Exception as e:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts == 1:
                self.get_logger().warn(f"Failed to connect to shared memory: {e}")
            return False

    def _read_robot_state_from_shm(self) -> Optional[dict]:
        if self._robot_state_shm is None:
            self._shm_connection_attempts += 1
            if self._shm_connection_attempts % 100 == 0:
                self._try_connect_shared_memory()
            return None
        try:
            with self._shm_lock:
                timestamp = int.from_bytes(self._robot_state_shm.buf[0:4], 'little')
                data_len = int.from_bytes(self._robot_state_shm.buf[4:8], 'little')
                if data_len == 0:
                    return None
                json_bytes = bytes(self._robot_state_shm.buf[8:8 + data_len])
                return json.loads(json_bytes.decode('utf-8'))
        except Exception:
            try:
                self._robot_state_shm.close()
            except Exception:
                pass
            self._robot_state_shm = None
            return None

    # ── Publish odometry ────────────────────────────────────────────

    def _publish(self):
        current_time = time.time()
        robot_state = self._read_robot_state_from_shm()

        with self._state_lock:
            dt = current_time - self._last_update_time
            self._last_update_time = current_time

            if robot_state and 'imu_data' in robot_state:
                imu = robot_state['imu_data']
                if len(imu) >= 13:
                    self._prev_position = self._position.copy()

                    self._position[0] = imu[0]
                    self._position[1] = imu[1]
                    self._position[2] = imu[2]

                    qw, qx, qy, qz = imu[3], imu[4], imu[5], imu[6]
                    self._orientation = np.array([qx, qy, qz, qw])

                    self._angular_vel[0] = imu[10]
                    self._angular_vel[1] = imu[11]
                    self._angular_vel[2] = imu[12]

                    if not self._first_reading and dt > 0.001:
                        world_vel = (self._position - self._prev_position) / dt
                        yaw = self._get_yaw_from_quaternion()
                        c, s = np.cos(-yaw), np.sin(-yaw)
                        self._linear_vel[0] = world_vel[0] * c - world_vel[1] * s
                        self._linear_vel[1] = world_vel[0] * s + world_vel[1] * c
                        self._linear_vel[2] = world_vel[2]

                    self._first_reading = False
            else:
                self._linear_vel = np.array([0.0, 0.0, 0.0])
                self._angular_vel = np.array([0.0, 0.0, 0.0])

            pos = self._position.copy()
            ori = self._orientation.copy()
            lin_vel = self._linear_vel.copy()
            ang_vel = self._angular_vel.copy()

        stamp = self.get_clock().now().to_msg()

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]
        msg.pose.pose.orientation.x = ori[0]
        msg.pose.pose.orientation.y = ori[1]
        msg.pose.pose.orientation.z = ori[2]
        msg.pose.pose.orientation.w = ori[3]

        msg.twist.twist.linear.x = lin_vel[0]
        msg.twist.twist.linear.y = lin_vel[1]
        msg.twist.twist.linear.z = lin_vel[2]
        msg.twist.twist.angular.x = ang_vel[0]
        msg.twist.twist.angular.y = ang_vel[1]
        msg.twist.twist.angular.z = ang_vel[2]

        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01

        self._pub.publish(msg)

    # ── Math helpers ────────────────────────────────────────────────

    def _get_yaw_from_quaternion(self) -> float:
        x, y, z, w = self._orientation
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ── Cleanup ─────────────────────────────────────────────────────

    def destroy_node(self):
        if self._robot_state_shm is not None:
            try:
                self._robot_state_shm.close()
                self.get_logger().info("Closed shared memory connection")
            except Exception as e:
                self.get_logger().warn(f"Error closing shared memory: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
