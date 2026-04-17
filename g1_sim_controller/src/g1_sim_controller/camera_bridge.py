#!/usr/bin/env python3

import ctypes
import numpy as np
import cv2
from multiprocessing import shared_memory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage


# Inline shared memory header (matches tools/shared_memory_utils.py)
class _SimpleImageHeader(ctypes.LittleEndianStructure):
    _fields_ = [
        ('timestamp', ctypes.c_uint64),
        ('height', ctypes.c_uint32),
        ('width', ctypes.c_uint32),
        ('channels', ctypes.c_uint32),
        ('image_name', ctypes.c_char * 16),
        ('data_size', ctypes.c_uint32),
        ('encoding', ctypes.c_uint32),
        ('quality', ctypes.c_uint32),
    ]


HEADER_SIZE = ctypes.sizeof(_SimpleImageHeader)


class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('camera_name', 'head')
        self.declare_parameter('frame_id', 'front_cam')

        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.raw_pub = self.create_publisher(Image, '/camera/image', 1)
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/camera/image/compressed', 1
        )

        self.shm = None
        self.last_timestamp = 0
        self._last_shm_ts = None
        self._shm_frame_count = 0
        self._shm_log_timer = self.create_timer(1.0, self._log_shm_rate)

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f"CameraBridge started: reading '{self.camera_name}' from shared memory, "
            f"publishing to /camera/image at {publish_rate} Hz"
        )

    def _read_frame(self):
        """Read frame from shared memory. Returns (header, payload) or (None, None)."""
        shm_name = f"isaac_{self.camera_name}_image_shm"

        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=shm_name)
            except FileNotFoundError:
                return None, None

        header_data = bytes(self.shm.buf[:HEADER_SIZE])
        header = _SimpleImageHeader.from_buffer_copy(header_data)

        if header.timestamp <= self.last_timestamp:
            return None, None

        # Track new frames arriving in shared memory
        self._shm_frame_count += 1
        self._last_shm_ts = header.timestamp

        payload = bytes(self.shm.buf[HEADER_SIZE:HEADER_SIZE + header.data_size])
        self.last_timestamp = header.timestamp
        return header, payload

    def _log_shm_rate(self):
        count = self._shm_frame_count
        self._shm_frame_count = 0
        # self.get_logger().info(f"[SHM] new frames last second: {count} Hz")

    def timer_callback(self):
        header, payload = self._read_frame()
        if header is None:
            return

        stamp = self.get_clock().now().to_msg()

        # Decode to BGR image
        if header.encoding == 1:  # JPEG
            encoded = np.frombuffer(payload, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                return
        else:  # RAW
            image = np.frombuffer(payload, dtype=np.uint8)
            expected = header.height * header.width * header.channels
            if image.size != expected:
                return
            image = image.reshape(header.height, header.width, header.channels)

        raw_msg = Image()
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id
        raw_msg.height = image.shape[0]
        raw_msg.width = image.shape[1]
        raw_msg.encoding = 'bgr8'
        raw_msg.is_bigendian = False
        raw_msg.step = image.shape[1] * image.shape[2]
        raw_msg.data = image.tobytes()
        self.raw_pub.publish(raw_msg)

        # Publish compressed (JPEG passthrough or re-encode)
        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.header.frame_id = self.frame_id
        comp_msg.format = 'jpeg'
        if header.encoding == 1:
            comp_msg.data = payload
        else:
            ok, buf = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 30])
            if not ok:
                return
            comp_msg.data = buf.tobytes()
        self.compressed_pub.publish(comp_msg)

    def destroy_node(self):
        if self.shm is not None:
            self.shm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
