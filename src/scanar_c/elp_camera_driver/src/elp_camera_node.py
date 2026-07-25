#!/usr/bin/env python3
"""
elp_camera_node.py — Production ELP 5MP Global Shutter USB Camera ROS 2 Driver
================================================================================
Hardware Driver for ELP 5MP Global Shutter USB Camera (Model: ELP-USBGS5MP01-L170).
Used in ScanAR C product profile for high-speed, zero-motion-blur 3D reconstruction.

Key Parameters:
- Format: MJPEG
- Resolution: 1920x1080 (or 1280x720)
- Target Frame Rate: 30.0 FPS / 60.0 FPS
- Device Binding: /dev/sensors/camera_elp symlink with dynamic /dev/video* fallback search.
- Exposure & WB: Manual Exposure, Manual White Balance, Fixed Focus.
"""

import os
import sys
import time
import glob
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

class ELPCameraNode(Node):
    def __init__(self):
        super().__init__('elp_camera_driver_node')

        # Single-Instance Process Lock
        try:
            import sys
            sys.path.append('/home/scanarstereo/scanAR_G/src/scanar_core')
            from scanar_profiles.process_lock import ProcessLock
            self._lock = ProcessLock('camera')
        except Exception as e:
            self.get_logger().warn(f"[ELP Camera Driver] Lock warning: {e}")

        # Parameters
        self.declare_parameter('video_device', '/dev/sensors/camera_elp')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'elp_camera_optical_frame')

        self.video_device_param = self.get_parameter('video_device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Low-latency QoS profile (Depth 1, Best Effort)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers
        self.pub_elp_image = self.create_publisher(Image, '/elp/camera/image_raw', sensor_qos)
        self.pub_info = self.create_publisher(CameraInfo, '/elp/camera/camera_info', sensor_qos)

        # Find camera device
        active_device, dev_idx = self._find_camera_device()
        self.get_logger().info(f"[ELP Camera Driver] Opening ELP 5MP Global Shutter Camera on {active_device} (Index {dev_idx})...")

        self.cap = cv2.VideoCapture(dev_idx)

        if not self.cap.isOpened():
            self.get_logger().error(f"[ELP Camera Driver] Failed to open {active_device}! Trying index 1 fallback...")
            self.cap = cv2.VideoCapture(1)

        # Configure hardware V4L2 properties for ELP Global Shutter camera
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Set manual exposure & white balance if supported by hardware driver
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # 1 = Manual Exposure
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)       # 0 = Manual White Balance
        except Exception:
            pass

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))

        self.get_logger().info(f"[ELP Camera Driver] ELP Global Shutter Stream Active: {actual_w}x{actual_h} @ {actual_fps:.1f} FPS (MJPEG)")

        # Polling timer at max frame rate
        timer_period = 1.0 / max(10.0, self.fps)
        self.timer = self.create_timer(timer_period, self._poll_frame)

    def _find_camera_device(self):
        # 1. Try persistent udev symlink if present
        if os.path.exists(self.video_device_param):
            real_path = os.path.realpath(self.video_device_param)
            try:
                dev_idx = int(real_path.replace('/dev/video', ''))
                return self.video_device_param, dev_idx
            except ValueError:
                pass

        # 2. Search available /dev/video* devices
        video_devs = sorted(glob.glob('/dev/video*'))
        for dev_path in video_devs:
            try:
                idx = int(dev_path.replace('/dev/video', ''))
                test_cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if test_cap.isOpened():
                    ret, frame = test_cap.read()
                    test_cap.release()
                    if ret and frame is not None:
                        self.get_logger().info(f"[ELP Camera Driver] Detected active video device at {dev_path} (Index {idx})")
                        return dev_path, idx
            except Exception:
                continue

        # Fallback to index 1 or 0
        return '/dev/video1', 1

    def _poll_frame(self):
        if not hasattr(self, 'cap') or self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        now = self.get_clock().now().to_msg()

        try:
            if len(frame.shape) == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif len(frame.shape) == 3 and frame.shape[2] == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            elif len(frame.shape) == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            msg = Image()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()

            # Publish exclusively to ELP camera topic
            self.pub_elp_image.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[ELP Camera Driver] Error publishing frame: {type(e).__name__} - {e}")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ELPCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
