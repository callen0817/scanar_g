#!/usr/bin/env python3
"""
elp_camera_node.py — Production ELP 5MP Global Shutter USB Camera ROS 2 Driver
================================================================================
Hardware Driver for ELP 5MP Global Shutter USB Camera (ScanAR C Configuration).
Replaces rolling shutter glasses camera with high-frame-rate, zero-distortion global shutter acquisition.
Publishes low-latency RGB/Mono frames to ROS 2 topic /viture/camera/image_raw and /elp/camera/image_raw.
"""

import os
import sys
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ELPCameraNode(Node):
    def __init__(self):
        super().__init__('elp_camera_node')

        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 60.0)
        self.declare_parameter('frame_id', 'elp_camera_optical_frame')

        self.video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Low-latency QoS profile (Depth 1, Best Effort)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Publishers
        self.pub_image = self.create_publisher(Image, '/viture/camera/image_raw', sensor_qos)
        self.pub_elp_image = self.create_publisher(Image, '/elp/camera/image_raw', sensor_qos)
        self.pub_info = self.create_publisher(CameraInfo, '/elp/camera/camera_info', sensor_qos)

        # Initialize V4L2 capture
        dev_idx = 0
        if self.video_device.startswith('/dev/video'):
            try:
                dev_idx = int(self.video_device.replace('/dev/video', ''))
            except ValueError:
                dev_idx = 0

        self.get_logger().info(f"Initializing ELP 5MP Global Shutter Camera on {self.video_device} ({self.width}x{self.height} @ {self.fps:.0f} FPS)...")
        self.cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open V4L2 camera device {self.video_device}! Retrying with default backend...")
            self.cap = cv2.VideoCapture(dev_idx)

        # Configure hardware V4L2 properties for ELP Global Shutter camera
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))

        self.get_logger().info(f"ELP Global Shutter Hardware Stream Active: {actual_w}x{actual_h} @ {actual_fps:.1f} FPS")

        # Timer for frame polling loop at 60 Hz
        timer_period = 1.0 / max(10.0, self.fps)
        self.timer = self.create_timer(timer_period, self._poll_frame)

    def _poll_frame(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("ELP Camera frame capture missed, retrying...")
            return

        now = self.get_clock().now().to_msg()

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id

            # Publish to both primary camera topic and ELP specific topic
            self.pub_image.publish(msg)
            self.pub_elp_image.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing ELP camera frame: {e}")

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
