#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math
import os

import fcntl
import struct

import threading
import queue

from rclpy.qos import qos_profile_sensor_data
# Import the ScanAR Hardware Abstraction Layer
from scanar_hal import MockVitureHAL
from viture_driver.product_capabilities import get_product_capability

class VitureDriverNode(Node):
    def __init__(self):
        super().__init__('viture_driver')
        
        # Declare parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('publish_rate', 30.0) # Hz for camera ROS topic
        self.declare_parameter('imu_rate', 100.0) # Hz for IMU
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('product', 'scanar_g')

        self.video_device = self.get_parameter('video_device').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_rate = self.get_parameter('imu_rate').value
        self.sim_mode = self.get_parameter('sim_mode').value
        self.product = self.get_parameter('product').value
        self.capability = get_product_capability(self.product)

        # Initialize bridge
        self.bridge = CvBridge()

        # Direct-to-disk recording queues & state
        self.active_dir = ""
        self.recording_frame_idx = 0
        self.recorder_queue = queue.Queue(maxsize=100)
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Publishers & Subscriptions
        self.image_pub = self.create_publisher(Image, '/viture/camera/image_raw', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/viture/imu', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/viture/pose', 10)
        self.fps_pub = self.create_publisher(Float32, '/viture/camera/fps', 10)
        self.imu_hz_pub = self.create_publisher(Float32, '/viture/imu/rate', 10)
        self.status_pub = self.create_publisher(String, '/viture/camera/status', 10)

        self.dir_sub = self.create_subscription(String, '/scanar/session/active_directory', self._cb_active_dir, 10)

        # HAL & Hardware configurations
        self.hal = None
        self.cap = None

        if self.sim_mode:
            self.get_logger().info("Initializing VITURE HAL Driver in DEVELOPER SIMULATION mode.")
            self.hal = MockVitureHAL()
        else:
            # Auto-probe UVC camera indices [0, 1, 2] if target device fails
            candidates = [self.video_device] + [idx for idx in [0, 1, 2] if idx != self.video_device]
            self.cap = None

            for dev_idx in candidates:
                dev_path = f"/dev/video{dev_idx}"
                if os.path.exists(dev_path):
                    # Verify V4L2 capability to filter out metadata nodes
                    is_capture_device = False
                    try:
                        fd = os.open(dev_path, os.O_RDWR)
                        buf = b'\x00' * 104
                        res = fcntl.ioctl(fd, 0x80685600, buf) # VIDIOC_QUERYCAP
                        _, _, _, _, capabilities, device_caps, _, _, _ = struct.unpack('16s 32s 32s I I I I I I', res)
                        caps = device_caps if device_caps != 0 else capabilities
                        is_capture_device = (caps & 0x00000001) != 0 # V4L2_CAP_VIDEO_CAPTURE
                        os.close(fd)
                    except Exception as e:
                        self.get_logger().warn(f"Failed to query V4L2 capabilities for {dev_path}: {e}")

                    if not is_capture_device:
                        self.get_logger().info(f"Skipping non-video capture node (metadata or sub-device): {dev_path}")
                        continue

                    self.get_logger().info(f"Probing VITURE Camera on hardware device node: {dev_path}")
                    cap_test = cv2.VideoCapture(dev_idx)
                    if cap_test.isOpened():
                        cap_test.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        cap_test.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                        cap_test.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                        cap_test.set(cv2.CAP_PROP_FPS, 30.0)
                        
                        # Apply sensor exposure optimization (ensuring well-exposed frames on hardware)
                        cap_test.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)  # Auto exposure mode
                        cap_test.set(cv2.CAP_PROP_GAIN, 0.0)           # Reset gain to default (auto-controlled)
                        cap_test.set(cv2.CAP_PROP_BRIGHTNESS, 0.0)     # Reset brightness offset
                        cap_test.set(cv2.CAP_PROP_CONTRAST, 32.0)      # Reset contrast to default
                        
                        # Read 10 frames for auto-exposure stabilization
                        success_count = 0
                        last_frame = None
                        for _ in range(10):
                            ret_w, frame_w = cap_test.read()
                            if ret_w and frame_w is not None:
                                success_count += 1
                                last_frame = frame_w
                        
                        if success_count > 0 and last_frame is not None:
                            self.cap = cap_test
                            h, w, c = last_frame.shape
                            self.get_logger().info(
                                f"VITURE Camera hardware verified on {dev_path}: "
                                f"{w}x{h} ({c} channels, mean intensity: {last_frame.mean():.1f}) @ 30 FPS"
                            )
                            
                            # Log negotiated settings
                            self.get_logger().info(
                                f"  [Camera Controls] Auto-Exposure: {cap_test.get(cv2.CAP_PROP_AUTO_EXPOSURE)}, "
                                f"Gain: {cap_test.get(cv2.CAP_PROP_GAIN)}, Brightness: {cap_test.get(cv2.CAP_PROP_BRIGHTNESS)}, "
                                f"Contrast: {cap_test.get(cv2.CAP_PROP_CONTRAST)}"
                            )
                            break
                        else:
                            cap_test.release()

            if self.cap is None or not self.cap.isOpened():
                self.get_logger().error(
                    f"\n===========================================================\n"
                    f"[VITURE Driver] FATAL: Failed to open physical camera at /dev/video{self.video_device}!\n"
                    f"Production appliance launch aborted. Connect hardware or set sim_mode:=true for dev.\n"
                    f"===========================================================\n"
                )
                raise RuntimeError("Physical camera device unavailable.")

        # Start decoupled high-rate acquisition thread & direct disk recorder thread
        self.running = True
        self.acq_thread = threading.Thread(target=self._hardware_acquisition_loop, daemon=True)
        self.recorder_thread = threading.Thread(target=self._direct_disk_recorder_loop, daemon=True)
        self.acq_thread.start()
        self.recorder_thread.start()

        # Timers
        self.cam_timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_frame)
        self.imu_timer = self.create_timer(1.0 / self.imu_rate, self.publish_imu_and_pose)

        # Profiling variables
        self.start_time = time.time()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.imu_count = 0
        self.last_imu_time = time.time()

        self.get_logger().info(f"ScanAR G VITURE Driver Node Initialized. publish_rate={self.publish_rate}, imu_rate={self.imu_rate}")

    def _cb_active_dir(self, msg):
        target_dir = msg.data
        if target_dir != self.active_dir:
            self.active_dir = target_dir
            self.recording_frame_idx = 0
            if self.active_dir:
                rgb_dir = os.path.join(self.active_dir, "rgb")
                os.makedirs(rgb_dir, exist_ok=True)
                self.get_logger().info(f"[viture_driver] Direct-to-disk recording active for target: {self.active_dir}")

    def _hardware_acquisition_loop(self):
        """High-rate background acquisition thread continuously reading camera hardware at 30 FPS."""
        while rclpy.ok() and getattr(self, 'running', True):
            if not self.sim_mode and self.cap is not None:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame
                    if self.active_dir:
                        try:
                            self.recorder_queue.put_nowait((self.active_dir, frame.copy()))
                        except queue.Full:
                            pass
            time.sleep(0.005)

    def _direct_disk_recorder_loop(self):
        """Decoupled direct-to-disk recorder thread for golden dataset preservation."""
        while rclpy.ok() and getattr(self, 'running', True):
            try:
                target_dir, frame = self.recorder_queue.get(timeout=0.1)
                rgb_dir = os.path.join(target_dir, "rgb")
                os.makedirs(rgb_dir, exist_ok=True)
                img_name = f"frame_{self.recording_frame_idx:05d}.jpg"
                img_path = os.path.join(rgb_dir, img_name)
                cv2.imwrite(img_path, frame)
                self.recording_frame_idx += 1
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"Direct recorder error: {e}")

    def publish_camera_frame(self):
        frame = None
        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
        
        if frame is None and self.sim_mode and self.hal is not None:
            ret, frame, _ = self.hal.get_frame()

        if frame is not None:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "viture_camera_frame"
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = frame.shape[1] * frame.shape[2] if frame.ndim == 3 else frame.shape[1]
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)

    def publish_imu_and_pose(self):
        stamp = self.get_clock().now().to_msg()

        if self.sim_mode and self.hal is not None:
            # Query IMU data from HAL
            success_imu, imu_data, t_imu = self.hal.get_imu_data()
            if success_imu:
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = "viture_imu_frame"
                imu_msg.linear_acceleration.x = float(imu_data["accel"][0])
                imu_msg.linear_acceleration.y = float(imu_data["accel"][1])
                imu_msg.linear_acceleration.z = float(imu_data["accel"][2])
                imu_msg.angular_velocity.x = float(imu_data["gyro"][0])
                imu_msg.angular_velocity.y = float(imu_data["gyro"][1])
                imu_msg.angular_velocity.z = float(imu_data["gyro"][2])
                self.imu_pub.publish(imu_msg)
                self.imu_count += 1

            # Query tracking pose from HAL
            success_pose, pos, quat, t_pose = self.hal.get_pose()
            if success_pose:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = "odom"
                pose_msg.pose.position.x = float(pos[0])
                pose_msg.pose.position.y = float(pos[1])
                pose_msg.pose.position.z = float(pos[2])
                pose_msg.pose.orientation.w = float(quat[0])
                pose_msg.pose.orientation.x = float(quat[1])
                pose_msg.pose.orientation.y = float(quat[2])
                pose_msg.pose.orientation.z = float(quat[3])
                self.pose_pub.publish(pose_msg)
        else:
            # Under hardware mode, raw IMU reading happens here.
            # Currently fallback to simulation if SDK endpoints are restricted.
            pass

        # Monitor IMU Rate
        now = time.time()
        if now - self.last_imu_time >= 2.0:
            hz = self.imu_count / (now - self.last_imu_time)
            hz_msg = Float32()
            hz_msg.data = float(hz)
            self.imu_hz_pub.publish(hz_msg)
            self.imu_count = 0
            self.last_imu_time = now

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VitureDriverNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
