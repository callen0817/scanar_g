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

# Import the ScanAR Hardware Abstraction Layer
from scanar_hal import MockVitureHAL

class VitureDriverNode(Node):
    def __init__(self):
        super().__init__('viture_driver')
        
        # Declare parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('publish_rate', 30.0) # Hz for camera
        self.declare_parameter('imu_rate', 100.0) # Hz for IMU
        self.declare_parameter('sim_mode', False)

        self.video_device = self.get_parameter('video_device').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.imu_rate = self.get_parameter('imu_rate').value
        self.sim_mode = self.get_parameter('sim_mode').value

        # Initialize bridge
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, '/viture/camera/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/viture/imu', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/viture/pose', 10)
        self.fps_pub = self.create_publisher(Float32, '/viture/camera/fps', 10)
        self.imu_hz_pub = self.create_publisher(Float32, '/viture/imu/rate', 10)

        # HAL & Hardware configurations
        self.hal = None
        self.cap = None

        if self.sim_mode:
            self.get_logger().info("Initializing VITURE HAL Driver in SIMULATION mode.")
            self.hal = MockVitureHAL()
        else:
            self.get_logger().info(f"Initializing VITURE Camera on device node: /dev/video{self.video_device}")
            self.cap = cv2.VideoCapture(self.video_device)
            if not self.cap.isOpened():
                self.get_logger().warn(f"Failed to open physical camera at /dev/video{self.video_device}. Falling back to simulation HAL.")
                self.sim_mode = True
                self.hal = MockVitureHAL()
        
        # Timers
        self.cam_timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_frame)
        self.imu_timer = self.create_timer(1.0 / self.imu_rate, self.publish_imu_and_pose)

        # Profiling variables
        self.start_time = time.time()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.imu_count = 0
        self.last_imu_time = time.time()

        self.get_logger().info("ScanAR G VITURE Driver Node Initialized.")

    def publish_camera_frame(self):
        ret = False
        frame = None
        t_sec = 0.0
        
        if not self.sim_mode and self.cap is not None:
            ret, frame = self.cap.read()
            t_sec = time.time()
        elif self.sim_mode and self.hal is not None:
            ret, frame, t_sec = self.hal.get_frame()

        if ret and frame is not None:
            # Publish image
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "viture_camera_frame"
            self.image_pub.publish(msg)

            # Monitor FPS
            self.frame_count += 1
            now = time.time()
            if now - self.last_fps_time >= 2.0:
                fps = self.frame_count / (now - self.last_fps_time)
                fps_msg = Float32()
                fps_msg.data = float(fps)
                self.fps_pub.publish(fps_msg)
                self.frame_count = 0
                self.last_fps_time = now

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
        self.imu_count += 1
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
