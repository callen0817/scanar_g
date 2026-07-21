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

        # Video capture init
        self.cap = None
        if not self.sim_mode:
            self.get_logger().info(f"Initializing VITURE Camera on device node: /dev/video{self.video_device}")
            self.cap = cv2.VideoCapture(self.video_device)
            if not self.cap.isOpened():
                self.get_logger().warn(f"Failed to open physical camera at /dev/video{self.video_device}. Falling back to simulation frames.")
                self.sim_mode = True
        
        # Timers
        self.cam_timer = self.create_timer(1.0 / self.publish_rate, self.publish_camera_frame)
        self.imu_timer = self.create_timer(1.0 / self.imu_rate, self.publish_imu_and_pose)

        # Simulation state variables
        self.start_time = time.time()
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.imu_count = 0
        self.last_imu_time = time.time()

        self.get_logger().info("ScanAR G VITURE Driver Node Initialized.")

    def publish_camera_frame(self):
        ret = False
        frame = None
        
        if not self.sim_mode and self.cap is not None:
            ret, frame = self.cap.read()
            
        if not ret:
            # Generate beautiful synthetic calibration / reality pattern
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            # Draw a grid pattern simulating features
            t = time.time() - self.start_time
            cx = int(320 + 80 * math.sin(t * 0.5))
            cy = int(240 + 60 * math.cos(t * 0.7))
            cv2.circle(frame, (cx, cy), 30, (70, 230, 120), -1)
            cv2.putText(frame, f"VITURE SIM FRAME", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(frame, f"TIME: {t:.2f}s", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 205, 0), 2)
            
            # Draw some grid points representing "features" in the environment
            for i in range(10):
                gx = int(320 + 150 * math.sin(i + t * 0.2))
                gy = int(240 + 100 * math.cos(i * 1.5 + t * 0.3))
                cv2.drawMarker(frame, (gx, gy), (0, 255, 255), cv2.MARKER_CROSS, 15, 2)
                
            ret = True

        if ret:
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
        t = time.time() - self.start_time
        stamp = self.get_clock().now().to_msg()

        # 1. IMU Msg
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = "viture_imu_frame"

        # Simulate natural walking vibrations on IMU
        # Walking frequency: ~1.8 Hz (approx 1.8 steps per second)
        walk_freq = 1.8 * 2 * math.pi
        
        # Accelerometer (adds gravity + walking bobbing)
        acc_x = 0.1 * math.sin(t * walk_freq)
        acc_y = 0.1 * math.cos(t * walk_freq)
        acc_z = 9.81 + 0.3 * math.sin(t * walk_freq * 2) # vertical acceleration has double walking frequency
        imu_msg.linear_acceleration.x = acc_x + np.random.normal(0, 0.02)
        imu_msg.linear_acceleration.y = acc_y + np.random.normal(0, 0.02)
        imu_msg.linear_acceleration.z = acc_z + np.random.normal(0, 0.02)

        # Gyroscope (rotational speed of head)
        gyro_x = 0.05 * math.sin(t * 0.5)
        gyro_y = 0.08 * math.cos(t * 0.7)
        gyro_z = 0.03 * math.sin(t * 0.3)
        imu_msg.angular_velocity.x = gyro_x + np.random.normal(0, 0.005)
        imu_msg.angular_velocity.y = gyro_y + np.random.normal(0, 0.005)
        imu_msg.angular_velocity.z = gyro_z + np.random.normal(0, 0.005)

        self.imu_pub.publish(imu_msg)

        # 2. Pose Msg
        # Simulates walking in a loop / figure 8 in the room
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "odom"

        # Trajectory simulation (smooth scale)
        scale_x = 2.0
        scale_y = 1.5
        pos_x = scale_x * math.sin(t * 0.1)
        pos_y = scale_y * math.sin(t * 0.2) # figure 8 path
        pos_z = 1.6 + 0.03 * math.sin(t * walk_freq) # height at 1.6m with walking bobbing

        pose_msg.pose.position.x = pos_x
        pose_msg.pose.position.y = pos_y
        pose_msg.pose.position.z = pos_z

        # Orientation: looking forward along trajectory
        yaw = math.atan2(2 * math.cos(t * 0.2) * scale_y * 0.2, math.cos(t * 0.1) * scale_x * 0.1)
        pitch = 0.05 * math.sin(t * walk_freq) # nodding
        roll = 0.02 * math.cos(t * walk_freq)  # swaying

        # Convert Euler angles to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy

        self.pose_pub.publish(pose_msg)

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
