#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import os
import json
import math
import csv

class ContinuousCaptureNode(Node):
    def __init__(self):
        super().__init__('continuous_capture')
        
        # Initialize bridge
        self.bridge = CvBridge()

        # State
        self.trajectory = []  # items: (stamp, tx, ty, tz, qx, qy, qz, qw)
        self.active_dir = ""
        self.frame_idx = 0
        self.image_save_decimation = 3  # Save every 3rd frame (e.g. 10 Hz from 30 FPS)
        self.frame_counter = 0

        self.imu_csv_file = None
        self.imu_writer = None

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/scanar/odometry', self.handle_odom, 10)
        self.img_sub = self.create_subscription(Image, '/viture/camera/image_raw', self.handle_image, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, '/viture/imu', self.handle_imu, 10)
        self.dir_sub = self.create_subscription(String, '/scanar/session/active_directory', self.handle_active_directory, 10)
        
        self.get_logger().info("ScanAR G Continuous Capture Recorder Node Initialized.")

    def handle_odom(self, msg):
        if not self.active_dir:
            return
            
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        self.trajectory.append((stamp, tx, ty, tz, qx, qy, qz, qw))

    def handle_image(self, msg):
        if not self.active_dir:
            return
            
        self.frame_counter += 1
        if self.frame_counter % self.image_save_decimation != 0:
            return

        try:
            import numpy as np
            cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            rgb_dir = os.path.join(self.active_dir, "rgb")
            os.makedirs(rgb_dir, exist_ok=True)
            img_name = f"frame_{self.frame_idx:05d}.jpg"
            img_path = os.path.join(rgb_dir, img_name)
            cv2.imwrite(img_path, cv_img)
            self.frame_idx += 1
        except Exception as e:
            self.get_logger().error(f"Failed to write image: {e}")
            self.get_logger().error(f"Failed to record frame: {e}")

    def handle_imu(self, msg):
        if not self.active_dir or self.imu_writer is None:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_writer.writerow([
            stamp,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

    def handle_active_directory(self, msg):
        target_dir = msg.data
        if target_dir == self.active_dir:
            return
            
        # Close previous IMU file if open
        if self.imu_csv_file is not None:
            try:
                self.imu_csv_file.close()
            except Exception:
                pass
            self.imu_csv_file = None
            self.imu_writer = None

        # Finalize trajectory
        if self.active_dir and self.trajectory:
            self.write_trajectory_products()
            
        self.active_dir = target_dir
        self.trajectory = []
        self.frame_idx = 0
        self.frame_counter = 0

        # Open new IMU file if recording starts
        if self.active_dir:
            imu_dir = os.path.join(self.active_dir, "imu")
            os.makedirs(imu_dir, exist_ok=True)
            imu_path = os.path.join(imu_dir, "imu_raw.csv")
            try:
                self.imu_csv_file = open(imu_path, 'w', newline='')
                self.imu_writer = csv.writer(self.imu_csv_file)
                self.imu_writer.writerow(["timestamp_sec", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])
            except Exception as e:
                self.get_logger().error(f"Failed to open IMU recording file: {e}")

    def write_trajectory_products(self):
        self.get_logger().info(f"Finalizing trajectory capture. Writing poses to: {self.active_dir}")
        
        poses_dir = os.path.join(self.active_dir, "poses")
        os.makedirs(poses_dir, exist_ok=True)
        
        tum_path = os.path.join(poses_dir, "poses.tum")
        csv_path = os.path.join(poses_dir, "poses.csv")
        
        try:
            with open(tum_path, 'w') as f_tum, open(csv_path, 'w') as f_csv:
                f_csv.write("timestamp_sec,tx,ty,tz,qx,qy,qz,qw\n")
                for stamp, tx, ty, tz, qx, qy, qz, qw in self.trajectory:
                    f_tum.write(f"{stamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
                    f_csv.write(f"{stamp:.6f},{tx:.6f},{ty:.6f},{tz:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n")
            self.get_logger().info("Recorded TUM and CSV trajectories written successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to write trajectory files: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
