#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from scanar_interfaces.msg import ScanConfidence, GaussianSplatArray
import json
import os
import time

class DiagnosticsDashboard(Node):
    def __init__(self):
        super().__init__('diagnostics_dashboard')

        # Cache variables
        self.vigs_status = "INITIALIZING"
        self.camera_fps = 0.0
        self.imu_hz = 0.0
        self.gaussian_count = 0
        
        self.cpu_load = 0.0
        self.mem_usage = 0.0
        self.temp_c = 0.0
        
        self.pipeline_latency_ms = 0.0
        self.overall_confidence = 0.0
        self.active_dir = "STANDBY"
        self.cam_status = "UNKNOWN"

        # Subscriptions
        self.create_subscription(String, '/vigs/status', self.cb_status, 10)
        self.create_subscription(Float32, '/viture/camera/fps', self.cb_fps, 10)
        self.create_subscription(Float32, '/viture/imu/rate', self.cb_imu_rate, 10)
        self.create_subscription(GaussianSplatArray, '/vigs/gaussian_splats', self.cb_splats, 10)
        self.create_subscription(Float32, '/scanar/scan_confidence', self.cb_confidence, 10)
        self.create_subscription(String, '/scanar/diagnostics/latency_report', self.cb_latency, 10)
        self.create_subscription(String, '/scanar/session/active_directory', self.cb_active_dir, 10)
        self.create_subscription(String, '/viture/camera/status', self.cb_cam_status, 10)
        
        # Periodic terminal print
        self.create_timer(1.0, self.render_terminal_dashboard)
        self.get_logger().info("ScanAR G Diagnostics Dashboard Node initialized.")

    def cb_status(self, msg):
        self.vigs_status = msg.data

    def cb_fps(self, msg):
        self.camera_fps = msg.data

    def cb_imu_rate(self, msg):
        self.imu_hz = msg.data

    def cb_splats(self, msg):
        self.gaussian_count = len(msg.splats)

    def cb_confidence(self, msg):
        self.overall_confidence = msg.data

    def cb_latency(self, msg):
        try:
            data = json.loads(msg.data)
            self.pipeline_latency_ms = data.get("latency_profile_ms", {}).get("total_pipeline_latency_ms", 0.0)
        except Exception:
            pass

    def cb_cam_status(self, msg):
        self.cam_status = msg.data

    def cb_active_dir(self, msg):
        self.active_dir = msg.data if msg.data else "STANDBY"

    def render_terminal_dashboard(self):
        # Clear screen ANSI escape
        os.system('clear')
        
        imu_display = "OFF" if self.imu_hz == 0.0 else f"{self.imu_hz:.1f} Hz"
        # Render a beautiful development status layout
        dashboard = f"""
===========================================================
               SCANAR G SYSTEM DIAGNOSTICS
===========================================================
[-] ACTIVE SESSION:
    - Path              : {self.active_dir}

[-] VITURE SENSOR LAYER:
    - RGB Camera Stream : {self.camera_fps:.1f} FPS ({self.cam_status})
    - IMU Sampling Rate : {imu_display}
    - Stereo Stream     : OFF

[-] VIGS SLAM BACKEND:
    - Status            : [{self.vigs_status}]
    - Gaussian Primitives: {self.gaussian_count:,} splats

[-] PIPELINE LATENCY:
    - E2E Processing    : {self.pipeline_latency_ms:.2f} ms

[-] QUALITY SCORE:
    - Tracking Quality  : {self.overall_confidence:.1f} / 100.0
===========================================================
"""
        print(dashboard)

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
