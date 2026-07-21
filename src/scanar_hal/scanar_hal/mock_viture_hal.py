import time
import math
import numpy as np
from typing import Tuple
from .interfaces import ICameraSource, IImuSource, IDepthSource, ITrackingSource

class MockVitureHAL(ICameraSource, IImuSource, IDepthSource, ITrackingSource):
    """
    Mock implementation of the ScanAR HAL interfaces for the VITURE Luma Ultra glasses.
    Generates high-fidelity simulated camera, IMU, depth, and tracking data.
    """
    def __init__(self):
        self.start_time = time.time()
        
        # Camera intrinsics
        self.fx = 580.0
        self.fy = 580.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Simulated trajectory variables
        self.last_pose_time = time.time()
        self.theta = 0.0
        
    # --- ICameraSource Implementation ---
    def get_frame(self) -> Tuple[bool, np.ndarray, float]:
        t = time.time()
        elapsed = t - self.start_time
        
        # Generate a simulated visual frame (dark scene with dynamic calibration target shapes)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw background stars / SLAM target points
        for i in range(12):
            phase = i * (2.0 * math.pi / 12)
            px = int(320 + 150 * math.cos(elapsed * 0.4 + phase))
            py = int(240 + 120 * math.sin(elapsed * 0.4 + phase))
            cv2_color = (0, 200, 100) if i % 2 == 0 else (200, 100, 0)
            # Draw crosshairs
            cv2_draw_cross(frame, (px, py), cv2_color)

        return True, frame, t

    def get_intrinsics(self) -> dict:
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "dist_coeffs": [0.0, 0.0, 0.0, 0.0]
        }

    # --- IImuSource Implementation ---
    def get_imu_data(self) -> Tuple[bool, dict, float]:
        t = time.time()
        elapsed = t - self.start_time
        
        # Simulate natural head bobbing / walking vibration dynamics (1.8 Hz step frequency)
        step_freq = 1.8
        walk_phase = 2.0 * math.pi * step_freq * elapsed
        
        # Angular velocities (rad/s)
        gx = 0.08 * math.sin(walk_phase)
        gy = 0.04 * math.cos(walk_phase * 2)
        gz = 0.02 * math.sin(walk_phase)
        
        # Linear accelerations (m/s^2)
        ax = 0.15 * math.cos(walk_phase)
        ay = 0.20 * math.sin(walk_phase)
        az = 9.81 + 0.35 * math.cos(walk_phase * 2) # gravity + step acceleration
        
        data = {
            "accel": [ax, ay, az],
            "gyro": [gx, gy, gz]
        }
        return True, data, t

    # --- IDepthSource Implementation ---
    def get_depth_map(self) -> Tuple[bool, np.ndarray, float]:
        t = time.time()
        # Mock depth: generate a gradient depth map (2.0m average depth)
        depth = np.ones((480, 640), dtype=np.float32) * 2.0
        return True, depth, t

    # --- ITrackingSource Implementation ---
    def get_pose(self) -> Tuple[bool, np.ndarray, np.ndarray, float]:
        t = time.time()
        elapsed = t - self.start_time
        
        # Simulate a loop walking trajectory (circle of radius 2.0m on XY plane)
        # Completes one loop every 20 seconds
        omega = 2.0 * math.pi / 20.0
        self.theta = omega * elapsed
        
        x = 2.0 * math.cos(self.theta)
        y = 2.0 * math.sin(self.theta)
        z = 1.6 + 0.05 * math.sin(2.0 * math.pi * 1.8 * elapsed) # head altitude oscillates
        
        # Calculate yaw heading from trajectory angle + add yaw rotation
        yaw = self.theta + math.pi / 2.0
        # Convert yaw to quaternion orientation (w, x, y, z)
        qw = math.cos(yaw * 0.5)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw * 0.5)
        
        pos = np.array([x, y, z], dtype=np.float32)
        quat = np.array([qw, qx, qy, qz], dtype=np.float32)
        
        return True, pos, quat, t

def cv2_draw_cross(img, pt, color):
    # Minimal cv2-free drawing helper if cv2 is not imported
    # Typically cv2 is available in this ROS context
    try:
        import cv2
        cv2.drawMarker(img, pt, color, markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)
    except:
        pass
