"""
scanar_l.py — ScanAR L Profile (Airy 3D LiDAR + FAST-LIO2)
===========================================================
- Hardware: Airy 3D LiDAR (No Camera)
- IMU: Internal Airy IMU
- Engine: FAST-LIO2
- Primary Topic: /airy/points
"""

from .base_profile import ScanARProfile

SCANAR_L_PROFILE = ScanARProfile(
    name="ScanAR L",
    product_key="scanar_l",
    camera_topic="",
    imu_topic="/airy/imu",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=False,
    color_mode="green",
    default_fps=10.0,
    resolution=(0, 0),
    reconstruction_engine="fast_lio2",
    sensor_description="Airy 3D LiDAR + Internal Airy IMU + FAST-LIO2",
    hardware_drivers=["airy_lidar_driver", "fast_lio2_engine", "viture_hud"]
)
