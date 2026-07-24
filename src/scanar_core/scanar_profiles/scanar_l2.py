"""
scanar_l2.py — ScanAR L2 Profile (Multi-Airy LiDAR + External Dual RTK IMU)
=============================================================================
- Hardware: Multi-Airy 3D LiDAR Array (No Camera)
- IMU: External Dual RTK GNSS/IMU (Internal Airy IMU IGNORED)
- Engine: FAST-LIO2
- Primary Topic: /airy/points
"""

from .base_profile import ScanARProfile

SCANAR_L2_PROFILE = ScanARProfile(
    name="ScanAR L2",
    product_key="scanar_l2",
    camera_topic="",
    imu_topic="/dual_rtk/imu",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=False,
    color_mode="green",
    default_fps=10.0,
    resolution=(0, 0),
    reconstruction_engine="fast_lio2",
    sensor_description="Multi-Airy 3D LiDAR + External Dual RTK IMU + FAST-LIO2",
    hardware_drivers=["multi_airy_lidar_driver", "dual_rtk_imu_driver", "fast_lio2_engine", "viture_hud"]
)
