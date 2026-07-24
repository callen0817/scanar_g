"""
scanar_pro.py — ScanAR Pro Profile (Multi-Modal Hybrid Fusion)
===============================================================
- Hardware: Stereo RGB Camera + Airy 3D LiDAR
- IMU: External Dual RTK GNSS/IMU
- Engine: FAST-LIVO2 (LiDAR-Inertial-Visual Fusion)
- Primary Topic: /oak_d_poe/camera/image_raw & /airy/points
"""

from .base_profile import ScanARProfile

SCANAR_PRO_PROFILE = ScanARProfile(
    name="ScanAR Pro",
    product_key="scanar_pro",
    camera_topic="/oak_d_poe/camera/image_raw",
    imu_topic="/dual_rtk/imu",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=True,
    color_mode="natural",
    default_fps=60.0,
    resolution=(1920, 1080),
    reconstruction_engine="fast_livo2",
    sensor_description="Stereo RGB + Airy 3D LiDAR + Dual RTK IMU + FAST-LIVO2",
    hardware_drivers=["oak_d_poe_driver", "airy_lidar_driver", "dual_rtk_imu_driver", "fast_livo2_engine", "viture_hud"]
)
