"""
scanar_g.py — ScanAR G Profile (Frozen Reference MVP)
======================================================
- Hardware: VITURE Luma Ultra Glasses RGB Camera + Monitor
- IMU: None (Pure Visual SLAM)
- Engine: LingBot-Map
- Primary Topic: /viture/camera/image_raw
"""

from .base_profile import ScanARProfile

SCANAR_G_PROFILE = ScanARProfile(
    name="ScanAR G",
    product_key="scanar_g",
    camera_topic="/viture/camera/image_raw",
    imu_topic="/viture/imu",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=True,
    color_mode="natural",
    default_fps=30.0,
    resolution=(1920, 1080),
    reconstruction_engine="lingbot_map",
    sensor_description="VITURE Glasses Camera + LingBot-Map",
    hardware_drivers=["viture_driver", "lingbot_engine", "viture_hud"]
)
