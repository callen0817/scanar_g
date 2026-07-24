"""
scanar_s.py — ScanAR S Profile (Luxonis OAK-D Stereo)
=====================================================
- Hardware: Luxonis OAK-D Stereo Camera
- IMU: Internal Luxonis OAK-D IMU
- Engine: VINS-Fusion
- Primary Topic: /oak_d/camera/image_raw
"""

from .base_profile import ScanARProfile

SCANAR_S_PROFILE = ScanARProfile(
    name="ScanAR S",
    product_key="scanar_s",
    camera_topic="/oak_d/camera/image_raw",
    imu_topic="/oak_d/imu",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=True,
    color_mode="natural",
    default_fps=30.0,
    resolution=(1920, 1080),
    reconstruction_engine="vins_fusion",
    sensor_description="Luxonis OAK-D Stereo + Internal IMU + VINS-Fusion",
    hardware_drivers=["oak_d_driver", "vins_fusion_engine", "viture_hud"]
)
