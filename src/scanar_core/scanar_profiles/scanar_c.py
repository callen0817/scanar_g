"""
scanar_c.py — ScanAR C Profile (ELP Global Shutter Camera)
===========================================================
- Hardware: ELP 5MP Global Shutter USB Camera (ELP-USBGS5MP01-L170)
- Monitor: VITURE Glasses (Monitor Display ONLY, viture_driver camera NOT started)
- IMU: None (Pure Visual SLAM)
- Engine: LingBot-Map
- Primary Topic: /elp/camera/image_raw
"""

from .base_profile import ScanARProfile

SCANAR_C_PROFILE = ScanARProfile(
    name="ScanAR C",
    product_key="scanar_c",
    camera_topic="/elp/camera/image_raw",
    imu_topic="",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=True,
    color_mode="natural",
    default_fps=30.0,
    resolution=(1920, 1080),
    reconstruction_engine="lingbot_map",
    sensor_description="ELP 5MP Global Shutter USB Camera + LingBot-Map",
    hardware_drivers=["elp_camera_driver", "lingbot_engine", "viture_hud"]
)
