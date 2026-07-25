"""
scanar_g/profile.py — ScanAR G Profile (Frozen Reference MVP)
==============================================================
- Hardware: VITURE Luma Ultra Glasses RGB Camera + Monitor
- Sensor Package: Self-contained VITURE glasses sensor assembly
- Calibration: Factory integrated camera intrinsics
- Engine: LingBot-Map
- Primary Topic: /viture/camera/image_raw
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_G_PROFILE = ScanARProfile(
    name="ScanAR G",
    product_key="scanar_g",
    sensor_assembly={
        "display": "VITURE Luma Ultra Micro-OLED Glasses",
        "camera": "VITURE RGB Camera",
        "imu": "None (Pure Visual SLAM)"
    },
    calibration_package="viture_factory_calibration",
    slam_engine="lingbot_map",
    camera_topic="/viture/camera/image_raw",
    imu_topic="/viture/imu",
    pointcloud_topic="",
    supported_views={"rgb": True, "pose": True, "map": True, "eng": True},
    recording_topics=["/viture/camera/image_raw", "/scanar/odometry", "/scanar/reconstruction"],
    hardware_drivers=["viture_driver", "lingbot_engine", "viture_hud"],
    color_mode="natural",
    default_fps=30.0,
    resolution=(1920, 1080),
    production_locked=True
)
