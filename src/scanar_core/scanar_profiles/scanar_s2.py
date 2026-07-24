"""
scanar_s2.py — ScanAR S2 Profile (Luxonis OAK-D Pro W PoE + External Dual RTK IMU)
=====================================================================================
- Hardware: Luxonis OAK-D Pro W PoE Camera
- IMU: External Dual RTK GNSS/IMU (Internal OAK-D IMU IGNORED)
- Engine: VINS-Fusion
- Primary Topic: /oak_d_poe/camera/image_raw
"""

from .base_profile import ScanARProfile

SCANAR_S2_PROFILE = ScanARProfile(
    name="ScanAR S2",
    product_key="scanar_s2",
    camera_topic="/oak_d_poe/camera/image_raw",
    imu_topic="/dual_rtk/imu",
    odom_topic="/scanar/odometry",
    reconstruction_topic="/scanar/reconstruction",
    has_rgb_camera=True,
    color_mode="natural",
    default_fps=60.0,
    resolution=(1920, 1080),
    reconstruction_engine="vins_fusion",
    sensor_description="Luxonis OAK-D Pro W PoE + External Dual RTK IMU + VINS-Fusion",
    hardware_drivers=["oak_d_poe_driver", "dual_rtk_imu_driver", "vins_fusion_engine", "viture_hud"]
)
