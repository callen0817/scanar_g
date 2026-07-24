"""
scanar_s2/profile.py — ScanAR S2 Profile (Luxonis OAK-D Pro W PoE + External Dual RTK IMU)
=============================================================================================
- Hardware: Luxonis OAK-D Pro W PoE Camera + External Dual RTK GNSS/IMU Rig
- Sensor Package: Rigidly mounted OAK-D Pro W PoE + Dual RTK IMU sensor assembly
- Calibration: Calibrated camera-to-RTK-IMU extrinsics & time synchronization
- Engine: VINS-Fusion
- Primary Topic: /oak_d_poe/camera/image_raw
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_S2_PROFILE = ScanARProfile(
    name="ScanAR S2",
    product_key="scanar_s2",
    sensor_assembly={
        "display": "VITURE Glasses (Monitor Display Only)",
        "camera": "Luxonis OAK-D Pro W PoE Camera",
        "imu": "External Dual RTK GNSS/IMU (Internal OAK IMU Ignored)"
    },
    calibration_package="oak_d_poe_dual_rtk_calibration_package",
    slam_engine="vins_fusion",
    camera_topic="/oak_d_poe/camera/image_raw",
    imu_topic="/dual_rtk/imu",
    pointcloud_topic="",
    supported_views={"rgb": True, "pose": True, "map": True, "eng": True},
    recording_topics=["/oak_d_poe/camera/image_raw", "/dual_rtk/imu", "/scanar/odometry", "/scanar/reconstruction"],
    hardware_drivers=["oak_d_poe_driver", "dual_rtk_imu_driver", "vins_fusion_engine", "viture_hud"],
    color_mode="natural",
    default_fps=60.0,
    resolution=(1920, 1080)
)
