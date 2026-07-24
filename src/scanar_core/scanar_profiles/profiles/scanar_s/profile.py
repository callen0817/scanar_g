"""
scanar_s/profile.py — ScanAR S Profile (Luxonis OAK-D Stereo)
=============================================================
- Hardware: Luxonis OAK-D Stereo Camera
- Sensor Package: Self-contained OAK-D Stereo Rig + Internal IMU
- Calibration: Luxonis factory stereo calibration & camera-IMU extrinsics
- Engine: VINS-Fusion
- Primary Topic: /oak_d/camera/image_raw
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_S_PROFILE = ScanARProfile(
    name="ScanAR S",
    product_key="scanar_s",
    sensor_assembly={
        "display": "VITURE Glasses (Monitor Display Only)",
        "camera": "Luxonis OAK-D Stereo Camera",
        "imu": "Internal Luxonis OAK-D IMU"
    },
    calibration_package="oak_d_factory_stereo_calibration",
    slam_engine="vins_fusion",
    camera_topic="/oak_d/camera/image_raw",
    imu_topic="/oak_d/imu",
    pointcloud_topic="",
    supported_views={"rgb": True, "pose": True, "map": True, "eng": True},
    recording_topics=["/oak_d/camera/image_raw", "/oak_d/imu", "/scanar/odometry", "/scanar/reconstruction"],
    hardware_drivers=["oak_d_driver", "vins_fusion_engine", "viture_hud"],
    color_mode="natural",
    default_fps=30.0,
    resolution=(1920, 1080)
)
