"""
scanar_pro/profile.py — ScanAR Pro Profile (Multi-Modal Hybrid Fusion)
=======================================================================
- Hardware: Stereo RGB Camera + Airy 3D LiDAR Array + Dual RTK IMU
- Sensor Package: Rigidly mounted Stereo Camera + Airy LiDAR + Dual RTK IMU assembly
- Calibration: Full camera-LiDAR-IMU extrinsic calibration & hardware trigger time sync
- Engine: FAST-LIVO2 (LiDAR-Inertial-Visual Fusion)
- Primary Topic: /oak_d_poe/camera/image_raw & /airy/points
- Supported Views: RGB=True, Pose=True, Map=True, Eng=True
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_PRO_PROFILE = ScanARProfile(
    name="ScanAR Pro",
    product_key="scanar_pro",
    sensor_assembly={
        "display": "VITURE Glasses (Monitor Display Only)",
        "camera": "Luxonis Stereo Camera",
        "lidar": "Airy 3D LiDAR Array",
        "imu": "External Dual RTK GNSS/IMU"
    },
    calibration_package="scanar_pro_hybrid_calibration_package",
    slam_engine="fast_livo2",
    camera_topic="/oak_d_poe/camera/image_raw",
    imu_topic="/dual_rtk/imu",
    pointcloud_topic="/airy/points",
    supported_views={"rgb": True, "pose": True, "map": True, "eng": True},
    recording_topics=["/oak_d_poe/camera/image_raw", "/airy/points", "/dual_rtk/imu", "/scanar/odometry", "/scanar/reconstruction"],
    hardware_drivers=["oak_d_poe_driver", "airy_lidar_driver", "dual_rtk_imu_driver", "fast_livo2_engine", "viture_hud"],
    color_mode="natural",
    default_fps=60.0,
    resolution=(1920, 1080)
)
