"""
scanar_l2/profile.py — ScanAR L2 Profile (Multi-Airy LiDAR + External Dual RTK IMU)
=====================================================================================
- Hardware: Multi-Airy 3D LiDAR Array (Left Airy + Right Airy, No Camera)
- Sensor Package: Rigidly mounted Multi-Airy LiDAR Array + Dual RTK IMU
- Calibration: Multi-LiDAR extrinsics, LiDAR-to-RTK-IMU extrinsics & time sync
- Engine: FAST-LIO2 (Multi-LiDAR)
- Primary Topic: /airy/points
- Supported Views: RGB=False, Pose=True, Map=True, Eng=True
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_L2_PROFILE = ScanARProfile(
    name="ScanAR L2",
    product_key="scanar_l2",
    sensor_assembly={
        "display": "VITURE Glasses (Monitor Display Only)",
        "lidar": "Multi-Airy 3D LiDAR Array (Left + Right)",
        "imu": "External Dual RTK GNSS/IMU (Internal Airy IMU Ignored)"
    },
    calibration_package="multi_airy_dual_rtk_calibration_package",
    slam_engine="fast_lio2",
    camera_topic="",
    imu_topic="/dual_rtk/imu",
    pointcloud_topic="/airy/points",
    supported_views={"rgb": False, "pose": True, "map": True, "eng": True},
    recording_topics=["/airy/points", "/dual_rtk/imu", "/scanar/odometry"],
    hardware_drivers=["multi_airy_lidar_driver", "dual_rtk_imu_driver", "fast_lio2_engine", "viture_hud"],
    color_mode="green",
    default_fps=10.0,
    resolution=(0, 0)
)
