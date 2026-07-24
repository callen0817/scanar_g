"""
scanar_l/profile.py — ScanAR L Profile (Airy 3D LiDAR + FAST-LIO2)
===================================================================
- Hardware: Airy 3D LiDAR (LiDAR Only, No Camera)
- Sensor Package: Self-contained Airy 3D LiDAR + Internal IMU
- Calibration: Factory LiDAR-IMU internal calibration
- Engine: FAST-LIO2
- Primary Topic: /airy/points
- Supported Views: RGB=False, Pose=True, Map=True, Eng=True
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_L_PROFILE = ScanARProfile(
    name="ScanAR L",
    product_key="scanar_l",
    sensor_assembly={
        "display": "VITURE Glasses (Monitor Display Only)",
        "lidar": "Airy 3D LiDAR",
        "imu": "Internal Airy IMU"
    },
    calibration_package="airy_lidar_internal_calibration",
    slam_engine="fast_lio2",
    camera_topic="",
    imu_topic="/airy/imu",
    pointcloud_topic="/airy/points",
    supported_views={"rgb": False, "pose": True, "map": True, "eng": True},
    recording_topics=["/airy/points", "/airy/imu", "/scanar/odometry"],
    hardware_drivers=["airy_lidar_driver", "fast_lio2_engine", "viture_hud"],
    color_mode="green",
    default_fps=10.0,
    resolution=(0, 0)
)
