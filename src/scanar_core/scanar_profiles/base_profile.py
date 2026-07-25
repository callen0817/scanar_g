"""
base_profile.py
================
Validated Hardware Configuration Profile for ScanAR Products.
Each profile represents a complete, calibrated sensor package, slam engine, and UI configuration.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Tuple

@dataclass
class ScanARProfile:
    name: str                                           # Display Name e.g. "ScanAR C"
    product_key: str                                    # Key e.g. "scanar_c"
    sensor_assembly: Dict[str, str]                     # Hardware details (camera, imu, lidar)
    calibration_package: str                            # Calibration config path / description
    slam_engine: str                                    # "lingbot_map", "vins_fusion", "fast_lio2", "fast_livo2"
    camera_topic: str                                   # Primary ROS 2 Image Topic
    imu_topic: str                                      # Primary ROS 2 IMU Topic
    pointcloud_topic: str                               # Primary ROS 2 PointCloud Topic
    odom_topic: str = "/scanar/odometry"                # ROS 2 Odometry Topic
    reconstruction_topic: str = "/scanar/reconstruction"# ROS 2 Reconstruction Topic
    supported_views: Dict[str, bool] = field(default_factory=lambda: {"rgb": True, "pose": True, "map": True, "eng": True})
    recording_topics: List[str] = field(default_factory=list)
    hardware_drivers: List[str] = field(default_factory=list)
    color_mode: str = "natural"                         # "natural" or "green"
    default_fps: float = 30.0                           # Camera / Sensor FPS
    resolution: Tuple[int, int] = (1920, 1080)          # Frame resolution (width, height)
    production_locked: bool = False                     # Reference MVP production lock flag

    def supports_view(self, view_key: str) -> bool:
        """Check if UI view (rgb, pose, map, eng) is supported by this hardware profile."""
        return self.supported_views.get(view_key.lower(), False)

    @property
    def has_rgb_camera(self) -> bool:
        """Property alias for RGB view support."""
        return self.supports_view("rgb")

    def get_camera_topic(self) -> str:
        """Get primary ROS 2 camera image topic."""
        return self.camera_topic if self.supports_view("rgb") else ""

    def get_imu_topic(self) -> str:
        """Get primary ROS 2 IMU topic."""
        return self.imu_topic

    def get_slam_engine(self) -> str:
        """Get SLAM / Reconstruction engine name."""
        return self.slam_engine
