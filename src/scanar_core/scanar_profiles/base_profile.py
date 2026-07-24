"""
base_profile.py
================
Hardware-agnostic base profile class for ScanAR products.
Each product profile implements this interface to specify topics, drivers, and UI capabilities.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class ScanARProfile:
    name: str                                  # Display Name e.g. "ScanAR C"
    product_key: str                           # Key e.g. "scanar_c"
    camera_topic: str                          # ROS 2 Image Topic e.g. "/elp/camera/image_raw"
    imu_topic: str                             # ROS 2 IMU Topic
    odom_topic: str                            # ROS 2 Odometry Topic
    reconstruction_topic: str                  # ROS 2 Reconstruction Topic
    has_rgb_camera: bool                       # UI Has RGB Camera Feed
    color_mode: str                            # "natural" or "green"
    default_fps: float                         # Target FPS
    resolution: Tuple[int, int]                # (width, height)
    reconstruction_engine: str                 # "lingbot", "vins_fusion", "fast_lio2", "fast_livo2"
    sensor_description: str                    # Detailed hardware description
    hardware_drivers: List[str] = field(default_factory=list) # ROS 2 node launch packages
