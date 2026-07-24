"""
scanar_c/profile.py — ScanAR C Profile (ELP Global Shutter Camera)
===================================================================
- Hardware: ELP 5MP Global Shutter USB Camera (ELP-USBGS5MP01-L170)
- Monitor: VITURE Glasses (Monitor Display ONLY, viture_driver camera NOT started)
- Sensor Package: ELP Global Shutter Camera mounted on rigid frame
- Calibration: ELP camera intrinsics & manual exposure/WB profile
- Engine: LingBot-Map
- Primary Topic: /elp/camera/image_raw
"""

from scanar_profiles.base_profile import ScanARProfile

SCANAR_C_PROFILE = ScanARProfile(
    name="ScanAR C",
    product_key="scanar_c",
    sensor_assembly={
        "display": "VITURE Glasses (Monitor Display Only)",
        "camera": "ELP 5MP Global Shutter USB Camera (ELP-USBGS5MP01-L170)",
        "imu": "None (Pure Visual SLAM)"
    },
    calibration_package="elp_usbgs5mp01_calibration",
    slam_engine="lingbot_map",
    camera_topic="/elp/camera/image_raw",
    imu_topic="",
    pointcloud_topic="",
    supported_views={"rgb": True, "pose": True, "map": True, "eng": True},
    recording_topics=["/elp/camera/image_raw", "/scanar/odometry", "/scanar/reconstruction"],
    hardware_drivers=["elp_camera_driver", "lingbot_engine", "viture_hud"],
    color_mode="natural",
    default_fps=30.0,
    resolution=(1920, 1080)
)
