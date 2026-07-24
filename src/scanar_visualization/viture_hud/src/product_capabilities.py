"""
product_capabilities.py
========================
Central capability-based configuration module for ScanAR products.
Replaces hardcoded string checks across nodes with modular capability queries.
"""

class ProductCapability:
    def __init__(self, name: str, has_rgb_camera: bool, color_mode: str, default_fps: float, default_resolution: tuple, reconstruction_engine: str = "lingbot", sensor_description: str = "", startup_services: list = None):
        self.name = name
        self.has_rgb_camera = has_rgb_camera
        self.color_mode = color_mode # "natural" or "green"
        self.default_fps = default_fps
        self.default_resolution = default_resolution # (width, height)
        self.reconstruction_engine = reconstruction_engine # "lingbot", "vins_fusion", "fast_lio2", "fast_livo2"
        self.sensor_description = sensor_description
        self.startup_services = startup_services if startup_services is not None else ["viture_driver", "viture_hud"]

PRODUCT_CAPABILITIES = {
    "scanar_g": ProductCapability(
        "ScanAR G", has_rgb_camera=True, color_mode="natural", default_fps=30.0, default_resolution=(1920, 1080),
        reconstruction_engine="lingbot", sensor_description="VITURE Glasses Camera + LingBot-Map",
        startup_services=["viture_driver", "lingbot_engine", "viture_hud"]),
        
    "scanar_c": ProductCapability(
        "ScanAR C", has_rgb_camera=True, color_mode="natural", default_fps=60.0, default_resolution=(1280, 720),
        reconstruction_engine="lingbot", sensor_description="ELP 5MP Global Shutter USB Camera + LingBot-Map",
        startup_services=["elp_camera_driver", "lingbot_engine", "viture_hud"]),
        
    "scanar_s": ProductCapability(
        "ScanAR S", has_rgb_camera=True, color_mode="natural", default_fps=30.0, default_resolution=(1920, 1080),
        reconstruction_engine="vins_fusion", sensor_description="Luxonis OAK-D Stereo + Internal OAK IMU + VINS-Fusion",
        startup_services=["oak_d_driver", "vins_fusion_engine", "viture_hud"]),
        
    "scanar_s2": ProductCapability(
        "ScanAR S2", has_rgb_camera=True, color_mode="natural", default_fps=60.0, default_resolution=(1920, 1080),
        reconstruction_engine="vins_fusion", sensor_description="Luxonis OAK-D Pro W PoE + External Dual RTK IMU + VINS-Fusion",
        startup_services=["oak_d_poe_driver", "dual_rtk_imu_driver", "vins_fusion_engine", "viture_hud"]),
        
    "scanar_l": ProductCapability(
        "ScanAR L", has_rgb_camera=False, color_mode="green", default_fps=10.0, default_resolution=(0, 0),
        reconstruction_engine="fast_lio2", sensor_description="Airy 3D LiDAR + Internal Airy IMU + FAST-LIO2",
        startup_services=["airy_lidar_driver", "fast_lio2_engine", "viture_hud"]),
        
    "scanar_l2": ProductCapability(
        "ScanAR L2", has_rgb_camera=False, color_mode="green", default_fps=10.0, default_resolution=(0, 0),
        reconstruction_engine="fast_lio2", sensor_description="Airy 3D LiDAR + External Dual RTK IMU + FAST-LIO2",
        startup_services=["airy_lidar_driver", "dual_rtk_imu_driver", "fast_lio2_engine", "viture_hud"]),
        
    "scanar_pro": ProductCapability(
        "ScanAR Pro", has_rgb_camera=True, color_mode="natural", default_fps=60.0, default_resolution=(1920, 1080),
        reconstruction_engine="fast_livo2", sensor_description="Luxonis Stereo + Airy LiDAR + Dual RTK IMU + FAST-LIVO2",
        startup_services=["oak_d_driver", "airy_lidar_driver", "dual_rtk_imu_driver", "fast_livo2_engine", "viture_hud"]),
}

def get_product_capability(product_name: str) -> ProductCapability:
    key = str(product_name).lower().strip()
    return PRODUCT_CAPABILITIES.get(key, PRODUCT_CAPABILITIES["scanar_g"])
