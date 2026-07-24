"""
product_capabilities.py
========================
Central capability-based configuration module for ScanAR products.
Replaces hardcoded string checks across nodes with modular capability queries.
"""

class ProductCapability:
    def __init__(self, name: str, has_rgb_camera: bool, color_mode: str, default_fps: float, default_resolution: tuple, reconstruction_engine: str = "lingbot", startup_services: list = None):
        self.name = name
        self.has_rgb_camera = has_rgb_camera
        self.color_mode = color_mode # "natural" or "green"
        self.default_fps = default_fps
        self.default_resolution = default_resolution # (width, height)
        self.reconstruction_engine = reconstruction_engine # "lingbot", "fast_lio", "none"
        self.startup_services = startup_services if startup_services is not None else ["viture_driver", "viture_hud"]

PRODUCT_CAPABILITIES = {
    "scanar_g": ProductCapability("ScanAR G", has_rgb_camera=True, color_mode="natural", default_fps=30.0, default_resolution=(1920, 1080), reconstruction_engine="lingbot", startup_services=["viture_driver", "lingbot_engine", "viture_hud"]),
    "scanar_s": ProductCapability("ScanAR S", has_rgb_camera=True, color_mode="natural", default_fps=30.0, default_resolution=(1920, 1080), reconstruction_engine="lingbot", startup_services=["viture_driver", "lingbot_engine", "viture_hud"]),
    "scanar_s2": ProductCapability("ScanAR S2", has_rgb_camera=True, color_mode="natural", default_fps=30.0, default_resolution=(1920, 1080), reconstruction_engine="lingbot", startup_services=["viture_driver", "lingbot_engine", "viture_hud"]),
    "scanar_pro": ProductCapability("ScanAR Pro", has_rgb_camera=True, color_mode="natural", default_fps=30.0, default_resolution=(1920, 1080), reconstruction_engine="lingbot", startup_services=["viture_driver", "lingbot_engine", "viture_hud"]),
    "scanar_l": ProductCapability("ScanAR L", has_rgb_camera=False, color_mode="green", default_fps=10.0, default_resolution=(0, 0), reconstruction_engine="fast_lio", startup_services=["fast_lio_driver", "viture_hud"]),
    "scanar_l2": ProductCapability("ScanAR L2", has_rgb_camera=False, color_mode="green", default_fps=10.0, default_resolution=(0, 0), reconstruction_engine="fast_lio", startup_services=["fast_lio_driver", "viture_hud"]),
}

def get_product_capability(product_name: str) -> ProductCapability:
    key = str(product_name).lower().strip()
    return PRODUCT_CAPABILITIES.get(key, PRODUCT_CAPABILITIES["scanar_g"])
