"""
scanar_profiles package
=======================
Central registry of hardware-agnostic product profiles for ScanAR.
"""

from .base_profile import ScanARProfile
from .scanar_g import SCANAR_G_PROFILE
from .scanar_c import SCANAR_C_PROFILE
from .scanar_s import SCANAR_S_PROFILE
from .scanar_s2 import SCANAR_S2_PROFILE
from .scanar_l import SCANAR_L_PROFILE
from .scanar_l2 import SCANAR_L2_PROFILE
from .scanar_pro import SCANAR_PRO_PROFILE

SCANAR_PROFILES = {
    "scanar_g": SCANAR_G_PROFILE,
    "scanar_c": SCANAR_C_PROFILE,
    "scanar_s": SCANAR_S_PROFILE,
    "scanar_s2": SCANAR_S2_PROFILE,
    "scanar_l": SCANAR_L_PROFILE,
    "scanar_l2": SCANAR_L2_PROFILE,
    "scanar_pro": SCANAR_PRO_PROFILE,
}

def get_scanar_profile(product_key: str) -> ScanARProfile:
    key = str(product_key).lower().strip()
    return SCANAR_PROFILES.get(key, SCANAR_G_PROFILE)
