"""
product_capabilities.py
========================
Bridge module inheriting from central scanar_profiles plugin architecture.
Replaces hardcoded string checks across nodes with modular capability queries.
"""

import os
import sys

# Add scanar_profiles path
profiles_dir = "/home/scanarstereo/scanAR_G/src/scanar_core"
if profiles_dir not in sys.path:
    sys.path.append(profiles_dir)

from scanar_profiles import get_scanar_profile, SCANAR_PROFILES, ScanARProfile

# Alias ProductCapability to ScanARProfile for backward compatibility
ProductCapability = ScanARProfile
PRODUCT_CAPABILITIES = SCANAR_PROFILES

def get_product_capability(product_name: str) -> ScanARProfile:
    return get_scanar_profile(product_name)
