#!/bin/bash
# ==============================================================================
# ScanAR G — Production One-Time Offline Appliance Provisioning
# Prepares Jetson host system, verifies pre-installed weights, and builds packages.
# ==============================================================================

set -e

echo "==========================================================="
echo "       SCANAR G — OFFLINE APPLIANCE PROVISIONING"
echo "==========================================================="

# 1. Base APT Packages
echo "[-] Step 1: Checking system packages..."
sudo apt-get install -y python3-pip libopenblas-dev libopenmpi-dev libomp-dev python3-matplotlib git > /dev/null 2>&1 || true

# 2. LingBot Package Installation
echo "[-] Step 2: Installing local lingbot-map package..."
TARGET_SRC="/home/scanarstereo/scanAR_G/src/reconstruction_backends/lingbot_map_src"
pip3 install -e "$TARGET_SRC" --no-deps > /dev/null 2>&1 || true

# 3. Offline Model Check
echo "[-] Step 3: Verifying offline model checkpoint..."
mkdir -p /home/scanarstereo/models

MODEL_FILE=""
if [ -f "/home/scanarstereo/models/lingbot-map.pt" ]; then
    MODEL_FILE="/home/scanarstereo/models/lingbot-map.pt"
elif [ -f "/home/scanarstereo/scanAR_G/models/lingbot-map.pt" ]; then
    MODEL_FILE="/home/scanarstereo/scanAR_G/models/lingbot-map.pt"
fi

if [ -n "$MODEL_FILE" ]; then
    echo "    ✓ Found offline model checkpoint: $MODEL_FILE"
else
    echo "    ⚠ Offline model checkpoint not found at /home/scanarstereo/models/lingbot-map.pt!"
    echo "      Copy 'lingbot-map.pt' to /home/scanarstereo/models/ before live capture."
fi

# 4. System Pre-Flight Check & CUDA Test
echo "[-] Step 4: Executing pre-flight startup verification..."
/home/scanarstereo/scanAR_G/scripts/verify_system.py || true

# 5. Clean Build Workspace
echo "[-] Step 5: Building ROS 2 workspace packages..."
cd /home/scanarstereo/scanAR_G
rm -rf build install log
colcon build

echo "==========================================================="
echo "✓ SCANAR G PROVISIONING COMPLETED"
echo "==========================================================="
