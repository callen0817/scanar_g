#!/bin/bash

# Exit on any error
set -e

echo "==========================================================="
echo "          INSTALLING SCANAR G LINGBOT-MAP DEPENDENCIES"
echo "==========================================================="

# 1. Base APT packages
echo "[-] Installing base system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-pip libopenblas-dev libopenmpi-dev libomp-dev python3-matplotlib git

# 2. PyTorch CUDA for Jetson Orin NX (JetPack 6)
echo "[-] Verifying PyTorch CUDA build..."
if python3 -c "import torch; print(f'Found existing PyTorch: {torch.__version__} (CUDA: {torch.cuda.is_available()})'); assert torch.cuda.is_available()" 2>/dev/null; then
    echo "    ✓ PyTorch with CUDA support is already installed."
else
    echo "    ✗ PyTorch CUDA not found. Installing Jetson-native PyTorch wheel for JetPack 6..."
    pip3 install torch torchvision --index-url=https://pypi.jetson-ai-lab.io/jp6/cu122 --extra-index-url=https://pypi.org/simple
fi

# 3. Pip Requirements
echo "[-] Installing requirements..."
pip3 install -r /home/scanarstereo/scanAR_G/requirements/jetson_requirements.txt
pip3 install -r /home/scanarstereo/scanAR_G/requirements/lingbot_requirements.txt

# 4. Clone and Install Robbyant/lingbot-map
echo "[-] Cloning and installing Robbyant/lingbot-map repository..."
TARGET_SRC="/home/scanarstereo/scanAR_G/src/reconstruction_backends/lingbot_map_src"
if [ ! -d "$TARGET_SRC" ]; then
    echo "Cloning official repository..."
    git clone https://github.com/Robbyant/lingbot-map.git "$TARGET_SRC"
else
    echo "Repository already exists at $TARGET_SRC."
fi

echo "Installing lingbot-map in editable development mode..."
pip3 install -e "$TARGET_SRC"

# 5. Run Verification
echo "[-] Verification check..."
/home/scanarstereo/scanAR_G/scripts/check_dependencies.sh

echo "==========================================================="
echo "✓ SUCCESS: Installation completed successfully!"
echo "==========================================================="
