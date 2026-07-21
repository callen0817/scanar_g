# Environment Report

## System Information
- **Operating System**: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- **Kernel & L4T**: NVIDIA L4T R36.5.0 (JetPack 6.x equivalent)
  - Revision: 5.0
  - Date: Fri Jan 16 03:50:45 UTC 2026
  - Architecture: aarch64 (ARM64)
- **Python Version**: Python 3.10.12
- **ROS 2 Distribution**: ROS 2 Humble (ROS_VERSION=2, ROS_PYTHON_VERSION=3)
  - Installed Prefix: `/opt/ros/humble`

## GPU & CUDA Capabilities
- **CUDA Driver Support**: `nvidia-l4t-cuda` is installed (36.5.0).
- **Libraries Available**: 
  - `libcuda.so.1` / `libcuda.so` is located in `/usr/lib/aarch64-linux-gnu/nvidia/`.
  - No default GPU command-line compiler (`nvcc`) is present in `PATH`.
- **TensorRT**: Not detected via default Debian package manager (`dpkg -l`).
