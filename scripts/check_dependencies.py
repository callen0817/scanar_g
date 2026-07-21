#!/usr/bin/env python3
import sys
import os

def check_jetson():
    print("[-] Checking Hardware Platform...")
    is_jetson = False
    
    # Check device tree model
    if os.path.exists("/proc/device-tree/model"):
        try:
            with open("/proc/device-tree/model", "r") as f:
                model = f.read().lower()
                if "nvidia" in model or "jetson" in model:
                    is_jetson = True
                    print(f"    ✓ Jetson Hardware Detected: {model.strip().upper()}")
        except Exception:
            pass
            
    # Check tegra release file
    if not is_jetson and os.path.exists("/etc/nv_tegra_release"):
        is_jetson = True
        print("    ✓ Jetson L4T Release File Detected.")
        
    if not is_jetson:
        print("    ⚠ Non-Jetson hardware detected. Running in validation simulation mode.")
    return True # return True to allow cross-development, but warning printed

def check_cuda():
    print("[-] Checking NVIDIA CUDA Driver & Runtime...")
    has_l4t_cuda = False
    # Check dpkg for Jetson CUDA packages
    try:
        import subprocess
        res = subprocess.run(["dpkg", "-l"], capture_output=True, text=True)
        if "nvidia-l4t-cuda" in res.stdout:
            has_l4t_cuda = True
            print("    ✓ nvidia-l4t-cuda runtime packages detected.")
    except Exception:
        pass
        
    # Check common library presence
    lib_exists = False
    common_paths = [
        "/usr/lib/aarch64-linux-gnu/libcuda.so",
        "/usr/lib/aarch64-linux-gnu/tegra/libcuda.so",
        "/usr/local/cuda/lib64/libcudart.so"
    ]
    for p in common_paths:
        if os.path.exists(p):
            lib_exists = True
            print(f"    ✓ Found CUDA Library: {p}")
            break
            
    if not has_l4t_cuda and not lib_exists:
        print("    ✗ NVIDIA CUDA Runtime libraries not found! CUDA driver is required.")
        return False
    return True

def check_pytorch():
    print("[-] Checking PyTorch installation & CUDA Support...")
    try:
        import torch
        print(f"    ✓ PyTorch Version: {torch.__version__}")
        
        # Verify CUDA support
        cuda_avail = torch.cuda.is_available()
        if cuda_avail:
            print("    ✓ PyTorch CUDA support: ENABLED")
            print(f"    ✓ CUDA Device Count: {torch.cuda.device_count()}")
            print(f"    ✓ Active Device Name: {torch.cuda.get_device_name(0)}")
            return True
        else:
            print("    ✗ PyTorch is installed, but CUDA is DISABLED (CPU-only build).")
            print("      ScanAR G requires a PyTorch build with CUDA enabled for real-time inference.")
            return False
    except ImportError:
        print("    ✗ PyTorch package not found! Please run the dependency installer.")
        return False

def check_lingbot_map():
    print("[-] Checking lingbot_map Python module...")
    try:
        import lingbot_map
        print("    ✓ lingbot_map package successfully imported.")
        return True
    except ImportError:
        print("    ✗ lingbot_map package not found in Python path!")
        print("      Please run scripts/install_dependencies.sh to clone and install the package.")
        return False

def main():
    print("===========================================================")
    print("              SCANAR G PRE-FLIGHT DEPENDENCY CHECK")
    print("===========================================================")
    
    success = True
    success &= check_jetson()
    success &= check_cuda()
    success &= check_pytorch()
    success &= check_lingbot_map()
    
    print("===========================================================")
    if success:
        print("✓ SUCCESS: All ScanAR G production dependencies are satisfied!")
        print("===========================================================")
        sys.exit(0)
    else:
        print("✗ ERROR: ScanAR G production dependencies check failed!")
        print("Please run dependency installer to fix the requirements:")
        print("  sudo ./scripts/install_dependencies.sh")
        print("===========================================================")
        sys.exit(1)

if __name__ == "__main__":
    main()
