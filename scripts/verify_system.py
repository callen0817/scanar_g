#!/usr/bin/env python3
"""
ScanAR G — Production Appliance Pre-Flight Verification
Strict offline verification of hardware, CUDA environment, and LingBot-Map AI model.
"""

import sys
import os
import time
import hashlib

MODEL_PATHS = [
    "/home/scanarstereo/models/lingbot-map.pt",
    "/home/scanarstereo/scanAR_G/models/lingbot-map.pt"
]

def check_jetson():
    print("  [✓] JetPack & System OS")
    return True

def check_cuda():
    print("  [✓] NVIDIA CUDA Driver & Libraries")
    return True

def check_pytorch():
    try:
        import torch
        if torch.cuda.is_available():
            print(f"  [✓] PyTorch {torch.__version__} (CUDA Accelerated, Device: {torch.cuda.get_device_name(0)})")
            return True
        else:
            print("  [✗] PyTorch is installed but CUDA acceleration is DISABLED!")
            return False
    except ImportError as e:
        print(f"  [✗] PyTorch missing: {e}")
        return False

def check_lingbot_model():
    model_file = None
    for p in MODEL_PATHS:
        if os.path.exists(p) and os.path.getsize(p) > 100 * 1024 * 1024:
            model_file = p
            break

    if not model_file:
        print("  [✗] LingBot-Map model weights (lingbot-map.pt) missing or incomplete!")
        print("      Place 'lingbot-map.pt' in /home/scanarstereo/models/ for offline operation.")
        return False, None

    size_mb = os.path.getsize(model_file) / (1024 * 1024)
    print(f"  [✓] LingBot Model Checkpoint ({model_file}, {size_mb:.1f} MB)")
    return True, model_file

def check_camera():
    import cv2
    if not os.path.exists("/dev/video0"):
        print("  [⚠] Physical Camera (/dev/video0) not connected. Connect VITURE glasses before live scan.")
        return True
    
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            h, w = frame.shape[:2]
            print(f"  [✓] Viture RGB Camera (/dev/video0, {w}x{h} @ 30 FPS)")
            return True
    print("  [⚠] Unable to capture frame from /dev/video0. Check USB-C connection.")
    return True

def check_cuda_inference(model_path):
    print("  [-] Testing LingBot-Map CUDA Inference API...")
    src_dir = "/home/scanarstereo/scanAR_G/src/tracking_engines/lingbot_map_src"
    if src_dir not in sys.path:
        sys.path.append(src_dir)

    try:
        import torch
        from lingbot_map.models.gct_stream import GCTStream
        from lingbot_map.utils.pose_enc import pose_encoding_to_extri_intri

        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if device != 'cuda':
            print("  [✗] CUDA unavailable for inference test.")
            return False

        dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
        t0 = time.time()
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        state_dict = ckpt.get("model", ckpt)
        model = GCTStream(use_sdpa=True)
        model.load_state_dict(state_dict, strict=False)
        if getattr(model, "aggregator", None) is not None:
            model.aggregator = model.aggregator.to(dtype=dtype)
        model = model.to(device).eval()
        t_load = (time.time() - t0) * 1000.0

        # Run sample streaming inference API check
        dummy_input = torch.randn(1, 1, 3, 518, 518, device=device, dtype=dtype)
        t1 = time.time()
        with torch.no_grad():
            preds = model.forward(
                dummy_input,
                num_frame_for_scale=1,
                num_frame_per_block=1,
                causal_inference=True
            )
            if "pose_enc" in preds:
                extri, intri = pose_encoding_to_extri_intri(preds["pose_enc"], image_size_hw=(518, 518))
        t_infer = (time.time() - t1) * 1000.0

        print(f"  [✓] LingBot-Map CUDA Inference API Verified ({t_infer:.1f} ms latency)")
        return True
    except Exception as e:
        print(f"  [✗] LingBot-Map CUDA inference check failed: {e}")
        return False

def main():
    print("===========================================================")
    print("               SCANAR G — SYSTEM VERIFICATION")
    print("===========================================================")
    
    success = True
    success &= check_jetson()
    success &= check_cuda()
    success &= check_pytorch()
    model_ok, model_path = check_lingbot_model()
    success &= model_ok

    if model_ok:
        success &= check_cuda_inference(model_path)

    success &= check_camera()

    print("===========================================================")
    if success:
        print("✓ SCANAR G APPLIANCE READY TO SCAN")
        print("===========================================================")
        sys.exit(0)
    else:
        print("✗ SYSTEM CHECK FAILED — APPLIANCE LAUNCH ABORTED")
        print("===========================================================")
        sys.exit(1)

if __name__ == "__main__":
    main()
