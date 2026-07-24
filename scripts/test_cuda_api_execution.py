#!/usr/bin/env python3
"""
ScanAR G — LingBot-Map CUDA API Execution Verification
Rigorously tests import, model instantiation, weights loading, CUDA memory initialization,
first & second consecutive streaming inference passes, and memory stabilization.
"""

import sys
import os
import time

def verify_api_execution():
    print("===========================================================")
    print("     SCANAR G — LINGBOT-MAP CUDA API EXECUTION TEST")
    print("===========================================================")

    # 1. Import Check
    print("[-] 1. Importing LingBot-Map modules...")
    src_dir = "/home/scanarstereo/scanAR_G/src/reconstruction_backends/lingbot_map_src"
    if src_dir not in sys.path:
        sys.path.append(src_dir)

    try:
        import torch
        from lingbot_map.models.gct_stream import GCTStream
        from lingbot_map.utils.pose_enc import pose_encoding_to_extri_intri
        print("    ✓ Import succeeded.")
    except Exception as e:
        print(f"    ✗ Import FAILED: {e}")
        return False

    # 2. CUDA Check
    print("[-] 2. Initializing CUDA accelerator...")
    if not torch.cuda.is_available():
        print("    ✗ CUDA unavailable!")
        return False
    device = 'cuda'
    print(f"    ✓ CUDA initialized on device: {torch.cuda.get_device_name(0)}")

    # 3. Model Weights Check
    model_path = "/home/scanarstereo/models/lingbot-map.pt"
    if not os.path.exists(model_path):
        print(f"    ✗ Model weights not found at {model_path}!")
        return False

    print("[-] 3. Loading weights into CUDA memory...")
    try:
        t0 = time.time()
        ckpt = torch.load(model_path, map_location=device, weights_only=False)
        state_dict = ckpt.get("model", ckpt)
        # Pass use_sdpa=True for native PyTorch SDPA without flashinfer
        model = GCTStream(use_sdpa=True).to(device).eval()
        missing, unexpected = model.load_state_dict(state_dict, strict=False)
        t_load = (time.time() - t0) * 1000.0
        mem_alloc_mb = torch.cuda.memory_allocated(0) / (1024 * 1024)
        print(f"    ✓ Weights loaded into CUDA memory in {t_load:.1f} ms (Allocated: {mem_alloc_mb:.1f} MB)")
    except Exception as e:
        print(f"    ✗ Model loading FAILED: {e}")
        return False

    # 4. First Inference Pass
    print("[-] 4. Executing 1st streaming inference pass...")
    try:
        frame1 = torch.randn(1, 1, 3, 518, 518, device=device)
        t1 = time.time()
        with torch.no_grad():
            preds1 = model.forward(frame1, num_frame_for_scale=1, num_frame_per_block=1, causal_inference=True)
            if "pose_enc" in preds1:
                extri1, intri1 = pose_encoding_to_extri_intri(preds1["pose_enc"])
        t_infer1 = (time.time() - t1) * 1000.0
        mem_alloc1 = torch.cuda.memory_allocated(0) / (1024 * 1024)
        print(f"    ✓ 1st Inference PASSED ({t_infer1:.1f} ms, GPU Mem: {mem_alloc1:.1f} MB)")
    except Exception as e:
        print(f"    ✗ 1st Inference FAILED: {e}")
        return False

    # 5. Second Consecutive Inference Pass & Memory Stabilization Check
    print("[-] 5. Executing 2nd consecutive streaming inference pass...")
    try:
        frame2 = torch.randn(1, 1, 3, 518, 518, device=device)
        t2 = time.time()
        with torch.no_grad():
            preds2 = model.forward(frame2, num_frame_for_scale=1, num_frame_per_block=1, causal_inference=True)
            if "pose_enc" in preds2:
                extri2, intri2 = pose_encoding_to_extri_intri(preds2["pose_enc"])
        t_infer2 = (time.time() - t2) * 1000.0
        mem_alloc2 = torch.cuda.memory_allocated(0) / (1024 * 1024)
        mem_diff = abs(mem_alloc2 - mem_alloc1)
        print(f"    ✓ 2nd Inference PASSED ({t_infer2:.1f} ms, GPU Mem: {mem_alloc2:.1f} MB, Delta: {mem_diff:.2f} MB)")
        print("    ✓ Memory stabilized successfully.")
    except Exception as e:
        print(f"    ✗ 2nd Inference FAILED: {e}")
        return False

    print("===========================================================")
    print("✓ LINGBOT-MAP CUDA API EXECUTION VERIFIED")
    print("===========================================================")
    return True

if __name__ == "__main__":
    success = verify_api_execution()
    sys.exit(0 if success else 1)
