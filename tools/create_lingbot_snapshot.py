#!/usr/bin/env python3
import os
import sys
import time
import json
import hashlib
import subprocess
import torch

src_dir = "/home/scanarstereo/scanAR_G/src/tracking_engines/lingbot_map_src"
if src_dir not in sys.path:
    sys.path.append(src_dir)

from lingbot_map.models.gct_stream import GCTStream

MODELS_DIR = "/home/scanarstereo/models"
FP16_WEIGHTS_PATH = os.path.join(MODELS_DIR, "lingbot-map-fp16.pt")
SNAPSHOT_PATH = os.path.join(MODELS_DIR, "lingbot-map-engine-v1.snapshot")
METADATA_PATH = os.path.join(MODELS_DIR, "snapshot_metadata.json")

def get_git_commit():
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd="/home/scanarstereo/scanAR_G").decode().strip()
    except Exception:
        return "unknown"

def compute_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()

def create_snapshot():
    print("=== Creating Initialized LingBot Deployment Snapshot (v1) ===")
    t_start = time.time()

    if not os.path.exists(FP16_WEIGHTS_PATH):
        raise FileNotFoundError(f"Source FP16 checkpoint not found at: {FP16_WEIGHTS_PATH}")

    print("1. Instantiating GCTStream model architecture...")
    t0 = time.time()
    model = GCTStream(use_sdpa=True)
    print(f"   Architecture constructed in {time.time() - t0:.2f}s")

    print("2. Loading FP16 weights into state dict...")
    t0 = time.time()
    ckpt = torch.load(FP16_WEIGHTS_PATH, map_location="cpu", weights_only=False)
    state_dict = ckpt.get("model", ckpt)
    model.load_state_dict(state_dict, strict=False)
    print(f"   Weights loaded in {time.time() - t0:.2f}s")

    print("3. Configuring evaluation mode and dtype rules...")
    dtype = torch.bfloat16 if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    with torch.inference_mode():
        if getattr(model, "aggregator", None) is not None:
            model.aggregator = model.aggregator.to(dtype=dtype)
        model = model.eval()

    print(f"4. Saving deployment snapshot artifact to: {SNAPSHOT_PATH}...")
    t0 = time.time()
    torch.save(model, SNAPSHOT_PATH)
    t_save = time.time() - t0
    snapshot_size_gb = os.path.getsize(SNAPSHOT_PATH) / (1024**3)
    print(f"   Snapshot saved ({snapshot_size_gb:.2f} GB) in {t_save:.2f}s")

    print("5. Computing SHA256 model hash and hardened metadata...")
    model_hash = compute_sha256(SNAPSHOT_PATH)
    git_commit = get_git_commit()

    gpu_cc = f"{torch.cuda.get_device_capability()[0]}.{torch.cuda.get_device_capability()[1]}" if torch.cuda.is_available() else "none"

    metadata = {
        "model": "lingbot-map",
        "architecture": "GCTStream",
        "precision": "fp16",
        "model_hash": model_hash,
        "git_commit": git_commit,
        "cuda_version": torch.version.cuda if torch.cuda.is_available() else "none",
        "torch_version": torch.__version__,
        "python_version": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
        "gpu_compute_capability": gpu_cc,
        "product_profile": "scanar_g",
        "created_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "snapshot_size_bytes": os.path.getsize(SNAPSHOT_PATH),
        "source_weights_path": FP16_WEIGHTS_PATH
    }

    with open(METADATA_PATH, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"6. Saved hardened metadata to: {METADATA_PATH}")

    total_time = time.time() - t_start
    print(f"\nDeployment Snapshot Generation Completed Successfully in {total_time:.2f}s!")

if __name__ == '__main__':
    create_snapshot()
