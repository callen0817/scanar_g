#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import time
import math
import os
import json
import struct
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from scanar_interfaces.msg import ReconstructionFrame, ScanConfidence
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class LingBotBackendNode(Node):
    def __init__(self):
        super().__init__('lingbot_backend_node')

        # Node parameters
        self.declare_parameter('cuda_enabled', True)
        self.cuda_enabled = self.get_parameter('cuda_enabled').get_parameter_value().bool_value
        self.declare_parameter('sim_mode', False)
        self.sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        # Initialize bridge
        self.bridge = CvBridge()

        # Cache variables
        self.prev_gray = None
        self.prev_pts = None
        self.state_pos = np.array([0.0, 0.0, 0.0])
        self.state_q = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z
        self.all_pts_3d = []
        self.all_colors = []
        self.active_dir = ""

        # Enforce Production dependencies if not in sim_mode
        self.has_lingbot = False
        self.model = None

        if not self.sim_mode:
            try:
                import sys
                src_dir = "/home/scanarstereo/scanAR_G/src/tracking_engines/lingbot_map_src"
                if src_dir not in sys.path:
                    sys.path.append(src_dir)
                t_start = time.time()
                
                t0 = time.time()
                import torch
                t_import = time.time() - t0

                t0 = time.time()
                self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
                if not torch.cuda.is_available():
                    raise RuntimeError("PyTorch package is CPU-only, CUDA support is required.")
                t_cuda_init = time.time() - t0

                from lingbot_map.models.gct_stream import GCTStream
                self.get_logger().info("[LingBot-Map] Initializing GCTStream Neural 3D Reconstruction Model...")
                
                snapshot_path = "/home/scanarstereo/models/lingbot-map-engine-v1.snapshot"
                metadata_path = "/home/scanarstereo/models/snapshot_metadata.json"
                fp16_model_path = "/home/scanarstereo/models/lingbot-map-fp16.pt"
                fp32_model_path = "/home/scanarstereo/models/lingbot-map.pt"

                snapshot_valid = False
                if os.path.exists(snapshot_path) and os.path.exists(metadata_path):
                    try:
                        with open(metadata_path, 'r') as mf:
                            meta = json.load(mf)
                        py_ver = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
                        gpu_cc = f"{torch.cuda.get_device_capability()[0]}.{torch.cuda.get_device_capability()[1]}" if torch.cuda.is_available() else "none"
                        
                        if (meta.get("torch_version") == torch.__version__ and 
                            meta.get("python_version") == py_ver and
                            meta.get("gpu_compute_capability") == gpu_cc):
                            snapshot_valid = True
                            self.get_logger().info(f"[LingBot-Map] Verified deployment snapshot artifact (Hash: {meta.get('model_hash', 'N/A')[:12]}..., Git: {meta.get('git_commit', 'N/A')[:7]}, CC: {gpu_cc})")
                        else:
                            self.get_logger().warn("[LingBot-Map] Snapshot environment mismatch detected. Falling back to FP16 checkpoint load.")
                    except Exception as e:
                        self.get_logger().warn(f"[LingBot-Map] Snapshot metadata check failed: {e}")

                if snapshot_valid:
                    self.get_logger().info(f"[LingBot-Map] Fast-Boot: Restoring deployment snapshot artifact from: {snapshot_path}")
                    t0 = time.time()
                    self.model = torch.load(snapshot_path, map_location="cpu", weights_only=False)
                    t_restore = time.time() - t0

                    t0 = time.time()
                    self.dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
                    if getattr(self.model, "aggregator", None) is not None:
                        self.model.aggregator = self.model.aggregator.to(dtype=self.dtype)
                    self.model = self.model.to(self.device).eval()
                    t_gpu_transfer = time.time() - t0

                    t_total = time.time() - t_start
                    self.has_lingbot = True

                    self.get_logger().info(
                        f"\n===========================================================\n"
                        f"[LingBot-Map] Snapshot Fast-Boot Breakdown (Total: {t_total:.2f}s):\n"
                        f"  1. Module Imports      : {t_import:.2f} s\n"
                        f"  2. CUDA Context Init   : {t_cuda_init:.2f} s\n"
                        f"  3. Snapshot Restore    : {t_restore:.2f} s\n"
                        f"  4. GPU VRAM Transfer   : {t_gpu_transfer:.2f} s\n"
                        f"===========================================================\n"
                        f"Production stack initialized. Model weights loaded successfully."
                    )
                else:
                    model_path = fp16_model_path if os.path.exists(fp16_model_path) else fp32_model_path
                    if os.path.exists(model_path):
                        self.get_logger().info(f"[LingBot-Map] Loading model weights from: {model_path}")
                        self.dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
                        
                        t0 = time.time()
                        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
                        t_nvme_read = time.time() - t0

                        t0 = time.time()
                        state_dict = ckpt.get("model", ckpt)
                        self.model = GCTStream(use_sdpa=True)
                        self.model.load_state_dict(state_dict, strict=False)
                        t_deserialize = time.time() - t0

                        t0 = time.time()
                        if getattr(self.model, "aggregator", None) is not None:
                            self.model.aggregator = self.model.aggregator.to(dtype=self.dtype)
                        self.model = self.model.to(self.device).eval()
                        t_gpu_transfer = time.time() - t0

                        t_total = time.time() - t_start
                        self.has_lingbot = True

                        self.get_logger().info(
                            f"\n===========================================================\n"
                            f"[LingBot-Map] Fallback Model Startup Profiling Breakdown (Total: {t_total:.2f}s):\n"
                            f"  1. Module Imports      : {t_import:.2f} s\n"
                            f"  2. CUDA Context Init   : {t_cuda_init:.2f} s\n"
                            f"  3. NVMe Checkpoint Read: {t_nvme_read:.2f} s\n"
                            f"  4. Deserialization     : {t_deserialize:.2f} s\n"
                            f"  5. GPU VRAM Transfer   : {t_gpu_transfer:.2f} s\n"
                            f"===========================================================\n"
                            f"Production stack initialized. Model weights loaded successfully."
                        )
                    else:
                        self.get_logger().warn(f"[LingBot-Map] Model checkpoint not yet complete at {model_path}.")
            except Exception as e:
                self.get_logger().error(
                    "\n===========================================================\n"
                    "[LingBot-Map] FATAL: Production dependencies not satisfied!\n"
                    f"Detail: {e}\n"
                    "===========================================================\n"
                )
                raise RuntimeError("Missing production dependencies for LingBot-Map reconstruction backend.")
        else:
            self.get_logger().warn("[LingBot-Map] Running in Simulation/Development mode. Bypassing production dependency requirements.")

        # Camera intrinsics
        self.fx = 960.0
        self.fy = 960.0
        self.cx = 960.0
        self.cy = 540.0

        # Publishers
        self.recon_pub = self.create_publisher(ReconstructionFrame, '/scanar/reconstruction', 10)
        self.odom_pub = self.create_publisher(Odometry, '/scanar/odometry', 10)
        self.status_pub = self.create_publisher(String, '/vigs/status', 10)
        self.conf_pub = self.create_publisher(Float32, '/scanar/scan_confidence', 10)

        # Subscribers
        self.create_subscription(Image, '/viture/camera/image_raw', self.handle_image, qos_profile_sensor_data)
        self.create_subscription(String, '/scanar/session/active_directory', self.handle_active_directory, 10)

        # Periodic status publisher
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info("ScanAR G LingBot-Map Backend Node Initialized.")

    def publish_status(self):
        msg = String()
        if self.sim_mode:
            msg.data = "LingBot-Map (Simulation)"
        else:
            msg.data = "LingBot-Map (Active)"
        self.status_pub.publish(msg)

    def handle_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert camera image: {e}")
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        self.cx = w / 2.0
        self.cy = h / 2.0

        if self.sim_mode:
            self.run_simulated_mapper(gray, cv_img, msg.header)
        else:
            if self.select_keyframe(cv_img, gray):
                self.run_lingbot_inference(cv_img, msg.header)

    def select_keyframe(self, cv_img, gray) -> bool:
        """
        Adaptive Inference Scheduler & Keyframe Selector.
        Evaluates Blur, Exposure, and Motion Scores to optimize GPU efficiency.
        """
        now = time.time()
        # Enforce max keyframe interval fallback (at least 1 Hz keyframe guaranteed)
        time_since_last = now - getattr(self, "last_keyframe_time", 0.0)

        # 1. Blur Score (Laplacian Variance)
        blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
        if blur_score < 25.0 and time_since_last < 2.0: # Skip severely motion-blurred frames unless timeout
            return False

        # 2. Exposure Score (Mean brightness range [12, 245])
        exposure_score = float(np.mean(gray))
        if (exposure_score < 10.0 or exposure_score > 248.0) and time_since_last < 2.0:
            return False

        # High-Velocity Motion Override: during fast head turns (motion_delta >= 1.5),
        # force immediate neural inference to preserve temporal feature correspondence & prevent tracking loss!
        if getattr(self, "prev_keyframe_gray", None) is not None:
            motion_delta = float(np.mean(np.abs(gray.astype(np.float32) - self.prev_keyframe_gray.astype(np.float32))))
            if motion_delta >= 1.5:
                self.last_keyframe_time = now
                self.prev_keyframe_gray = gray.copy()
                return True

        # 3. Motion Score (Pixel difference delta)
        if getattr(self, "prev_keyframe_gray", None) is not None and time_since_last < 0.5:
            motion_delta = float(np.mean(np.abs(gray.astype(np.float32) - self.prev_keyframe_gray.astype(np.float32))))
            if motion_delta < 0.8: # Accept dynamic camera viewpoints quickly
                return False

        self.last_keyframe_time = now
        self.prev_keyframe_gray = gray.copy()
        return True

    def run_lingbot_inference(self, cv_img, header):
        if not self.has_lingbot or self.model is None:
            self.get_logger().error("[LingBot-Map] Model not loaded. Inference cannot proceed.")
            return

        import torch
        from lingbot_map.utils.pose_enc import pose_encoding_to_extri_intri

        t0 = time.time()

        # Preprocess frame: resize to 518x518, convert BGR->RGB, normalize to [0,1]
        # Tensor shape for GCTStream: [1, 1, 3, 518, 518] (Batch=1, Seq=1, C=3, H=518, W=518)
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (518, 518))
        dtype = getattr(self, "dtype", torch.float32)
        img_tensor = torch.from_numpy(resized).permute(2, 0, 1).unsqueeze(0).unsqueeze(0).to(device=self.device, dtype=dtype) / 255.0

        try:
            with torch.no_grad():
                preds = self.model.forward(
                    img_tensor,
                    num_frame_for_scale=1,
                    num_frame_per_block=1,
                    causal_inference=True
                )

            t_ms = (time.time() - t0) * 1000.0

            # Decode camera extrinsics using official LingBot API
            if isinstance(preds, dict) and "pose_enc" in preds:
                try:
                    extri, intri = pose_encoding_to_extri_intri(preds["pose_enc"], image_size_hw=(518, 518))
                    extri_np = extri.squeeze().cpu().numpy()
                    if extri_np.ndim == 2 and extri_np.shape[1] == 4 and extri_np.shape[0] >= 3:
                        R_w2c = extri_np[:3, :3]
                        t_w2c = extri_np[:3, 3]
                        
                        # Invert transformation to get camera-to-world (c2w)
                        R_c2w = R_w2c.T
                        t_c2w = -R_c2w @ t_w2c
                        
                        self.state_pos = t_c2w
                        
                        # Convert R_c2w to quaternion (w, x, y, z)
                        tr = np.trace(R_c2w)
                        if tr > 0:
                            S = np.sqrt(tr + 1.0) * 2.0
                            qw = 0.25 * S
                            qx = (R_c2w[2, 1] - R_c2w[1, 2]) / S
                            qy = (R_c2w[0, 2] - R_c2w[2, 0]) / S
                            qz = (R_c2w[1, 0] - R_c2w[0, 1]) / S
                        elif (R_c2w[0, 0] > R_c2w[1, 1]) and (R_c2w[0, 0] > R_c2w[2, 2]):
                            S = np.sqrt(1.0 + R_c2w[0, 0] - R_c2w[1, 1] - R_c2w[2, 2]) * 2.0
                            qw = (R_c2w[2, 1] - R_c2w[1, 2]) / S
                            qx = 0.25 * S
                            qy = (R_c2w[0, 1] + R_c2w[1, 0]) / S
                            qz = (R_c2w[0, 2] + R_c2w[2, 0]) / S
                        elif R_c2w[1, 1] > R_c2w[2, 2]:
                            S = np.sqrt(1.0 + R_c2w[1, 1] - R_c2w[0, 0] - R_c2w[2, 2]) * 2.0
                            qw = (R_c2w[0, 2] - R_c2w[2, 0]) / S
                            qx = (R_c2w[0, 1] + R_c2w[1, 0]) / S
                            qy = 0.25 * S
                            qz = (R_c2w[1, 2] + R_c2w[2, 1]) / S
                        else:
                            S = np.sqrt(1.0 + R_c2w[2, 2] - R_c2w[0, 0] - R_c2w[1, 1]) * 2.0
                            qw = (R_c2w[1, 0] - R_c2w[0, 1]) / S
                            qx = (R_c2w[0, 2] + R_c2w[2, 0]) / S
                            qy = (R_c2w[1, 2] + R_c2w[2, 1]) / S
                            qz = 0.25 * S
                            
                        self.state_q = np.array([qw, qx, qy, qz])
                except Exception as pe_err:
                    self.get_logger().warn(f"[LingBot-Map] pose_encoding_to_extri_intri warning: {pe_err}")

            # Decode predicted 3D world points from LingBot-Map
            if isinstance(preds, dict):
                sampled_pts = None
                flat_colors = None

                if "world_points" in preds and preds["world_points"] is not None:
                    pts_tensor = preds["world_points"].squeeze().cpu().numpy()
                    if pts_tensor.ndim == 3:
                        pts_tensor = pts_tensor.reshape(-1, 3)
                    mask = ~np.isnan(pts_tensor).any(axis=1) & ~np.isinf(pts_tensor).any(axis=1)
                    valid_pts = pts_tensor[mask]
                    if len(valid_pts) > 0:
                        step = max(1, len(valid_pts) // 800)
                        sampled_pts = valid_pts[::step]
                        flat_colors = resized.reshape(-1, 3)[mask][::step]
                elif "depth" in preds and preds["depth"] is not None and "intri" in locals() and "extri_np" in locals():
                    try:
                        depth_np = preds["depth"].squeeze().cpu().numpy()
                        intri_np = intri.squeeze().cpu().numpy()
                        h, w = depth_np.shape[:2]
                        fx, fy = intri_np[0, 0], intri_np[1, 1]
                        cx, cy = intri_np[0, 2], intri_np[1, 2]
                        
                        u, v = np.meshgrid(np.arange(w), np.arange(h))
                        mask = (depth_np > 0.1) & (depth_np < 10.0) & ~np.isnan(depth_np)
                        z_c = depth_np[mask]
                        u_c = u[mask]
                        v_c = v[mask]
                        
                        x_c = (u_c - cx) * z_c / fx
                        y_c = (v_c - cy) * z_c / fy
                        cam_pts = np.stack([x_c, y_c, z_c], axis=-1)
                        
                        R_w2c = extri_np[:3, :3]
                        t_w2c = extri_np[:3, 3]
                        R_c2w = R_w2c.T
                        t_c2w = -R_c2w @ t_w2c
                        world_pts = (cam_pts @ R_c2w.T) + t_c2w
                        
                        if len(world_pts) > 0:
                            step = max(1, len(world_pts) // 800)
                            sampled_pts = world_pts[::step]
                            flat_colors = resized.reshape(-1, 3)[mask.reshape(-1)][::step]
                    except Exception as dp_err:
                        self.get_logger().warn(f"[LingBot-Map] Depth backprojection warning: {dp_err}")

                if sampled_pts is not None and flat_colors is not None:
                    for p, c in zip(sampled_pts, flat_colors):
                        self.all_pts_3d.append(p.tolist())
                        self.all_colors.append(c.tolist())

            if len(self.all_pts_3d) > 20000:
                self.all_pts_3d = self.all_pts_3d[-20000:]
                self.all_colors = self.all_colors[-20000:]

            # Publish Standard Odometry on /scanar/odometry
            odom = Odometry()
            odom.header = header
            odom.header.frame_id = "odom"
            odom.child_frame_id = "viture_camera_frame"
            odom.pose.pose.position.x = float(self.state_pos[0])
            odom.pose.pose.position.y = float(self.state_pos[1])
            odom.pose.pose.position.z = float(self.state_pos[2])
            odom.pose.pose.orientation.w = float(self.state_q[0])
            odom.pose.pose.orientation.x = float(self.state_q[1])
            odom.pose.pose.orientation.y = float(self.state_q[2])
            odom.pose.pose.orientation.z = float(self.state_q[3])
            self.odom_pub.publish(odom)

            # Publish Generic ReconstructionFrame
            recon = ReconstructionFrame()
            recon.header = header
            recon.tracking_engine = "lingbot_map"
            recon.pose = odom.pose.pose
            recon.confidence = 95.0

            if self.all_pts_3d:
                pts_np = np.array(self.all_pts_3d, dtype=np.float32)
                colors_np = np.array(self.all_colors, dtype=np.uint8)
                recon.x = pts_np[:, 0].tolist()
                recon.y = pts_np[:, 1].tolist()
                recon.z = pts_np[:, 2].tolist()
                recon.r = colors_np[:, 0].tolist()
                recon.g = colors_np[:, 1].tolist()
                recon.b = colors_np[:, 2].tolist()

            # Export active reconstruction deliverables (throttled to 5.0s interval)
            now = time.time()
            if self.active_dir and len(self.all_pts_3d) > 0:
                if (now - getattr(self, 'last_export_time', 0.0)) >= 5.0:
                    self.export_reconstruction_pipeline()
                    self.last_export_time = now

        except Exception as e:
            self.get_logger().error(f"[LingBot-Map] CUDA inference error: {e}")

    def run_simulated_mapper(self, gray, cv_img, header):
        # Developer simulation mode only
        pass

    def handle_active_directory(self, msg):
        target_dir = msg.data
        self.get_logger().info(f"[LingBot-Map] handle_active_directory received: '{target_dir}' (current active_dir: '{self.active_dir}')")
        if target_dir == self.active_dir:
            return

        if self.active_dir:
            self.export_reconstruction_pipeline()

        self.active_dir = target_dir
        if self.active_dir:
            self.all_pts_3d = []
            self.all_colors = []
            self.prev_gray = None
            self.prev_pts = None

    def export_reconstruction_pipeline(self):
        self.get_logger().info(f"[LingBot-Map] Export pipeline triggered. Target: {self.active_dir}")

        capture_dataset_dir = os.path.join(self.active_dir, "capture_dataset")
        os.makedirs(capture_dataset_dir, exist_ok=True)
        os.makedirs(os.path.join(capture_dataset_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(capture_dataset_dir, "imu"), exist_ok=True)
        os.makedirs(os.path.join(capture_dataset_dir, "poses"), exist_ok=True)

        splat_out_path = os.path.join(self.active_dir, "scene.splat")
        ply_out_path = os.path.join(self.active_dir, "scene.ply")

        # 1. Export scene.ply
        try:
            with open(ply_out_path, 'w') as f:
                f.write("ply\n")
                f.write("format ascii 1.0\n")
                f.write(f"element vertex {len(self.all_pts_3d)}\n")
                f.write("property float x\n")
                f.write("property float y\n")
                f.write("property float z\n")
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")
                f.write("end_header\n")
                for i in range(len(self.all_pts_3d)):
                    p = self.all_pts_3d[i]
                    c = self.all_colors[i]
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
            self.get_logger().info(f"Exported PLY point cloud: {ply_out_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to export PLY point cloud: {e}")

        # 2. Export scene.splat
        try:
            with open(splat_out_path, 'wb') as f:
                for i in range(len(self.all_pts_3d)):
                    p = self.all_pts_3d[i]
                    c = self.all_colors[i]
                    # Position
                    f.write(struct.pack('fff', p[0], p[1], p[2]))
                    # Scale (0.05 nominal scale)
                    f.write(struct.pack('fff', 0.05, 0.05, 0.05))
                    # Color RGBA
                    f.write(bytes([int(c[0]), int(c[1]), int(c[2]), 255]))
                    # Rotation
                    f.write(bytes([128, 128, 128, 255]))
            self.get_logger().info(f"Exported WebGL Gaussian Splat: {splat_out_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to export splat: {e}")

        # 3. Export metadata.json
        try:
            metadata_path = os.path.join(capture_dataset_dir, "metadata.json")
            metadata = {
                "system": "ScanAR G",
                "sensor": "VITURE Luma Ultra XR Glasses",
                "gaussian_count": len(self.all_pts_3d),
                "fps_camera": 30.0,
                "fx": self.fx,
                "fy": self.fy,
                "cx": self.cx,
                "cy": self.cy,
                "export_time": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            self.get_logger().info(f"Exported dataset metadata: {metadata_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to export metadata: {e}")

    def quat_to_rot(self, qw, qx, qy, qz):
        R = np.zeros((3, 3), dtype=np.float32)
        R[0, 0] = 1 - 2 * (qy**2 + qz**2)
        R[0, 1] = 2 * (qx*qy - qz*qw)
        R[0, 2] = 2 * (qx*qz + qy*qw)
        R[1, 0] = 2 * (qx*qy + qz*qw)
        R[1, 1] = 1 - 2 * (qx**2 + qz**2)
        R[1, 2] = 2 * (qy*qz - qx*qw)
        R[2, 0] = 2 * (qx*qz - qy*qw)
        R[2, 1] = 2 * (qy*qz + qx*qw)
        R[2, 2] = 1 - 2 * (qx**2 + qy**2)
        return R

def main(args=None):
    rclpy.init(args=args)
    node = LingBotBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
