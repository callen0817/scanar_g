#!/usr/bin/env python3
"""
hud_renderer.py — ScanAR G Production HUD Renderer
======================================================
ROS2 node that receives live telemetry and renders the operator HUD
to the connected VITURE glasses display at 60 Hz using OpenCV.

Architecture:
- Main Display: 100% Optical See-Through AR (Black canvas = 0.0 nits emissive light)
- Right-Side Tool PIP Stack:
    1. SLAM Map PIP (2D Floorplan, trajectory, heading arrow, point cloud projection)
    2. RGB Camera PIP (Live 60 FPS RGB stream sensor health monitor)
    3. Pose/Path PIP (Directional navigation arrow, confidence meter, 6DoF distance)
- Left-Side: Engineering Overlay (Pinned at X=0, Y=44)
- Top Bar: Action Buttons + Tool PIP Toggles + Config Switcher ([K] CFG: SCANAR G / SCANAR C)
- Full Keyboard Hotkeys: R (Record/Stop), X (Reset), S (Save), E (ENG), M (Map), C (RGB), P (Pose), K (Config Cycle), H (Hide All), F (Fullscreen), Q/ESC (Quit)
- ROS 2 Session Publisher: Publishes session control commands to `/scanar/session/command`
"""

import os
import cv2
import numpy as np
import math
import time
import json
import collections

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from scanar_interfaces.msg import ScanConfidence, GaussianSplatArray, SystemHealth, ReconstructionFrame
from cv_bridge import CvBridge

from hud_widgets.keyplan import KeyplanWidget
try:
    from viture_hud.product_capabilities import get_product_capability
except ImportError:
    from product_capabilities import get_product_capability

# Display constants
WIDTH, HEIGHT = 1920, 1080
FPS           = 60
DISPLAY       = os.environ.get("DISPLAY", ":1")

# Theme colors (BGR) — Pure pitch black (0,0,0) turns Micro-OLED subpixels OFF completely for 0.0 nit AR see-through transparency
BG_COLOR = (0, 0, 0)
C_CYAN  = (220, 205,   0)
C_GREEN = ( 70, 230, 120)
C_AMBER = (  0, 175, 255)
C_RED   = ( 55,  55, 240)
C_WHITE = (235, 235, 235)
C_DIM   = ( 90,  90,  90)
C_PANEL = ( 0,   0,   0)
C_EDGE  = ( 50, 185, 205)

FONT = cv2.FONT_HERSHEY_SIMPLEX

# PIP Stack geometry — Right-side vertical stack
KP_W = 340
KP_X = WIDTH - KP_W - 24  # 1556

LAT_HIST = collections.deque(maxlen=100)

def _blink(ca, cb, hz=1.0):
    return ca if math.sin(time.time() * hz * 2 * math.pi) > 0 else cb

def _lat_color(ms):
    return C_GREEN if ms < 33 else (C_AMBER if ms < 50 else C_RED)

def _hbar(frame, x, y, w, h, val, maxv, col):
    cv2.rectangle(frame, (x, y), (x + w, y + h), (30, 30, 30), -1)
    fill = int(w * min(val, maxv) / maxv)
    if fill > 0:
        cv2.rectangle(frame, (x, y), (x + fill, y + h), col, -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), C_DIM, 1)

def _panel(frame, x, y, w, h, alpha=0.82):
    ov = frame.copy()
    cv2.rectangle(ov, (x, y), (x + w, y + h), C_PANEL, -1)
    cv2.addWeighted(ov, alpha, frame, 1 - alpha, 0, frame)
    cv2.rectangle(frame, (x, y), (x + w, y + h), C_EDGE, 1)

def _get_gpu_load():
    try:
        with open("/sys/class/devfreq/17000000.gpu/device/load", "r") as f:
            return float(f.read().strip()) / 10.0
    except Exception:
        return 15.0

def _get_gpu_memory():
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
        total, free = 0, 0
        for line in lines:
            if line.startswith("MemTotal:"):
                total = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                free = int(line.split()[1])
        if total > 0:
            used_gb = (total - free) / (1024 * 1024)
            total_gb = total / (1024 * 1024)
            return used_gb, total_gb
    except Exception:
        pass
    return 3.2, 16.0

def _draw_engineering(frame, cpu, ram, temp, lat_ms, confidence, stats_db, dropped_f, profile_name):
    EX, EY, EW, EH = 0, 44, 360, 520
    _panel(frame, EX, EY, EW, EH)
    cv2.putText(frame, "ENGINEERING STATUS", (EX + 10, EY + 18),
                FONT, 0.40, (0, 255, 0), 1, cv2.LINE_AA)
    y = EY + 40

    gpu_load = _get_gpu_load()
    vram_used, vram_total = _get_gpu_memory()

    def row(lbl, val, col=C_WHITE):
        nonlocal y
        cv2.putText(frame, lbl,  (EX + 14, y), FONT, 0.37, C_DIM,  1, cv2.LINE_AA)
        cv2.putText(frame, val,  (EX + 185, y), FONT, 0.39, col,    1, cv2.LINE_AA)
        y += 24

    def bar(lbl, val, maxv, col):
        nonlocal y
        cv2.putText(frame, lbl, (EX + 14, y), FONT, 0.37, C_DIM, 1, cv2.LINE_AA)
        _hbar(frame, EX + 110, y - 13, EW - 138, 13, val, maxv, col)
        cv2.putText(frame, f"{val:.0f}", (EX + EW - 42, y),
                    FONT, 0.34, col, 1, cv2.LINE_AA)
        y += 26

    profile_label = self.profile.name.upper()
    row("PROFILE", profile_label, C_CYAN)
    y += 4

    cpu_c = C_GREEN if cpu < 60 else (C_AMBER if cpu < 85 else C_RED)
    row("CPU LOAD",   f"{cpu:.0f}%",  cpu_c)
    bar("",           cpu, 100,       cpu_c)

    gpu_c = C_GREEN if gpu_load < 60 else (C_AMBER if gpu_load < 85 else C_RED)
    row("GPU UTIL",   f"{gpu_load:.1f}%", gpu_c)
    bar("",           gpu_load, 100,       gpu_c)

    ram_c = C_GREEN if ram < 70 else C_AMBER
    row("VRAM USED",  f"{vram_used:.1f} / {vram_total:.0f} GB", ram_c)
    bar("",           (vram_used/vram_total)*100.0, 100, ram_c)

    tmp_c = C_GREEN if temp < 65 else (C_AMBER if temp < 78 else C_RED)
    row("CPU TEMP",   f"{temp:.0f}°C", tmp_c)
    bar("",           temp, 100,       tmp_c)

    y += 4
    row("ENGINE", self.profile.reconstruction_engine, C_GREEN)
    row("MAP UPDATE", f"{stats_db.get('optimization_fps', 0.0):.1f} Hz", C_CYAN)
    row("RECON POINTS", f"{stats_db.get('active_splats', 0)} pts", C_CYAN)
    row("DROPPED FRAMES", f"{dropped_f}", C_RED if dropped_f > 10 else C_CYAN)

class HudRenderer(Node):
    def __init__(self):
        super().__init__('hud_renderer')

        # Single-Instance Process Lock
        try:
            import sys
            sys.path.append('/home/scanarstereo/scanAR_G/src/scanar_core')
            from scanar_profiles.process_lock import ProcessLock
            self._lock = ProcessLock('hud')
        except Exception as e:
            self.get_logger().warn(f"[HUD Renderer] Lock warning: {e}")

        # Declare product profile parameters
        self.confidence = 95.0
        self.camera_fps = 30.0
        self.imu_rate_hz = 100.0
        self.vigs_status = "INITIALIZING"
        self.gaussian_count = 0
        self.active_dir = ""
        self.recording_state = "STBY"  # "STBY", "PREPARING", "RECORDING"
        self._prep_start_t = 0.0
        
        # System health metrics
        self.cpu_load = 0.0
        self.ram_pct = 0.0
        self.cpu_temp = 0.0
        self.latency_ms = 0.0
        self.dist_m = 0.0
        self.calib_ok = True
        
        # PIP & Overlay Toggles (Pass-Through AR is ALWAYS the main view)
        self.show_eng = False
        self.show_slam_pip = True
        self.show_rgb_pip = True
        self.show_pose_pip = True
        self.is_fullscreen = False
        self.should_exit = False
        
        # V1.5 statistics & metrics
        self.stats_db = {}
        self.dropped_frames = 0
        self.last_draw_time = time.time()
        self._t_start = time.time()
        self.mouse_pos = (0, 0)
        self._last_click_msg = ""
        self._last_click_t = 0.0

        # Keyplan widget — maps the real-time floor plan
        self.keyplan = KeyplanWidget()

        # Camera Intrinsics
        self.fx = 960.0
        self.fy = 960.0
        self.cx = WIDTH / 2.0
        self.cy = HEIGHT / 2.0

        # Current pose
        self.pose_pos = np.array([0.0, 0.0, 0.0])
        self.pose_rot_mat = np.eye(3)

        # Current 3D Gaussian splats
        self.splat_positions = np.zeros((0, 3), dtype=np.float32)
        self.splat_colors = np.zeros((0, 3), dtype=np.uint8)
        self.splat_scales = np.zeros((0,), dtype=np.float32)
        self.splat_opacities = np.zeros((0,), dtype=np.float32)

        self.bridge = CvBridge()
        self.camera_image = None
        self.pip_image = None

        # Declare and get product parameter & capability profile
        self.declare_parameter('product', 'scanar_g')
        self.product = self.get_parameter('product').get_parameter_value().string_value
        self.profile = get_product_capability(self.product)
        self.capability = self.profile

        # Low-latency camera QoS profile (Depth 1, Keep Last, Best Effort)
        cam_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ROS 2 Command Publisher
        self.pub_cmd = self.create_publisher(String, '/scanar/session/command', 10)

        # ROS subscriptions derived directly from profile plugin
        if self.profile.camera_topic:
            self.sub_cam = self.create_subscription(Image, self.profile.camera_topic, self._cb_image, cam_qos)
        else:
            self.sub_cam = None
        self.create_subscription(Imu, '/viture/imu', self._cb_imu_data, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/scanar/odometry', self._cb_odom, 10)
        self.create_subscription(ReconstructionFrame, '/scanar/reconstruction', self._cb_reconstruction, 10)
        self.create_subscription(Float32, '/scanar/scan_confidence', self._cb_confidence, 10)
        self.create_subscription(String, '/vigs/status', self._cb_status, 10)
        self.create_subscription(Float32, '/viture/camera/fps', self._cb_fps, 10)
        self.create_subscription(Float32, '/viture/imu/rate', self._cb_imu_rate, 10)
        self.create_subscription(String, '/scanar/session/active_directory', self._cb_active_dir, 10)
        self.create_subscription(SystemHealth, '/scanar/diagnostics/system_health', self._cb_health, 10)
        self.create_subscription(String, '/scanar/diagnostics/latency_report', self._cb_latency, 10)
        self.create_subscription(Bool, '/scanar/hud/eng_mode', self._cb_eng, 10)
        self.create_subscription(String, '/vigs/statistics', self._cb_stats_json, 10)

        self.get_logger().info("ScanAR G V1.5 HUD Renderer Node Initialized (Pass-Through AR Mode + PIP Stack).")

    def cycle_product_config(self):
        configs = ["scanar_g", "scanar_c", "scanar_s", "scanar_s2", "scanar_l", "scanar_l2", "scanar_pro"]
        curr_idx = configs.index(self.product) if self.product in configs else 0
        next_idx = (curr_idx + 1) % len(configs)
        self.product = configs[next_idx]
        self.profile = get_product_capability(self.product)
        self.capability = self.profile

        if hasattr(self, 'sub_cam') and self.sub_cam is not None:
            self.destroy_subscription(self.sub_cam)
            self.sub_cam = None

        if self.profile.camera_topic:
            cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
            self.sub_cam = self.create_subscription(Image, self.profile.camera_topic, self._cb_image, cam_qos)

        self.camera_image = None
        self.pip_image = None
        self._last_click_msg = f"CONFIGURATION SWITCHED TO: {self.profile.name.upper()}"
        self._last_click_t = time.time()
        self.get_logger().info(f"Product configuration switched via HUD -> {self.profile.name} ({self.profile.camera_topic})")

    def trigger_action(self, action_id: str):
        msg = String()
        msg.data = action_id
        self.pub_cmd.publish(msg)
        
        if action_id in ("start", "rec", "toggle_record", "stop"):
            if not self.active_dir and self.recording_state != "RECORDING":
                self.recording_state = "PREPARING"
                self._prep_start_t = time.time()
                self._last_click_msg = "INITIALIZING SLAM & RECORDING PIPELINE..."
                self.get_logger().info("Action -> START SCAN / PREPARING RECORDING")
            else:
                self.recording_state = "STBY"
                self.active_dir = ""
                self._last_click_msg = "SCAN STOPPED. RECORDING SAVED."
                self.get_logger().info("Action -> STOP SCAN / RECORDING SAVED")
        elif action_id == "reset":
            if hasattr(self, 'keyplan') and self.keyplan is not None:
                self.keyplan.reset()
            self.dist_m = 0.0
            self._last_click_msg = "MINIMAP & TRAJECTORY RESET OK"
            self.get_logger().info("Action -> RESET MAP")
        elif action_id == "save":
            self._last_click_msg = "CAPTURE SESSION SAVED OK"
            self.get_logger().info("Action -> SAVE CAPTURE")
        elif action_id == "export":
            self._last_click_msg = "EXPORTING 3D GAUSSIAN SPLATS (.splat & .ply)..."
            self.get_logger().info("Action -> EXPORT SPLATS")
        
        self._last_click_t = time.time()

    def _cb_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.camera_image = cv_img
            # Direct bilinear downsampling for 60 FPS PIP camera monitor (< 30ms latency)
            self.pip_image = cv2.resize(cv_img, (KP_W, 200), interpolation=cv2.INTER_LINEAR)
        except Exception:
            pass

    def _cb_imu_data(self, msg: Imu):
        dt = 0.01
        gyro_y = msg.angular_velocity.y
        if hasattr(self, 'keyplan') and self.keyplan is not None:
            self.keyplan.heading += gyro_y * dt

    def _cb_odom(self, msg: Odometry):
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.y - q.x * q.z),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)) + math.pi / 2.0

        if len(self.keyplan._trajectory) > 0:
            self.dist_m += math.hypot(tx - self.pose_pos[0], tz - self.pose_pos[2])

        self.pose_pos = np.array([tx, ty, tz])
        self.pose_rot_mat = self.quat_to_rot(q.w, q.x, q.y, q.z)
        self.keyplan.update_pose(tx, tz, yaw)

    def _cb_reconstruction(self, msg: ReconstructionFrame):
        if msg.tracking_engine:
            self.tracking_engine = msg.tracking_engine
        self.gaussian_count = len(msg.x)
        if self.gaussian_count == 0:
            self.splat_positions = np.zeros((0, 3), dtype=np.float32)
            self.splat_colors = np.zeros((0, 3), dtype=np.uint8)
            self.splat_scales = np.zeros((0,), dtype=np.float32)
            self.splat_opacities = np.zeros((0,), dtype=np.float32)
            return

        self.splat_positions = np.zeros((self.gaussian_count, 3), dtype=np.float32)
        self.splat_positions[:, 0] = msg.x
        self.splat_positions[:, 1] = msg.y
        self.splat_positions[:, 2] = msg.z

        self.splat_colors = np.zeros((self.gaussian_count, 3), dtype=np.uint8)
        self.splat_colors[:, 0] = msg.b
        self.splat_colors[:, 1] = msg.g
        self.splat_colors[:, 2] = msg.r

        self.splat_scales = np.ones((self.gaussian_count,), dtype=np.float32) * 0.05
        self.splat_opacities = np.ones((self.gaussian_count,), dtype=np.float32) * 1.0

        map_points_2d = list(zip(msg.x, msg.z))
        if self.capability.color_mode == "natural":
            map_colors_bgr = list(zip(msg.b, msg.g, msg.r))
            self.keyplan.update_map_points(map_points_2d, colors=map_colors_bgr)
        else:
            self.keyplan.update_map_points(map_points_2d, colors=None)

    def _cb_confidence(self, msg: Float32):
        self.confidence = msg.data

    def _cb_status(self, msg: String):
        self.vigs_status = msg.data

    def _cb_fps(self, msg: Float32):
        self.camera_fps = msg.data

    def _cb_imu_rate(self, msg: Float32):
        self.imu_rate_hz = msg.data

    def _cb_active_dir(self, msg: String):
        self.active_dir = msg.data
        if self.active_dir:
            self.recording_state = "RECORDING"

    def _cb_health(self, msg: SystemHealth):
        self.cpu_load = msg.cpu_load
        self.ram_pct = msg.memory_usage
        self.cpu_temp = msg.cpu_temperature

    def _cb_latency(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latency_ms = float(data.get("latency_profile_ms", {}).get("total_pipeline_latency_ms", 0.0))
            LAT_HIST.append(self.latency_ms)
        except Exception:
            pass

    def _cb_eng(self, msg: Bool):
        self.show_eng = msg.data

    def _cb_stats_json(self, msg: String):
        try:
            self.stats_db = json.loads(msg.data)
        except Exception:
            pass

    def draw(self, frame: np.ndarray) -> None:
        now = time.time()
        dt = now - self.last_draw_time
        self.last_draw_time = now
        if dt > 1.5 * (1.0 / 60.0):
            self.dropped_frames += int(dt / (1.0 / 60.0)) - 1

        # 100% Optical See-Through AR: Pitch black background (0,0,0) turns Micro-OLED subpixels OFF completely
        frame[:] = BG_COLOR

        # Transition from PREPARING to RECORDING state after 1.5s simulation if no active_dir callback received
        if self.recording_state == "PREPARING" and (now - self._prep_start_t) > 1.5:
            self.recording_state = "RECORDING"
            if not self.active_dir:
                self.active_dir = "/tmp/scanar_session_active"

        # 1. Project and render 3D Gaussian Splats over 100% See-Through AR Passthrough
        if len(self.splat_positions) > 0:
            rel_pos = self.splat_positions - self.pose_pos
            p_cam = rel_pos @ self.pose_rot_mat

            mask = p_cam[:, 2] > 0.1
            if np.any(mask):
                p_cam_filt = p_cam[mask]
                colors_filt = self.splat_colors[mask]
                scales_filt = self.splat_scales[mask]

                u = (self.fx * p_cam_filt[:, 0] / p_cam_filt[:, 2]) + self.cx
                v = (self.fy * p_cam_filt[:, 1] / p_cam_filt[:, 2]) + self.cy

                screen_mask = (u >= 0) & (u < WIDTH) & (v >= 0) & (v < HEIGHT)
                if np.any(screen_mask):
                    u_scr = u[screen_mask]
                    v_scr = v[screen_mask]
                    z_scr = p_cam_filt[:, 2][screen_mask]
                    col_scr = colors_filt[screen_mask]
                    scale_scr = scales_filt[screen_mask]

                    sort_idx = np.argsort(z_scr)[::-1]
                    for idx in sort_idx:
                        px = int(u_scr[idx])
                        py = int(v_scr[idx])
                        if self.product in ('scanar_g', 'scanar_c'):
                            r_pix = 2
                        else:
                            r_pix = int((scale_scr[idx] * self.fx) / z_scr[idx])
                            r_pix = max(2, min(r_pix, 150))
                        color = (int(col_scr[idx][0]), int(col_scr[idx][1]), int(col_scr[idx][2]))
                        cv2.circle(frame, (px, py), r_pix, color, -1, cv2.LINE_AA)

        # 2. Render Top Strip Dashboard (Transparent glass bar with cyan bottom border)
        cv2.rectangle(frame, (0, 0), (WIDTH, 44), (0, 0, 0), -1)
        cv2.line(frame, (0, 44), (WIDTH, 44), C_EDGE, 1)

        # REC / PREP / STBY indicator
        if self.recording_state == "RECORDING" or self.active_dir:
            rec_col = _blink(C_RED, C_DIM, hz=1.5)
            rec_lbl = "REC"
        elif self.recording_state == "PREPARING":
            rec_col = _blink(C_AMBER, C_GREEN, hz=3.0)
            rec_lbl = "PREP"
        else:
            rec_col = C_DIM
            rec_lbl = "STBY"

        cv2.circle(frame, (24, 22), 9, rec_col, -1)
        cv2.putText(frame, rec_lbl, (40, 28), FONT, 0.46, rec_col, 1, cv2.LINE_AA)

        # Configuration Rating (SCANAR G / C: HIGH)
        conf_str = "LOST"
        conf_col = C_RED
        if self.confidence >= 90.0:
            conf_str = "HIGH"
            conf_col = (0, 255, 0)
        elif self.confidence >= 70.0:
            conf_str = "MEDIUM"
            conf_col = C_CYAN
        elif self.confidence >= 40.0:
            conf_str = "LOW"
            conf_col = (0, 255, 0)

        config_name = self.capability.name.upper()
        cv2.circle(frame, (118, 22), 6, conf_col, -1)
        cv2.putText(frame, f"{config_name}: {conf_str}", (132, 28), FONT, 0.44, conf_col, 1, cv2.LINE_AA)

        # PIP & Overlay Toggle Buttons + Configuration Cycle Button on Top Bar
        toggle_buttons = [
            ("eng", "[E] ENG", 300, 75, self.show_eng),
            ("map", "[M] MAP", 385, 85, self.show_slam_pip and self.profile.supports_view("map")),
            ("rgb", "[C] RGB", 480, 80, self.show_rgb_pip and self.profile.supports_view("rgb")),
            ("pose", "[P] POSE", 570, 85, self.show_pose_pip and self.profile.supports_view("pose")),
            ("cfg", f"[K] CFG: {config_name}", 665, 160, True),
        ]

        for t_id, t_label, bx, bw, is_act in toggle_buttons:
            btn_col = C_CYAN if t_id == "cfg" else ((0, 255, 0) if is_act else C_CYAN)
            lbl_txt = t_label if t_id == "cfg" else (f"{t_label} ✓" if is_act else t_label)
            cv2.rectangle(frame, (bx, 8), (bx + bw, 36), (0, 0, 0), -1)
            cv2.rectangle(frame, (bx, 8), (bx + bw, 36), btn_col, 1)
            cv2.putText(frame, lbl_txt, (bx + 8, 26), FONT, 0.35, btn_col, 1, cv2.LINE_AA)

        # Action Menu Buttons on Right Side
        rec_btn_lbl = "STOP REC" if (self.active_dir or self.recording_state == "RECORDING") else ("PREPARING..." if self.recording_state == "PREPARING" else "START REC")
        rec_btn_col = C_RED if (self.active_dir or self.recording_state == "RECORDING") else (C_AMBER if self.recording_state == "PREPARING" else (0, 255, 0))

        action_buttons = [
            ("rec", rec_btn_lbl, 1420, 130, rec_btn_col),
            ("reset", "RESET MAP", 1560, 105, C_CYAN),
            ("save", "SAVE", 1675, 75, C_CYAN),
            ("export", "EXPORT", 1760, 90, C_CYAN),
        ]

        for act_id, act_label, bx, bw, act_col in action_buttons:
            cv2.rectangle(frame, (bx, 8), (bx + bw, 36), (0, 0, 0), -1)
            cv2.rectangle(frame, (bx, 8), (bx + bw, 36), act_col, 1)
            cv2.putText(frame, act_label, (bx + 8, 26), FONT, 0.34, act_col, 1, cv2.LINE_AA)

        # Click Confirmation Visual Flash Banner
        if hasattr(self, '_last_click_msg') and time.time() - self._last_click_t < 2.5:
            cv2.rectangle(frame, (WIDTH // 2 - 300, 48), (WIDTH // 2 + 300, 84), (0, 0, 0), -1)
            cv2.rectangle(frame, (WIDTH // 2 - 300, 48), (WIDTH // 2 + 300, 84), (0, 255, 0), 2)
            cv2.putText(frame, self._last_click_msg, (WIDTH // 2 - 285, 72), FONT, 0.48, (0, 255, 0), 2, cv2.LINE_AA)

        # High-Contrast Bold Hover Tooltip Popup Rendering (Positioned safely at Y=52)
        tooltip_txt = None
        if hasattr(self, 'mouse_pos') and self.mouse_pos is not None:
            mx, my = self.mouse_pos
            if 0 <= my <= 44:
                if 300 <= mx <= 375:
                    tooltip_txt = "Toggle real-time CPU, RAM, IMU & SLAM engineering metrics panel."
                elif 385 <= mx <= 470:
                    tooltip_txt = "Toggle live 2D SLAM floorplan & trajectory PIP map."
                elif 480 <= mx <= 560:
                    tooltip_txt = "Toggle live 60 FPS RGB camera feed PIP monitor."
                elif 570 <= mx <= 655:
                    tooltip_txt = "Toggle navigation directional arrow & 6DoF pose PIP window."
                elif 665 <= mx <= 825:
                    tooltip_txt = "Cycle product profile (ScanAR G, ScanAR C, ScanAR S, ScanAR L) (Hotkey: K)."
                elif 1420 <= mx <= 1550:
                    tooltip_txt = "Start or stop continuous dataset session recording (Hotkey: R)."
                elif 1560 <= mx <= 1665:
                    tooltip_txt = "Reset 2D minimap & clear trajectory path (Hotkey: X)."
                elif 1675 <= mx <= 1750:
                    tooltip_txt = "Save current capture session state (Hotkey: S)."
                elif 1760 <= mx <= 1850:
                    tooltip_txt = "Export 3D Gaussian Splats (.splat & .ply) (Hotkey: E)."

        if tooltip_txt:
            tt_w = len(tooltip_txt) * 10 + 30
            tx = max(10, min(mx, WIDTH - tt_w - 10))
            ty = 52
            cv2.rectangle(frame, (tx, ty), (tx + tt_w, ty + 36), (0, 0, 0), -1)
            cv2.rectangle(frame, (tx, ty), (tx + tt_w, ty + 36), (0, 255, 0), 2)
            cv2.putText(frame, tooltip_txt, (tx + 12, ty + 24), FONT, 0.50, (255, 255, 255), 2, cv2.LINE_AA)

        # 3. RIGHT-SIDE TOOL PIP STACK
        # (A) Top PIP: SLAM Map PIP (2D Floorplan, Trajectory, Point Cloud Projection)
        if self.show_slam_pip:
            pip_slam_y = 60
            pip_slam_h = 240
            cv2.putText(frame, "LIVE SLAM MAP PIP", (KP_X, pip_slam_y - 6), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)
            self.keyplan.draw(frame, KP_X, pip_slam_y, KP_W, pip_slam_h)

        # (B) Middle PIP: Live RGB Camera PIP Monitor
        if self.show_rgb_pip and self.profile.supports_view("rgb"):
            pip_rgb_y = 320
            pip_rgb_h = 200
            cv2.putText(frame, "LIVE RGB STREAM (60 FPS)", (KP_X, pip_rgb_y - 6), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)
            if self.pip_image is not None and self.pip_image.shape[:2] == (pip_rgb_h, KP_W):
                frame[pip_rgb_y:pip_rgb_y + pip_rgb_h, KP_X:KP_X + KP_W] = self.pip_image
            else:
                cv2.rectangle(frame, (KP_X, pip_rgb_y), (KP_X + KP_W, pip_rgb_y + pip_rgb_h), (20, 20, 20), -1)
                cv2.putText(frame, "NO RGB CAMERA FEED", (KP_X + 60, pip_rgb_y + 100), FONT, 0.42, C_DIM, 1, cv2.LINE_AA)
            cv2.rectangle(frame, (KP_X, pip_rgb_y), (KP_X + KP_W, pip_rgb_y + pip_rgb_h), C_EDGE, 1)

        # (C) Bottom PIP: Pose & Trajectory HUD PIP
        if self.show_pose_pip:
            pip_pose_y = 540
            pip_pose_h = 180
            _panel(frame, KP_X, pip_pose_y, KP_W, pip_pose_h)
            cv2.putText(frame, "POSE & TRAJECTORY PIP", (KP_X + 10, pip_pose_y + 20), FONT, 0.40, (0, 255, 0), 1, cv2.LINE_AA)
            
            # Draw Directional Navigation Arrow
            cx_arrow = KP_X + 60
            cy_arrow = pip_pose_y + 100
            yaw = getattr(self.keyplan, 'heading', 0.0)
            a_len = 35
            tip_x = int(cx_arrow + a_len * math.sin(yaw))
            tip_y = int(cy_arrow - a_len * math.cos(yaw))
            
            # Triangle base
            b1_x = int(cx_arrow + 14 * math.sin(yaw + 2.4))
            b1_y = int(cy_arrow - 14 * math.cos(yaw + 2.4))
            b2_x = int(cx_arrow + 14 * math.sin(yaw - 2.4))
            b2_y = int(cy_arrow - 14 * math.cos(yaw - 2.4))
            
            arrow_pts = np.array([[tip_x, tip_y], [b1_x, b1_y], [b2_x, b2_y]], np.int32)
            cv2.fillPoly(frame, [arrow_pts], (0, 255, 0), cv2.LINE_AA)
            cv2.polylines(frame, [arrow_pts], True, C_WHITE, 1, cv2.LINE_AA)
            
            # Pose telemetry details
            cv2.putText(frame, f"CONFIDENCE: {self.confidence:.0f}%", (KP_X + 130, pip_pose_y + 60), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)
            cv2.putText(frame, f"DISTANCE: {self.dist_m:.1f} m", (KP_X + 130, pip_pose_y + 90), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)
            cv2.putText(frame, f"LATENCY: {self.latency_ms:.0f} ms", (KP_X + 130, pip_pose_y + 120), FONT, 0.38, C_GREEN if self.latency_ms < 50 else C_AMBER, 1, cv2.LINE_AA)
            cv2.putText(frame, f"POSE XYZ: [{self.pose_pos[0]:.1f}, {self.pose_pos[1]:.1f}, {self.pose_pos[2]:.1f}]", (KP_X + 130, pip_pose_y + 150), FONT, 0.34, C_WHITE, 1, cv2.LINE_AA)

        # 4. LEFT-SIDE: Engineering Overlay (Pinned at X=0, Y=44)
        if self.show_eng:
            _draw_engineering(frame, self.cpu_load, self.ram_pct, self.cpu_temp, self.latency_ms, self.confidence, self.stats_db, self.dropped_frames, self.product)

        # 5. Warnings overlay
        if self.confidence < 70:
            wc = _blink(C_AMBER, C_DIM, hz=1.2)
            cv2.putText(frame, "⚠  SLAM QUALITY DEGRADED — SLOW DOWN",
                        (WIDTH // 2 - 290, HEIGHT // 2), FONT, 0.75, wc, 2, cv2.LINE_AA)
        if self.cpu_temp > 78:
            cv2.putText(frame, "⚠  HIGH TEMPERATURE",
                        (WIDTH // 2 - 160, HEIGHT // 2 + 50), FONT, 0.65, _blink(C_RED, C_DIM), 2, cv2.LINE_AA)

        # 6. Bottom Status Bar (Floating neon text over 100% transparent space)
        elapsed = time.time() - self._t_start
        ts = time.strftime("%H:%M:%S")
        cv2.putText(frame,
                    f"ScanAR G V1.5  ·  {ts}  ·  "
                    f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d} elapsed  ·  [K] Switch Config  [H] Hide PIPs  [F] Fullscreen  [Q] Quit",
                    (16, HEIGHT - 10), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)

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

    def on_mouse_click(self, event, x, y, flags, param):
        self.mouse_pos = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info(f"Mouse clicked at coords: x={x}, y={y}")
            if 0 <= y <= 60:
                # PIP & Overlay Toggles
                if 300 <= x <= 375:
                    self.show_eng = not self.show_eng
                    self._last_click_msg = f"ACTION: ENG PANEL {'ON' if self.show_eng else 'OFF'}"
                    self._last_click_t = time.time()
                    return
                elif 385 <= x <= 470:
                    self.show_slam_pip = not self.show_slam_pip
                    self._last_click_msg = f"ACTION: SLAM MAP PIP {'ON' if self.show_slam_pip else 'OFF'}"
                    self._last_click_t = time.time()
                    return
                elif 480 <= x <= 560:
                    self.show_rgb_pip = not self.show_rgb_pip
                    self._last_click_msg = f"ACTION: RGB STREAM PIP {'ON' if self.show_rgb_pip else 'OFF'}"
                    self._last_click_t = time.time()
                    return
                elif 570 <= x <= 655:
                    self.show_pose_pip = not self.show_pose_pip
                    self._last_click_msg = f"ACTION: POSE PIP {'ON' if self.show_pose_pip else 'OFF'}"
                    self._last_click_t = time.time()
                    return
                elif 665 <= x <= 825:
                    self.cycle_product_config()
                    return

                # Action buttons on right side
                elif 1420 <= x <= 1550:
                    self.trigger_action("start")
                    return
                elif 1560 <= x <= 1665:
                    self.trigger_action("reset")
                    return
                elif 1675 <= x <= 1750:
                    self.trigger_action("save")
                    return
                elif 1760 <= x <= 1850:
                    self.trigger_action("export")
                    return

def main(args=None):
    rclpy.init(args=args)
    node = HudRenderer()

    window = "ScanAR G — HUD"
    cv2.namedWindow(window, cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(window, WIDTH, HEIGHT)

    def _on_mouse(event, x, y, flags, param):
        try:
            w_rect = cv2.getWindowImageRect(window)
            if w_rect[2] > 0 and w_rect[3] > 0:
                x = int(x * (WIDTH / float(w_rect[2])))
                y = int(y * (HEIGHT / float(w_rect[3])))
        except Exception:
            pass
        node.on_mouse_click(event, x, y, flags, param)

    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    delay_ms = max(1, int(1000 / FPS))
    mouse_cb_set = False

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.draw(frame)
            cv2.imshow(window, frame)
            
            # Handle window fullscreen toggle
            if hasattr(node, 'is_fullscreen'):
                if node.is_fullscreen:
                    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                else:
                    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

            key = cv2.waitKey(delay_ms) & 0xFF

            if not mouse_cb_set:
                try:
                    cv2.setMouseCallback(window, _on_mouse)
                    mouse_cb_set = True
                except Exception:
                    pass

            if key in (ord('q'), ord('Q'), 27):
                break
            elif key in (ord('r'), ord('R')):
                node.trigger_action("start")
            elif key in (ord('x'), ord('X')):
                node.trigger_action("reset")
            elif key in (ord('s'), ord('S')):
                node.trigger_action("save")
            elif key in (ord('k'), ord('K')):
                node.cycle_product_config()
            elif key in (ord('e'), ord('E')):
                node.show_eng = not node.show_eng
                node._last_click_msg = f"HOTKEY [E]: ENG OVERLAY {'ON' if node.show_eng else 'OFF'}"
                node._last_click_t = time.time()
            elif key in (ord('m'), ord('M')):
                node.show_slam_pip = not node.show_slam_pip
                node._last_click_msg = f"HOTKEY [M]: SLAM MAP PIP {'ON' if node.show_slam_pip else 'OFF'}"
                node._last_click_t = time.time()
            elif key in (ord('c'), ord('C')):
                node.show_rgb_pip = not node.show_rgb_pip
                node._last_click_msg = f"HOTKEY [C]: RGB STREAM PIP {'ON' if node.show_rgb_pip else 'OFF'}"
                node._last_click_t = time.time()
            elif key in (ord('p'), ord('P')):
                node.show_pose_pip = not node.show_pose_pip
                node._last_click_msg = f"HOTKEY [P]: POSE PIP {'ON' if node.show_pose_pip else 'OFF'}"
                node._last_click_t = time.time()
            elif key in (ord('h'), ord('H')):
                all_on = not (node.show_slam_pip or node.show_rgb_pip or node.show_pose_pip or node.show_eng)
                node.show_slam_pip = all_on
                node.show_rgb_pip = all_on
                node.show_pose_pip = all_on
                node.show_eng = all_on
                node._last_click_msg = f"HOTKEY [H]: ALL PIPS {'SHOWN' if all_on else 'HIDDEN'}"
                node._last_click_t = time.time()
            elif key in (ord('f'), ord('F')):
                node.is_fullscreen = not node.is_fullscreen
                node._last_click_msg = f"HOTKEY [F]: FULLSCREEN {'ON' if node.is_fullscreen else 'OFF'}"
                node._last_click_t = time.time()
            elif key == ord('0'):
                node.show_slam_pip = True
                node.show_rgb_pip = True
                node.show_pose_pip = True
                node._last_click_msg = "HOTKEY [0]: DEFAULT PASSTHROUGH VIEW (ALL PIPS ACTIVE)"
                node._last_click_t = time.time()
            elif key == ord('1'):
                node.show_rgb_pip = not node.show_rgb_pip
                node._last_click_msg = f"HOTKEY [1]: RGB PIP {'ON' if node.show_rgb_pip else 'OFF'}"
                node._last_click_t = time.time()
            elif key == ord('2'):
                node.show_slam_pip = not node.show_slam_pip
                node._last_click_msg = f"HOTKEY [2]: SLAM MAP PIP {'ON' if node.show_slam_pip else 'OFF'}"
                node._last_click_t = time.time()
            elif key == ord('3'):
                node.show_pose_pip = not node.show_pose_pip
                node._last_click_msg = f"HOTKEY [3]: POSE PIP {'ON' if node.show_pose_pip else 'OFF'}"
                node._last_click_t = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
