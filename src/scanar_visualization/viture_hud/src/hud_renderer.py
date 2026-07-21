#!/usr/bin/env python3
"""
hud_renderer.py — ScanAR G Production HUD Renderer
======================================================
ROS2 node that receives live telemetry and renders the operator HUD
to the connected VITURE glasses display at 60 Hz using OpenCV.
Features a 3D Gaussian Splat viewer running in real-time.
"""

import os
import cv2
import numpy as np
import math
import time
import collections

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from nav_msgs.msg import Odometry
from scanar_interfaces.msg import ScanConfidence, GaussianSplatArray

# Display constants
WIDTH, HEIGHT = 1920, 1080
FPS           = 60
DISPLAY       = os.environ.get("DISPLAY", ":1")

# Theme colors (BGR)
BG_COLOR = (6, 8, 10)
C_CYAN  = (220, 205,   0)
C_GREEN = ( 70, 230, 120)
C_AMBER = (  0, 175, 255)
C_RED   = ( 55,  55, 240)
C_WHITE = (235, 235, 235)
C_DIM   = ( 90,  90,  90)
C_PANEL = ( 16,  20,  26)
C_EDGE  = ( 50, 185, 205)

FONT = cv2.FONT_HERSHEY_SIMPLEX

def _blink(ca, cb, hz=1.0):
    return ca if math.sin(time.time() * hz * 2 * math.pi) > 0 else cb

class HudRenderer(Node):
    def __init__(self):
        super().__init__('hud_renderer')

        # HUD State variables
        self.confidence = 95.0
        self.camera_fps = 30.0
        self.imu_rate_hz = 100.0
        self.vigs_status = "INITIALIZING"
        self.gaussian_count = 0
        self.active_dir = ""
        self._t_start = time.time()

        # Visual mode (1: Natural RGB, 2: ScanAR Green, 3: Hybrid)
        self.visual_mode = 1

        # Camera Intrinsics (same as vigs_backend for consistent projection)
        self.fx = 960.0 # scaled for 1920x1080
        self.fy = 960.0
        self.cx = WIDTH / 2.0
        self.cy = HEIGHT / 2.0

        # Current pose
        self.pose_pos = np.array([0.0, 0.0, 0.0])
        self.pose_rot_mat = np.eye(3)
        self.trajectory = []

        # Current 3D Gaussian splats
        self.splat_positions = np.zeros((0, 3), dtype=np.float32)
        self.splat_colors = np.zeros((0, 3), dtype=np.uint8)
        self.splat_scales = np.zeros((0,), dtype=np.float32)
        self.splat_opacities = np.zeros((0,), dtype=np.float32)

        # ROS subscriptions
        self.create_subscription(Odometry, '/fast_lio/odometry', self._cb_odom, 10)
        self.create_subscription(GaussianSplatArray, '/vigs/gaussian_splats', self._cb_splats, 10)
        self.create_subscription(Float32, '/scanar/scan_confidence', self._cb_confidence, 10)
        self.create_subscription(String, '/vigs/status', self._cb_status, 10)
        self.create_subscription(Float32, '/viture/camera/fps', self._cb_fps, 10)
        self.create_subscription(Float32, '/viture/imu/rate', self._cb_imu_rate, 10)
        self.create_subscription(String, '/scanar/session/active_directory', self._cb_active_dir, 10)

        self.get_logger().info("ScanAR G HUD Renderer Node Initialized.")

    def _cb_odom(self, msg: Odometry):
        tx = msg.pose.pose.position.x
        ty = msg.pose.pose.position.y
        tz = msg.pose.pose.position.z
        self.pose_pos = np.array([tx, ty, tz])

        qw = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        self.pose_rot_mat = self.quat_to_rot(qw, qx, qy, qz)

        # Accumulate trajectory (x, y) for minimap / stats
        self.trajectory.append((tx, ty))
        if len(self.trajectory) > 5000:
            self.trajectory.pop(0)

    def _cb_splats(self, msg: GaussianSplatArray):
        self.gaussian_count = len(msg.splats)
        if self.gaussian_count == 0:
            self.splat_positions = np.zeros((0, 3), dtype=np.float32)
            self.splat_colors = np.zeros((0, 3), dtype=np.uint8)
            self.splat_scales = np.zeros((0,), dtype=np.float32)
            self.splat_opacities = np.zeros((0,), dtype=np.float32)
            return

        # Vectorized parsing of splats
        positions = []
        colors = []
        scales = []
        opacities = []
        for s in msg.splats:
            positions.append([s.x, s.y, s.z])
            colors.append([int(s.b), int(s.g), int(s.r)]) # BGR for OpenCV
            scales.append(s.scale)
            opacities.append(s.opacity)

        self.splat_positions = np.array(positions, dtype=np.float32)
        self.splat_colors = np.array(colors, dtype=np.uint8)
        self.splat_scales = np.array(scales, dtype=np.float32)
        self.splat_opacities = np.array(opacities, dtype=np.float32)

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

    def draw(self, frame: np.ndarray) -> None:
        # Clear screen to BG_COLOR
        frame[:] = BG_COLOR

        # 1. Project and render 3D Gaussian Splats if available
        if len(self.splat_positions) > 0:
            # Transform splats to camera frame
            # P_cam = R_cam^T * (P_world - t_cam)
            rel_pos = self.splat_positions - self.pose_pos
            # Matrix multiplication to transform all points: P_cam = rel_pos @ R_cam
            # Since R_cam is the orientation of the camera/head, the inverse transform is R_cam^T
            # So rel_pos @ R_cam translates to multiplying by R_cam matrix.
            p_cam = rel_pos @ self.pose_rot_mat

            # Filter points in front of the camera (Z_cam > 0.1)
            mask = p_cam[:, 2] > 0.1
            if np.any(mask):
                p_cam_filt = p_cam[mask]
                colors_filt = self.splat_colors[mask]
                scales_filt = self.splat_scales[mask]
                opacities_filt = self.splat_opacities[mask]

                # Project to image plane
                z = p_cam_filt[:, 2]
                u = (self.fx * p_cam_filt[:, 0] / z) + self.cx
                v = (self.fy * p_cam_filt[:, 1] / z) + self.cy

                # Filter points on screen
                screen_mask = (u >= 0) & (u < WIDTH) & (v >= 0) & (v < HEIGHT)
                if np.any(screen_mask):
                    u_scr = u[screen_mask]
                    v_scr = v[screen_mask]
                    z_scr = z[screen_mask]
                    col_scr = colors_filt[screen_mask]
                    scale_scr = scales_filt[screen_mask]
                    op_scr = opacities_filt[screen_mask]

                    # Sort points by depth (back to front) for alpha blending
                    sort_idx = np.argsort(z_scr)[::-1]

                    for idx in sort_idx:
                        px = int(u_scr[idx])
                        py = int(v_scr[idx])
                        
                        # Size is scale projected onto screen
                        r_pix = int((scale_scr[idx] * self.fx) / z_scr[idx])
                        r_pix = max(2, min(r_pix, 150))

                        # Determine color based on mode
                        if self.visual_mode == 1:
                            # Natural RGB Mode
                            color = (int(col_scr[idx][0]), int(col_scr[idx][1]), int(col_scr[idx][2]))
                        elif self.visual_mode == 2:
                            # ScanAR Green Mode
                            # Map green intensity to confidence
                            intensity = int(120 + 135 * (self.confidence / 100.0))
                            color = (50, intensity, 50)
                        else:
                            # Hybrid Mode (RGB splats + green confidence halo)
                            color = (int(col_scr[idx][0]), int(col_scr[idx][1]), int(col_scr[idx][2]))

                        # Draw splat
                        if self.visual_mode == 3:
                            # Draw outer confidence green ring
                            cv2.circle(frame, (px, py), r_pix + 2, (30, 180, 50), 1, cv2.LINE_AA)
                        
                        # Filled circle
                        cv2.circle(frame, (px, py), r_pix, color, -1, cv2.LINE_AA)

        # 2. Render Top Strip Dashboard
        ov = frame.copy()
        cv2.rectangle(ov, (0, 0), (WIDTH, 44), C_PANEL, -1)
        cv2.addWeighted(ov, 0.78, frame, 0.22, 0, frame)
        cv2.line(frame, (0, 44), (WIDTH, 44), C_EDGE, 1)

        # REC indicator
        rec_col = _blink(C_RED, C_DIM, hz=1.5) if self.active_dir else C_DIM
        cv2.circle(frame, (24, 22), 9, rec_col, -1)
        cv2.putText(frame, "REC" if self.active_dir else "STBY", (40, 28), FONT, 0.46, rec_col, 1, cv2.LINE_AA)

        # Mode tag
        mode_str = ["NATURAL RGB", "SCANAR GREEN", "HYBRID OVERLAY"][self.visual_mode - 1]
        cv2.putText(frame, f"MODE: {mode_str}", (120, 28), FONT, 0.44, C_CYAN, 1, cv2.LINE_AA)

        # Title
        cv2.putText(frame, "ScanAR  G", (WIDTH // 2 - 50, 29), FONT, 0.52, C_CYAN, 1, cv2.LINE_AA)

        # Stats strip
        stats_str = f"CAM {self.camera_fps:.1f} FPS  ·  IMU {self.imu_rate_hz:.0f} Hz  ·  SPLATS {self.gaussian_count:,}  ·  VIGS: {self.vigs_status}"
        sw, _ = cv2.getTextSize(stats_str, FONT, 0.44, 1)[0]
        cv2.putText(frame, stats_str, (WIDTH - sw - 200, 28), FONT, 0.44, C_WHITE, 1, cv2.LINE_AA)

        # Quality indicator
        qual_col = C_GREEN if self.confidence >= 80 else (C_AMBER if self.confidence >= 50 else C_RED)
        cv2.circle(frame, (WIDTH - 150, 22), 6, qual_col, -1)
        cv2.putText(frame, f"QUALITY {self.confidence:.0f}%", (WIDTH - 135, 28), FONT, 0.44, qual_col, 1, cv2.LINE_AA)

        # 3. Mode Help Panel (Bottom-Left)
        b_panel_w, b_panel_h = 360, 100
        _panel_x, _panel_y = 20, HEIGHT - b_panel_h - 60
        ov_b = frame.copy()
        cv2.rectangle(ov_b, (_panel_x, _panel_y), (_panel_x + b_panel_w, _panel_y + b_panel_h), C_PANEL, -1)
        cv2.addWeighted(ov_b, 0.8, frame, 0.2, 0, frame)
        cv2.rectangle(frame, (_panel_x, _panel_y), (_panel_x + b_panel_w, _panel_y + b_panel_h), C_EDGE, 1)

        cv2.putText(frame, "CONTROLS & SHORTCUTS", (_panel_x + 10, _panel_y + 20), FONT, 0.40, C_CYAN, 1, cv2.LINE_AA)
        cv2.putText(frame, "[1] Natural RGB Mode", (_panel_x + 15, _panel_y + 40), FONT, 0.36, C_WHITE if self.visual_mode == 1 else C_DIM, 1, cv2.LINE_AA)
        cv2.putText(frame, "[2] ScanAR Green Mode", (_panel_x + 15, _panel_y + 60), FONT, 0.36, C_WHITE if self.visual_mode == 2 else C_DIM, 1, cv2.LINE_AA)
        cv2.putText(frame, "[3] Hybrid View Mode", (_panel_x + 15, _panel_y + 80), FONT, 0.36, C_WHITE if self.visual_mode == 3 else C_DIM, 1, cv2.LINE_AA)

        # 4. Warnings overlay (if tracking is lost / poor)
        if self.confidence < 70:
            wc = _blink(C_AMBER, C_DIM, hz=1.2)
            cv2.putText(frame, "⚠  TRACKING QUALITY DEGRADED — ROTATE HEAD SLOWLY",
                        (WIDTH // 2 - 380, HEIGHT // 2), FONT, 0.75, wc, 2, cv2.LINE_AA)

        # 5. Bottom Status Bar
        bov = frame.copy()
        cv2.rectangle(bov, (0, HEIGHT - 34), (WIDTH, HEIGHT), C_PANEL, -1)
        cv2.addWeighted(bov, 0.75, frame, 0.25, 0, frame)
        elapsed = time.time() - self._t_start
        ts = time.strftime("%H:%M:%S")
        cv2.putText(frame,
                    f"ScanAR G Appliance  ·  {ts}  ·  "
                    f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d} elapsed",
                    (16, HEIGHT - 10), FONT, 0.38, C_DIM, 1, cv2.LINE_AA)

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
    node = HudRenderer()

    window = "ScanAR G — HUD"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, WIDTH, HEIGHT)

    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    delay_ms = max(1, int(1000 / FPS))

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.draw(frame)
            cv2.imshow(window, frame)
            
            key = cv2.waitKey(delay_ms) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key == ord('1'):
                node.visual_mode = 1
                node.get_logger().info("HUD Mode set to: NATURAL RGB")
            elif key == ord('2'):
                node.visual_mode = 2
                node.get_logger().info("HUD Mode set to: SCANAR GREEN")
            elif key == ord('3'):
                node.visual_mode = 3
                node.get_logger().info("HUD Mode set to: HYBRID")
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
