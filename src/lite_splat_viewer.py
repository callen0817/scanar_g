#!/usr/bin/env python3
import sys
import os
import cv2
import numpy as np
import math

class LiteSplatViewer:
    def __init__(self, ply_path):
        self.ply_path = ply_path
        self.points = np.zeros((0, 3), dtype=np.float32)
        self.colors = np.zeros((0, 3), dtype=np.uint8)

        # Viewer parameters
        self.width = 1024
        self.height = 768
        self.fx = 800.0
        self.fy = 800.0
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # View state
        self.yaw = 0.0
        self.pitch = 0.0
        self.zoom = 5.0
        self.center = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        # Rendering mode (1: RGB, 2: Green)
        self.mode = 1

        self.load_ply()

    def load_ply(self):
        print(f"Loading PLY point cloud/splats from: {self.ply_path}")
        if not os.path.exists(self.ply_path):
            print(f"Error: file not found: {self.ply_path}")
            return False

        try:
            with open(self.ply_path, 'r') as f:
                header = []
                num_vertices = 0
                while True:
                    line = f.readline().strip()
                    header.append(line)
                    if line.startswith("element vertex"):
                        num_vertices = int(line.split()[-1])
                    if line == "end_header":
                        break

                print(f"Reading {num_vertices} vertices...")
                positions = []
                colors = []
                for _ in range(num_vertices):
                    line = f.readline().strip()
                    if not line:
                        break
                    parts = line.split()
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    r, g, b = int(parts[3]), int(parts[4]), int(parts[5])
                    positions.append([x, y, z])
                    colors.append([b, g, r]) # OpenCV uses BGR

                self.points = np.array(positions, dtype=np.float32)
                self.colors = np.array(colors, dtype=np.uint8)

                if len(self.points) > 0:
                    self.center = np.mean(self.points, axis=0)
                    print(f"Loaded successfully. Center of cloud: {self.center}")
                return True
        except Exception as e:
            print(f"Failed to read PLY file: {e}")
            return False

    def render(self):
        # Create blank display frame
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        if len(self.points) == 0:
            cv2.putText(frame, "No point data loaded.", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return frame

        # Compute viewer camera pose rotation matrix based on yaw/pitch
        cy = math.cos(self.yaw)
        sy = math.sin(self.yaw)
        cp = math.cos(self.pitch)
        sp = math.sin(self.pitch)

        # Camera Rotation Matrix
        R_yaw = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1]
        ], dtype=np.float32)

        R_pitch = np.array([
            [1, 0, 0],
            [0, cp, -sp],
            [0, sp, cp]
        ], dtype=np.float32)

        R = R_yaw @ R_pitch

        # Camera position
        # Position camera back along the viewing direction by zoom distance from cloud center
        cam_pos = self.center - R @ np.array([0, 0, self.zoom], dtype=np.float32)

        # Project points to camera frame: P_cam = R^T * (P_world - cam_pos)
        rel_pos = self.points - cam_pos
        p_cam = rel_pos @ R

        # Filter points in front of the camera
        mask = p_cam[:, 2] > 0.1
        if np.any(mask):
            p_cam_filt = p_cam[mask]
            colors_filt = self.colors[mask]

            z = p_cam_filt[:, 2]
            u = (self.fx * p_cam_filt[:, 0] / z) + self.cx
            v = (self.fy * p_cam_filt[:, 1] / z) + self.cy

            # Filter points on screen
            screen_mask = (u >= 0) & (u < self.width) & (v >= 0) & (v < self.height)
            if np.any(screen_mask):
                u_scr = u[screen_mask]
                v_scr = v[screen_mask]
                z_scr = z[screen_mask]
                col_scr = colors_filt[screen_mask]

                # Sort by depth back-to-front
                sort_idx = np.argsort(z_scr)[::-1]

                for idx in sort_idx:
                    px = int(u_scr[idx])
                    py = int(v_scr[idx])
                    
                    # Compute size based on depth
                    radius = max(1, int(15 / z_scr[idx]))

                    if self.mode == 1:
                        color = (int(col_scr[idx][0]), int(col_scr[idx][1]), int(col_scr[idx][2]))
                    else:
                        color = (50, 220, 50) # Green Mode

                    cv2.circle(frame, (px, py), radius, color, -1)

        # Draw HUD overlays on viewer
        cv2.putText(frame, "ScanAR G Lite Splat Viewer", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 205, 0), 2)
        cv2.putText(frame, f"Splat count: {len(self.points):,}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1)
        cv2.putText(frame, f"Zoom: {self.zoom:.2f}m  Yaw: {math.degrees(self.yaw):.1f}  Pitch: {math.degrees(self.pitch):.1f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1)
        
        # Shortcuts panel
        cv2.rectangle(frame, (10, self.height - 120), (320, self.height - 10), (20, 24, 30), -1)
        cv2.rectangle(frame, (10, self.height - 120), (320, self.height - 10), (50, 185, 205), 1)
        cv2.putText(frame, "CONTROLS:", (20, self.height - 95), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 205, 0), 1)
        cv2.putText(frame, "Arrow Keys / drag: Rotate View", (20, self.height - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1)
        cv2.putText(frame, "+ / - Keys: Zoom In / Out", (20, self.height - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1)
        cv2.putText(frame, "[1] RGB Color | [2] ScanAR Green Mode", (20, self.height - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (235, 235, 235), 1)
        cv2.putText(frame, "Press ESC / Q to Quit", (20, self.height - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (55, 55, 240), 1)

        return frame

    def run(self):
        window_name = "ScanAR G — Lite Splat Previewer"
        cv2.namedWindow(window_name)
        
        drag = False
        last_x, last_y = 0, 0
        
        def mouse_callback(event, x, y, flags, param):
            nonlocal drag, last_x, last_y
            if event == cv2.EVENT_LBUTTONDOWN:
                drag = True
                last_x, last_y = x, y
            elif event == cv2.EVENT_LBUTTONUP:
                drag = False
            elif event == cv2.EVENT_MOUSEMOVE:
                if drag:
                    dx = x - last_x
                    dy = y - last_y
                    self.yaw += dx * 0.005
                    self.pitch -= dy * 0.005
                    last_x, last_y = x, y
        
        cv2.setMouseCallback(window_name, mouse_callback)

        while True:
            frame = self.render()
            cv2.imshow(window_name, frame)
            
            key = cv2.waitKey(10) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            elif key == ord('1'):
                self.mode = 1
            elif key == ord('2'):
                self.mode = 2
            elif key == ord('+') or key == ord('='):
                self.zoom = max(0.5, self.zoom - 0.2)
            elif key == ord('-') or key == ord('_'):
                self.zoom += 0.2
            elif key == 82: # Up arrow
                self.pitch += 0.05
            elif key == 84: # Down arrow
                self.pitch -= 0.05
            elif key == 81: # Left arrow
                self.yaw -= 0.05
            elif key == 83: # Right arrow
                self.yaw += 0.05

        cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 lite_splat_viewer.py <path_to_ply_file>")
        sys.exit(1)
    
    viewer = LiteSplatViewer(sys.argv[1])
    viewer.run()
