#!/usr/bin/env python3
"""
ScanAR Dual — Data Collection Simulation
Simulates a live scan capture session with:
  - Accumulating 3D point cloud (top-down map view)
  - Live trajectory path drawing
  - Packet counter / MCAP recorder ticking
  - Storage consumption
  - Pipeline latency graph
  - Full HUD overlay reflecting capture state
Press Q or ESC to quit.
"""

import cv2
import numpy as np
import time
import math
import random
import collections

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
WIDTH  = 1920
HEIGHT = 1080
FPS    = 60

# Colour palette (BGR)
C_BG        = (8,   10,  14)
C_CYAN      = (220, 200, 0)
C_GREEN     = (80,  240, 130)
C_AMBER     = (0,   175, 255)
C_RED       = (60,  60,  255)
C_WHITE     = (240, 240, 240)
C_DIM       = (100, 100, 100)
C_PANEL     = (18,  22,  28)
C_EDGE      = (55,  190, 210)
C_TRAJ      = (0,   230, 255)    # trajectory line — bright yellow
C_PT_LEFT   = (0,   200, 80)     # left LiDAR points — green
C_PT_RIGHT  = (200, 80,  0)      # right LiDAR points — blue-ish
C_GRID      = (28,  35,  42)

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ---------------------------------------------------------------------------
# Map viewport (left two-thirds of screen)
# ---------------------------------------------------------------------------
MAP_X, MAP_Y   = 20,  60
MAP_W, MAP_H   = 1260, 960
MAP_CX, MAP_CY = MAP_X + MAP_W//2, MAP_Y + MAP_H//2
MAP_SCALE      = 5.0   # pixels per metre

# Right panel
RP_X, RP_W = MAP_X + MAP_W + 20, 600
RP_Y       = MAP_Y

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def panel(frame, x, y, w, h, title=None, alpha=0.88):
    ov = frame.copy()
    cv2.rectangle(ov, (x, y), (x+w, y+h), C_PANEL, -1)
    cv2.addWeighted(ov, alpha, frame, 1-alpha, 0, frame)
    cv2.rectangle(frame, (x, y), (x+w, y+h), C_EDGE, 1)
    if title:
        cv2.putText(frame, title, (x+10, y+20), FONT, 0.42, C_CYAN, 1, cv2.LINE_AA)

def hbar(frame, x, y, w, h, val, maxv, col):
    cv2.rectangle(frame, (x, y), (x+w, y+h), (35, 35, 35), -1)
    fill = int(w * min(val, maxv) / maxv)
    if fill > 0:
        cv2.rectangle(frame, (x, y), (x+fill, y+h), col, -1)
    cv2.rectangle(frame, (x, y), (x+w, y+h), C_DIM, 1)

def blink(ca, cb, hz=1.0):
    return ca if math.sin(time.time() * hz * 2 * math.pi) > 0 else cb

def draw_grid_map(frame):
    for x in range(MAP_X, MAP_X+MAP_W, 40):
        cv2.line(frame, (x, MAP_Y), (x, MAP_Y+MAP_H), C_GRID, 1)
    for y in range(MAP_Y, MAP_Y+MAP_H, 40):
        cv2.line(frame, (MAP_X, y), (MAP_X+MAP_W, y), C_GRID, 1)
    # Origin crosshair
    cv2.line(frame, (MAP_CX-15, MAP_CY), (MAP_CX+15, MAP_CY), C_EDGE, 1)
    cv2.line(frame, (MAP_CX, MAP_CY-15), (MAP_CX, MAP_CY+15), C_EDGE, 1)
    cv2.putText(frame, "0,0", (MAP_CX+6, MAP_CY-4), FONT, 0.32, C_DIM, 1)

def world_to_map(wx, wy):
    px = int(MAP_CX + wx * MAP_SCALE)
    py = int(MAP_CY - wy * MAP_SCALE)
    return px, py

def latency_color(ms):
    if ms < 33:  return C_GREEN
    if ms < 50:  return C_AMBER
    return C_RED

# ---------------------------------------------------------------------------
# Simulated robot trajectory (figure-8 / office loop)
# ---------------------------------------------------------------------------

def trajectory_position(t):
    """Returns (x, y) in metres for elapsed time t."""
    speed   = 0.8           # m/s
    omega   = 2 * math.pi / 45.0   # full loop in 45 s
    r       = 30.0
    x = r * math.sin(omega * t)
    y = r * math.sin(omega * t * 2) * 0.5
    return x, y

# ---------------------------------------------------------------------------
# Point cloud simulation — scatter around robot position with sensor geometry
# ---------------------------------------------------------------------------

def emit_scan_ring(rx, ry, heading_rad, n_left=900, n_right=900):
    """Returns two lists of (px, py) in world coords for left+right LiDARs."""
    pts_l, pts_r = [], []
    for _ in range(n_left):
        angle = random.uniform(0, 2*math.pi)
        dist  = random.gauss(18, 6)
        dist  = max(0.5, min(dist, 40))
        # Add structural bias (simulate walls)
        if abs(math.cos(angle)) > 0.7:
            dist = random.uniform(12, 16)
        wx = rx + dist * math.cos(angle)
        wy = ry + dist * math.sin(angle)
        pts_l.append((wx, wy))
    for _ in range(n_right):
        angle = random.uniform(0, 2*math.pi)
        dist  = random.gauss(18, 6)
        dist  = max(0.5, min(dist, 40))
        if abs(math.sin(angle)) > 0.7:
            dist = random.uniform(10, 14)
        wx = rx + dist * math.cos(angle)
        wy = ry + dist * math.sin(angle)
        pts_r.append((wx, wy))
    return pts_l, pts_r

# ---------------------------------------------------------------------------
# Latency history ring buffer
# ---------------------------------------------------------------------------
LAT_HISTORY = collections.deque(maxlen=120)

def draw_latency_graph(frame, x, y, w, h):
    panel(frame, x, y, w, h, "PIPELINE LATENCY (ms)")
    if len(LAT_HISTORY) < 2:
        return
    vals = list(LAT_HISTORY)
    maxv = max(60, max(vals))
    pts  = []
    for i, v in enumerate(vals):
        px = x + int(i * w / len(vals))
        py = y + h - int((v / maxv) * (h - 30)) - 4
        pts.append((px, py))
    # 33ms target line
    target_y = y + h - int((33.0 / maxv) * (h - 30)) - 4
    cv2.line(frame, (x, target_y), (x+w, target_y), C_GREEN, 1)
    cv2.putText(frame, "33ms", (x+4, target_y-3), FONT, 0.3, C_GREEN, 1)
    # Graph line
    for i in range(1, len(pts)):
        col = latency_color(vals[i])
        cv2.line(frame, pts[i-1], pts[i], col, 2, cv2.LINE_AA)
    # Current value
    cur = vals[-1]
    cv2.putText(frame, f"{cur:.1f} ms", (x+w-90, y+h-8),
                FONT, 0.5, latency_color(cur), 1, cv2.LINE_AA)

# ---------------------------------------------------------------------------
# Packet ticker
# ---------------------------------------------------------------------------
def packet_rate_str(t):
    # Simulate packet bursts
    base = 7200  # ~7200 packets/s at 10Hz 720-line LiDAR × 2 sensors
    jitter = int(random.gauss(0, 120))
    return base + jitter

# ---------------------------------------------------------------------------
# Main render
# ---------------------------------------------------------------------------

def main():
    window = "ScanAR Dual — Data Collection Simulation  (Q/ESC to quit)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, WIDTH, HEIGHT)

    t_start    = time.time()
    traj_pts   = []                        # world-coord trajectory
    cloud_l    = []                        # accumulated left points  (world)
    cloud_r    = []                        # accumulated right points (world)
    scan_timer = 0.0
    scan_hz    = 10.0                      # LiDAR @ 10 Hz
    scan_interval = 1.0 / scan_hz

    total_packets  = 0
    mcap_bytes     = 0
    storage_free   = 128.0                 # GB
    session_start  = time.strftime("%Y%m%d_%H%M%S")
    poses_written  = 0
    frame_idx      = 0

    delay_ms = max(1, int(1000 / FPS))

    print("=" * 60)
    print("  ScanAR Dual — Data Collection Simulation")
    print("  Press  Q  or  ESC  to quit.")
    print("=" * 60)

    last_t = time.time()

    while True:
        now = time.time()
        elapsed = now - t_start
        dt = now - last_t
        last_t = now

        # ---- Robot position ----
        rx, ry = trajectory_position(elapsed)
        heading = math.atan2(
            trajectory_position(elapsed + 0.1)[1] - ry,
            trajectory_position(elapsed + 0.1)[0] - rx
        )

        # ---- Accumulate trajectory ----
        traj_pts.append((rx, ry))
        if len(traj_pts) > 5000:
            traj_pts = traj_pts[-5000:]

        # ---- Emit LiDAR scan at 10 Hz ----
        scan_timer += dt
        new_scan = False
        if scan_timer >= scan_interval:
            scan_timer -= scan_interval
            pl, pr = emit_scan_ring(rx, ry, heading, n_left=200, n_right=200)
            cloud_l.extend(pl)
            cloud_r.extend(pr)
            if len(cloud_l) > 80000: cloud_l = cloud_l[-80000:]
            if len(cloud_r) > 80000: cloud_r = cloud_r[-80000:]
            total_packets += random.randint(700, 750)
            mcap_bytes    += random.randint(180_000, 220_000)
            poses_written += 1
            new_scan = True

        # ---- Simulated latency ----
        base_lat = 29.3 + 2.0 * math.sin(elapsed * 0.3) + random.gauss(0, 1.5)
        LAT_HISTORY.append(max(10, base_lat))

        # ---- Derived metrics ----
        dist_m       = elapsed * 0.8
        points_total = len(cloud_l) + len(cloud_r)
        points_m     = points_total / 1_000_000
        cpu_load     = 45.0 + 8.0 * math.sin(elapsed * 0.2) + random.gauss(0, 2)
        cpu_temp     = 52.0 + 4.0 * math.sin(elapsed * 0.15)
        ram_pct      = 61.0 + 5.0 * math.sin(elapsed * 0.1)
        imu_hz       = 379.0 + random.gauss(0, 2)
        storage_free = max(0, 128.0 - mcap_bytes / 1e9)
        confidence   = min(99, 80 + int(min(elapsed / 5.0, 1.0) * 18))

        # ================================================================
        # RENDER
        # ================================================================
        frame = np.full((HEIGHT, WIDTH, 3), C_BG, dtype=np.uint8)

        # ---- Top bar ----
        panel(frame, 0, 0, WIDTH, 50)
        ts = time.strftime("%Y-%m-%d  %H:%M:%S")
        cv2.putText(frame, f"SCANAR DUAL  ·  CAPTURE SESSION  {session_start}  ·  {ts}",
                    (16, 32), FONT, 0.52, C_CYAN, 1, cv2.LINE_AA)
        rec_col = blink(C_RED, C_DIM, hz=1.5)
        cv2.circle(frame, (WIDTH - 30, 25), 10, rec_col, -1)
        cv2.putText(frame, "● REC", (WIDTH - 100, 32), FONT, 0.48, rec_col, 1, cv2.LINE_AA)

        # ---- Map viewport ----
        panel(frame, MAP_X, MAP_Y, MAP_W, MAP_H, "POINT CLOUD MAP  (TOP-DOWN VIEW)")
        draw_grid_map(frame)

        # Draw accumulated point cloud (subsample for speed)
        step_l = max(1, len(cloud_l) // 6000)
        for i in range(0, len(cloud_l), step_l):
            px, py = world_to_map(*cloud_l[i])
            if MAP_X < px < MAP_X+MAP_W and MAP_Y < py < MAP_Y+MAP_H:
                frame[py, px] = C_PT_LEFT

        step_r = max(1, len(cloud_r) // 6000)
        for i in range(0, len(cloud_r), step_r):
            px, py = world_to_map(*cloud_r[i])
            if MAP_X < px < MAP_X+MAP_W and MAP_Y < py < MAP_Y+MAP_H:
                frame[py, px] = C_PT_RIGHT

        # Draw trajectory path
        if len(traj_pts) > 1:
            step_t = max(1, len(traj_pts) // 2000)
            traj_map = [world_to_map(x, y) for x, y in traj_pts[::step_t]]
            traj_map = [(p[0], p[1]) for p in traj_map
                        if MAP_X < p[0] < MAP_X+MAP_W and MAP_Y < p[1] < MAP_Y+MAP_H]
            for i in range(1, len(traj_map)):
                age = i / len(traj_map)
                alpha_col = tuple(int(c * age) for c in C_TRAJ)
                cv2.line(frame, traj_map[i-1], traj_map[i], alpha_col, 2, cv2.LINE_AA)

        # Robot icon
        rpx, rpy = world_to_map(rx, ry)
        if MAP_X < rpx < MAP_X+MAP_W and MAP_Y < rpy < MAP_Y+MAP_H:
            cv2.circle(frame, (rpx, rpy), 10, C_WHITE, -1)
            cv2.circle(frame, (rpx, rpy), 10, C_CYAN, 2)
            hx = int(rpx + 18 * math.cos(heading))
            hy = int(rpy - 18 * math.sin(heading))
            cv2.arrowedLine(frame, (rpx, rpy), (hx, hy), C_CYAN, 2,
                            tipLength=0.4, line_type=cv2.LINE_AA)

        # Map legend
        cv2.circle(frame, (MAP_X+20, MAP_Y+MAP_H-20), 5, C_PT_LEFT, -1)
        cv2.putText(frame, "Left Airy", (MAP_X+30, MAP_Y+MAP_H-15), FONT, 0.38, C_PT_LEFT, 1)
        cv2.circle(frame, (MAP_X+120, MAP_Y+MAP_H-20), 5, C_PT_RIGHT, -1)
        cv2.putText(frame, "Right Airy", (MAP_X+130, MAP_Y+MAP_H-15), FONT, 0.38, C_PT_RIGHT, 1)
        cv2.putText(frame, f"Scale: 1px = {1/MAP_SCALE:.2f}m",
                    (MAP_X+MAP_W-180, MAP_Y+MAP_H-15), FONT, 0.38, C_DIM, 1)

        # ================================================================
        # RIGHT PANEL
        # ================================================================
        rp = RP_X
        ry_cur = RP_Y

        # ---- Session stats ----
        panel(frame, rp, ry_cur, RP_W, 170, "SESSION STATISTICS")
        ry_cur += 30
        stats = [
            ("Elapsed",       f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"),
            ("Distance",      f"{dist_m:.1f} m"),
            ("Poses written", f"{poses_written:,}"),
            ("Total points",  f"{points_m:.2f} M"),
            ("MCAP size",     f"{mcap_bytes/1e6:.1f} MB"),
            ("Free storage",  f"{storage_free:.1f} GB"),
        ]
        for label, val in stats:
            cv2.putText(frame, label, (rp+14, ry_cur+16), FONT, 0.40, C_DIM, 1, cv2.LINE_AA)
            cv2.putText(frame, val,   (rp+200, ry_cur+16), FONT, 0.42, C_WHITE, 1, cv2.LINE_AA)
            ry_cur += 24
        ry_cur += 10

        # ---- Sensor streams ----
        panel(frame, rp, ry_cur, RP_W, 175, "SENSOR STREAMS")
        ry_cur += 32
        streams = [
            ("LEFT AIRY",  "192.168.10.10", "10.0 Hz", blink(C_GREEN, (0,80,40), hz=10)),
            ("RIGHT AIRY", "192.168.11.11", "10.0 Hz", blink(C_GREEN, (0,80,40), hz=10)),
            ("IG-2 IMU",   "/dev/scanar/ig2", f"{imu_hz:.0f} Hz", blink(C_CYAN, (0,60,90), hz=5)),
            ("PTP SYNC",   "Grandmaster",   "15 ns lock", C_GREEN),
        ]
        for name, addr, rate, dot_col in streams:
            cv2.circle(frame, (rp+18, ry_cur+6), 7, dot_col, -1)
            cv2.putText(frame, name, (rp+32, ry_cur+10), FONT, 0.38, C_WHITE, 1, cv2.LINE_AA)
            cv2.putText(frame, addr, (rp+160, ry_cur+10), FONT, 0.35, C_DIM, 1, cv2.LINE_AA)
            cv2.putText(frame, rate, (rp+RP_W-110, ry_cur+10), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)
            ry_cur += 32
        ry_cur += 8

        # ---- Packet counter ----
        panel(frame, rp, ry_cur, RP_W, 80, "PACKET COUNTER")
        cv2.putText(frame, f"{total_packets:,}", (rp+16, ry_cur+55),
                    FONT, 1.0, C_GREEN, 2, cv2.LINE_AA)
        cv2.putText(frame, "total packets captured", (rp+200, ry_cur+55),
                    FONT, 0.38, C_DIM, 1, cv2.LINE_AA)
        ry_cur += 90

        # ---- System health bars ----
        panel(frame, rp, ry_cur, RP_W, 160, "SYSTEM HEALTH")
        bar_labels = [
            ("CPU",  cpu_load,  100, C_GREEN if cpu_load < 70 else C_AMBER),
            ("RAM",  ram_pct,   100, C_GREEN if ram_pct < 75 else C_AMBER),
            ("TEMP", cpu_temp,  100, C_GREEN if cpu_temp < 65 else C_AMBER),
        ]
        by = ry_cur + 35
        for lbl, val, mx, col in bar_labels:
            cv2.putText(frame, lbl, (rp+14, by+12), FONT, 0.40, C_DIM, 1, cv2.LINE_AA)
            hbar(frame, rp+70, by, RP_W-120, 16, val, mx, col)
            unit = "°C" if lbl == "TEMP" else "%"
            cv2.putText(frame, f"{val:.0f}{unit}", (rp+RP_W-54, by+12),
                        FONT, 0.38, col, 1, cv2.LINE_AA)
            by += 36
        ry_cur += 170

        # ---- SLAM confidence ----
        panel(frame, rp, ry_cur, RP_W, 80, "SLAM CONFIDENCE")
        conf_col = C_GREEN if confidence >= 80 else C_AMBER
        hbar(frame, rp+16, ry_cur+38, RP_W-80, 20, confidence, 100, conf_col)
        cv2.putText(frame, f"{confidence}%", (rp+RP_W-62, ry_cur+55),
                    FONT, 0.52, conf_col, 1, cv2.LINE_AA)
        ry_cur += 90

        # ---- Latency graph ----
        draw_latency_graph(frame, rp, ry_cur, RP_W, 170)
        ry_cur += 180

        # ---- Bottom bar ----
        panel(frame, 0, HEIGHT-40, WIDTH, 40)
        cv2.putText(frame, f"FPS: {FPS}  |  Points: {points_m:.2f}M  |  "
                           f"Poses: {poses_written}  |  "
                           f"Latency: {LAT_HISTORY[-1]:.1f}ms  |  "
                           f"SLAM: TRACKING  |  AUTH: ✓",
                    (20, HEIGHT-14), FONT, 0.44, C_CYAN, 1, cv2.LINE_AA)

        cv2.imshow(window, frame)
        frame_idx += 1

        key = cv2.waitKey(delay_ms) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break

    cv2.destroyAllWindows()
    print("Simulation closed.")


if __name__ == "__main__":
    main()
