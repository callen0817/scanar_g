#!/usr/bin/env python3
"""
ScanAR Dual — Operator HUD  (Passthrough-First Design)
=======================================================
Layer 1  OPERATOR  — always visible
  · corner keyplan: live 2D map building as data is collected
    (trajectory line + accumulated scan points + heading arrow)
  · minimal status strip (REC, TRACKING, LATENCY)
  · centre warnings only when triggered

Layer 2  ENGINEERING  — press E to toggle
  · CPU / RAM / TEMP bars
  · IMU + PTP rates
  · pipeline latency graph
  · packet counter
  · SLAM confidence

No CAD overlay. No coverage %. No pre-existing floor plan.
The map builds from scratch as the operator walks — like NavVis VLX.

Controls
--------
  E          toggle engineering overlay
  Q / ESC    quit
"""

import cv2
import numpy as np
import math
import time
import random
import collections

# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
WIDTH, HEIGHT = 1920, 1080
FPS           = 60

# Near-black — simulates AR passthrough (real world shows through)
BG_COLOR = (6, 8, 10)

# AR palette (BGR)
C_CYAN   = (220, 205,   0)
C_GREEN  = ( 70, 230, 120)
C_AMBER  = (  0, 175, 255)
C_RED    = ( 55,  55, 240)
C_WHITE  = (235, 235, 235)
C_DIM    = ( 90,  90,  90)
C_PANEL  = ( 16,  20,  26)
C_EDGE   = ( 50, 185, 205)
C_TRAJ   = (  0, 220, 255)   # trajectory — bright yellow/cyan
C_PT     = ( 55, 200, 100)   # accumulated scan points — muted green

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ---------------------------------------------------------------------------
# Minimap geometry (bottom-right corner)
# ---------------------------------------------------------------------------
MAP_PX = 340    # pixel width
MAP_PH = 300    # pixel height
MAP_X  = WIDTH  - MAP_PX - 24
MAP_Y  = HEIGHT - MAP_PH - 60

# World bounds for auto-scaling (grow as points arrive)
_wx_min = _wy_min = -1.0
_wx_max = _wy_max =  1.0


def _update_bounds(wx, wy, pad=8.0):
    global _wx_min, _wy_min, _wx_max, _wy_max
    _wx_min = min(_wx_min, wx - pad)
    _wy_min = min(_wy_min, wy - pad)
    _wx_max = max(_wx_max, wx + pad)
    _wy_max = max(_wy_max, wy + pad)


def w2m(wx, wy):
    """World coords → minimap pixel, auto-scaled to fit all seen points."""
    rng_x = max(_wx_max - _wx_min, 1.0)
    rng_y = max(_wy_max - _wy_min, 1.0)
    scale = min(MAP_PX / rng_x, MAP_PH / rng_y) * 0.88  # 88% margin
    cx = MAP_X + MAP_PX // 2
    cy = MAP_Y + MAP_PH // 2
    mid_wx = (_wx_min + _wx_max) / 2
    mid_wy = (_wy_min + _wy_max) / 2
    px = int(cx + (wx - mid_wx) * scale)
    py = int(cy - (wy - mid_wy) * scale)
    return px, py, scale


# ---------------------------------------------------------------------------
# Robot trajectory  (figure-8 / office loop in world metres)
# ---------------------------------------------------------------------------
WAYPOINTS = [
    (-25,  2), (-20, 14), (-10, 14), (-7,  6),
    (  4,  6), ( 14,  6), ( 24, 12), ( 34,  8),
    ( 30, -8), ( 16,-10), (  8,  3), ( -7,  6),
    ( -7, -2), ( -8,-18), (-20,-22), (-25,-14),
    (-15,-10), ( -7, -2), (-25,  2),
]
WALK_SPEED = 1.6   # m/s


def robot_state(t):
    """Return (x, y, heading_rad) for elapsed time t."""
    segs = []
    total = 0.0
    for i in range(len(WAYPOINTS)):
        a = WAYPOINTS[i]
        b = WAYPOINTS[(i + 1) % len(WAYPOINTS)]
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        segs.append((a, b, d))
        total += d

    t_mod = t % (total / WALK_SPEED)
    accum = 0.0
    for (a, b, d) in segs:
        seg_t = d / WALK_SPEED
        if accum + seg_t >= t_mod or seg_t == 0:
            frac = (t_mod - accum) / max(seg_t, 1e-6)
            frac = max(0.0, min(1.0, frac))
            rx = a[0] + frac * (b[0] - a[0])
            ry = a[1] + frac * (b[1] - a[1])
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            return rx, ry, heading
        accum += seg_t
    return WAYPOINTS[0][0], WAYPOINTS[0][1], 0.0


# ---------------------------------------------------------------------------
# LiDAR scan point emission (2D top-down, world metres)
# ---------------------------------------------------------------------------
def emit_scan(rx, ry, n=120):
    """Emit n scan returns around (rx, ry) simulating a 2D lidar ring."""
    pts = []
    for _ in range(n):
        angle = random.uniform(0, 2 * math.pi)
        dist  = random.gauss(14, 5)
        dist  = max(0.3, min(dist, 30))
        # bias returns to simulate wall structure
        if abs(math.cos(angle)) > 0.75:
            dist = random.uniform(8, 16)
        pts.append((rx + dist * math.cos(angle),
                    ry + dist * math.sin(angle)))
    return pts


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def panel(frame, x, y, w, h, alpha=0.82):
    ov = frame.copy()
    cv2.rectangle(ov, (x, y), (x + w, y + h), C_PANEL, -1)
    cv2.addWeighted(ov, alpha, frame, 1 - alpha, 0, frame)
    cv2.rectangle(frame, (x, y), (x + w, y + h), C_EDGE, 1)


def hbar(frame, x, y, w, h, val, maxv, col):
    cv2.rectangle(frame, (x, y), (x + w, y + h), (30, 30, 30), -1)
    fill = int(w * min(val, maxv) / maxv)
    if fill > 0:
        cv2.rectangle(frame, (x, y), (x + fill, y + h), col, -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), C_DIM, 1)


def blink(ca, cb, hz=1.0):
    return ca if math.sin(time.time() * hz * 2 * math.pi) > 0 else cb


def lat_color(ms):
    return C_GREEN if ms < 33 else (C_AMBER if ms < 50 else C_RED)


LAT_HIST = collections.deque(maxlen=100)


# ---------------------------------------------------------------------------
# KEYPLAN — live map renderer
# ---------------------------------------------------------------------------

def draw_keyplan(frame, rx, ry, heading, traj_pts, scan_pts_2d):
    # Outer panel
    panel(frame, MAP_X - 8, MAP_Y - 26, MAP_PX + 16, MAP_PH + 42)
    cv2.putText(frame, "MAP",
                (MAP_X - 2, MAP_Y - 10), FONT, 0.42, C_CYAN, 1, cv2.LINE_AA)

    # Map background
    cv2.rectangle(frame, (MAP_X, MAP_Y),
                  (MAP_X + MAP_PX, MAP_Y + MAP_PH), (5, 8, 6), -1)

    # Accumulated scan points (subsample for performance)
    step = max(1, len(scan_pts_2d) // 12000)
    for i in range(0, len(scan_pts_2d), step):
        wx, wy = scan_pts_2d[i]
        px, py, _ = w2m(wx, wy)
        if MAP_X <= px < MAP_X + MAP_PX and MAP_Y <= py < MAP_Y + MAP_PH:
            frame[py, px] = C_PT

    # Trajectory path with age fade
    if len(traj_pts) > 1:
        step_t = max(1, len(traj_pts) // 600)
        tmapped = []
        for wx, wy in traj_pts[::step_t]:
            px, py, _ = w2m(wx, wy)
            if MAP_X <= px < MAP_X + MAP_PX and MAP_Y <= py < MAP_Y + MAP_PH:
                tmapped.append((px, py))
        for i in range(1, len(tmapped)):
            age = i / len(tmapped)
            c = tuple(int(v * (0.3 + 0.7 * age)) for v in C_TRAJ)
            cv2.line(frame, tmapped[i - 1], tmapped[i], c, 1, cv2.LINE_AA)

    # Robot icon + heading arrow
    rpx, rpy, scale = w2m(rx, ry)
    arrow_len = max(8, int(4 * scale))
    ax = int(rpx + arrow_len * math.cos(heading))
    ay = int(rpy - arrow_len * math.sin(heading))
    if MAP_X < rpx < MAP_X + MAP_PX and MAP_Y < rpy < MAP_Y + MAP_PH:
        cv2.arrowedLine(frame, (rpx, rpy), (ax, ay),
                        C_WHITE, 2, tipLength=0.45, line_type=cv2.LINE_AA)
        cv2.circle(frame, (rpx, rpy), 5, C_WHITE, -1)
        cv2.circle(frame, (rpx, rpy), 5, C_CYAN, 1)

    # Border
    cv2.rectangle(frame, (MAP_X, MAP_Y),
                  (MAP_X + MAP_PX, MAP_Y + MAP_PH), C_EDGE, 1)


# ---------------------------------------------------------------------------
# Engineering overlay (left panel)
# ---------------------------------------------------------------------------

def draw_engineering(frame, cpu, ram, temp, imu_hz, lat_ms, packets, conf):
    EX, EY, EW = 20, 60, 360
    EH = 560
    panel(frame, EX, EY, EW, EH)
    cv2.putText(frame, "ENGINEERING", (EX + 10, EY + 18),
                FONT, 0.40, C_CYAN, 1, cv2.LINE_AA)

    y = EY + 40

    def row(label, val, col=C_WHITE):
        nonlocal y
        cv2.putText(frame, label,  (EX + 14, y), FONT, 0.37, C_DIM, 1, cv2.LINE_AA)
        cv2.putText(frame, val,    (EX + 185, y), FONT, 0.39, col,   1, cv2.LINE_AA)
        y += 24

    def bar_row(label, val, maxv, col):
        nonlocal y
        cv2.putText(frame, label, (EX + 14, y), FONT, 0.37, C_DIM, 1, cv2.LINE_AA)
        hbar(frame, EX + 110, y - 13, EW - 138, 13, val, maxv, col)
        cv2.putText(frame, f"{val:.0f}", (EX + EW - 42, y),
                    FONT, 0.34, col, 1, cv2.LINE_AA)
        y += 26

    cpu_c = C_GREEN if cpu < 60 else (C_AMBER if cpu < 85 else C_RED)
    row("CPU LOAD",  f"{cpu:.0f}%",  cpu_c)
    bar_row("",      cpu,  100,       cpu_c)

    ram_c = C_GREEN if ram < 70 else C_AMBER
    row("RAM",       f"{ram:.0f}%",  ram_c)
    bar_row("",      ram,  100,       ram_c)

    tmp_c = C_GREEN if temp < 65 else (C_AMBER if temp < 78 else C_RED)
    row("CPU TEMP",  f"{temp:.0f}°C", tmp_c)
    bar_row("",      temp, 100,        tmp_c)

    y += 6
    row("IMU RATE",   f"{imu_hz:.0f} Hz",  C_CYAN)
    row("PTP SYNC",   "15 ns lock",         C_GREEN)
    row("LATENCY",    f"{lat_ms:.1f} ms",   lat_color(lat_ms))
    row("CONFIDENCE", f"{conf:.0f}%",
        C_GREEN if conf >= 80 else C_AMBER)
    row("PACKETS",    f"{packets:,}",        C_WHITE)

    y += 10
    # Latency mini-graph
    if len(LAT_HIST) > 2:
        gx, gy, gw, gh = EX + 14, y, EW - 28, 80
        cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), (18, 18, 18), -1)
        cv2.rectangle(frame, (gx, gy), (gx + gw, gy + gh), C_DIM, 1)
        vals = list(LAT_HIST)
        maxv = max(60, max(vals))
        t33y = gy + gh - int((33.0 / maxv) * gh)
        cv2.line(frame, (gx, t33y), (gx + gw, t33y), C_GREEN, 1)
        cv2.putText(frame, "33ms", (gx + 3, t33y - 2), FONT, 0.28, C_GREEN, 1)
        pts = [(gx + int(i * gw / len(vals)),
                gy + gh - int((v / maxv) * gh))
               for i, v in enumerate(vals)]
        for i in range(1, len(pts)):
            cv2.line(frame, pts[i - 1], pts[i],
                     lat_color(vals[i]), 1, cv2.LINE_AA)
        cv2.putText(frame, "LATENCY", (gx, gy - 3), FONT, 0.29, C_DIM, 1)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    window = "ScanAR Dual — Operator HUD  (E=engineering  Q/ESC=quit)"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, WIDTH, HEIGHT)

    t_start    = time.time()
    traj_pts   = []          # (wx, wy) history
    scan_pts   = []          # accumulated 2D scan returns (world coords)
    scan_timer = 0.0
    show_eng   = False
    packets    = 0
    conf       = 80
    delay_ms   = max(1, int(1000 / FPS))
    last_t     = time.time()

    print("=" * 60)
    print("  ScanAR Dual — Operator HUD")
    print("  Map builds live as data is collected.")
    print("  E → toggle engineering overlay  |  Q/ESC → quit")
    print("=" * 60)

    while True:
        now     = time.time()
        elapsed = now - t_start
        dt      = now - last_t
        last_t  = now

        rx, ry, heading = robot_state(elapsed)
        traj_pts.append((rx, ry))
        if len(traj_pts) > 10000:
            traj_pts = traj_pts[-10000:]

        # Update auto-scale bounds
        _update_bounds(rx, ry)

        # Emit scan at 10 Hz
        scan_timer += dt
        if scan_timer >= 0.1:
            scan_timer -= 0.1
            new_pts = emit_scan(rx, ry, n=150)
            scan_pts.extend(new_pts)
            if len(scan_pts) > 200000:
                scan_pts = scan_pts[-200000:]
            packets += random.randint(700, 750)

        # Telemetry
        lat_ms = max(10.0, 29.3 + 2.0 * math.sin(elapsed * 0.3)
                     + random.gauss(0, 1.2))
        LAT_HIST.append(lat_ms)
        cpu    = 45.0 + 8.0 * math.sin(elapsed * 0.2) + random.gauss(0, 2)
        ram    = 61.0 + 5.0 * math.sin(elapsed * 0.1)
        temp   = 52.0 + 4.0 * math.sin(elapsed * 0.15)
        imu_hz = 379.0 + random.gauss(0, 2)
        conf   = min(99, 80 + int(min(elapsed / 6.0, 1.0) * 19))
        dist_m = elapsed * WALK_SPEED

        # ================================================================
        # RENDER
        # ================================================================
        frame = np.full((HEIGHT, WIDTH, 3), BG_COLOR, dtype=np.uint8)

        # ---- Top strip ----
        ov = frame.copy()
        cv2.rectangle(ov, (0, 0), (WIDTH, 44), C_PANEL, -1)
        cv2.addWeighted(ov, 0.78, frame, 0.22, 0, frame)
        cv2.line(frame, (0, 44), (WIDTH, 44), C_EDGE, 1)

        rec_col = blink(C_RED, C_DIM, hz=1.5)
        cv2.circle(frame, (24, 22), 9, rec_col, -1)
        cv2.putText(frame, "REC", (40, 28), FONT, 0.46, rec_col, 1, cv2.LINE_AA)

        trk_txt = "TRACKING" if conf >= 80 else "DEGRADED"
        trk_col = C_GREEN if conf >= 80 else blink(C_AMBER, C_DIM, hz=1.5)
        cv2.circle(frame, (118, 22), 6, trk_col, -1)
        cv2.putText(frame, trk_txt, (132, 28), FONT, 0.44, trk_col, 1, cv2.LINE_AA)

        cv2.putText(frame, "SCANAR  DUAL",
                    (WIDTH // 2 - 72, 29), FONT, 0.52, C_CYAN, 1, cv2.LINE_AA)

        right_txt = f"LAT {lat_ms:.0f}ms   {dist_m:.0f}m"
        rw, _ = cv2.getTextSize(right_txt, FONT, 0.44, 1)[0]
        cv2.putText(frame, right_txt, (WIDTH - rw - 20, 28),
                    FONT, 0.44, C_WHITE, 1, cv2.LINE_AA)

        # ---- Centre warnings (conditional) ----
        if conf < 70:
            wc = blink(C_AMBER, C_DIM, hz=1.2)
            cv2.putText(frame, "⚠  SLAM QUALITY DEGRADED — SLOW DOWN",
                        (WIDTH // 2 - 290, HEIGHT // 2),
                        FONT, 0.75, wc, 2, cv2.LINE_AA)
        if temp > 78:
            cv2.putText(frame, "⚠  HIGH TEMPERATURE",
                        (WIDTH // 2 - 160, HEIGHT // 2 + 50),
                        FONT, 0.65, blink(C_RED, C_DIM), 2, cv2.LINE_AA)

        # ---- Keyplan (bottom-right) ----
        draw_keyplan(frame, rx, ry, heading, traj_pts, scan_pts)

        # ---- Engineering overlay ----
        if show_eng:
            draw_engineering(frame, cpu, ram, temp, imu_hz,
                             lat_ms, packets, conf)
            cv2.putText(frame, "[E] ENGINEERING ON",
                        (20, HEIGHT - 50), FONT, 0.38, C_CYAN, 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, "[E] ENGINEERING",
                        (20, HEIGHT - 50), FONT, 0.38, C_DIM, 1, cv2.LINE_AA)

        # ---- Bottom bar ----
        bov = frame.copy()
        cv2.rectangle(bov, (0, HEIGHT - 34), (WIDTH, HEIGHT), C_PANEL, -1)
        cv2.addWeighted(bov, 0.75, frame, 0.25, 0, frame)
        ts = time.strftime("%H:%M:%S")
        cv2.putText(frame,
                    f"ScanAR Dual  ·  {ts}  ·  "
                    f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d} elapsed",
                    (16, HEIGHT - 10), FONT, 0.38, C_DIM, 1, cv2.LINE_AA)

        # ================================================================
        cv2.imshow(window, frame)
        key = cv2.waitKey(delay_ms) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break
        elif key in (ord('e'), ord('E')):
            show_eng = not show_eng

    cv2.destroyAllWindows()
    print("HUD closed.")


if __name__ == "__main__":
    main()
