#!/usr/bin/env python3
"""
ScanAR Dual — Visual HUD Demo
Renders a graphical stereoscopic HUD overlay using OpenCV.
Cycles through all 4 simulation scenarios so the operator can visually
validate the layout before physical deployment.

Run:
    python3 hud_visual_demo.py
Press Q or Escape to quit.
"""

import cv2
import numpy as np
import time
import math

# ---------------------------------------------------------------------------
# Display & rendering constants
# ---------------------------------------------------------------------------
WIDTH  = 1920
HEIGHT = 1080
FPS    = 60

# AR-style colour palette (BGR for OpenCV)
C_BG         = (0,   0,   0)          # pure black background
C_CYAN       = (255, 210, 0)          # primary accent — teal/cyan
C_GREEN      = (80,  255, 140)        # OK / success
C_AMBER      = (0,   180, 255)        # warning
C_RED        = (60,  60,  255)        # critical / error
C_WHITE      = (255, 255, 255)
C_DIM        = (120, 120, 120)
C_PANEL_BG   = (20,  20,  20)        # semi-dark panel fill
C_PANEL_EDGE = (60,  200, 220)        # teal border

FONT       = cv2.FONT_HERSHEY_SIMPLEX
FONT_MONO  = cv2.FONT_HERSHEY_PLAIN

# ---------------------------------------------------------------------------
# Scenario definitions (mirrors scenarios.yaml)
# ---------------------------------------------------------------------------
SCENARIOS = [
    {
        "name": "BOOT / INIT",
        "label_color": C_AMBER,
        "confidence": 80,
        "slam_ready": False,
        "slam_auth": False,
        "poses": 0,
        "distance_m": 0.0,
        "points_m": 0.0,
        "cpu_load": 10.0,
        "cpu_temp": 38.0,
        "ram_pct": 22.0,
        "latency_ms": 0.0,
        "warnings": ["SLAM NOT AUTHORIZED", "Awaiting calibration gate"],
        "recording": False,
        "storage_gb": 124.0,
    },
    {
        "name": "NORMAL MAPPING",
        "label_color": C_GREEN,
        "confidence": 98,
        "slam_ready": True,
        "slam_auth": True,
        "poses": 427,
        "distance_m": 42.7,
        "points_m": 12.4,
        "cpu_load": 45.0,
        "cpu_temp": 52.0,
        "ram_pct": 61.0,
        "latency_ms": 29.3,
        "warnings": [],
        "recording": True,
        "storage_gb": 118.6,
    },
    {
        "name": "WARNING STATE",
        "label_color": C_AMBER,
        "confidence": 65,
        "slam_ready": True,
        "slam_auth": True,
        "poses": 620,
        "distance_m": 61.2,
        "points_m": 18.1,
        "cpu_load": 78.0,
        "cpu_temp": 71.0,
        "ram_pct": 82.0,
        "latency_ms": 38.5,
        "warnings": ["HIGH CPU TEMPERATURE", "SLAM QUALITY DEGRADED"],
        "recording": True,
        "storage_gb": 105.2,
    },
    {
        "name": "FAILURE RECOVERY",
        "label_color": C_RED,
        "confidence": 10,
        "slam_ready": True,
        "slam_auth": True,
        "poses": 620,
        "distance_m": 61.2,
        "points_m": 18.1,
        "cpu_load": 35.0,
        "cpu_temp": 58.0,
        "ram_pct": 55.0,
        "latency_ms": 0.0,
        "warnings": ["SLAM TRACKING LOST", "HOLD POSITION — RELOCALIZING"],
        "recording": False,
        "storage_gb": 105.2,
    },
]

SCENARIO_DURATION = 8.0   # seconds per scenario

# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def draw_panel(frame, x, y, w, h, label=None, alpha=0.85):
    """Draw a semi-transparent dark panel with a teal border."""
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x+w, y+h), C_PANEL_BG, -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
    cv2.rectangle(frame, (x, y), (x+w, y+h), C_PANEL_EDGE, 1)
    if label:
        cv2.putText(frame, label, (x+10, y+20), FONT, 0.45, C_CYAN, 1, cv2.LINE_AA)


def draw_bar(frame, x, y, w, h, value, max_val, color):
    """Draw a horizontal progress bar."""
    cv2.rectangle(frame, (x, y), (x+w, y+h), (40, 40, 40), -1)
    fill = int(w * min(value, max_val) / max_val)
    if fill > 0:
        cv2.rectangle(frame, (x, y), (x+fill, y+h), color, -1)
    cv2.rectangle(frame, (x, y), (x+w, y+h), C_DIM, 1)


def draw_arc_dial(frame, cx, cy, r, value, max_val, color, thickness=6):
    """Draw a sweeping arc dial (progress circle)."""
    cv2.circle(frame, (cx, cy), r, (40, 40, 40), thickness)
    angle = int(270 * value / max_val)
    cv2.ellipse(frame, (cx, cy), (r, r), -90, 0, angle, color, thickness, cv2.LINE_AA)


def draw_scanlines(frame, alpha=0.04):
    """Subtle horizontal scanline CRT effect for AR feel."""
    overlay = frame.copy()
    for y in range(0, HEIGHT, 4):
        cv2.line(overlay, (0, y), (WIDTH, y), (0, 0, 0), 1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


def draw_grid(frame, spacing=80, alpha=0.06):
    """Faint perspective grid to simulate AR world anchor."""
    overlay = frame.copy()
    for x in range(0, WIDTH, spacing):
        cv2.line(overlay, (x, 0), (x, HEIGHT), C_PANEL_EDGE, 1)
    for y in range(0, HEIGHT, spacing):
        cv2.line(overlay, (0, y), (WIDTH, y), C_PANEL_EDGE, 1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


def blink_color(color_on, color_off, hz=1.0):
    t = time.time()
    return color_on if math.sin(t * hz * 2 * math.pi) > 0 else color_off


# ---------------------------------------------------------------------------
# Per-frame render
# ---------------------------------------------------------------------------

def render_frame(sc, frame_idx, t_global):
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    # -- Background grid --
    draw_grid(frame)

    # -- Top bar --
    draw_panel(frame, 0, 0, WIDTH, 46)
    ts = time.strftime("%Y-%m-%d  %H:%M:%S")
    cv2.putText(frame, f"SCANAR DUAL  ·  HUD v1.0  ·  {ts}",
                (16, 30), FONT, 0.55, C_CYAN, 1, cv2.LINE_AA)

    # Scenario badge (top-right)
    sc_label = f"[ {sc['name']} ]"
    sc_w, _ = cv2.getTextSize(sc_label, FONT, 0.6, 2)[0]
    cv2.putText(frame, sc_label, (WIDTH - sc_w - 20, 30),
                FONT, 0.6, sc["label_color"], 2, cv2.LINE_AA)

    # -- LEFT PANEL: Confidence & SLAM --
    PX, PY, PW, PH = 20, 60, 320, 420
    draw_panel(frame, PX, PY, PW, PH, "SLAM  CONFIDENCE")

    conf = sc["confidence"]
    dial_cx, dial_cy, dial_r = PX + PW//2, PY + 140, 80
    dial_color = C_GREEN if conf >= 80 else (C_AMBER if conf >= 50 else C_RED)
    draw_arc_dial(frame, dial_cx, dial_cy, dial_r, conf, 100, dial_color, 10)
    cv2.putText(frame, f"{conf}%",
                (dial_cx - 28, dial_cy + 14), FONT, 1.1, dial_color, 2, cv2.LINE_AA)
    cv2.putText(frame, "CONFIDENCE",
                (dial_cx - 55, dial_cy + dial_r + 22), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)

    # SLAM authorized indicator
    auth_color = C_GREEN if sc["slam_auth"] else blink_color(C_AMBER, C_DIM, hz=2)
    auth_text  = "SLAM  AUTHORIZED" if sc["slam_auth"] else "SLAM  NOT AUTH"
    cv2.circle(frame, (PX + 20, PY + 250), 8, auth_color, -1)
    cv2.putText(frame, auth_text, (PX + 36, PY + 255), FONT, 0.45, auth_color, 1, cv2.LINE_AA)

    # SLAM ready
    rdy_color = C_GREEN if sc["slam_ready"] else C_AMBER
    cv2.circle(frame, (PX + 20, PY + 280), 8, rdy_color, -1)
    rdy_txt = "ENGINE READY" if sc["slam_ready"] else "ENGINE OFFLINE"
    cv2.putText(frame, rdy_txt, (PX + 36, PY + 285), FONT, 0.45, rdy_color, 1, cv2.LINE_AA)

    # Pose count
    cv2.putText(frame, f"Poses:", (PX + 16, PY + 325), FONT, 0.42, C_DIM, 1, cv2.LINE_AA)
    cv2.putText(frame, f"{sc['poses']:,}", (PX + 90, PY + 325), FONT, 0.48, C_WHITE, 1, cv2.LINE_AA)
    cv2.putText(frame, f"Distance:", (PX + 16, PY + 350), FONT, 0.42, C_DIM, 1, cv2.LINE_AA)
    cv2.putText(frame, f"{sc['distance_m']:.1f} m", (PX + 110, PY + 350), FONT, 0.48, C_WHITE, 1, cv2.LINE_AA)
    cv2.putText(frame, f"Points:", (PX + 16, PY + 375), FONT, 0.42, C_DIM, 1, cv2.LINE_AA)
    cv2.putText(frame, f"{sc['points_m']:.1f} M", (PX + 90, PY + 375), FONT, 0.48, C_WHITE, 1, cv2.LINE_AA)

    # -- RIGHT PANEL: System Health --
    RX, RY, RW, RH = WIDTH - 340, 60, 320, 350
    draw_panel(frame, RX, RY, RW, RH, "SYSTEM  HEALTH")

    # CPU bar
    cpu_color = C_GREEN if sc["cpu_load"] < 60 else (C_AMBER if sc["cpu_load"] < 85 else C_RED)
    cv2.putText(frame, "CPU", (RX+16, RY+55), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)
    draw_bar(frame, RX+70, RY+43, RW-90, 16, sc["cpu_load"], 100, cpu_color)
    cv2.putText(frame, f"{sc['cpu_load']:.0f}%", (RX+RW-55, RY+55), FONT, 0.42, cpu_color, 1, cv2.LINE_AA)

    # RAM bar
    ram_color = C_GREEN if sc["ram_pct"] < 70 else (C_AMBER if sc["ram_pct"] < 90 else C_RED)
    cv2.putText(frame, "RAM", (RX+16, RY+90), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)
    draw_bar(frame, RX+70, RY+78, RW-90, 16, sc["ram_pct"], 100, ram_color)
    cv2.putText(frame, f"{sc['ram_pct']:.0f}%", (RX+RW-55, RY+90), FONT, 0.42, ram_color, 1, cv2.LINE_AA)

    # Temperature bar
    temp_color = C_GREEN if sc["cpu_temp"] < 60 else (C_AMBER if sc["cpu_temp"] < 75 else C_RED)
    cv2.putText(frame, "TEMP", (RX+16, RY+125), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)
    draw_bar(frame, RX+70, RY+113, RW-90, 16, sc["cpu_temp"], 100, temp_color)
    cv2.putText(frame, f"{sc['cpu_temp']:.0f}°C", (RX+RW-60, RY+125), FONT, 0.42, temp_color, 1, cv2.LINE_AA)

    # Latency
    lat_color = C_GREEN if sc["latency_ms"] < 33 else (C_AMBER if sc["latency_ms"] < 50 else C_RED)
    cv2.putText(frame, "PIPELINE LATENCY", (RX+16, RY+180), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)
    lat_txt = f"{sc['latency_ms']:.1f} ms" if sc["latency_ms"] > 0 else "-- ms"
    cv2.putText(frame, lat_txt, (RX+16, RY+210), FONT, 0.9, lat_color, 2, cv2.LINE_AA)

    # Storage
    cv2.putText(frame, "FREE STORAGE", (RX+16, RY+260), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)
    stor_color = C_GREEN if sc["storage_gb"] > 50 else C_AMBER
    cv2.putText(frame, f"{sc['storage_gb']:.1f} GB", (RX+16, RY+290), FONT, 0.9, stor_color, 2, cv2.LINE_AA)

    # -- RECORDING indicator (top-right corner of right panel) --
    if sc["recording"]:
        rec_color = blink_color(C_RED, C_DIM, hz=1.5)
        cv2.circle(frame, (RX + RW - 24, RY + 24), 10, rec_color, -1)
        cv2.putText(frame, "REC", (RX + RW - 56, RY + 18), FONT, 0.4, rec_color, 1, cv2.LINE_AA)

    # -- CENTER PANEL: Warnings / Status --
    CX, CY, CW, CH = 360, 60, WIDTH-720, 220
    draw_panel(frame, CX, CY, CW, CH, "OPERATOR  STATUS")

    if sc["warnings"]:
        warn_color = blink_color(sc["label_color"], C_DIM, hz=1.0)
        for i, w in enumerate(sc["warnings"]):
            cv2.putText(frame, f"⚠  {w}", (CX+20, CY+80+i*50),
                        FONT, 0.72, warn_color, 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, "ALL SYSTEMS NOMINAL", (CX+30, CY+110),
                    FONT, 0.9, C_GREEN, 2, cv2.LINE_AA)
        cv2.putText(frame, "Mapping in progress", (CX+50, CY+155),
                    FONT, 0.55, C_DIM, 1, cv2.LINE_AA)

    # -- LiDAR streaming indicators --
    LX, LY, LW, LH = 360, 300, WIDTH-720, 120
    draw_panel(frame, LX, LY, LW, LH, "LIDAR  STREAMS")

    pulse = blink_color(C_GREEN, (0, 100, 50), hz=10.0)
    cv2.circle(frame, (LX+30,  LY+60), 10, pulse, -1)
    cv2.putText(frame, "LEFT AIRY  192.168.10.10  10.0 Hz",
                (LX+50, LY+65), FONT, 0.5, C_WHITE, 1, cv2.LINE_AA)
    cv2.circle(frame, (LX+30,  LY+95), 10, pulse, -1)
    cv2.putText(frame, "RIGHT AIRY  192.168.11.11  10.0 Hz",
                (LX+50, LY+100), FONT, 0.5, C_WHITE, 1, cv2.LINE_AA)
    # IMU
    imu_col = blink_color(C_CYAN, (0, 80, 120), hz=5.0)
    cv2.circle(frame, (LX+LW-200, LY+60), 8, imu_col, -1)
    cv2.putText(frame, "IG-2 IMU  379 Hz",
                (LX+LW-185, LY+65), FONT, 0.5, C_WHITE, 1, cv2.LINE_AA)
    ptp_col = C_GREEN
    cv2.circle(frame, (LX+LW-200, LY+95), 8, ptp_col, -1)
    cv2.putText(frame, "PTP SYNC  15 ns",
                (LX+LW-185, LY+100), FONT, 0.5, C_WHITE, 1, cv2.LINE_AA)

    # -- BOTTOM BAR: FPS & scenario progress --
    BY = HEIGHT - 50
    draw_panel(frame, 0, BY, WIDTH, 50)

    elapsed_in_scene = (t_global % (SCENARIO_DURATION * len(SCENARIOS))) % SCENARIO_DURATION
    draw_bar(frame, 20, BY+18, 300, 12, elapsed_in_scene, SCENARIO_DURATION, C_CYAN)
    cv2.putText(frame, f"SCENARIO  {sc['name']}  [{elapsed_in_scene:.1f}s / {SCENARIO_DURATION:.0f}s]",
                (330, BY+28), FONT, 0.45, C_DIM, 1, cv2.LINE_AA)

    act_fps = min(FPS, 1.0 / max(0.001, 1.0/FPS))
    cv2.putText(frame, f"HUD  60 FPS", (WIDTH - 200, BY+28),
                FONT, 0.48, C_CYAN, 1, cv2.LINE_AA)

    # Corner crosshair reticle (centre point)
    mid = (WIDTH//2, HEIGHT//2)
    cv2.line(frame, (mid[0]-20, mid[1]), (mid[0]+20, mid[1]), C_CYAN, 1)
    cv2.line(frame, (mid[0], mid[1]-20), (mid[0], mid[1]+20), C_CYAN, 1)
    cv2.circle(frame, mid, 6, C_CYAN, 1)

    # Scanline overlay for CRT / AR aesthetics
    draw_scanlines(frame)

    return frame


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main():
    window_name = "ScanAR Dual — HUD Visual Demo  (Q / ESC to quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, WIDTH, HEIGHT)

    frame_idx  = 0
    t_start    = time.time()
    delay_ms   = max(1, int(1000 / FPS))

    print("=" * 60)
    print("  ScanAR Dual — HUD Visual Demo")
    print("  Cycling through 4 scenarios automatically.")
    print("  Press  Q  or  ESC  to quit.")
    print("=" * 60)

    while True:
        t_now    = time.time() - t_start
        sc_idx   = int(t_now / SCENARIO_DURATION) % len(SCENARIOS)
        sc       = SCENARIOS[sc_idx]

        frame = render_frame(sc, frame_idx, t_now)
        cv2.imshow(window_name, frame)

        frame_idx += 1

        key = cv2.waitKey(delay_ms) & 0xFF
        if key in (ord('q'), ord('Q'), 27):   # Q or ESC
            break

    cv2.destroyAllWindows()
    print("HUD demo closed.")


if __name__ == "__main__":
    main()
