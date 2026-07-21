# GUI Inventory

This document lists the GUI, HUD, and rendering assets identified inside `~/scanar_dual`.

## 1. ROS Package: `viture_hud`
The primary HUD renderer that runs at 60 Hz to display the operator view onto the VITURE glasses.

- **Main Nodes / Scripts** (`src/`):
  - `hud_renderer.py` — ROS2 node carrying the main CV2 display window loop. Subscribes to `/fast_lio/odometry`, `/scanar/simulation/map_points`, `/scanar/scan_confidence_breakdown`, `/scanar/calibration/state`, `/scanar/diagnostics/system_health`, `/scanar/diagnostics/latency_report`, and `/scanar/hud/eng_mode`.
  - `hud_overlay.py` — Script defining HUD overlay configurations.
  - `hud_state_manager.py` — Tracks rendering state.
  - `hud_operator.py` — Standalone mockup simulation driving HUD render tests.
  - `hud_visual_demo.py` — Visual demo script.
  - `hud_capture_sim.py` — Capture scenario simulation.

- **Widgets** (`src/hud_widgets/`):
  - `keyplan.py` — Custom minimap widget displaying accumulated 2D scan points, trajectory lines with fade effects, and current pose arrow.
  - `confidence.py` — Placeholder confidence rendering widget.
  - `imu.py` — Placeholder IMU status widget.
  - `lidar.py` — Placeholder LiDAR status widget.
  - `map.py` — Placeholder map status widget.
  - `notifications.py` — Placeholder notifications widget.
  - `recording.py` — Recording status widget.
  - `storage.py` — Storage capacity widget.

## 2. ROS Package: `scanar_diagnostics_dashboard`
A terminal-based text dashboard displaying live topic status.

- **Nodes** (`src/`):
  - `dashboard_node.py` — Subscribes to diagnostics topics and renders an ANSI text-based dashboard layout in the terminal at 1 Hz.
