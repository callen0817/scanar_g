"""
hud_widgets/keyplan.py
======================
Live 2D map widget for the ScanAR Dual operator HUD.

Responsibilities
----------------
- Accept pose updates (x, y, heading) — source-agnostic
- Accept map point updates (list of (x, y) world coords) — source-agnostic
- Render: accumulated scan points + trajectory + heading arrow → cv2 frame

The widget does NOT know whether data arrives from:
  · FAST-LIO odometry (production)
  · scanar_simulation topics (development)

It only knows:
  "I receive poses. I receive map points. I draw them."
"""

import cv2
import numpy as np
import math

# AR colour palette (BGR)
_C_PANEL  = (16,  20,  26)
_C_EDGE   = (50, 185, 205)
_C_CYAN   = (220, 205,  0)
_C_TRAJ   = (  0, 220, 255)
_C_PT     = ( 55, 200, 100)
_C_WHITE  = (235, 235, 235)

_FONT = cv2.FONT_HERSHEY_SIMPLEX


class KeyplanWidget:
    """
    Stateful minimap widget. Call update_pose() and update_map_points()
    as data arrives, then call draw() each render frame.
    """

    MAX_TRAJECTORY_PTS = 10_000
    MAX_MAP_PTS        = 200_000
    RENDER_MAP_PTS     = 12_000   # subsample limit for performance

    def __init__(self):
        self.pose_x  = 0.0
        self.pose_y  = 0.0
        self.heading = 0.0

        self._trajectory: list[tuple[float, float]] = []
        self._map_points: list[tuple[float, float]] = []

        # Auto-scaling world bounds
        self._wx_min = -1.0
        self._wy_min = -1.0
        self._wx_max =  1.0
        self._wy_max =  1.0

    # ------------------------------------------------------------------
    # Public update API (source-agnostic)
    # ------------------------------------------------------------------

    def update_pose(self, x: float, y: float, heading: float) -> None:
        """Update robot position and heading (world metres, radians)."""
        self.pose_x  = x
        self.pose_y  = y
        self.heading = heading
        self._trajectory.append((x, y))
        if len(self._trajectory) > self.MAX_TRAJECTORY_PTS:
            self._trajectory = self._trajectory[-self.MAX_TRAJECTORY_PTS:]
        self._expand_bounds(x, y, pad=8.0)

    def update_map_points(self, pts: list[tuple[float, float]]) -> None:
        """Append a batch of 2D world-frame scan returns."""
        self._map_points.extend(pts)
        if len(self._map_points) > self.MAX_MAP_PTS:
            self._map_points = self._map_points[-self.MAX_MAP_PTS:]
        for wx, wy in pts:
            self._expand_bounds(wx, wy, pad=2.0)

    # ------------------------------------------------------------------
    # Render
    # ------------------------------------------------------------------

    def draw(self, frame: np.ndarray,
             map_x: int, map_y: int,
             map_w: int, map_h: int) -> None:
        """
        Render the keyplan into `frame` at pixel rect (map_x, map_y, map_w, map_h).
        Call once per render frame.
        """
        # Panel background
        ov = frame.copy()
        cv2.rectangle(ov, (map_x - 8, map_y - 26),
                      (map_x + map_w + 8, map_y + map_h + 16),
                      _C_PANEL, -1)
        cv2.addWeighted(ov, 0.85, frame, 0.15, 0, frame)
        cv2.rectangle(frame, (map_x - 8, map_y - 26),
                      (map_x + map_w + 8, map_y + map_h + 16),
                      _C_EDGE, 1)
        cv2.putText(frame, "MAP", (map_x - 2, map_y - 10),
                    _FONT, 0.42, _C_CYAN, 1, cv2.LINE_AA)

        # Map interior background
        cv2.rectangle(frame, (map_x, map_y),
                      (map_x + map_w, map_y + map_h), (5, 8, 6), -1)

        # Accumulated scan points
        if self._map_points:
            step = max(1, len(self._map_points) // self.RENDER_MAP_PTS)
            for i in range(0, len(self._map_points), step):
                wx, wy = self._map_points[i]
                px, py = self._w2px(wx, wy, map_x, map_y, map_w, map_h)
                if map_x <= px < map_x + map_w and map_y <= py < map_y + map_h:
                    frame[py, px] = _C_PT

        # Trajectory with age-based fade
        if len(self._trajectory) > 1:
            step = max(1, len(self._trajectory) // 600)
            mapped = []
            for wx, wy in self._trajectory[::step]:
                px, py = self._w2px(wx, wy, map_x, map_y, map_w, map_h)
                if map_x <= px < map_x + map_w and map_y <= py < map_y + map_h:
                    mapped.append((px, py))
            for i in range(1, len(mapped)):
                age = i / len(mapped)
                c = tuple(int(v * (0.25 + 0.75 * age)) for v in _C_TRAJ)
                cv2.line(frame, mapped[i - 1], mapped[i], c, 1, cv2.LINE_AA)

        # Robot position + heading arrow
        rpx, rpy = self._w2px(self.pose_x, self.pose_y,
                               map_x, map_y, map_w, map_h)
        _, _, scale = self._map_scale(map_w, map_h)
        arrow_len = max(8, int(4 * scale))
        ax = int(rpx + arrow_len * math.cos(self.heading))
        ay = int(rpy - arrow_len * math.sin(self.heading))
        if map_x < rpx < map_x + map_w and map_y < rpy < map_y + map_h:
            cv2.arrowedLine(frame, (rpx, rpy), (ax, ay),
                            _C_WHITE, 2, tipLength=0.45,
                            line_type=cv2.LINE_AA)
            cv2.circle(frame, (rpx, rpy), 5, _C_WHITE, -1)
            cv2.circle(frame, (rpx, rpy), 5, _C_CYAN,  1)

        # Border
        cv2.rectangle(frame, (map_x, map_y),
                      (map_x + map_w, map_y + map_h), _C_EDGE, 1)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _expand_bounds(self, wx: float, wy: float, pad: float) -> None:
        self._wx_min = min(self._wx_min, wx - pad)
        self._wy_min = min(self._wy_min, wy - pad)
        self._wx_max = max(self._wx_max, wx + pad)
        self._wy_max = max(self._wy_max, wy + pad)

    def _map_scale(self, map_w: int, map_h: int):
        rng_x = max(self._wx_max - self._wx_min, 1.0)
        rng_y = max(self._wy_max - self._wy_min, 1.0)
        scale = min(map_w / rng_x, map_h / rng_y) * 0.88
        mid_x = (self._wx_min + self._wx_max) / 2.0
        mid_y = (self._wy_min + self._wy_max) / 2.0
        return mid_x, mid_y, scale

    def _w2px(self, wx: float, wy: float,
              map_x: int, map_y: int,
              map_w: int, map_h: int) -> tuple[int, int]:
        mid_x, mid_y, scale = self._map_scale(map_w, map_h)
        cx = map_x + map_w // 2
        cy = map_y + map_h // 2
        px = int(cx + (wx - mid_x) * scale)
        py = int(cy - (wy - mid_y) * scale)
        return px, py
