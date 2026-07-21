#!/usr/bin/env python3

class HudOverlay:
    def __init__(self, width=1920, height=1080):
        self.width = width
        self.height = height

    def project_coordinates(self, x, y, z):
        # Project 3D SLAM coordinate points into Vitures HUD pixel overlay
        return int(x * 100 + self.width / 2), int(y * 100 + self.height / 2)
