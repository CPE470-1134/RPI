#!/usr/bin/env python3

import numpy as np

# Configuration constants
MIN_DISTANCE_MM = 100  # Ignore points closer than 100mm (noise)
MAX_DISTANCE_MM = 10000  # Ignore points farther than 10m
INTENSITY_THRESHOLD = 10  # Ignore very low intensity points
DISCONTINUITY_THRESHOLD_MM = 500  # Gap threshold: 500mm indicates opening edge


class PointCloud:
    def __init__(self):
        self.points = []  # List of (x, y, angle, distance, intensity)

    def add_point(self, distance_mm, intensity, angle_deg):
        # Filter noisy/invalid points
        if distance_mm < MIN_DISTANCE_MM or distance_mm > MAX_DISTANCE_MM:
            return
        if intensity < INTENSITY_THRESHOLD:
            return

        # Convert polar to Cartesian (mm)
        angle_rad = np.radians(angle_deg)
        x = distance_mm * np.cos(angle_rad)
        y = distance_mm * np.sin(angle_rad)

        self.points.append({
            'x': x,
            'y': y,
            'angle': angle_deg,
            'distance': distance_mm,
            'intensity': intensity
        })

    def get_sorted_by_angle(self):
        return sorted(self.points, key=lambda p: p['angle'])

    def size(self):
        return len(self.points)




def get_opening(point_cloud):
    if point_cloud.size() < 50:
        print("ERROR: Insufficient points in cloud")
        return None

    pts = point_cloud.get_sorted_by_angle()

    max_gap = 0
    gap_p1 = None
    gap_p2 = None

    for i in range(len(pts)):
        p1 = pts[i]
        p2 = pts[(i + 1) % len(pts)]

        # Compute true geometric gap
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        gap = np.sqrt(dx*dx + dy*dy)

        if gap > max_gap:
            max_gap = gap
            gap_p1 = p1
            gap_p2 = p2

    if max_gap < DISCONTINUITY_THRESHOLD_MM:
        print(f"No opening found (max gap {max_gap:.1f} < threshold {DISCONTINUITY_THRESHOLD_MM})")
        return None

    return gap_p1, gap_p2, max_gap