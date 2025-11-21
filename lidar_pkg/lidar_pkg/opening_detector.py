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
    if point_cloud.size() < 10:
        print("ERROR: Insufficient points in cloud")
        return None

    sorted_points = point_cloud.get_sorted_by_angle()

    # Find the largest gap/discontinuity
    max_gap = 0
    gap_edge1_idx = -1
    gap_edge2_idx = -1

    for i in range(len(sorted_points)):
        current = sorted_points[i]
        next_point = sorted_points[(i + 1) % len(sorted_points)]

        # Calculate distance jump between consecutive angle positions
        distance_diff = abs(next_point['distance'] - current['distance'])

        if distance_diff > max_gap:
            max_gap = distance_diff
            gap_edge1_idx = i
            gap_edge2_idx = (i + 1) % len(sorted_points)

    # Verify gap is significant
    if max_gap < DISCONTINUITY_THRESHOLD_MM:
        print(f"WARNING: Max discontinuity ({max_gap:.0f}mm) below threshold ({DISCONTINUITY_THRESHOLD_MM}mm)")
        return None

    # Extract edge points
    point1 = sorted_points[gap_edge1_idx]
    point2 = sorted_points[gap_edge2_idx]

    return (point1, point2, max_gap)
