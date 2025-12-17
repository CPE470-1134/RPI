#!/usr/bin/env python3
"""
Localization Geometry Utilities

Pure mathematical functions for robot localization using ArUco markers.
No ROS dependencies - testable geometry and triangulation algorithms.
"""

import math
from typing import Optional, List, Tuple
import numpy as np


class MarkerObservation:
    """
    Single marker observation from robot's perspective

    Attributes:
        marker_id (int): Marker ID
        bearing_rad (float): Bearing angle from robot to marker (in robot frame)
        distance_m (float): Distance from robot to marker
        quality (float): Observation quality score (0-1)
    """
    def __init__(self, marker_id: int, bearing_rad: float, distance_m: float, quality: float = 1.0):
        self.marker_id = marker_id
        self.bearing_rad = bearing_rad
        self.distance_m = distance_m
        self.quality = quality


class ArenaPose:
    """
    Robot pose in arena coordinate frame

    Attributes:
        x (float): X position in meters
        y (float): Y position in meters
        theta (float): Heading in radians
        covariance (Optional[np.ndarray]): 3x3 covariance matrix
    """
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, 
                 covariance: Optional[np.ndarray] = None):
        self.x = x
        self.y = y
        self.theta = theta
        self.covariance = covariance if covariance is not None else np.eye(3) * 0.01


class LocalizationGeometry:
    """
    Geometry and triangulation utilities for localization
    
    All methods are static - this is a utility class with no state.
    Handles coordinate transforms, triangulation, and geometric calculations.
    """

    @staticmethod
    def triangulate_position_from_two_markers(
        obs1: MarkerObservation, marker1_pos: Tuple[float, float],
        obs2: MarkerObservation, marker2_pos: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        """
        Triangulate robot position from two marker observations using trilateration
        
        Args:
            obs1: First marker observation
            marker1_pos: Known position of first marker (x, y) in arena frame
            obs2: Second marker observation  
            marker2_pos: Known position of second marker (x, y) in arena frame
            
        Returns:
            Robot position (x, y) in arena frame, or None if no valid solution
        """
        # Use circle-circle intersection based on distances
        intersections = LocalizationGeometry._circle_circle_intersection(
            marker1_pos, obs1.distance_m,
            marker2_pos, obs2.distance_m
        )
        
        if len(intersections) == 0:
            return None
        elif len(intersections) == 1:
            return intersections[0]
        else:
            # Two possible positions - need to disambiguate
            # Use bearing information to select correct one
            return LocalizationGeometry._select_position_from_bearing(
                intersections, obs1, marker1_pos, obs2, marker2_pos
            )

    @staticmethod
    def triangulate_position_from_multiple_markers(
        observations: List[MarkerObservation],
        marker_positions: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        """
        Triangulate robot position from 3+ markers using least squares
        
        Args:
            observations: List of marker observations
            marker_positions: List of known marker positions (x, y)
            
        Returns:
            Best-fit robot position (x, y), or None if insufficient data
        """
        if len(observations) < 2:
            return None
        
        if len(observations) == 2:
            return LocalizationGeometry.triangulate_position_from_two_markers(
                observations[0], marker_positions[0],
                observations[1], marker_positions[1]
            )
        
        # Use least squares for overdetermined system (3+ markers)
        return LocalizationGeometry._least_squares_trilateration(
            observations, marker_positions
        )

    @staticmethod
    def compute_heading_from_marker_bearing(
        robot_pos: Tuple[float, float],
        marker_obs: MarkerObservation,
        marker_pos: Tuple[float, float]
    ) -> float:
        """
        Compute robot heading from position and marker bearing observation
        
        Args:
            robot_pos: Robot position (x, y) in arena frame
            marker_obs: Marker observation with bearing in robot frame
            marker_pos: Known marker position (x, y) in arena frame
            
        Returns:
            Robot heading in radians (arena frame)
            
        Logic:
            - Compute absolute bearing from robot to marker in arena frame
            - Robot heading = absolute_bearing - observed_bearing
        """
        # Absolute bearing from robot to marker in arena frame
        absolute_bearing = math.atan2(
            marker_pos[1] - robot_pos[1],
            marker_pos[0] - robot_pos[0]
        )
        
        # Robot heading = where robot is facing
        # If robot faces angle θ, and sees marker at bearing β (relative),
        # then absolute bearing to marker = θ + β
        # Therefore: θ = absolute_bearing - β
        robot_heading = absolute_bearing - marker_obs.bearing_rad
        
        return LocalizationGeometry.normalize_angle(robot_heading)

    @staticmethod
    def compute_expected_marker_observation(
        robot_pose: ArenaPose,
        marker_pos: Tuple[float, float]
    ) -> Tuple[float, float]:
        """
        Compute expected bearing and distance to marker from robot pose
        
        Args:
            robot_pose: Current robot pose estimate
            marker_pos: Known marker position (x, y)
            
        Returns:
            (expected_bearing_rad, expected_distance_m)
        """
        # Distance from robot to marker
        dx = marker_pos[0] - robot_pose.x
        dy = marker_pos[1] - robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Absolute bearing from robot to marker in arena frame
        absolute_bearing = math.atan2(dy, dx)
        
        # Relative bearing in robot frame
        relative_bearing = absolute_bearing - robot_pose.theta
        relative_bearing = LocalizationGeometry.normalize_angle(relative_bearing)
        
        return (relative_bearing, distance)

    @staticmethod
    def transform_pose_by_odometry_delta(
        current_pose: ArenaPose,
        delta_x: float, delta_y: float, delta_theta: float
    ) -> ArenaPose:
        """
        Update pose by applying odometry motion in robot's local frame
        
        Args:
            current_pose: Current robot pose in arena frame
            delta_x: Forward motion in robot frame (meters)
            delta_y: Lateral motion in robot frame (meters)  
            delta_theta: Rotation change (radians)
            
        Returns:
            Updated pose in arena frame
        """
        # Rotate delta motion from robot frame to arena frame
        cos_theta = math.cos(current_pose.theta)
        sin_theta = math.sin(current_pose.theta)
        
        arena_delta_x = delta_x * cos_theta - delta_y * sin_theta
        arena_delta_y = delta_x * sin_theta + delta_y * cos_theta
        
        new_pose = ArenaPose(
            x=current_pose.x + arena_delta_x,
            y=current_pose.y + arena_delta_y,
            theta=LocalizationGeometry.normalize_angle(current_pose.theta + delta_theta)
        )
        
        return new_pose

    # ========================================================================
    # Private Helper Methods
    # ========================================================================

    @staticmethod
    def _circle_circle_intersection(
        center1: Tuple[float, float], radius1: float,
        center2: Tuple[float, float], radius2: float,
        tolerance: float = 0.001
    ) -> List[Tuple[float, float]]:
        """
        Find intersection points of two circles
        
        Args:
            center1: (x, y) of first circle
            radius1: Radius of first circle
            center2: (x, y) of second circle
            radius2: Radius of second circle
            tolerance: Numerical tolerance for edge cases
            
        Returns:
            List of intersection points (0, 1, or 2 points)
        """
        x1, y1 = center1
        x2, y2 = center2
        
        # Distance between centers
        d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Check if circles don't intersect or are identical
        if d > radius1 + radius2 + tolerance:
            return []  # Too far apart
        if d < abs(radius1 - radius2) - tolerance:
            return []  # One inside the other
        if d < tolerance and abs(radius1 - radius2) < tolerance:
            return []  # Identical circles (infinite solutions)
        
        # Check for single tangent point
        if abs(d - (radius1 + radius2)) < tolerance or abs(d - abs(radius1 - radius2)) < tolerance:
            # Single intersection point
            t = radius1 / d
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return [(x, y)]
        
        # Two intersection points
        # Using formula from: http://paulbourke.net/geometry/circlesphere/
        a = (radius1**2 - radius2**2 + d**2) / (2 * d)
        h = math.sqrt(max(0, radius1**2 - a**2))  # max for numerical stability
        
        # Point on line between centers
        px = x1 + a * (x2 - x1) / d
        py = y1 + a * (y2 - y1) / d
        
        # Two intersection points
        p1 = (px + h * (y2 - y1) / d, py - h * (x2 - x1) / d)
        p2 = (px - h * (y2 - y1) / d, py + h * (x2 - x1) / d)
        
        return [p1, p2]

    @staticmethod
    def _select_position_from_bearing(
        candidates: List[Tuple[float, float]],
        obs1: MarkerObservation, marker1_pos: Tuple[float, float],
        obs2: MarkerObservation, marker2_pos: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        """
        Select correct position from multiple candidates using bearing info
        
        Args:
            candidates: List of possible robot positions
            obs1, obs2: Marker observations with bearings
            marker1_pos, marker2_pos: Known marker positions
            
        Returns:
            Best matching position or None
        """
        if len(candidates) == 0:
            return None
        if len(candidates) == 1:
            return candidates[0]
        
        # For each candidate, compute what the bearing would be
        # Choose the one that best matches observed bearings
        best_candidate = None
        best_error = float('inf')
        
        for candidate in candidates:
            # Compute heading for this candidate using first marker
            heading = LocalizationGeometry.compute_heading_from_marker_bearing(
                candidate, obs1, marker1_pos
            )
            
            # Compute expected bearing to second marker
            pose = ArenaPose(candidate[0], candidate[1], heading)
            expected_bearing, _ = LocalizationGeometry.compute_expected_marker_observation(
                pose, marker2_pos
            )
            
            # Error between expected and observed bearing
            bearing_error = abs(LocalizationGeometry.normalize_angle(
                expected_bearing - obs2.bearing_rad
            ))
            
            if bearing_error < best_error:
                best_error = bearing_error
                best_candidate = candidate
        
        return best_candidate

    @staticmethod
    def _least_squares_trilateration(
        observations: List[MarkerObservation],
        marker_positions: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        """
        Solve for robot position using least squares (3+ markers)
        
        Args:
            observations: List of marker observations
            marker_positions: List of known marker positions
            
        Returns:
            Best-fit robot position (x, y)
        """
        if len(observations) < 3:
            return None
        
        # Set up least squares problem
        # For each marker i: (x - x_i)^2 + (y - y_i)^2 = d_i^2
        # Linearize: 2*x*x_i + 2*y*y_i = x_i^2 + y_i^2 + d_1^2 - d_i^2
        
        # Use first marker as reference
        x1, y1 = marker_positions[0]
        d1 = observations[0].distance_m
        
        A = []
        b = []
        
        for i in range(1, len(observations)):
            xi, yi = marker_positions[i]
            di = observations[i].distance_m
            
            A.append([2 * (xi - x1), 2 * (yi - y1)])
            b.append([xi**2 - x1**2 + yi**2 - y1**2 + d1**2 - di**2])
        
        A = np.array(A)
        b = np.array(b)
        
        # Solve least squares: A * [x, y]^T = b
        try:
            solution, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            return (float(solution[0][0]), float(solution[1][0]))
        except np.linalg.LinAlgError:
            return None

    @staticmethod
    def normalize_angle(angle_rad: float) -> float:
        """
        Normalize angle to range [-π, π]
        
        Args:
            angle_rad: Angle in radians
            
        Returns:
            Normalized angle in radians
        """
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad

    @staticmethod
    def compute_distance(x1: float, y1: float, x2: float, y2: float) -> float:
        """
        Compute Euclidean distance between two points
        
        Args:
            x1, y1: First point
            x2, y2: Second point
            
        Returns:
            Distance in same units as input
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    @staticmethod
    def compute_bearing(from_x: float, from_y: float, to_x: float, to_y: float) -> float:
        """
        Compute bearing angle from one point to another
        
        Args:
            from_x, from_y: Starting point
            to_x, to_y: Target point
            
        Returns:
            Bearing angle in radians (arena frame convention)
        """
        return math.atan2(to_y - from_y, to_x - from_x)
