#!/usr/bin/env python3
"""
Localizer Node - Robot Pose Estimation

This node maintains the robot's pose in the arena coordinate frame using
ArUco marker-based triangulation and odometry integration. It provides
continuous pose estimates and on-demand localization services.

Published Topics:
    /localizer/pose (geometry_msgs/PoseStamped):
        - Continuously published robot pose in arena frame
        - Origin at point A, X-axis right, Y-axis down
        - Updated via odometry integration and marker corrections

    /localizer/status (std_msgs/String):
        - Localization status: "NOT_LOCALIZED", "LOCALIZED", "TRACKING"
        - Quality indicators and error messages

Subscribed Topics:
    /aruco/marker_info (custom_msgs/ArucoMarkerArray):
        - Detected markers with ID, bearing, distance
        - Used for triangulation and drift correction

    /odom (nav_msgs/Odometry):
        - Wheel odometry from robot base
        - Integrated to maintain pose between marker observations

Services:
    /localizer/localize (std_srvs/Trigger):
        - Trigger new localization from markers
        - Returns: success flag and estimated pose
        - Requires robot to be stationary or rotating

    /localizer/set_pose (geometry_msgs/PoseStamped):
        - Manually set pose estimate
        - Used for initial pose or corrections

    /localizer/reset (std_srvs/Empty):
        - Reset to unlocalized state

Parameters:
    marker_20_x (float): X position of marker 20 (default: 0.0)
    marker_20_y (float): Y position of marker 20 (default: 0.575)
    marker_10_x (float): X position of marker 10 (default: 0.465)
    marker_10_y (float): Y position of marker 10 (default: 1.15)
    marker_30_x (float): X position of marker 30 (default: 0.465)
    marker_30_y (float): Y position of marker 30 (default: 0.0)
    min_markers_for_localization (int): Minimum markers needed (default: 2)
    pose_correction_enabled (bool): Auto-correct from markers (default: True)
    publish_rate_hz (float): Pose publishing rate (default: 10.0)
"""

import math
from typing import Optional, Dict, List, Tuple
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
import numpy as np

from custom_interfaces.msg import ArucoMarkerArray

# Import geometry utilities
from localization_geometry import LocalizationGeometry, MarkerObservation, ArenaPose


class LocalizationStatus:
    """
    Enumeration of localization states
    """
    NOT_LOCALIZED = "NOT_LOCALIZED"
    LOCALIZED = "LOCALIZED"
    TRACKING = "TRACKING"


class OdometryIntegrator:
    """
    Integrates odometry to track pose changes

    Attributes:
        last_odom_x (float): Last odometry x
        last_odom_y (float): Last odometry y
        last_odom_theta (float): Last odometry heading
        is_initialized (bool): Whether first odom received
    """
    def __init__(self):
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.is_initialized = False

    def update(self, odom_x: float, odom_y: float, odom_theta: float) -> Tuple[float, float, float]:
        """
        Compute delta motion since last update

        Returns:
            (delta_x, delta_y, delta_theta) in odometry frame
        """
        if not self.is_initialized:
            self.last_odom_x = odom_x
            self.last_odom_y = odom_y
            self.last_odom_theta = odom_theta
            self.is_initialized = True
            return (0.0, 0.0, 0.0)

        # Compute motion in odometry frame
        delta_x = odom_x - self.last_odom_x
        delta_y = odom_y - self.last_odom_y
        delta_theta = LocalizationGeometry.normalize_angle(odom_theta - self.last_odom_theta)

        # Update last values
        self.last_odom_x = odom_x
        self.last_odom_y = odom_y
        self.last_odom_theta = odom_theta

        return (delta_x, delta_y, delta_theta)


class LocalizerNode(Node):
    """
    ROS 2 node for robot pose estimation in arena frame

    Provides continuous pose tracking using marker-based localization
    and odometry integration. Uses LocalizationGeometry for all math.
    """

    def __init__(self) -> None:
        """
        Initialize the localizer node

        Sets up:
        - Known marker positions in arena
        - Pose tracking state
        - Odometry integrator
        - Publishers for pose and status
        - Subscribers for markers and odometry
        - Service servers for localization requests
        """
        super().__init__("localizer_node")

        # Declare parameters
        self._declare_parameters()

        # Load marker positions from parameters
        self.marker_positions: Dict[int, Tuple[float, float]] = {}
        self._load_marker_positions()

        # Localization state
        self.current_pose: Optional[ArenaPose] = None
        self.status: str = LocalizationStatus.NOT_LOCALIZED

        # Odometry integration
        self.odom_integrator = OdometryIntegrator()

        # Recent marker observations {marker_id: (observation, timestamp)}
        self.marker_observations: Dict[int, Tuple[MarkerObservation, float]] = {}
        self.observation_max_age = 1.0  # seconds

        # Setup ROS interfaces
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()

        self.get_logger().info("Localizer node initialized")
        self.get_logger().info(f"Known markers: {list(self.marker_positions.keys())}")

    def _declare_parameters(self) -> None:
        """
        Declare ROS 2 parameters

        Parameters for marker positions and localization settings
        """
        # Marker positions in arena frame
        self.declare_parameter('marker_20_x', 0.0)
        self.declare_parameter('marker_20_y', 0.575)
        self.declare_parameter('marker_10_x', 0.465)
        self.declare_parameter('marker_10_y', 1.15)
        self.declare_parameter('marker_30_x', 0.465)
        self.declare_parameter('marker_30_y', 0.0)

        # Localization settings
        self.declare_parameter('min_markers_for_localization', 2)
        self.declare_parameter('pose_correction_enabled', True)
        self.declare_parameter('publish_rate_hz', 10.0)

    def _load_marker_positions(self) -> None:
        """
        Load known marker positions from parameters

        Creates dictionary mapping marker_id -> (x, y) position
        """
        self.marker_positions[20] = (
            self.get_parameter('marker_20_x').value,
            self.get_parameter('marker_20_y').value
        )
        self.marker_positions[10] = (
            self.get_parameter('marker_10_x').value,
            self.get_parameter('marker_10_y').value
        )
        self.marker_positions[30] = (
            self.get_parameter('marker_30_x').value,
            self.get_parameter('marker_30_y').value
        )

    def _setup_publishers(self) -> None:
        """
        Create publishers

        Publishers:
        - /localizer/pose: Continuous pose estimate
        - /localizer/status: Localization status
        """
        self.pose_pub = self.create_publisher(PoseStamped, '/localizer/pose', 10)
        self.status_pub = self.create_publisher(String, '/localizer/status', 10)

    def _setup_subscribers(self) -> None:
        """
        Create subscribers

        Subscribers:
        - /aruco/marker_info: Marker detections
        - /odom: Odometry for integration
        """
        self.create_subscription(
            ArucoMarkerArray,
            '/aruco/marker_info',
            self._aruco_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

    def _setup_services(self) -> None:
        """
        Create service servers

        Services:
        - /localizer/localize: Trigger localization
        - /localizer/set_pose: Set pose manually
        - /localizer/reset: Reset localization
        """
        self.create_service(
            Trigger,
            '/localizer/localize',
            self._handle_localize_request
        )
        self.create_service(
            Empty,
            '/localizer/reset',
            self._handle_reset_request
        )

    def _setup_timers(self) -> None:
        """
        Create periodic timers

        Timers:
        - Pose publishing timer
        """
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate_hz, self._publish_pose)

    # ========================================================================
    # Service Handlers
    # ========================================================================

    def _handle_localize_request(self, request, response):
        """
        Handle localization service request

        Args:
            request: Trigger request
            response: Trigger response

        Returns:
            Response with success flag and message

        Attempts to localize from currently visible markers
        """
        # Get recent marker observations
        recent_obs = self._get_recent_observations()

        if len(recent_obs) < 2:
            response.success = False
            response.message = f"Insufficient markers visible ({len(recent_obs)}/2)"
            self.get_logger().warn(response.message)
            return response

        # Attempt localization
        pose = self._localize_from_markers(recent_obs)

        if pose is None:
            response.success = False
            response.message = "Triangulation failed"
            self.get_logger().error(response.message)
            return response

        # Success!
        self.current_pose = pose
        self.status = LocalizationStatus.LOCALIZED
        response.success = True
        response.message = f"Localized at ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.3f})"

        self.get_logger().info(response.message)
        self._publish_status(self.status, response.message)

        return response

    def _handle_reset_request(self, request, response):
        """
        Handle reset service request

        Args:
            request: Empty request
            response: Empty response

        Returns:
            Response

        Resets to unlocalized state
        """
        self.current_pose = None
        self.status = LocalizationStatus.NOT_LOCALIZED
        self.odom_integrator = OdometryIntegrator()
        self.marker_observations.clear()

        self.get_logger().info("Localization reset")
        self._publish_status(self.status, "Reset to NOT_LOCALIZED")

        return response

    # ========================================================================
    # Subscriber Callbacks
    # ========================================================================

    def _aruco_callback(self, msg: ArucoMarkerArray) -> None:
        """
        Process ArUco marker detections

        Args:
            msg: ArucoMarkerArray message

        Stores observations and optionally corrects pose if localized
        """
        current_time = time.time()

        # Store each marker observation
        for marker in msg.markers:
            if marker.marker_id in self.marker_positions:
                obs = MarkerObservation(
                    marker.marker_id,
                    marker.bearing_rad,
                    marker.distance_m
                )
                self.marker_observations[marker.marker_id] = (obs, current_time)

        # If localized and correction enabled, apply drift correction
        if self.current_pose and self.get_parameter('pose_correction_enabled').value:
            self._correct_pose_from_markers()

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Process odometry for pose integration

        Args:
            msg: Odometry message

        Updates pose estimate if localized
        """
        if self.current_pose is None:
            # Not localized yet, just initialize integrator
            odom_theta = self._quaternion_to_yaw(msg.pose.pose.orientation)
            self.odom_integrator.update(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                odom_theta
            )
            return

        # Get delta motion from odometry
        odom_theta = self._quaternion_to_yaw(msg.pose.pose.orientation)
        delta_x, delta_y, delta_theta = self.odom_integrator.update(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            odom_theta
        )

        # Update pose using geometry utilities
        self.current_pose = LocalizationGeometry.transform_pose_by_odometry_delta(
            self.current_pose,
            delta_x, delta_y, delta_theta
        )

        # Update status to TRACKING
        if self.status == LocalizationStatus.LOCALIZED:
            self.status = LocalizationStatus.TRACKING

    # ========================================================================
    # Localization - Triangulation
    # ========================================================================

    def _localize_from_markers(self, observations: Dict[int, MarkerObservation]) -> Optional[ArenaPose]:
        """
        Compute pose from marker observations using geometry utilities

        Args:
            observations: Dict of marker_id -> observation

        Returns:
            Estimated ArenaPose or None if insufficient data

        Requires at least 2 markers for triangulation
        """
        if len(observations) < 2:
            return None

        # Build lists for geometry function
        obs_list = []
        pos_list = []

        for marker_id, observation in observations.items():
            if marker_id in self.marker_positions:
                obs_list.append(observation)
                pos_list.append(self.marker_positions[marker_id])

        if len(obs_list) < 2:
            return None

        # Use geometry utilities to triangulate position
        robot_pos = LocalizationGeometry.triangulate_position_from_multiple_markers(
            obs_list, pos_list
        )

        if robot_pos is None:
            return None

        # Compute heading from first marker observation
        heading = LocalizationGeometry.compute_heading_from_marker_bearing(
            robot_pos,
            obs_list[0],
            pos_list[0]
        )

        return ArenaPose(robot_pos[0], robot_pos[1], heading)

    def _correct_pose_from_markers(self) -> None:
        """
        Apply drift correction using recent marker observations
        """
        if not self.current_pose:
            return

        recent_obs = self._get_recent_observations()
        if len(recent_obs) < 1:
            return

        # Simple correction: re-localize if we have enough markers
        if len(recent_obs) >= 2:
            corrected_pose = self._localize_from_markers(recent_obs)
            if corrected_pose:
                # Blend old and new pose (simple weighted average)
                alpha = 0.3  # Weight for new observation
                self.current_pose.x = (1 - alpha) * self.current_pose.x + alpha * corrected_pose.x
                self.current_pose.y = (1 - alpha) * self.current_pose.y + alpha * corrected_pose.y
                self.current_pose.theta = (1 - alpha) * self.current_pose.theta + alpha * corrected_pose.theta
                self.current_pose.theta = LocalizationGeometry.normalize_angle(self.current_pose.theta)

    # ========================================================================
    # Observation Management
    # ========================================================================

    def _get_recent_observations(self, max_age_sec: float = 1.0) -> Dict[int, MarkerObservation]:
        """
        Get recent marker observations

        Args:
            max_age_sec: Maximum age of observations to return

        Returns:
            Dict of marker_id -> most recent observation
        """
        current_time = time.time()
        recent = {}

        for marker_id, (obs, timestamp) in self.marker_observations.items():
            if (current_time - timestamp) <= max_age_sec:
                recent[marker_id] = obs

        return recent

    # ========================================================================
    # Publishing
    # ========================================================================

    def _publish_pose(self) -> None:
        """
        Publish current pose estimate

        Called periodically by timer
        """
        if self.current_pose is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arena'

        msg.pose.position.x = self.current_pose.x
        msg.pose.position.y = self.current_pose.y
        msg.pose.position.z = 0.0

        msg.pose.orientation = self._yaw_to_quaternion(self.current_pose.theta)

        self.pose_pub.publish(msg)

    def _publish_status(self, status: str, message: str = "") -> None:
        """
        Publish localization status

        Args:
            status: Status string (NOT_LOCALIZED, LOCALIZED, TRACKING)
            message: Additional status message
        """
        msg = String()
        msg.data = f"{status}: {message}" if message else status
        self.status_pub.publish(msg)

    # ========================================================================
    # Utility Functions
    # ========================================================================

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """
        Extract yaw from quaternion

        Args:
            q: Quaternion

        Returns:
            Yaw angle in radians
        """
        # Convert quaternion to yaw using standard formula
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """
        Convert yaw to quaternion

        Args:
            yaw: Yaw angle in radians

        Returns:
            Quaternion
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None) -> None:
    """
    Main entry point for localizer node

    Args:
        args: Command-line arguments
    """
    rclpy.init(args=args)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
