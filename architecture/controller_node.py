#!/usr/bin/env python3
"""
Controller Node - Navigation Command Decomposition

This node receives navigation goals from the director and breaks them down into
simple driver commands (rotate, drive). It queries the localizer for pose information
and reports progress back to the director.

Published Topics:
    /controller/driver_command (custom_msgs/DriverCommand):
        - Simple commands to driver: ROTATE, DRIVE, STOP
        - Includes target angle/distance and speed

    /controller/status (custom_msgs/NavigationStatus):
        - Current phase (IDLE, ALIGNING, MOVING, DONE)
        - Progress percentage, distance remaining

Subscribed Topics:
    /localizer/pose (geometry_msgs/PoseStamped):
        - Robot pose in arena frame from localizer
        - Used to compute navigation goals and track progress

    /aruco/marker_info (custom_msgs/ArucoMarkerArray):
        - Detected markers with ID, bearing, and distance
        - Used ONLY for visual servoing (FollowMarker action)

    /driver/motion_complete (std_msgs/Bool):
        - Notification from driver when commanded motion finishes
        - Triggers next phase of navigation

Action Servers:
    /controller/go_to_pose (custom_actions/GoToPose):
        - Navigate to target pose relative to current position
        - Takes target (x, y, theta) from director (in whatever frame director uses)
        - Breaks into: rotate to heading → drive distance → rotate to final angle
        - Reports progress and completion

    /controller/follow_marker (custom_actions/FollowMarker):
        - Visual servoing to approach specific ArUco marker
        - Uses marker bearing/distance from aruco node
        - Iteratively: align to marker → approach → repeat until at distance
        - Reports marker visibility and alignment status

Parameters:
    default_linear_speed_mps (float): Default drive speed (default: 0.15)
    default_angular_speed_rps (float): Default rotation speed (default: 0.3)
    alignment_tolerance_rad (float): Marker alignment tolerance (default: 0.052)
    approach_step_m (float): Distance to approach per iteration (default: 0.10)
"""

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from std_msgs.msg import Bool


class ControllerState(Enum):
    """
    Enumeration of controller states
    """
    IDLE = 0
    ALIGNING = 1
    MOVING = 2
    FINAL_ROTATION = 3
    APPROACHING_MARKER = 4
    DONE = 5


class NavigationPhase(Enum):
    """
    Phases for GoToPose navigation
    """
    ROTATE_TO_HEADING = 0
    DRIVE_TO_TARGET = 1
    ROTATE_TO_FINAL = 2
    COMPLETE = 3


class MarkerObservation:
    """
    Data class for ArUco marker observation

    Attributes:
        marker_id (int): Marker ID
        bearing_rad (float): Bearing angle to marker
        distance_m (float): Distance to marker
    """
    def __init__(self):
        pass


class PoseSnapshot:
    """
    Stores a snapshot of robot pose at start of navigation command

    Attributes:
        x (float): Starting x from localizer
        y (float): Starting y from localizer
        theta (float): Starting heading from localizer
    """
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta


class ControllerNode(Node):
    """
    ROS 2 node for decomposing navigation goals into driver commands

    This node takes action goals (GoToPose, FollowMarker) and breaks them
    into simple commands for the driver (rotate X, drive Y). Uses localizer
    for pose information.
    """

    def __init__(self) -> None:
        """
        Initialize the controller node

        Sets up:
        - ROS 2 parameters for speeds and tolerances
        - Action servers for navigation behaviors
        - Subscribers for pose and marker data
        - Publishers for driver commands and status
        """
        super().__init__("controller_node")

        # Declare parameters
        self._declare_parameters()

        # Current state
        self.state = ControllerState.IDLE
        self.current_pose: Optional[PoseStamped] = None
        self.start_pose: Optional[PoseSnapshot] = None

        # Marker observations for visual servoing
        self.marker_observations: dict = {}

        # Driver completion flag
        self.motion_complete = False

        # Setup ROS interfaces
        self._setup_action_servers()
        self._setup_subscribers()
        self._setup_publishers()

        self.get_logger().info("Controller node initialized")

    def _declare_parameters(self) -> None:
        """
        Declare ROS 2 parameters

        Parameters for speeds, tolerances, and approach distances
        """
        self.declare_parameter('default_linear_speed_mps', 0.15)
        self.declare_parameter('default_angular_speed_rps', 0.3)
        self.declare_parameter('alignment_tolerance_rad', 0.052)  # ~3 degrees
        self.declare_parameter('approach_step_m', 0.10)

    def _setup_action_servers(self) -> None:
        """
        Create action servers

        Action Servers:
        - /controller/go_to_pose: Navigate to relative target pose
        - /controller/follow_marker: Visual servoing to marker
        """
        pass

    def _setup_subscribers(self) -> None:
        """
        Create subscribers

        Subscribers:
        - /localizer/pose: Robot pose from localizer
        - /aruco/marker_info: Marker detections for visual servoing
        - /driver/motion_complete: Driver completion notifications
        """
        self.create_subscription(
            PoseStamped,
            '/localizer/pose',
            self._pose_callback,
            10
        )
        self.create_subscription(
            Bool,  # Will be ArucoMarkerArray when defined
            '/aruco/marker_info',
            self._aruco_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/driver/motion_complete',
            self._driver_complete_callback,
            10
        )

    def _setup_publishers(self) -> None:
        """
        Create publishers

        Publishers:
        - /controller/driver_command: Commands to driver
        - /controller/status: Current state and progress
        """
        self.driver_cmd_pub = self.create_publisher(
            Twist,  # Will be DriverCommand when defined
            '/controller/driver_command',
            10
        )
        self.status_pub = self.create_publisher(
            Twist,  # Will be NavigationStatus when defined
            '/controller/status',
            10
        )

    # ========================================================================
    # Action Server - GoToPose
    # ========================================================================

    def _go_to_pose_goal_callback(self, goal_request) -> GoalResponse:
        """
        Handle GoToPose goal request

        Args:
            goal_request: Target pose from director

        Returns:
            GoalResponse.ACCEPT or REJECT
        """
        pass

    def _go_to_pose_cancel_callback(self, goal_handle) -> CancelResponse:
        """
        Handle GoToPose cancellation

        Args:
            goal_handle: Goal being cancelled

        Returns:
            CancelResponse.ACCEPT or REJECT
        """
        pass

    async def _execute_go_to_pose(self, goal_handle) -> object:
        """
        Execute GoToPose - break into rotate/drive/rotate phases

        Args:
            goal_handle: Goal with target (x, y, theta)

        Returns:
            Result with success status

        Steps:
        1. Compute heading angle to target (x, y)
        2. Send ROTATE command to driver
        3. Wait for completion
        4. Compute distance to target
        5. Send DRIVE command to driver
        6. Wait for completion
        7. Send ROTATE to final theta (if specified)
        8. Return result
        """
        pass

    # ========================================================================
    # Action Server - FollowMarker
    # ========================================================================

    def _follow_marker_goal_callback(self, goal_request) -> GoalResponse:
        """
        Handle FollowMarker goal request

        Args:
            goal_request: Target marker ID

        Returns:
            GoalResponse.ACCEPT or REJECT
        """
        pass

    def _follow_marker_cancel_callback(self, goal_handle) -> CancelResponse:
        """
        Handle FollowMarker cancellation

        Args:
            goal_handle: Goal being cancelled

        Returns:
            CancelResponse.ACCEPT or REJECT
        """
        pass

    async def _execute_follow_marker(self, goal_handle) -> object:
        """
        Execute FollowMarker - iterative align and approach

        Args:
            goal_handle: Goal with marker ID and stop distance

        Returns:
            Result with success status

        Loop:
        1. Wait for marker to be visible
        2. Get marker bearing and distance
        3. If not aligned: send ROTATE to align
        4. If aligned but not at distance: send DRIVE to approach
        5. Repeat until at target distance
        6. Return result
        """
        pass

    # ========================================================================
    # Subscriber Callbacks
    # ========================================================================

    def _pose_callback(self, msg: PoseStamped) -> None:
        """
        Store latest pose from localizer

        Args:
            msg: PoseStamped from localizer
        """
        self.current_pose = msg

    def _aruco_callback(self, msg) -> None:
        """
        Store marker observations for visual servoing

        Args:
            msg: ArucoMarkerArray message
        """
        # TODO: Parse actual ArucoMarkerArray when defined
        # Store marker observations {marker_id: (bearing, distance)}
        pass

    def _driver_complete_callback(self, msg: Bool) -> None:
        """
        Handle driver completion notification

        Args:
            msg: Completion flag
        """
        if msg.data:
            self.motion_complete = True

    # ========================================================================
    # GoToPose Logic
    # ========================================================================

    def _compute_heading_to_target(self, target_x: float, target_y: float) -> float:
        """
        Compute heading angle from current position to target

        Args:
            target_x: Target x in arena frame
            target_y: Target y in arena frame

        Returns:
            Heading angle in radians (arena frame)
        """
        if not self.current_pose:
            return 0.0

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        return math.atan2(target_y - current_y, target_x - current_x)

    def _compute_distance_to_target(self, target_x: float, target_y: float) -> float:
        """
        Compute distance to target

        Args:
            target_x: Target x in arena frame
            target_y: Target y in arena frame

        Returns:
            Distance in meters
        """
        if not self.current_pose:
            return 0.0

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y

        return math.sqrt(dx**2 + dy**2)

    def _send_rotate_command(self, angle_rad: float, speed: float) -> None:
        """
        Send rotate command to driver

        Args:
            angle_rad: Angle to rotate
            speed: Angular speed
        """
        pass

    def _send_drive_command(self, distance_m: float, speed: float) -> None:
        """
        Send drive command to driver

        Args:
            distance_m: Distance to drive
            speed: Linear speed
        """
        pass

    # ========================================================================
    # FollowMarker Logic
    # ========================================================================

    def _get_marker_observation(self, marker_id: int) -> Optional[MarkerObservation]:
        """
        Get latest observation of marker

        Args:
            marker_id: Target marker ID

        Returns:
            MarkerObservation or None if not visible
        """
        pass

    def _is_aligned_to_marker(self, bearing_rad: float) -> bool:
        """
        Check if aligned with marker

        Args:
            bearing_rad: Bearing to marker

        Returns:
            True if within alignment tolerance
        """
        pass

    def _is_at_target_distance(self, current_distance: float, target_distance: float) -> bool:
        """
        Check if at target distance from marker

        Args:
            current_distance: Current distance to marker
            target_distance: Desired distance

        Returns:
            True if close enough
        """
        pass

    def _compute_approach_distance(self, current_distance: float, target_distance: float) -> float:
        """
        Compute how far to drive toward marker

        Args:
            current_distance: Current distance
            target_distance: Target distance

        Returns:
            Distance to drive (limited to step size)
        """
        pass

    # ========================================================================
    # Pose Tracking
    # ========================================================================

    def _save_start_pose(self) -> None:
        """
        Save current pose as start of navigation command
        """
        if self.current_pose:
            self.start_pose = PoseSnapshot(
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self._quaternion_to_yaw(self.current_pose.pose.orientation)
            )

    def _get_current_yaw(self) -> float:
        """
        Get current heading from localizer pose

        Returns:
            Current heading in radians
        """
        if self.current_pose:
            return self._quaternion_to_yaw(self.current_pose.pose.orientation)
        return 0.0

    # ========================================================================
    # Status Publishing
    # ========================================================================

    def _publish_status(self, state: ControllerState, progress: float, message: str) -> None:
        """
        Publish controller status

        Args:
            state: Current state
            progress: Progress percentage
            message: Status message
        """
        pass

    # ========================================================================
    # Utilities
    # ========================================================================

    def _normalize_angle(self, angle_rad: float) -> float:
        """
        Normalize angle to [-π, π]

        Args:
            angle_rad: Angle

        Returns:
            Normalized angle
        """
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """
        Extract yaw from quaternion

        Args:
            q: Quaternion

        Returns:
            Yaw angle in radians
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    """
    Main entry point

    Args:
        args: Command-line arguments
    """
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

