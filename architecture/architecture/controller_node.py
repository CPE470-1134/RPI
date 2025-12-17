#!/usr/bin/env python3
"""
Controller Node - Navigation Command Decomposition

This node receives navigation goals from the director and breaks them down into
simple driver commands (rotate, drive). It queries the localizer for pose information
and reports progress back to the director.
"""

import asyncio
import math
from enum import Enum
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool

from custom_interfaces.action import FollowMarker, GoToPose
from custom_interfaces.msg import ArucoMarkerArray, DriverCommand, NavigationStatus


class ControllerState(Enum):
    IDLE = 0
    ALIGNING = 1
    MOVING = 2
    FINAL_ROTATION = 3
    APPROACHING_MARKER = 4
    DONE = 5


class MarkerObservation:
    def __init__(self, marker_id, bearing_rad, distance_m):
        self.marker_id = marker_id
        self.bearing_rad = bearing_rad
        self.distance_m = distance_m


class ControllerNode(Node):
    """
    ROS 2 node for decomposing navigation goals into driver commands
    """

    def __init__(self) -> None:
        super().__init__("controller_node")

        # Declare parameters
        self._declare_parameters()

        # Current state
        self.state = ControllerState.IDLE
        self.current_pose: Optional[PoseStamped] = None

        # Marker observations {marker_id: MarkerObservation}
        self.marker_observations: Dict[int, MarkerObservation] = {}

        # Driver completion flag
        self.motion_complete = False

        # Callback group for actions
        self._action_cb_group = ReentrantCallbackGroup()

        # Setup ROS interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_action_servers()

        self.get_logger().info("Controller node initialized")

    def _declare_parameters(self) -> None:
        self.declare_parameter("default_linear_speed_mps", 0.15)
        self.declare_parameter("default_angular_speed_rps", 0.5)
        self.declare_parameter("alignment_tolerance_rad", 0.052)  # ~3 degrees
        self.declare_parameter("approach_step_m", 0.10)

    def _setup_action_servers(self) -> None:
        self._goto_action_server = ActionServer(
            self,
            GoToPose,
            "/controller/go_to_pose",
            self._execute_go_to_pose,
            callback_group=self._action_cb_group,
            goal_callback=self._go_to_pose_goal_callback,
            cancel_callback=self._go_to_pose_cancel_callback,
        )

        self._follow_action_server = ActionServer(
            self,
            FollowMarker,
            "/controller/follow_marker",
            self._execute_follow_marker,
            callback_group=self._action_cb_group,
            goal_callback=self._follow_marker_goal_callback,
            cancel_callback=self._follow_marker_cancel_callback,
        )

    def _setup_subscribers(self) -> None:
        self.create_subscription(
            PoseStamped, "/localizer/pose", self._pose_callback, 10
        )
        self.create_subscription(
            ArucoMarkerArray, "/aruco/marker_info", self._aruco_callback, 10
        )
        self.create_subscription(
            Bool, "/driver/motion_complete", self._driver_complete_callback, 10
        )

    def _setup_publishers(self) -> None:
        self.driver_cmd_pub = self.create_publisher(
            DriverCommand, "/controller/driver_command", 10
        )
        self.status_pub = self.create_publisher(
            NavigationStatus, "/controller/status", 10
        )

    # ========================================================================
    # Action Server - GoToPose
    # ========================================================================

    def _go_to_pose_goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(
            f"Received GoToPose goal: ({goal_request.target_x:.2f}, {goal_request.target_y:.2f})"
        )
        return GoalResponse.ACCEPT

    def _go_to_pose_cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("GoToPose cancel requested")
        return CancelResponse.ACCEPT

    async def _execute_go_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = GoToPose.Result()

        self.get_logger().info(
            f"Executing GoToPose to ({goal.target_x:.2f}, {goal.target_y:.2f})"
        )

        if not self.current_pose:
            self.get_logger().error("Cannot navigate: No pose estimate")
            goal_handle.abort()
            result.success = False
            result.message = "No pose estimate"
            return result

        # 1. Rotate to heading
        heading = self._compute_heading_to_target(goal.target_x, goal.target_y)
        current_yaw = self._get_current_yaw()
        delta_yaw = self._normalize_angle(heading - current_yaw)

        self.state = ControllerState.ALIGNING
        self._publish_status(self.state, 0.0, "Aligning to target")

        if abs(delta_yaw) > self.get_parameter("alignment_tolerance_rad").value:
            await self._wait_for_motion(
                DriverCommand.ROTATE,
                delta_yaw,
                self.get_parameter("default_angular_speed_rps").value,
            )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            return result

        # 2. Drive to target
        distance = self._compute_distance_to_target(goal.target_x, goal.target_y)
        self.state = ControllerState.MOVING
        self._publish_status(self.state, 0.33, "Driving to target")

        if distance > 0.01:
            await self._wait_for_motion(
                DriverCommand.DRIVE,
                distance,
                self.get_parameter("default_linear_speed_mps").value,
            )

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            return result

        # 3. Rotate to final heading (if needed/specified)

        if (
            hasattr(goal, "target_theta") and goal.target_theta is not None
        ):  # Check if field exists
            delta_yaw = self._normalize_angle(goal.target_theta - current_yaw)
            await self._wait_for_motion(
                DriverCommand.ROTATE,
                delta_yaw,
                self.get_parameter("default_angular_speed_rps").value,
            )
        else:  # No final heading specified, just stop and report done
            pass

        self.state = ControllerState.DONE
        self._publish_status(self.state, 1.0, "Reached target")

        goal_handle.succeed()
        result.success = True
        result.message = "Arrived at target"
        return result

    # ========================================================================
    # Action Server - FollowMarker
    # ========================================================================

    def _follow_marker_goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info(
            f"Received FollowMarker goal: ID {goal_request.marker_id}"
        )
        return GoalResponse.ACCEPT

    def _follow_marker_cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("FollowMarker cancel requested")
        return CancelResponse.ACCEPT

    async def _execute_follow_marker(self, goal_handle):
        goal = goal_handle.request
        result = FollowMarker.Result()
        marker_id = goal.marker_id
        stop_dist = goal.stop_distance

        self.state = ControllerState.APPROACHING_MARKER
        self.get_logger().info(
            f"Executing FollowMarker ID {marker_id}, stop at {stop_dist:.2f}m"
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                return result

            obs = self._get_marker_observation(marker_id)
            if not obs:
                # Wait for marker to be visible
                self.get_logger().warn(f"Marker {marker_id} not visible, waiting...")
                await asyncio.sleep(0.5)
                continue

            # Check if reached
            if obs.distance_m <= stop_dist + 0.05:  # Tolerance
                self.get_logger().info("Reached marker stop distance")
                break

            # Align
            if (
                abs(obs.bearing_rad)
                > self.get_parameter("alignment_tolerance_rad").value
            ):
                await self._wait_for_motion(
                    DriverCommand.ROTATE,
                    obs.bearing_rad,
                    self.get_parameter("default_angular_speed_rps").value,
                )
                await asyncio.sleep(0.5)  # Re-acquire visual
                continue

            # Approach
            step = min(
                self.get_parameter("approach_step_m").value, obs.distance_m - stop_dist
            )
            if step > 0.01:
                await self._wait_for_motion(
                    DriverCommand.DRIVE,
                    step,
                    self.get_parameter("default_linear_speed_mps").value,
                )
                await asyncio.sleep(0.5)  # Re-acquire visual
            else:
                break

        self.state = ControllerState.DONE
        goal_handle.succeed()
        result.success = True
        return result

    # ========================================================================
    # Helpers
    # ========================================================================

    async def _wait_for_motion(self, cmd_type, value, speed):
        """Send command and wait for completion"""
        self.motion_complete = False

        msg = DriverCommand()
        msg.command_type = cmd_type
        msg.target_value = float(value)
        msg.speed = float(speed)
        self.driver_cmd_pub.publish(msg)

        while not self.motion_complete and rclpy.ok():
            await asyncio.sleep(0.1)

    def _compute_heading_to_target(self, target_x: float, target_y: float) -> float:
        if not self.current_pose:
            return 0.0
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        return math.atan2(target_y - current_y, target_x - current_x)

    def _compute_distance_to_target(self, target_x: float, target_y: float) -> float:
        if not self.current_pose:
            return 0.0
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        dx = target_x - current_x
        dy = target_y - current_y
        return math.sqrt(dx**2 + dy**2)

    def _get_current_yaw(self) -> float:
        if self.current_pose:
            return self._quaternion_to_yaw(self.current_pose.pose.orientation)
        return 0.0

    def _get_marker_observation(self, marker_id: int) -> Optional[MarkerObservation]:
        return self.marker_observations.get(marker_id)

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def _aruco_callback(self, msg: ArucoMarkerArray) -> None:
        for marker in msg.markers:
            self.marker_observations[marker.marker_id] = MarkerObservation(
                marker.marker_id, marker.bearing_rad, marker.distance_m
            )

    def _driver_complete_callback(self, msg: Bool) -> None:
        if msg.data:
            self.motion_complete = True

    def _publish_status(self, state, progress, message):
        msg = NavigationStatus()
        msg.phase = state.value
        msg.progress = float(progress)
        msg.message = message
        self.status_pub.publish(msg)

    def _normalize_angle(self, angle_rad: float) -> float:
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
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
