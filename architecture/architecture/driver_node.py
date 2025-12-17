#!/usr/bin/env python3
"""
Driver Node - Low-Level Motor Control

This node executes simple motor control primitives. It receives commands from the
controller node to rotate by an angle or drive a distance, and reports when the
motion is complete. It does NOT track odometry or make navigation decisions.
"""

import math
from enum import Enum
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from custom_interfaces.msg import DriverCommand as DriverCommandMsg


class DriverState(Enum):
    IDLE = 0
    ROTATING = 1
    DRIVING = 2
    STOPPING = 3


class MotionTracker:
    def __init__(self):
        self.start_position_x = 0.0
        self.start_position_y = 0.0
        self.start_heading = 0.0
        self.target_heading = 0.0
        self.target_distance = 0.0
        self.distance_traveled = 0.0
        self.rotation_completed = 0.0


class DriverNode(Node):
    """
    ROS 2 node for low-level motor control
    """

    def __init__(self) -> None:
        super().__init__("driver_node")

        self._declare_parameters()

        self.state = DriverState.IDLE
        self.current_cmd: Optional[DriverCommandMsg] = None
        self.tracker = MotionTracker()

        # Current odometry state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        self._setup_subscribers()
        self._setup_publishers()

        # 20Hz control loop
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info("Driver node initialized")

    def _declare_parameters(self) -> None:
        self.declare_parameter("default_linear_speed_mps", 0.15)
        self.declare_parameter("default_angular_speed_rps", 0.5)
        self.declare_parameter("rotation_tolerance_rad", 0.02)
        self.declare_parameter("distance_tolerance_m", 0.01)

    def _setup_subscribers(self) -> None:
        self.create_subscription(
            DriverCommandMsg, "/controller/driver_command", self._command_callback, 10
        )
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

    def _setup_publishers(self) -> None:
        self.vel_pub = self.create_publisher(Twist, "/driver/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/driver/status", 10)
        self.complete_pub = self.create_publisher(Bool, "/driver/motion_complete", 10)

    def _command_callback(self, msg: DriverCommandMsg) -> None:
        self.get_logger().info(
            f"Received command: Type={msg.command_type}, Val={msg.target_value:.2f}"
        )

        if msg.command_type == DriverCommandMsg.STOP:
            self._execute_stop_command()
        elif msg.command_type == DriverCommandMsg.ROTATE:
            self._execute_rotate_command(msg.target_value, msg.speed)
        elif msg.command_type == DriverCommandMsg.DRIVE:
            self._execute_drive_command(msg.target_value, msg.speed)

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self._quaternion_to_yaw(msg.pose.pose.orientation)
        self.odom_received = True

        # Update trackers
        if self.state == DriverState.DRIVING:
            dx = self.current_x - self.tracker.start_position_x
            dy = self.current_y - self.tracker.start_position_y
            self.tracker.distance_traveled = math.sqrt(dx**2 + dy**2)

        elif self.state == DriverState.ROTATING:
            diff = self.current_theta - self.tracker.start_heading
            self.tracker.rotation_completed = self._normalize_angle(diff)

    def _execute_stop_command(self) -> None:
        self.state = DriverState.STOPPING
        self._stop_robot()
        self.state = DriverState.IDLE

    def _execute_rotate_command(
        self, target_angle_rad: float, angular_speed: float
    ) -> None:
        if not self.odom_received:
            self.get_logger().warn("Cannot rotate: No odometry received")
            return

        self._reset_motion_tracker()
        # Interpretation: target_angle_rad is RELATIVE rotation
        self.tracker.target_heading = target_angle_rad
        # Actually, for rotation, we track delta.
        # So we just need to rotate until accumulated rotation equals target.

        self.current_cmd = DriverCommandMsg()
        self.current_cmd.command_type = DriverCommandMsg.ROTATE
        self.current_cmd.target_value = target_angle_rad
        self.current_cmd.speed = angular_speed

        self.state = DriverState.ROTATING

    def _execute_drive_command(self, distance_m: float, linear_speed: float) -> None:
        if not self.odom_received:
            self.get_logger().warn("Cannot drive: No odometry received")
            return

        self._reset_motion_tracker()
        self.tracker.target_distance = distance_m

        self.current_cmd = DriverCommandMsg()
        self.current_cmd.command_type = DriverCommandMsg.DRIVE
        self.current_cmd.target_value = distance_m
        self.current_cmd.speed = linear_speed

        self.state = DriverState.DRIVING

    def _control_loop(self) -> None:
        if self.state == DriverState.IDLE:
            self._publish_state(self.state)
            return

        if self.state == DriverState.ROTATING:
            self._update_rotation()
        elif self.state == DriverState.DRIVING:
            self._update_drive()

        self._publish_state(self.state)

    def _update_rotation(self) -> None:
        # Check completion
        # We rotate until the relative change matches the target
        remaining = abs(self.tracker.target_heading) - abs(
            self.tracker.rotation_completed
        )

        if remaining <= self.get_parameter("rotation_tolerance_rad").value:
            self._complete_motion()
            return

        # Publish velocity
        speed = abs(self.current_cmd.speed)
        # Determine direction
        direction = 1.0 if self.tracker.target_heading > 0 else -1.0

        # Simple P-control for slowdown? Or just bang-bang with ramp down?
        # Let's use constant speed for simplicity as requested by architecture doc ("simple driver commands")
        # But maybe slow down at end
        if remaining < 0.2:
            speed = max(0.1, speed * 0.5)

        self._publish_velocity(0.0, direction * speed)

    def _update_drive(self) -> None:
        remaining = self.tracker.target_distance - self.tracker.distance_traveled

        if remaining <= self.get_parameter("distance_tolerance_m").value:
            self._complete_motion()
            return

        speed = abs(self.current_cmd.speed)
        if remaining < 0.05:
            speed = max(0.05, speed * 0.5)

        self._publish_velocity(speed, 0.0)

    def _reset_motion_tracker(self) -> None:
        self.tracker.start_position_x = self.current_x
        self.tracker.start_position_y = self.current_y
        self.tracker.start_heading = self.current_theta
        self.tracker.distance_traveled = 0.0
        self.tracker.rotation_completed = 0.0

    def _complete_motion(self) -> None:
        self._stop_robot()
        self.state = DriverState.IDLE
        self._publish_completion()

    def _publish_completion(self) -> None:
        msg = Bool()
        msg.data = True
        self.complete_pub.publish(msg)

    def _publish_velocity(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.vel_pub.publish(msg)

    def _stop_robot(self) -> None:
        self._publish_velocity(0.0, 0.0)

    def _publish_state(self, state: DriverState) -> None:
        msg = String()
        msg.data = state.name
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
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
