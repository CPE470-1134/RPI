#!/usr/bin/env python3
"""
Safety Monitor Node

This node monitors LiDAR data for potential collisions and can override velocity
commands to prevent the robot from hitting obstacles. It acts as a safety layer
between the driver node and the robot base.
"""


import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleZone:
    def __init__(self):
        self.front_clear = True
        self.rear_clear = True
        self.left_clear = True
        self.right_clear = True
        self.min_front_distance = float("inf")
        self.min_rear_distance = float("inf")
        self.min_left_distance = float("inf")
        self.min_right_distance = float("inf")


class SafetyNode(Node):
    """
    ROS 2 node for collision avoidance using LiDAR data
    """

    def __init__(self) -> None:
        super().__init__("safety_node")

        self._declare_parameters()

        # Internal state
        self.current_obstacles = ObstacleZone()

        self._setup_subscribers()
        self._setup_publishers()

        self.get_logger().info("Safety node initialized")

    def _declare_parameters(self) -> None:
        self.declare_parameter("front_obstacle_threshold_m", 0.25)
        self.declare_parameter("rear_obstacle_threshold_m", 0.20)
        self.declare_parameter("front_sector_angle_deg", 60.0)
        self.declare_parameter("enable_override", True)

    def _setup_subscribers(self) -> None:
        self.create_subscription(LaserScan, "/lidar/scan", self._lidar_callback, 10)
        self.create_subscription(
            Twist, "/driver/cmd_vel", self._driver_cmd_callback, 10
        )

    def _setup_publishers(self) -> None:
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(Bool, "/safety/status", 10)
        self.dist_pub = self.create_publisher(Float32, "/safety/obstacle_distance", 10)

    def _lidar_callback(self, msg: LaserScan) -> None:
        self.current_obstacles = self._analyze_scan(msg)

        # Publish nearest distance
        min_dist = min(
            self.current_obstacles.min_front_distance,
            self.current_obstacles.min_rear_distance,
            self.current_obstacles.min_left_distance,
            self.current_obstacles.min_right_distance,
        )
        self.dist_pub.publish(Float32(data=min_dist))

    def _driver_cmd_callback(self, cmd: Twist) -> None:
        if not self.get_parameter("enable_override").value:
            self._publish_safe_command(cmd)
            self._publish_status(False)
            return

        safe_cmd = self._compute_safe_velocity(cmd, self.current_obstacles)

        # Check if modified
        is_modified = (safe_cmd.linear.x != cmd.linear.x) or (
            safe_cmd.angular.z != cmd.angular.z
        )

        self._publish_safe_command(safe_cmd)
        self._publish_status(is_modified)

        if is_modified and (cmd.linear.x != 0 or cmd.angular.z != 0):
            self.get_logger().warn("Safety override active! Stopping unsafe motion.")

    def _analyze_scan(self, scan: LaserScan) -> ObstacleZone:
        zone = ObstacleZone()

        # Scan parameters
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = scan.ranges

        # Define sectors (assuming 0 is Forward, CCW positive)
        # Front: [-30, 30]
        # Left: [30, 150]
        # Rear: [150, 210] -> [150, 180] U [-180, -150]
        # Right: [-150, -30]

        half_front = math.radians(
            self.get_parameter("front_sector_angle_deg").value / 2.0
        )

        for i, r in enumerate(ranges):
            if r < scan.range_min or r > scan.range_max:
                continue

            angle = angle_min + i * angle_inc
            # Normalize to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Check sectors
            if -half_front <= angle <= half_front:
                zone.min_front_distance = min(zone.min_front_distance, r)
            elif half_front < angle < (math.pi - half_front):  # Leftish
                zone.min_left_distance = min(zone.min_left_distance, r)
            elif (math.pi - half_front) <= abs(angle):  # Rear
                zone.min_rear_distance = min(zone.min_rear_distance, r)
            elif -(math.pi - half_front) < angle < -half_front:  # Rightish
                zone.min_right_distance = min(zone.min_right_distance, r)

        # Determine clearance
        front_thresh = self.get_parameter("front_obstacle_threshold_m").value
        rear_thresh = self.get_parameter("rear_obstacle_threshold_m").value

        zone.front_clear = zone.min_front_distance > front_thresh
        zone.rear_clear = zone.min_rear_distance > rear_thresh
        # Relax side checks for rotation, mostly care about front/rear for drive

        return zone

    def _compute_safe_velocity(self, cmd: Twist, obstacles: ObstacleZone) -> Twist:
        safe_cmd = Twist()
        safe_cmd.linear.x = cmd.linear.x
        safe_cmd.angular.z = cmd.angular.z

        # Safety Logic
        if cmd.linear.x > 0 and not obstacles.front_clear:
            safe_cmd.linear.x = 0.0

        if cmd.linear.x < 0 and not obstacles.rear_clear:
            safe_cmd.linear.x = 0.0

        return safe_cmd

    def _publish_safe_command(self, cmd: Twist) -> None:
        self.cmd_vel_pub.publish(cmd)

    def _publish_status(self, override_active: bool) -> None:
        self.status_pub.publish(Bool(data=override_active))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
