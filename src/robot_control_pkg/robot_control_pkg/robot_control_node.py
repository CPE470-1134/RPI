
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class ArucoAlignAndApproach(Node):
    """
    Controller node for Questions 2b and 2c.

    Assumptions:
    - Another node (from Question 2a) publishes the alignment error α (radians)
      and distance δ (meters) as a 2‑element Float32MultiArray on the topic
      `aruco_alignment` with:
         data[0] = alpha (alignment error, rad)
         data[1] = delta (distance to marker, m)
    - The robot exposes odometry on `/odom` (nav_msgs/Odometry), which is
      used as encoder‑based feedback to measure distance travelled.
    - We command the robot using `/cmd_vel` (geometry_msgs/Twist).
    """

    def __init__(self) -> None:
        super().__init__("aruco_align_and_approach")

        # === Parameters / constants ===
        # Angular speed for alignment (Question 2b)
        self.align_angular_speed = 0.005  # rad/s (constant magnitude)
        # Alignment tolerance: 3 degrees in radians
        self.align_tolerance_rad = math.radians(3.0)

        # Linear speed for approach (Question 2c)
        self.approach_linear_speed = 0.20  # m/s
        # Desired travel distance towards marker
        self.approach_distance = 0.30  # m

        # === Publishers & subscribers ===
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.alignment_sub = self.create_subscription(
            Float32MultiArray,
            "aruco_alignment",
            self.alignment_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",    
            self.odom_callback,
            10,
        )

        # === Internal state ===
        self.current_alpha: Optional[float] = None  # latest alpha (rad)
        self.current_delta: Optional[float] = None  # latest delta (m)

        self.aligned: bool = False  # have we satisfied |alpha| < 3° ?

        # For distance tracking during 2c
        self.approach_started: bool = False
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.last_travel_distance: float = 0.0

        # Control loop timer (runs at 20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("ArucoAlignAndApproach node started (2b & 2c).")

    # === Subscribers =====================================================

    def alignment_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback for alignment feedback (Question 2a output).

        Expects:
        - msg.data[0] = alpha (rad)
        - msg.data[1] = delta (m)
        """
        if len(msg.data) < 2:
            self.get_logger().warn(
                "Received alignment message with insufficient length; "
                "expected 2 elements [alpha, delta]."
            )
            return
        
        self.get_logger().info(
            f"RECEIVED: "
            f"Alpha: {self.current_alpha} --> {msg.data[0]:.4f} rad "
            f"({math.degrees(msg.data[0]):.2f} deg), "
            f"Delta: {self.current_delta} --> {msg.data[1]:.3f} m"
        )
        self.current_alpha = float(msg.data[0])
        self.current_delta = float(msg.data[1])

    def odom_callback(self, msg: Odometry) -> None:
        """
        Odometry callback used as encoder‑based feedback for distance.

        so we use pose changes in the odometry
        frame to estimate how far the robot has traveled during 2c.
        """
        if not self.approach_started:
            # We only integrate distance once 2c has begun.
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Record starting pose the first time we get odometry after
        # approach_started becomes True.
        if self.start_x is None or self.start_y is None:
            self.start_x = x
            self.start_y = y
            self.last_travel_distance = 0.0
            return

        dx = x - self.start_x
        dy = y - self.start_y
        self.last_travel_distance = math.hypot(dx, dy)

    # === Control logic ===================================================

    def control_loop(self) -> None:
        """
        Main control loop that implements:
        - Question 2b: use α to align robot until |α| < 3°
        - Question 2c: move 0.30 m forward using encoder/odom feedback
        """
        # We require at least one alignment message before doing anything.
        if self.current_alpha is None or self.current_delta is None:
            return

        twist = Twist()

        # --- Phase 1: Alignment (Question 2b) ---------------------------
        if not self.aligned:
            alpha = self.current_alpha

            if abs(alpha) > self.align_tolerance_rad:
                # Not yet aligned: apply constant‑magnitude angular velocity
                # in the direction that reduces alpha.
                twist.angular.z = (
                    self.align_angular_speed if alpha > 0.0 else -self.align_angular_speed
                )
                twist.linear.x = 0.0

                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(
                    f"[2b] Aligning: alpha = {alpha:.4f} rad "
                    f"({math.degrees(alpha):.2f} deg), "
                    f"delta = {self.current_delta:.3f} m"
                )
                return

            # Alignment condition satisfied: stop rotation and mark aligned.
            self.aligned = True
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)

            # Print final alignment error and current delta
            self.get_logger().info(
                f"[2b] Alignment complete. "
                f"alpha = {self.current_alpha:.4f} rad "
                f"({math.degrees(self.current_alpha):.2f} deg), "
                f"delta = {self.current_delta:.3f} m"
            )

            # Prepare for phase 2 (approach).
            self.approach_started = True
            # Distance tracking will be initialized in odom_callback.
            return

        # --- Phase 2: Approach (Question 2c) ----------------------------
        if self.approach_started:
            # If we do not yet have a starting odom pose, odom_callback
            # will set it on the next odom message. For now, command
            # forward motion.
            if self.start_x is None or self.start_y is None:
                twist.linear.x = self.approach_linear_speed
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # Check how far we've traveled according to odom.
            distance = self.last_travel_distance

            if distance < self.approach_distance:
                # Keep moving forward with constant velocity 0.20 m/s.
                twist.linear.x = self.approach_linear_speed
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)

                self.get_logger().info(
                    f"[2c] Approaching: traveled = {distance:.3f} m / "
                    f"{self.approach_distance:.3f} m, "
                    f"delta = {self.current_delta:.3f} m"
                )
                return

            # Desired travel distance reached: stop and report.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            self.approach_started = False  # prevent re‑running

            self.get_logger().info(
                f"[2c] Approach complete. "
                f"distance_traveled = {distance:.3f} m, "
                f"current delta = {self.current_delta:.3f} m"
            )

    # === Shutdown helper ================================================

    def stop_robot(self) -> None:
        """Publish a zero Twist to stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None) -> None:
    """
    Entry point for the `robot_control_node` executable.

    This node implements the behavior requested in Questions 2b and 2c:
    1) Rotate to reduce the alignment error α to within 3° of zero.
    2) Then drive 0.30 m forward using odometry as encoder‑based feedback.
    """
    rclpy.init(args=args)
    node = ArucoAlignAndApproach()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

