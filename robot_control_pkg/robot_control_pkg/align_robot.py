#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy
)


class AlignRobot(Node):
    """
    Phase 1 only:
    - Align robot to ArUco marker until |alpha| < 3°.
    - Publish alignment information to /alignment_info topic:
         data[0] = aligned_flag (0.0 or 1.0)
         data[1] = alpha_rad
         data[2] = delta_m
    """

    def __init__(self):
        super().__init__("align_robot")

        # === Alignment config ===
        self.align_angular_speed = 0.005               # rad/s
        self.align_tolerance_rad = math.radians(3.0)  # 3 degrees in radians

        # === Internal state ===
        self.current_alpha: Optional[float] = None
        self.current_delta: Optional[float] = None
        self.aligned: bool = False

        # === Publishers ===
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.align_pub = self.create_publisher(Float32MultiArray, "/alignment_info", 10)

        # === Subscribers ===
        align_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.align_sub = self.create_subscription(
            Float32MultiArray,
            "/aruco_alignment",
            self.alignment_callback,
            align_qos
        )

        # Control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("AlignRobot (Phase 1 only) started.")

    # ----------------------------------------------------------------------
    # Subscriber Callback
    # ----------------------------------------------------------------------
    def alignment_callback(self, msg: Float32MultiArray):
        """
        Incoming data from ArucoAlignmentNode:
        msg.data = [alpha_rad, delta_m]
        """
        new_alpha = float(msg.data[0])
        new_delta = float(msg.data[1])
        
        self.current_alpha = new_alpha
        self.current_delta = new_delta
        
        #if (self.current_alpha != new_alpha) or (self.current_delta != new_delta):
            
            #self.newPoseReceived = True
            #self.current_alpha = new_alpha
            #self.current_delta = new_delta
            

    # ----------------------------------------------------------------------
    # Main Loop
    # ----------------------------------------------------------------------
    def control_loop(self):
        if self.current_alpha is None or self.current_delta is None:
            return

        twist = Twist()

        # ---------------------------------------
        # PHASE 1: ALIGNMENT ONLY
        # ---------------------------------------
        if abs(self.current_alpha) > self.align_tolerance_rad:
    
            self.aligned = False
            
            # Determine rotation direction
            
            if self.current_alpha > 0:
                direction = -1.0  # Rotate CCW
            else:
                direction = 1.0  # Rotate CW
                
            twist.angular.z = direction * self.align_angular_speed

            self.get_logger().info(
                f"[ALIGNING] α={self.current_alpha:.3f} rad "
                f"({math.degrees(self.current_alpha):.1f}°), δ={self.current_delta:.3f} m"
            )
            
        else:
            self.aligned = True
            twist.angular.z = 0.0
            #twist.linear.x = 0.0

            self.get_logger().info(
                f"[ALIGNED] α={self.current_alpha:.4f} rad "
                f"({math.degrees(self.current_alpha):.2f}°), δ={self.current_delta:.3f} m"
            )

        # Publish cmd_vel
        self.cmd_vel_pub.publish(twist)

        # Publish alignment info to be used by ApproachMarker
        msg = Float32MultiArray()
        msg.data = [
            float(self.aligned), # 0.0 or 1.0 (aligned flag)
            self.current_alpha,
            self.current_delta
        ]
        self.align_pub.publish(msg)

    # ----------------------------------------------------------------------
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AlignRobot()

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
