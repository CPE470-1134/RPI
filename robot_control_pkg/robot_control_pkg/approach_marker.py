
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ApproachMarker(Node):
    """
    Phase 2 only:
      - Listens for /alignment_info from AlignRobot
      - When aligned_flag == 1.0, drive forward 0.30 m using odometry
      - Publish constant linear velocity of 0.20 m/s
      - Stop and print distance + δ
    """

    def __init__(self):
        super().__init__("approach_marker")

        # === Parameters ===
        self.target_distance = 0.30   # 30 cm
        self.forward_speed = 0.20     # m/s

        # === Internal state ===
        self.aligned_flag = 0.0
        self.delta = None
        self.alpha = None

        self.start_x = None
        self.start_y = None
        self.traveled = 0.0

        self.approach_started = False
        self.approach_completed = False

        # === Publishers ===
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # === QoS ===
        qos_alignment = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # === Subscribers ===
        self.align_sub = self.create_subscription(
            Float32MultiArray,
            "/alignment_info",
            self.alignment_callback,
            qos_alignment
            
        )
        
        # QOS for odom, BEST_EFFORT
        qos_odom = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        
        #  SUBSCRIBER for /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_odom
        )

        # Timer
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("ApproachMarker node started (Phase 2 only).")

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def alignment_callback(self, msg: Float32MultiArray):
        """
        msg.data = [aligned_flag, alpha_rad, delta_m]
        """
        self.aligned_flag = msg.data[0]
        self.alpha = msg.data[1]
        self.delta = msg.data[2]

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            return

        self.traveled = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)

    # ----------------------------------------------------------------------
    # Main Control Logic
    # ----------------------------------------------------------------------
    def control_loop(self):
        if self.approach_completed:
            return  # done

        # ---------------------------------------------------------
        # Wait for alignment flag from AlignRobot
        # ---------------------------------------------------------
        if self.aligned_flag != 1.0:
            self.get_logger().info("[WAITING] Still aligning...",throttle_duration_sec=5.0)
            return  # still aligning

        # ---------------------------------------------------------
        # Start approach
        # ---------------------------------------------------------
        if not self.approach_started:
            self.start_x = None
            self.start_y = None
            #self.traveled = 0.0

            self.approach_started = True
            self.get_logger().info("[APPROACH] Alignment confirmed, starting forward motion.")

        twist = Twist()

        # ---------------------------------------------------------
        # Drive forward until target distance
        # ---------------------------------------------------------
        if self.traveled < self.target_distance:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            # Publish velocity
            self.cmd_pub.publish(twist)
            
            self.get_logger().info(
                f"[MOVING] Traveled = {self.traveled:.3f} m / {self.target_distance:.2f} m "
                f"(delta = {self.delta:.3f} m)"
            )

        else:
            # Arrived at target distance
            self.approach_completed = True
            self.get_logger().info("[STOPPED] Target distance reached.")
            
            # Ensure robot is stopped, does not populate cmd_vel buffer
            if not self.final_state_sent:
                self.stop_robot()
                self.final_state_sent = True
                
            return
        

    # ----------------------------------------------------------------------
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Stopping robot. Final cmd_vel published.")
        self.cmd_pub.publish(twist)
  


def main(args=None):
    rclpy.init(args=args)
    node = ApproachMarker()

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
