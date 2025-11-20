import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class RotateForMarker(Node):
    def __init__(self):
        super().__init__('rotate_for_marker')

        # === Rotation config ===
        self.ROTATION_SEARCH_SPEED = 0.05              # rad/s
        
        # Consectuve detections needed to confirm marker presence
        self.REQUIRED_DETECTION_AMOUNT = 5
        self.detection_counter = 0
        
        self.prev_msg = None
        
        # SUBSCRIBERS - /cmd_vel
        self.cmd_vel_sub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # SUBSCRIBERS - /aruco_alignment
        self.aruco_sub = self.create_subscription(
            Float32MultiArray,
            '/aruco_alignment',
            self.aruco_callback,
            10
        )
        # TIMER - Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("RotateForMarker node initialized.")
        
    def aruco_callback(self, msg: Float32MultiArray):
        # Check if marker is detected (assuming msg.data[0] is alpha_rad,delta_m)
         if msg.data and len(msg.data) >= 2:
            self.detection_counter += 1

         else:
            self.detection_counter = 0
            
    def control_loop(self):
        twist_msg = Twist()
        
        if self.detection_counter >= self.REQUIRED_DETECTION_AMOUNT:
            # Marker detected consistently - stop rotation
            twist_msg.angular.z = 0.0
            self.get_logger().info("Marker detected consistently. Stopping rotation.")
            
            rclpy.shutdown()
            return
        else:
            # Rotate to search for marker
            twist_msg.angular.z = self.ROTATION_SEARCH_SPEED
            self.get_logger().info("Searching for marker by rotating.")
        
        # Publish cmd_vel message
        self.cmd_vel_sub.publish(twist_msg) 
        
# ----------------------------------------------------------------------
# Main function        
def main(args=None):
    rclpy.init(args=args)
    rotate_for_marker_node = RotateForMarker()
    rclpy.spin(rotate_for_marker_node)
    rotate_for_marker_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()