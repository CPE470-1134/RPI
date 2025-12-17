import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from arena_interfaces.action import GoToTarget
from arena_interfaces.msg import MarkerRelative
from .controllers import PoseController, VisualServoController
import math
import time

class GoToTargetServer(Node):
    def __init__(self):
        super().__init__('go_to_target_server')
        
        # Logic Helpers
        self.pose_ctrl = PoseController()
        self.vis_ctrl = VisualServoController()
        
        # ROS Infrastructure
        self._action_server = ActionServer(
            self, GoToTarget, 'go_to_target', 
            execute_callback=self.execute_callback, # Callback function for the action server to execute the goal
            cancel_callback=self.cancel_callback
        )
        
        # Output Publisher - Velocity Command
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(MarkerRelative, '/aruco/marker_relative', self.marker_cb, 10)

        # State
        self.pose = None # [x, y, yaw]
        self.marker = None
        self.last_marker_time = 0.0

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))
        self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

    def marker_cb(self, msg):
        if msg.visible:
            self.marker = msg
            self.last_marker_time = time.time()

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = GoToTarget.Result()
        feedback = GoToTarget.Feedback()
        
        start_time = time.time()
        rate = self.create_rate(20)

        self.get_logger().info(f"Start Action Mode: {goal.mode}")

        while rclpy.ok():
            # 1. Checks
            if goal_handle.is_cancel_requested:
                self.stop(); goal_handle.canceled(); return GoToTarget.Result()
            
            if (time.time() - start_time) > goal.timeout_s:
                self.stop(); goal_handle.abort(); result.success=False; result.message="Timeout"; return result

            cmd = Twist()
            done = False

            # 2. Logic Dispatch
            if goal.mode == GoToTarget.MODE_POSE:
                done = self.run_pose_mode(goal, cmd, feedback)
            elif goal.mode == GoToTarget.MODE_MARKER:
                done = self.run_marker_mode(goal, cmd, feedback)

            # 3. Output
            self.vel_pub.publish(cmd)
            goal_handle.publish_feedback(feedback)

            if done:
                self.stop()
                goal_handle.succeed()
                result.success = True
                result.message = "Success"
                return result
            
            rate.sleep()

     # Pose Mode - Navigate to a specific pose Using Odom
    def run_pose_mode(self, goal, cmd, fb): # Pose Mode - Navigate to a specific pose
        if not self.pose: return False
        
        # Compute the target pose, distance and angle to the target pose
        target = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]
        v, w, dist, ang = self.pose_ctrl.compute(self.pose, target)
        
        # Construct the velocity command
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        
        # Update the feedback
        fb.distance_error = dist
        fb.heading_error = ang
        fb.current_state = "NAV_ODOM" # Current state of the robot
        
        return (dist < goal.pos_tol_m) # Return True if the robot has reached the target pose

    def run_marker_mode(self, goal, cmd, fb):
        is_fresh = (time.time() - self.last_marker_time) < 0.5
        
        if not is_fresh:
            cmd.angular.z = float(goal.search_omega_radps)
            fb.current_state = "SEARCHING"
            return False

        v, w, delta_err, alpha_err = self.vis_ctrl.compute(
            self.marker.alpha, self.marker.delta, goal.desired_delta_m
        )
        
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        fb.distance_error = delta_err
        fb.heading_error = alpha_err
        fb.current_state = "VISUAL_SERVO"

        return (abs(delta_err) < goal.delta_tol_m and abs(alpha_err) < goal.alpha_tol_rad)

    def stop(self):
        self.vel_pub.publish(Twist())

def main():
    rclpy.init()
    rclpy.spin(GoToTargetServer())
    rclpy.shutdown()