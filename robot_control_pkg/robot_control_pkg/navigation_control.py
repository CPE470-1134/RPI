

import math

def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class OdomLocalizationController:
    """Drive to (x,y) using Odom"""
    def __init__(self):
        self.k_v = 0.5
        self.k_w = 1.8
        self.max_v = 0.22

    def compute(self, curr_pose, target_pose):
        # curr_pose = [x, y, yaw]
        dx = target_pose[0] - curr_pose[0]
        dy = target_pose[1] - curr_pose[1]
        dist_err = math.sqrt(dx**2 + dy**2)
        
        target_head = math.atan2(dy, dx)
        head_err = wrap_angle(target_head - curr_pose[2])

        w = self.k_w * head_err
        
        # Turn-then-move heuristic
        if abs(head_err) > 0.2:
            v = 0.0
        else:
            v = min(self.max_v, self.k_v * dist_err)
            
        return v, w, dist_err, head_err

class CameraLocalizationController:
    """Dock to ArUco using Camera"""
    def __init__(self):
        self.k_alpha = 1.5
        self.k_delta = 0.6
        self.max_v = 0.15

    def compute(self, alpha, delta, desired_delta):
        alpha_err = alpha
        delta_err = delta - desired_delta

        w = self.k_alpha * alpha_err
        
        # Don't drive forward if not facing tag
        if abs(alpha_err) < 0.2:
            v = self.k_delta * delta_err
            v = max(min(v, self.max_v), -self.max_v)
        else:
            v = 0.0
            
        return v, w, delta_err, alpha_err