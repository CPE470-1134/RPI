#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')
        
        print("Initializing ArUco Pose Estimator Node...")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # ArUco dictionary (using 4x4 dictionary to match estpose.py)
        print("Setting up ArUco detector...")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        print("ArUco detector initialized")
        
        # Camera parameters from camera_matrix.npz
        self.camera_matrix_cal = None
        self.dist_coeffs_cal = None
        self.load_camera_calibration()
        
        # Camera parameters from estpose.py
        self.camera_matrix_preset = np.array([
            [821.993, 0, 330.489],
            [0, 821.993, 248.997],
            [0, 0, 1]
        ])
        self.dist_coeffs_preset = np.array([[-0.018522, 1.03979, 0, 0, -3.3171, 0, 0, 0]])
        
        # Marker size in meters
        self.marker_size = 0.05  # 5cm
        
        # Video capture
        print("Opening camera...")
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("ERROR: Could not open camera!")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("Camera initialized successfully")
        
        # Create timer for frame processing
        self.create_timer(0.1, self.process_frame)  # 10 FPS
        
        # Statistics storage
        self.pos_diffs = []
        self.rot_diffs = []
        
        self.get_logger().info('ArUco Pose Estimator Node initialized')
    
    def load_camera_calibration(self):
        """Load camera calibration parameters from file."""
        try:
            # Load camera calibration from the npz file
            calib_data = np.load('/root/RPI/calibration_data.npz')
            self.camera_matrix_cal = calib_data['camera_matrix']
            self.dist_coeffs_cal = calib_data['dist_coeffs']
            self.get_logger().info('Camera calibration loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load camera calibration: {str(e)}')
    
    def estimate_pose(self, corners, use_preset=False):
        """Estimate pose of ArUco marker using either preset or calibrated parameters."""
        if use_preset:
            camera_matrix = self.camera_matrix_preset
            dist_coeffs = self.dist_coeffs_preset
        else:
            camera_matrix = self.camera_matrix_cal
            dist_coeffs = self.dist_coeffs_cal
        
        # Estimate pose
        rvecs, tvecs, _ = cv2.solvePnP(
            objectPoints=np.array([
                [-self.marker_size/2, self.marker_size/2, 0],
                [self.marker_size/2, self.marker_size/2, 0],
                [self.marker_size/2, -self.marker_size/2, 0],
                [-self.marker_size/2, -self.marker_size/2, 0]
            ], dtype=np.float32),
            imagePoints=corners,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs
        )
        
        return rvecs, tvecs
    
    def create_pose_msg(self, rvec, tvec, frame_id):
        """Create PoseStamped message from rotation and translation vectors."""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Convert rotation vector to quaternion
        rot_matrix, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rot_matrix)
        
        # Set position
        pose_msg.pose.position.x = float(tvec[0])
        pose_msg.pose.position.y = float(tvec[1])
        pose_msg.pose.position.z = float(tvec[2])
        
        # Set orientation
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        return pose_msg
    
    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert rotation matrix to quaternion."""
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
            qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
            qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            S = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
            qx = 0.25 * S
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            S = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
            qy = 0.25 * S
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
        else:
            S = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
            qz = 0.25 * S
            
        return np.array([qx, qy, qz, qw])
    
    def process_frame(self):
        """Process camera frame and detect ArUco markers."""
        print("\nTrying to capture frame...")
        ret, frame = self.cap.read()
        if not ret:
            print("ERROR: Failed to capture frame from camera")
            self.get_logger().error('Failed to capture frame')
            return
        print(f"Frame captured successfully. Shape: {frame.shape}")
            
        # Convert frame to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers
        print("Looking for ArUco markers...")
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is None:
            print("No ArUco markers detected in this frame")
            return
            
        print(f"Found {len(ids)} ArUco marker(s)!")
        if ids is not None:
            for i in range(len(ids)):
                # Estimate pose with calibrated matrix
                rvec_cal, tvec_cal = self.estimate_pose(corners[i], use_preset=False)
                
                # Estimate pose with preset matrix
                rvec_preset, tvec_preset = self.estimate_pose(corners[i], use_preset=True)
                
                # Calculate differences
                pos_diff = np.linalg.norm(tvec_cal - tvec_preset)
                rot_diff = np.linalg.norm(rvec_cal - rvec_preset)
                
                # Store differences for statistics
                self.pos_diffs.append(pos_diff)
                self.rot_diffs.append(rot_diff)
                
                # Print results
                print("\n" + "="*50)
                print(f"Frame Analysis Results:")
                print("="*50)
                print(f"Marker ID: {ids[i][0]}")
                print("\nPositions (x, y, z):")
                print(f"Calibrated:  ({tvec_cal[0,0]:7.3f}, {tvec_cal[1,0]:7.3f}, {tvec_cal[2,0]:7.3f}) meters")
                print(f"Preset:      ({tvec_preset[0,0]:7.3f}, {tvec_preset[1,0]:7.3f}, {tvec_preset[2,0]:7.3f}) meters")
                print(f"Difference:   {pos_diff:7.3f} meters")
                
                print("\nRotations (roll, pitch, yaw):")
                print(f"Calibrated:  ({rvec_cal[0,0]:7.3f}, {rvec_cal[1,0]:7.3f}, {rvec_cal[2,0]:7.3f}) radians")
                print(f"Preset:      ({rvec_preset[0,0]:7.3f}, {rvec_preset[1,0]:7.3f}, {rvec_preset[2,0]:7.3f}) radians")
                print(f"Difference:   {rot_diff:7.3f} radians")
                
                # Print running statistics if we have enough samples
                if len(self.pos_diffs) >= 10:
                    print("\nRunning Statistics (last 10 frames):")
                    print(f"Position Difference - Mean: {np.mean(self.pos_diffs[-10:]):.3f}m, Std: {np.std(self.pos_diffs[-10:]):.3f}m")
                    print(f"Rotation Difference - Mean: {np.mean(self.rot_diffs[-10:]):.3f}rad, Std: {np.std(self.rot_diffs[-10:]):.3f}rad")
                print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()