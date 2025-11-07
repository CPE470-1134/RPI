#!/usr/bin/env python3

import numpy as np
import cv2 as cv

def load_calibration():
    """Load camera calibration data."""
    try:
        data = np.load('camera_matrix.npz')
        return data['mtx'], data.get('dist', None)
    except:
        print("No calibration file found. Will use uncalibrated estimation.")
        return None, None

def estimate_pose(frame, aruco_dict_type=cv.aruco.DICT_5X5_250, marker_length=0.05):
    """
    Detect ArUco markers and estimate their poses using both calibrated and uncalibrated approaches.
    
    Args:
        frame: Input image
        aruco_dict_type: Type of ArUco dictionary to use
        marker_length: Size of the marker in meters
    
    Returns:
        frame: Annotated image
        calibrated_poses: List of (rvec, tvec) for calibrated estimation
        uncalibrated_poses: List of (rvec, tvec) for uncalibrated estimation
    """
    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Load ArUco dictionary
    aruco_dict = cv.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect markers
    corners, ids, rejected = detector.detectMarkers(gray)
    
    if ids is None:
        return frame, [], []

    # Draw detected markers
    frame = aruco.drawDetectedMarkers(frame, corners, ids)

    # Load calibration data
    camera_matrix, dist_coeffs = load_calibration()
    
    calibrated_poses = []
    uncalibrated_poses = []
    
    # Estimate poses with calibration
    if camera_matrix is not None:
        for corner in corners:
            rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corner, marker_length, camera_matrix, dist_coeffs)
            calibrated_poses.append((rvecs[0][0], tvecs[0][0]))
            
            # Draw axis for each marker
            cv.drawFrameAxes(frame, camera_matrix, dist_coeffs, 
                           rvecs[0][0], tvecs[0][0], marker_length)
    
    # Estimate poses without calibration (using approximate camera matrix)
    h, w = frame.shape[:2]
    uncalib_camera_matrix = np.array([[w, 0, w/2],
                                     [0, w, h/2],
                                     [0, 0, 1]], dtype=np.float32)
    
    for corner in corners:
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
            corner, marker_length, uncalib_camera_matrix, None)
        uncalibrated_poses.append((rvecs[0][0], tvecs[0][0]))
    
    return frame, calibrated_poses, uncalibrated_poses

def main():
    # Open the same camera used for calibration
    cap = cv.VideoCapture(0)  # Assuming camera index 0
    
    # Load camera matrix
    camera_data = load_calibration()
    if camera_data[0] is not None:
        print("Successfully loaded camera calibration data")
    else:
        print("Warning: No calibration data found, will use uncalibrated estimation")
    
    print("Press 'q' to quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        # Process frame
        frame, calibrated_poses, uncalibrated_poses = estimate_pose(frame)
            
        # Display comparison if markers were detected
        if calibrated_poses and uncalibrated_poses:
            for i, ((rvec_cal, tvec_cal), (rvec_uncal, tvec_uncal)) in enumerate(zip(calibrated_poses, uncalibrated_poses)):
                print(f"\nMarker {i}:")
                print("Calibrated pose:")
                print(f"Position (x,y,z): {tvec_cal}")
                print(f"Rotation (rad): {rvec_cal}")
                print("\nUncalibrated pose:")
                print(f"Position (x,y,z): {tvec_uncal}")
                print(f"Rotation (rad): {rvec_uncal}")
                print("\nDifference:")
                print(f"Position difference: {np.abs(tvec_cal - tvec_uncal)}")
                print(f"Rotation difference: {np.abs(rvec_cal - rvec_uncal)}")
        
        # Save the frame with detected markers
        cv.imwrite('detected_markers.jpg', frame)
        
        # Check for quit command
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()