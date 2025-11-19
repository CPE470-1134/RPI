import numpy as np
import cv2 as cv
import os
import time

# Disable GUI features if no display is available
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

# Create directory for calibration images if it doesn't exist
if not os.path.exists('calibration_images'):
    os.makedirs('calibration_images')

# Initialize camera
cap = cv.VideoCapture(0)  # Use 0 for default camera

if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

# Set camera resolution (optional) - using a lower resolution for better compatibility
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

# Wait for camera to initialize
time.sleep(2)

# Counter for saved images
img_counter = 0
num_images_needed = 10  # Number of images we want to capture

# Chessboard parameters
chessboard_size = (8, 6)  # Number of inner corners (width, height)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

print("Press 'c' to capture when chessboard is detected")
print("Press 'q' to quit")
print(f"Need {num_images_needed} good images for calibration")
CAPTURE_BASE_DIR = Path(__file__).resolve().parents[3] / "camera_pkg" / "camera_pkg" / "images"

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Convert to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Try to find the chessboard corners
    ret_chess, corners = cv.findChessboardCorners(gray, chessboard_size, None)
    
    # If chessboard is found
    if ret_chess:
        # Refine corner positions
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # Save image automatically when chessboard is detected
        img_name = f"chessboard_{img_counter}.jpg"
        cv.imwrite(os.path.join(CAPTURE_BASE_DIR, img_name), frame)
        print(f"Captured {img_name}")
        img_counter += 1
        # Wait a bit before next capture to move the board
        time.sleep(2)
        
        if img_counter >= num_images_needed:
            print(f"Successfully captured {num_images_needed} images!")
            break
    
    # Small delay to reduce CPU usage
    time.sleep(0.1)

# Release everything when done
cap.release()
cv.destroyAllWindows()

print("\nCapture complete!")
print(f"Captured {img_counter} images in the 'calibration_images' directory")
print("You can now run calibration.py to perform the camera calibration")