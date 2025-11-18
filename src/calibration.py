import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) .... ,(7,5,0)
objp = np.zeros((7*6, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane
images = glob.glob('images/*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        
        # Draw the corners and save the image
        cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        output_path = fname.replace('.jpg', '_corners.jpg')
        cv.imwrite(output_path, img)
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f"Camera matrix:\n{mtx}")
# Save camera matrix for later
np.savez('camera_matrix.npz', mtx=mtx)