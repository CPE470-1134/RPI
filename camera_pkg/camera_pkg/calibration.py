import os
from pathlib import Path

import cv2 as cv
import numpy as np


def main():
    CALIBRATION_BASE_DIR = Path(__file__).resolve().parents[3] / "src" / "camera_pkg"
    CALIBRATION_MATRIX_DIR = CALIBRATION_BASE_DIR / "camera_pkg"
    IMAGES_DIR = CALIBRATION_BASE_DIR / "images"
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) .... ,(7,5,0)
    objp = np.zeros((7 * 6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane
    # images = glob.glob(os.path.join(CALIBRATION_BASE_DIR,'images/*.jpg'))

    # Find all jpg images in the images directory
    p = Path(IMAGES_DIR)

    images = list(p.glob("chessboard_[0-9].jpg"))
    print(f"Found {len(images)} images for calibration.")
    print(list(images))  # Convert generator to list
    if not images:
        return

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
            output_path = os.path.join(
                CALIBRATION_MATRIX_DIR, f"{os.path.basename(fname)}_corners.jpg"
            )
            cv.imwrite(output_path, img)
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print(f"Mtx:\n{mtx}")
    print(f"Dist:\n{dist}")
    # Save dist_coeffs and camera_matrix to .npz file
    np.savez(
        os.path.join(CALIBRATION_MATRIX_DIR, "calibration_data.npz"),
        camera_matrix=mtx,
        dist_coeffs=dist,
    )

    print(f"Camera matrix:\n{mtx}")
    # Save camera matrix for later


if __name__ == "__main__":
    main()
