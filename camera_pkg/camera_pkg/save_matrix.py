import numpy as np

# Camera matrix from estpose.py (preset matrix)
camera_matrix = np.array([
    [821.993, 0, 330.489],
    [0, 821.993, 248.997],
    [0, 0, 1]
])

dist_coeffs = np.array([[-0.018522, 1.03979, 0, 0, -3.3171, 0, 0, 0]])

# Save both matrices
np.savez(Path(__file__).resolve().parents[3] / "camera_pkg" / "camera_pkg" / "camera_matrix.npz", 
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs)