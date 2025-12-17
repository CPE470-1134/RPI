# Single ArUco marker detection with bearing and distance

# Marker ID
int32 marker_id

# Bearing angle from robot center to marker (radians)
# Positive: marker is to the right
# Negative: marker is to the left
# Zero: marker is straight ahead
float32 bearing_rad

# Distance from robot to marker (meters)
# 3D Euclidean distance from camera to marker center
float32 distance_m

# Detection quality/confidence (0.0 to 1.0)
# Based on marker visibility, pose estimation reprojection error, etc.
float32 quality
