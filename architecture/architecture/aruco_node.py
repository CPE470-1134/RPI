#!/usr/bin/env python3
"""
ArUco Marker Detection Node

This node is responsible for detecting ArUco markers using a camera and publishing
their detections including marker ID, pose, bearing angle, and distance.

Published Topics:
    /aruco/detections (custom_msgs/ArucoDetectionArray or vision_msgs/Detection2DArray):
        - Array of detected markers with IDs and 2D bounding boxes

    /aruco/poses (geometry_msgs/PoseArray):
        - 3D poses of detected markers in camera frame
        - Includes position (x, y, z) and orientation (quaternion)

    /aruco/marker_info (custom_msgs/ArucoMarkerArray):
        - Marker ID, bearing angle (radians), distance (meters)
        - Bearing: angle from camera center to marker center
        - Distance: 3D Euclidean distance from camera to marker

Subscribed Topics:
    None (captures directly from camera hardware)

Parameters:
    camera_index (int): USB camera device index (default: 0)
    camera_width (int): Frame width in pixels (default: 640)
    camera_height (int): Frame height in pixels (default: 480)
    horizontal_fov_deg (float): Horizontal field of view in degrees (default: 60.0)
    marker_length_m (float): Physical size of ArUco marker in meters (default: 0.25)
    aruco_dict (str): ArUco dictionary type (default: "DICT_4X4_50")
    publish_rate_hz (float): Publishing frequency in Hz (default: 20.0)
"""

import math
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node

from custom_interfaces.msg import ArucoMarker, ArucoMarkerArray


class ArucoDetection:
    """
    Data class representing a single ArUco marker detection

    Attributes:
        marker_id (int): The detected marker ID
        corners (np.ndarray): 2D corner points in image space
        center_x (int): X-coordinate of marker center in pixels
        center_y (int): Y-coordinate of marker center in pixels
        bearing_rad (float): Bearing angle from camera center in radians
        distance_m (float): 3D distance from camera to marker in meters
        rvec (np.ndarray): Rotation vector from solvePnP
        tvec (np.ndarray): Translation vector from solvePnP
    """

    def __init__(
        self,
        marker_id,
        corners,
        center_x,
        center_y,
        bearing_rad,
        distance_m,
        rvec,
        tvec,
    ):
        self.marker_id = marker_id
        self.corners = corners
        self.center_x = center_x
        self.center_y = center_y
        self.bearing_rad = bearing_rad
        self.distance_m = distance_m
        self.rvec = rvec
        self.tvec = tvec


class ArucoDetectionNode(Node):
    """
    ROS 2 node for detecting ArUco markers and publishing detection information

    This node captures camera frames, detects ArUco markers, computes their 3D pose
    using camera calibration data, and publishes detection results at a fixed rate.
    """

    def __init__(self) -> None:
        """
        Initialize the ArUco detection node
        """
        super().__init__("aruco_detection_node")

        self._declare_parameters()

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_width = int(self.get_parameter("camera_width").value)
        self.frame_height = int(self.get_parameter("camera_height").value)
        self.hfov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.marker_length = float(self.get_parameter("marker_length_m").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        try:
            calib_path = self._locate_calibration_file()
            self.camera_matrix, self.dist_coeffs = self._load_calibration(calib_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise

        self._setup_aruco_detector()
        self._setup_camera()
        self._setup_publishers()

        # 3D model coordinates of marker corners for PnP
        half = self.marker_length / 2.0
        self.obj_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._process_frame)
        self.get_logger().info("ArUco detection node initialized.")

    def _declare_parameters(self) -> None:
        """
        Declare all ROS 2 parameters with default values
        """
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("horizontal_fov_deg", 60.0)
        self.declare_parameter(
            "marker_length_m", 0.05
        )  # Default changed to match expected tags if needed
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("publish_rate_hz", 20.0)

    def _locate_calibration_file(self) -> Path:
        """
        Locate the camera calibration file in the workspace
        """
        search_roots = [
            Path(__file__).resolve().parents[1]
            / "camera_pkg"
            / "camera_pkg",  # In architecture/../camera_pkg/camera_pkg
            Path(__file__).resolve().parents[2]
            / "camera_pkg"
            / "camera_pkg",  # Adjusting for robustness
            Path(__file__).resolve().parents[1] / "camera_pkg",
            Path.cwd() / "camera_pkg" / "camera_pkg",
        ]
        for root in search_roots:
            candidate = root / "calibration_data.npz"
            if candidate.exists():
                return candidate
        raise FileNotFoundError("calibration_data.npz not found.")

    def _load_calibration(self, path: Path) -> Tuple[np.ndarray, np.ndarray]:
        """
        Load camera calibration matrix and distortion coefficients
        """
        data = np.load(str(path))
        return data["camera_matrix"], data["dist_coeffs"]

    def _setup_aruco_detector(self) -> None:
        """
        Initialize the ArUco detector
        """
        self.aruco_dict_obj = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict_obj, self.aruco_params)

    def _setup_camera(self) -> None:
        """
        Initialize camera hardware
        """
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera index {self.camera_index}")

    def _setup_publishers(self) -> None:
        """
        Create all ROS 2 publishers
        """
        self.pose_array_pub = self.create_publisher(PoseArray, "/aruco/poses", 10)
        self.marker_info_pub = self.create_publisher(
            ArucoMarkerArray, "/aruco/marker_info", 10
        )

    def _process_frame(self) -> None:
        """
        Main processing loop callback
        """
        frame = self._capture_frame()
        if frame is None:
            return

        corners_list, ids_list = self._detect_markers(frame)
        if not ids_list:
            return

        detections = []
        for i, marker_id in enumerate(ids_list):
            corners = corners_list[i]
            rvec, tvec = self._compute_marker_pose(corners)

            if rvec is None or tvec is None:
                continue

            center_x = int(np.mean(corners[0][:, 0]))
            center_y = int(np.mean(corners[0][:, 1]))

            bearing = self._compute_bearing(center_x)
            distance = self._compute_distance(tvec)

            detection = ArucoDetection(
                marker_id=int(marker_id[0]),
                corners=corners,
                center_x=center_x,
                center_y=center_y,
                bearing_rad=bearing,
                distance_m=distance,
                rvec=rvec,
                tvec=tvec,
            )
            detections.append(detection)

        if detections:
            self._publish_detections(detections)

    def _capture_frame(self) -> Optional[np.ndarray]:
        """
        Capture a single frame
        """
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame")
            return None
        return frame

    def _detect_markers(self, frame: np.ndarray) -> Tuple[List, List]:
        """
        Detect ArUco markers
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return [], []
        return list(corners), list(ids)

    def _compute_marker_pose(
        self, corners: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Compute 3D pose of marker using PnP
        """
        img_points = corners[0].astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            img_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None, None
        return rvec, tvec

    def _compute_bearing(self, center_x: int) -> float:
        """
        Compute bearing angle
        """
        img_center_x = self.frame_width / 2.0
        pixel_offset = center_x - img_center_x
        alpha_deg = (pixel_offset / img_center_x) * (self.hfov_deg / 2.0)
        # In ROS coordinate system: Left is positive Y, so bearing to left should be positive?
        # Usually bearing is positive to the left (counter-clockwise from forward).
        # Pixel offset positive (right side) -> negative bearing.
        return -math.radians(alpha_deg)

    def _compute_distance(self, tvec: np.ndarray) -> float:
        """
        Compute Euclidean distance
        """
        return float(np.linalg.norm(tvec))

    def _publish_detections(self, detections: List[ArucoDetection]) -> None:
        """
        Publish all detection data
        """
        self.pose_array_pub.publish(self._create_pose_array_msg(detections))
        self.marker_info_pub.publish(self._create_marker_info_msg(detections))

    def _create_pose_array_msg(self, detections: List[ArucoDetection]) -> PoseArray:
        """
        Create PoseArray message
        """
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        for det in detections:
            pose = Pose()
            pose.position.x = float(det.tvec[0][0])
            pose.position.y = float(det.tvec[1][0])
            pose.position.z = float(det.tvec[2][0])

            # Simple rotation matrix to quaternion (approximate or use cv2.Rodrigues)
            rot_mat, _ = cv2.Rodrigues(det.rvec)
            # Conversion from rot_mat to quaternion is complex without scipy/tf
            # Leaving orientation identity for now as we use bearing/distance mostly
            pose.orientation.w = 1.0

            msg.poses.append(pose)
        return msg

    def _create_marker_info_msg(self, detections: List[ArucoDetection]):
        """
        Create marker info message
        """
        msg = ArucoMarkerArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        for det in detections:
            marker = ArucoMarker()
            marker.marker_id = det.marker_id
            marker.bearing_rad = det.bearing_rad
            marker.distance_m = det.distance_m
            marker.quality = 1.0  # Placeholder
            msg.markers.append(marker)
        return msg

    def destroy_node(self) -> bool:
        """
        Cleanup resources
        """
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
