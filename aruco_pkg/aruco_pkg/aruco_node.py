
import math
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ArucoAlignmentNode(Node):
    """
    Detect a 4x4-50 ArUco marker and publish (alpha, delta).

    - alpha: alignment error (radians) computed from marker center vs. image center.
    - delta: distance (meters) from robot/camera to the marker (Euclidean norm of tvec).
    """

    def __init__(self) -> None:
        super().__init__("aruco_alignment_node")

        # --- Parameters / constants ----------------------------------------
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("horizontal_fov_deg", 60.0)
        self.declare_parameter("marker_length_m", 0.25) # Approx Standard Paper Size
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_width = int(self.get_parameter("camera_width").value)
        self.frame_height = int(self.get_parameter("camera_height").value)
        self.hfov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.marker_length = float(self.get_parameter("marker_length_m").value)
        
        # --- Internal state -----------------------------------------------
        self.frame_num = 0
        self.hit_count = 0
        self.miss_count = 0
        
        self.calib_path = self._locate_calibration_file()
   
        # --- Load calibration ----------------------------------------------
        self.camera_matrix, self.dist_coeffs = self._load_calibration(self.calib_path)
        self.aruco_frame_save_path = Path(__file__).resolve().parents[1] / "aruco_frames"
        self.aruco_frame_save_path.mkdir(parents=True, exist_ok=True)

        # --- ArUco configuration ------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # --- Camera --------------------------------------------------------
        self.cap = cv2.VideoCapture(self.camera_index)
        cap_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        cap_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera index {}".format(self.camera_index))

        ## Camera opened successfully, Logging
        self.get_logger().info(f"Camera opened: index={self.camera_index}, "
                                 f"resolution={cap_w}x{cap_h}")
        
        # --- Publisher -----------------------------------------------------
        self.alignment_pub = self.create_publisher(
            Float32MultiArray,
            "aruco_alignment",
            10,
        )

        # --- Timer ---------------------------------------------------------
        self.timer = self.create_timer(0.05, self._process_frame)  # 20 Hz
        self.get_logger().info("ArUco alignment node started.")

    # ----------------------------------------------------------------------
    def _locate_calibration_file(self) -> Path:
        """
        Locate camera calibration produced by camera_pkg.

       Search for the camera_matrix.npz file in the following directories:
       - .../create3_ws/src/RPI/src/camera_pkg/camera_pkg
       - .../create3_ws/src/RPI/src
       - .../create3_ws/src/RPI
       If it isn't found in any of these directories, raise a FileNotFoundError.
       """
       
        search_roots = [
            Path(__file__).resolve().parents[2] / "camera_pkg" / "camera_pkg",
            Path(__file__).resolve().parents[2] / "src",
            Path(__file__).resolve().parents[2],
        ]
        for root in search_roots:
            candidate = root / "calibration_data.npz"
            if candidate.exists():
                return candidate
        raise FileNotFoundError("calibration_data.npz not found. Please run the camera calibration script in camera_pkg to generate it.")

    def _load_calibration(self, path: Path) -> Tuple[np.ndarray, np.ndarray]:
        if not path.exists():
            raise FileNotFoundError(f"Calibration file not found: {path}")

        data = np.load(str(path))
        camera_matrix = data["camera_matrix"]
        dist_coeffs = data["dist_coeffs"]
        self.get_logger().info(f"Loaded camera calibration from {path}")
        self.get_logger().info(f"CV2 Version: {cv2.__version__}")
        return camera_matrix, dist_coeffs


 # ============================================================
    # MAIN PROCESS: Capture, Detect, Compute Pose
    # ============================================================
    def _process_frame(self) -> None:
        """Capture frame, detect ArUco marker, compute (alpha, delta), and publish."""   

        #self.get_logger().info(f"Reading Frame {self.frame_num}") 
        ret, frame = self.cap.read()
        self.frame_num += 1

        if not ret:
            self.get_logger().warning("Failed to capture frame.")
            return

        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Equalize image histogram to improve contrast between marker and background
        #gray = cv2.equalizeHist(gray) # Did not see much improvement (Disable for now)
        
        #cv2.imshow("Frame", gray)
        #cv2.waitKey(1)

        # Detect ArUco markers
        corners, ids, _ = self.detector.detectMarkers(gray)
        

        # If nothing detected
        if ids is None or len(corners) == 0:
            self.miss_count += 1
            self.get_logger().info(f"MISS: Frame {self.frame_num}")
            return

        self.hit_count += 1
        
        # Use *first* marker 
        marker_corners = corners[0]
        marker_id = ids[0][0]

        # -----------------------------------------------------------
        # Compute alpha (bearing) USING PIXEL-BASED METHOD
        # -----------------------------------------------------------
        cX = int(np.mean(marker_corners[0][:, 0]))

        alpha_rad   = self.compute_alpha_pixel_based(cX=cX)
        alpha_deg   = math.degrees(alpha_rad)

        # -----------------------------------------------------------
        # Compute delta USING PnP
        # -----------------------------------------------------------
        # 3D model coordinates of marker corners
        half = self.marker_length / 2.0
        obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        img_points = marker_corners[0].astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            self.get_logger().warning("PnP failed.")
            return

        # delta = Euclidean distance
        delta = float(np.linalg.norm(tvec))

        # -----------------------------------------------------------
        # Make ArucoAlignment Message & Publish 
        # -----------------------------------------------------------

        alignment_msg = Float32MultiArray()
        alignment_msg.data = [alpha_rad, delta]
        self.alignment_pub.publish(alignment_msg)   
        
        # -----------------------------------------------------------
        # Visualization
        # -----------------------------------------------------------
        #cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        #cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)
        #cv2.imshow("Aruco Alignment", frame)
        #cv2.waitKey(1) & 0xFF   

        self.get_logger().info(
            f"HIT: Frame {self.frame_num} | "
            f"ID {marker_id} | "
            f"alpha =({alpha_deg:.2f} deg) | "
            f"delta =({delta:.2f}) m"
        )
        
        save_every_n_hits= 50
        # Save every nth hit frame
        if self.hit_count % save_every_n_hits == 0:
            self.save_frame(frame, self.aruco_frame_save_path / f"aruco_id_{ids[0][0]}_delta_cm_{int(delta*100)}.png")
        
    # ============================================================
    # Compute alpha using pixel-based method
    # ============================================================
    def compute_alpha_pixel_based(self, cX: int) -> float:
        """
        Compute alpha (bearing) using pixel offset from image center
        Args:
            cX (int): x-coordinate of the marker center in the image.
        Returns:
            float: alpha (bearing) in radians.
        """
        # Image midpoint
        img_center_x = self.frame_width / 2.0

        # Pixel offset from image center
        pixel_offset = cX - img_center_x

        # Convert to degrees using half-FOV normalization
        alpha_deg = (pixel_offset / img_center_x) * (self.hfov_deg / 2.0)

        # Convert to radians
        return math.radians(alpha_deg)

    
    # ============================================================
    # Cleanup
    # ============================================================
    def destroy_node(self) -> bool:
        if self.cap.isOpened():
            self.cap.release()
        
        self.get_logger().info(f"Total hits: {self.hit_count}, Total misses: {self.miss_count}")
        self.get_logger().info(f"Hit Rate: {self.hit_count / (self.hit_count + self.miss_count):.2%}")
        
        cv2.destroyAllWindows()
        return super().destroy_node()

    # ============================================================
    # Helper Functions (Display, Save)
    # ============================================================
    def display_frame(self, frame: np.ndarray) -> None:
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def display_marker(self, frame: np.ndarray, corners: np.ndarray, ids: np.ndarray) -> None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)
        cv2.imshow("Aruco Marker", frame)
        cv2.waitKey(1) & 0xFF
        
    def save_frame(self, frame: np.ndarray, filename: str) -> None:
        cv2.imwrite(filename, frame)
        cv2.waitKey(1)
        self.get_logger().info(f"Saved frame to {filename}")
        

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
