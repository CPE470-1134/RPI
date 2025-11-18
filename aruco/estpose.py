import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco


class ArucoPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.camera_fov = 60  # degrees
        ROS2_NAMESPACE = ""  # set your ROS2_NAMESPACE (if you set one)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        self.camera_matrix = np.array([
            [821.993, 0, 330.489],
            [0, 821.993, 248.997],
            [0, 0, 1]
        ])
        
        self.dist_coeffs = np.array([[-0.018522, 1.03979, 0, 0, -3.3171, 0, 0, 0]])
        self.marker_length = 0.05  # meters
        self.frame_num = 0
        self.timer = self.create_timer(0.1, self.process_frame)  # ~10 FPS
    def estimatePoseUsingPnP(self, corners, marker_size, mtx, distortion):
        marker_points = np.array([
            [-marker_size/2, marker_size/2, 0],
            [marker_size/2, marker_size/2, 0],
            [marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)
        
        trash = []
        rvecs = []
        tvecs = []
        
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
            
        return rvecs, tvecs, trash
    def aruco_display(self, corners, ids, image):
        if len(corners) > 0:
            ids = ids.flatten()
            for markerCorner, markerID in zip(corners, ids):
                # Corner order: top-left, top-right, bottom-right, and bottom-left
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            cX, cY = image.shape[1]//2, image.shape[1]//2
        return image, (cX, cY)
    def process_frame(self):
        ret, frame = self.cap.read()
        self.frame_num += 1
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        frame, (cX, cY) = self.aruco_display(corners, ids, frame)
        
        print(cX, cY)
        print(frame.shape)
        
        if ids is None:
            self.get_logger().error("No ArUco detected")
        elif len(ids) > 1:
            self.get_logger().error("More than one ArUco detected")
        else:
            img_center_x = frame.shape[1]//2
            horiz_dist_ci_cx = cX - img_center_x
            print(self.camera_fov, horiz_dist_ci_cx, frame.shape[1])
            rvecs, tvecs, _ = self.estimatePoseUsingPnP(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            rvec = rvecs[0]
            tvec = tvecs[0]
            processed_x = float(tvec[0])
            processed_y = float(tvec[1])
            processed_z = float(tvec[2])
            
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
            print("Rotation Vector (rvec):", rvec.ravel())
            print("Translation Vector (tvec):", tvec.ravel())
            
            if self.frame_num % 10 == 0:
                cv2.imwrite('/root/create3_ws/aruco_detection.png', frame)
    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoPoseNode()
    
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        aruco_detector.cap.release()
        cv2.destroyAllWindows()
        aruco_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()