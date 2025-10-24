import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.subscription = self.create_subscription(
            Image, "camera/image", self.detector_callback, 10
        )
        self.bridge = CvBridge()
        
        # Initialize ArUco detector
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

    def detector_callback(self, msg):
        try:
            # Convert ROS Image → OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Detect ArUco markers
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
            )

            # If markers are detected, draw them
            if len(corners) > 0:
                # Draw green bounding box around the detected markers
                for corner in corners:
                    pts = corner.reshape((-1, 2)).astype(int)
                    cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                    
                    # Draw the center of the marker as a red dot
                    center = tuple(map(int, corner[0].mean(axis=0)))
                    cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

                # Display marker IDs
                for i, marker_id in enumerate(marker_ids):
                    pos = tuple(corners[i][0][0].astype(int) + 30)
                    cv2.putText(
                        cv_image,
                        f"ID: {marker_id[0]}",
                        pos,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 55, 55),
                        2,
                    )
                    self.get_logger().info(f"Detected ArUco marker with ID: {marker_id[0]}")

            # Display the frame
            cv2.imshow("ArUco Detector", cv_image)
            cv2.waitKey(1)  # Brief pause to update display

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()