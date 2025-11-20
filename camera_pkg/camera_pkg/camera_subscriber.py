import matplotlib.pyplot as plt
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscription = self.create_subscription(
            Image, "camera/image", self.listener_callback, 10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        print(msg)
        try:
            # Convert ROS Image → OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            
            # Load ArUco dictionary and parameters
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            aruco_parameters = cv2.aruco.DetectorParameters_create()
            corners, marker_ids, _ = cv2.aruco.detectMarkers(
                cv_image, aruco_dictionary, parameters=aruco_parameters
            )

            # If no markers were detected, skip drawing/saving altogether.
            if not corners:
                self.get_logger().debug("No ArUco markers detected in this frame.")
                return

            # Draw green bounding box around the detected marker(s)
            for corner in corners:
                pts = corner.reshape((-1, 2))
                pts = np.int32(pts)
                cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw the center of the first marker as a red dot
            center = np.mean(corners[0][0], axis=0)
            center = tuple(np.int32(center))
            cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

            # Display the marker ID at the top left of the image
            if marker_ids is not None:
                for i in range(len(marker_ids)):
                    cv2.putText(
                        cv_image,
                        f"ID: {marker_ids[i][0]}",
                        tuple(np.int32(corners[i][0][0]) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 55, 55),
                        2,
                    )

            cv2.imwrite("annotated_image.png", cv_image)

            # Display frame
            plt.imshow(cv_image)
            plt.axis("off")
            plt.show(block=False)
            plt.pause(0.1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
