import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher = self.create_publisher(Image, "camera/image", 10)
        self.bridge = CvBridge()

        # Open USB camera
        self.capture = cv2.VideoCapture(0)
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open camera.")
            return

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().error("Failed to capture frame.")
            return

        # Convert OpenCV frame → ROS Image
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher.publish(ros_image)
        self.get_logger().info("Publishing frame.")

    def __del__(self):
        if self.capture.isOpened():
            self.capture.release()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
