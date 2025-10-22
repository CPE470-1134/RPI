import cv2
import numpy as np

image = cv2.imread("ArUco.png")
aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_parameters = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
corners, marker_ids, rejected = aruco_detector.detectMarkers(image=image)
if len(corners) > 0:
    # Draw green bounding box around the detected marker
    for corner in corners:
        pts = corner.reshape((-1, 2))
        pts = np.int32(pts)
        cv2.polylines(image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
    # Draw the center of the marker as a red dot
    center = np.mean(corners[0][0], axis=0)
    center = tuple(np.int32(center))
    cv2.circle(image, center, 5, (0, 0, 255), -1)
    # Display the marker ID at the top left of the image
    for i in range(len(marker_ids)):
        cv2.putText(
            image,
            f"ID: {marker_ids[i][0]}",
            tuple(np.int32(corners[i][0][0]) + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 55, 55),
            2,
        )
cv2.imshow("Detected Aruco Marker", image)
cv2.waitKey(0)
cv2.imwrite("annotated_image", image)
cv2.destroyAllWindows()
