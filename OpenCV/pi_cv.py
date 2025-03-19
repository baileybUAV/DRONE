
#aruco detection using pi camera

import cv2
import numpy as np
from picamera2 import Picamera2

class HomogeneousBgDetector:
    def __init__(self, aruco_dict, aruco_parameters):
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)

    def detect_aruco_objects(self, frame):
        # Convert image to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        return corners, ids

# Initialize ArUco Detector
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

# Camera matrix and distortion coefficients
# Replace these with your camera's actual calibration data
camera_matrix = np.array([[1000.0, 0.0, 640.0],
                           [0.0, 1000.0, 360.0],
                           [0.0, 0.0, 1.0]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Assuming no distortion

# Marker size in meters
marker_size = 0.05  # Adjust to your marker's actual size

# Define 3D points of the marker's corners in real-world space
marker_corners_3d = np.array([
    [-marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32)

# Load Object Detector
detector = HomogeneousBgDetector(aruco_dict, parameters)

# Initialize Raspberry Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "SRGGB8"})
picam2.configure(config)
picam2.start()

# Main loop
while True:
    img = picam2.capture_array()
    
    # Convert to BGR for OpenCV compatibility
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # Detect ArUco objects
    corners, ids = detector.detect_aruco_objects(img)

    if ids is not None and len(corners) > 0:
        for i, corner in enumerate(corners):
            # Extract 2D corners of the marker
            marker_corners_2d = corner[0].astype(np.float32)

            # SolvePnP to find rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(
                marker_corners_3d, marker_corners_2d, camera_matrix, dist_coeffs
            )

            if success:
                # Draw marker boundaries
                cv2.polylines(img, [np.int32(marker_corners_2d)], True, (0, 255, 0), 5)

                # Display marker ID
                x, y = marker_corners_2d[0]
                cv2.putText(img, f"ID: {ids[i][0]}", (int(x), int(y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                # Draw axis
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, marker_size)

                # Print pose information
                print(f"Marker ID: {ids[i][0]}")
                print(f"Rotation Vector (rvec):\n{rvec}")
                print(f"Translation Vector (tvec):\n{tvec}")

    # Display the image
    cv2.imshow("Image", img)
    key = cv2.waitKey(1)
    if key == 27:  # Press 'Esc' to exit
        break

# Release resources
cv2.destroyAllWindows()
