import cv2
import numpy as np
import pickle
from picamera2 import Picamera2

# Load camera calibration data
CALIBRATION_DIR = "./calibrationFiles"

with open(f"{CALIBRATION_DIR}/calibration_data.pkl", "rb") as f:
    calib_data = pickle.load(f)
    camera_matrix = np.array(calib_data['camera_matrix'])
    dist_coeffs = np.array(calib_data['dist_coefs'])

# Initialize ArUco Detector
parameters = cv2.aruco.DetectorParameters()
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

def detect_aruco_objects(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

# Marker size in meters
marker_size = 0.05  # Adjust to your marker's actual size

# Define 3D points of the marker's corners in real-world space
marker_corners_3d = np.array([
    [-marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, marker_size / 2, 0],
    [marker_size / 2, -marker_size / 2, 0],
    [-marker_size / 2, -marker_size / 2, 0]
], dtype=np.float32)

# Initialize Raspberry Pi Camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(config)
picam2.start()

print("Press 'Esc' to exit.")

# Main loop
while True:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert to BGR for OpenCV
    
    # Detect ArUco objects
    corners, ids = detect_aruco_objects(img)
    
    if ids is not None and len(corners) > 0:
        for i, corner in enumerate(corners):
            marker_corners_2d = corner[0].astype(np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                marker_corners_3d, marker_corners_2d, camera_matrix, dist_coeffs
            )
            
            if success:
                cv2.polylines(img, [np.int32(marker_corners_2d)], True, (0, 255, 0), 5)
                x, y = marker_corners_2d[0]
                cv2.putText(img, f"ID: {ids[i][0]}", (int(x), int(y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, marker_size)
                print(f"Marker ID: {ids[i][0]}\nRotation Vector: {rvec}\nTranslation Vector: {tvec}")
    
    cv2.imshow("ArUco Detection", img)
    key = cv2.waitKey(1)
    if key == 27:  # Press 'Esc' to exit
        break

cv2.destroyAllWindows()