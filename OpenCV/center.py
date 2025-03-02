import cv2
import numpy as np

class ArucoDetector:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.camera_matrix = np.array([[1000.0, 0.0, 640.0],
                                       [0.0, 1000.0, 360.0],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.marker_size = 0.05
        self.marker_corners_3d = np.array([[-self.marker_size / 2, self.marker_size / 2, 0],
                                           [self.marker_size / 2, self.marker_size / 2, 0],
                                           [self.marker_size / 2, -self.marker_size / 2, 0],
                                           [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return self.detector.detectMarkers(gray)

    def process_markers(self, frame, corners, ids):
        for i, corner in enumerate(corners):
            marker_corners_2d = corner[0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(self.marker_corners_3d, marker_corners_2d, self.camera_matrix, self.dist_coeffs)
            if success:
                cv2.polylines(frame, [np.int32(marker_corners_2d)], True, (0, 255, 0), 5)
                x, y = marker_corners_2d[0]
                cv2.putText(frame, f"ID: {ids[i][0]}", (int(x), int(y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size)
                print(f"Marker ID: {ids[i][0]}\nRotation Vector:\n{rvec}\nTranslation Vector:\n{tvec}")

# Initialize and start capture
detector = ArucoDetector()
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    corners, ids = detector.detect_markers(frame)
    if ids is not None:
        detector.process_markers(frame, corners, ids)
    cv2.imshow("Image", frame)
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
