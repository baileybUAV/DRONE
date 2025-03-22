import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
from picamera2 import Picamera2

class ArucoDetector:
    def __init__(self, id_to_find, marker_size, camera_matrix, camera_distortion, show_video=True, width=1280, height=720):
        self.id_to_find = id_to_find
        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion
        self.show_video = show_video

        # Flip matrix for consistent orientation
        self.R_flip = np.array([[1, 0, 0],
                                [0, -1, 0],
                                [0, 0, -1]], dtype=np.float32)

        # ArUco setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        # Camera init
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
        self.picam2.configure(config)
        self.picam2.start()

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.last_time = time.time()

    def rotation_matrix_to_euler_angles(self, R):
        assert np.allclose(np.dot(R, R.T), np.identity(3), atol=1e-6)
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def detect(self):
        while True:
            img = self.picam2.capture_array()
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            img = cv2.undistort(img, self.camera_matrix, self.camera_distortion)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            fps = 1.0 / (time.time() - self.last_time)
            self.last_time = time.time()

            if ids is not None and self.id_to_find in ids:
                idx = np.where(ids == self.id_to_find)[0][0]
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)

                rvec, tvec = rvecs[idx][0], tvecs[idx][0]
                R_ct = cv2.Rodrigues(rvec)[0]
                R_tc = R_ct.T

                euler = self.rotation_matrix_to_euler_angles(self.R_flip @ R_tc)
                pos_camera = -R_tc @ tvec.reshape((3, 1))

                print(f"[INFO] Marker ID {self.id_to_find} Found")
                print(f"Position (m): x={tvec[0]:.2f} y={tvec[1]:.2f} z={tvec[2]:.2f}")
                print(f"Rotation (deg): roll={math.degrees(euler[0]):.1f}, pitch={math.degrees(euler[1]):.1f}, yaw={math.degrees(euler[2]):.1f}")
                print(f"FPS: {fps:.2f}")

                if self.show_video:
                    aruco.drawDetectedMarkers(img, corners)
                    cv2.drawFrameAxes(img, self.camera_matrix, self.camera_distortion, rvec, tvec, 0.1)

                    cv2.putText(img, f"Position x={tvec[0]:.2f} y={tvec[1]:.2f} z={tvec[2]:.2f}", (10, 30), self.font, 0.6, (0, 255, 0), 2)
                    cv2.putText(img, f"Euler r={math.degrees(euler[0]):.1f} p={math.degrees(euler[1]):.1f} y={math.degrees(euler[2]):.1f}", (10, 60), self.font, 0.6, (0, 255, 0), 2)
                    cv2.putText(img, f"FPS: {fps:.2f}", (10, 90), self.font, 0.6, (0, 255, 255), 2)

            else:
                print(f"[INFO] Marker ID {self.id_to_find} not found - FPS: {fps:.2f}")

            if self.show_video:
                cv2.imshow("Aruco Detection", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        self.picam2.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
    cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
    cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

    detector = ArucoDetector(
        id_to_find=1,
        marker_size=0.253,
        camera_matrix=cameraMatrix,
        camera_distortion=cameraDistortion,
        show_video=True
    )
    detector.detect()
