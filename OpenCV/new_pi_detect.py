import cv2
import cv2.aruco as aruco
import numpy as np
import time
import sys
from picamera2 import Picamera2

#############################

# Initialize Pi Camera
width = 1280
height = 720
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
picam2.configure(config)
picam2.start()

viewVideo = True
if len(sys.argv) > 1:
    viewVideo = sys.argv[1]
    if viewVideo == '0' or viewVideo.lower() == 'false':
        viewVideo = False

############ ARUCO/CV2 SETTINGS ############
id_to_find = 1
marker_size = 0.253  # meters

realWorldEfficiency = 0.7  # Account for real-world drone speed variation
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Enable corner refinement

# Load Camera Calibration Data
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

#############################

while True:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    # Undistort the image to improve accuracy
    img = cv2.undistort(img, cameraMatrix, cameraDistortion)
    
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    
    if ids is not None and id_to_find in ids:
        print("Found these IDs in the frame:", ids)
        index = np.where(ids == id_to_find)[0][0]  # Find index of the target marker
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, cameraDistortion)
        rvec, tvec = ret[0][index, 0, :], ret[1][index, 0, :]
        
        # Output position in meters
        print(f"MARKER POSITION (meters): x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f}")
        
        if viewVideo:
            aruco.drawDetectedMarkers(img, corners)
            cv2.drawFrameAxes(img, cameraMatrix, cameraDistortion, rvec, tvec, 0.1)
    else:
        print(f"ARUCO {id_to_find} NOT FOUND IN FRAME.")
    
    if viewVideo:
        cv2.imshow('Aruco Detection', img)
    
    # Ensure window updates and allow exit with 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
picam2.close()
exit()