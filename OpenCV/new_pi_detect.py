import cv2
import cv2.aruco as aruco
import numpy as np
import time
import sys
from picamera2 import Picamera2

#############################

# Initialize Pi Camera
width = 640
height = 480
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
marker_size = 0.305  # meters

realWorldEfficiency = 0.7  # Account for real-world drone speed variation
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# Load Camera Calibration Data
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

#############################

seconds = 1000000 if viewVideo else 5
start_time = time.time()
while time.time() - start_time < seconds:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    
    if ids is not None:
        print("Found these IDs in the frame:", ids)
        
        if id_to_find in ids:
            index = np.where(ids == id_to_find)[0][0]  # Find index of the target marker
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, cameraDistortion)
            rvec, tvec = ret[0][index, 0, :], ret[1][index, 0, :]
            
            x, y, z = tvec[0], tvec[1], tvec[2]  # Keep values as floating-point meters
            print(f"MARKER POSITION (meters): x={x:.3f} y={y:.3f} z={z:.3f}")
            
            if viewVideo:
                aruco.drawDetectedMarkers(img, corners)
                aruco.drawAxis(img, cameraMatrix, cameraDistortion, rvec, tvec, 0.1)
                cv2.imshow('Aruco Detection', img)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    else:
        print(f"ARUCO {id_to_find} NOT FOUND IN FRAME.")

if not viewVideo:
    print("Performance Diagnosis:")
    print("Make sure the system is running efficiently on the Raspberry Pi.")

cv2.destroyAllWindows()
