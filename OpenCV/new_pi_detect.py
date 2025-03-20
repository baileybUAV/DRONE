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
            
            # Convert meters to feet (1 meter = 3.28084 feet)
            x_ft, y_ft, z_ft = tvec[0] * 3.28084, tvec[1] * 3.28084, tvec[2] * 3.28084
            print(f"MARKER POSITION (feet): x={x_ft:.3f} y={y_ft:.3f} z={z_ft:.3f}")
            
            if viewVideo:
                aruco.drawDetectedMarkers(img, corners)
                cv2.drawFrameAxes(img, cameraMatrix, cameraDistortion, rvec, tvec, 0.1)
                cv2.imshow('Aruco Detection', img)
                
    else:
        print(f"ARUCO {id_to_find} NOT FOUND IN FRAME.")
        time.sleep(1)
        
    # Ensure window updates and allow exit with 'q'
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

if not viewVideo:
    print("Performance Diagnosis:")
    print("Make sure the system is running efficiently on the Raspberry Pi.")

cv2.destroyAllWindows()
picam2.close()
exit()