#!/usr/bin/env python
import numpy as np
import cv2
import os
import pickle
from picamera2 import Picamera2

# Camera settings
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAMESTEP = 60  # Capture every 20th frame

# Checkerboard settings
CHECKERBOARD = (9, 6)  # Number of inner corners
SQUARE_SIZE = 0.024  # Size of a square in meters
IMAGE_GOAL = 40  # Number of images needed for calibration
DEBUG_DIR = "./calibrationpic"  # Directory for debugging images
OUTPUT_DIR = "./calibrationFiles"  # Directory for calibration data

# Prepare object points for a 3D checkerboard
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # Scale to real-world size

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
picam2.configure(config)
picam2.start()

print("Press 'q' to quit and calibrate after collecting images.")

i = -1
image_count = 0
while True:
    i += 1
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert to BGR for OpenCV
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if i % FRAMESTEP != 0:
        continue

    print(f"Searching for chessboard in frame {i}...")
    found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, flags=cv2.CALIB_CB_FILTER_QUADS)

    if found:
        term_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term_criteria)
        imgpoints.append(corners.reshape(1, -1, 2))
        objpoints.append(objp.reshape(1, -1, 3))
        image_count += 1
        print(f"Captured {image_count}/{IMAGE_GOAL} images.")

        if DEBUG_DIR:
            img_chess = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(img_chess, CHECKERBOARD, corners, found)
            cv2.imwrite(os.path.join(DEBUG_DIR, f'{image_count:04d}.png'), img_chess)

        if image_count >= IMAGE_GOAL:
            break

    cv2.imshow("Calibration", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

# Perform camera calibration
print("\nPerforming calibration...")
rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (FRAME_WIDTH, FRAME_HEIGHT), None, None)

print("RMS:", rms)
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs.ravel())

# Save calibration data
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

np.savetxt(os.path.join(OUTPUT_DIR, "cameraMatrix.txt"), camera_matrix, delimiter=',')
np.savetxt(os.path.join(OUTPUT_DIR, "cameraDistortion.txt"), dist_coeffs, delimiter=',')

with open(os.path.join(OUTPUT_DIR, "calibration_data.pkl"), 'wb') as f:
    pickle.dump({'rms': rms, 'camera_matrix': camera_matrix, 'dist_coefs': dist_coeffs}, f)

print("Calibration complete. Data saved.")
