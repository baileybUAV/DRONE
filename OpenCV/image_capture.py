import cv2
import numpy as np
import glob
from picamera2 import Picamera2

# Define checkerboard pattern size
CHECKERBOARD = (7, 6)  # Adjust based on your checkerboard pattern
square_size = 0.025  # Size of a square in meters

# Termination criteria for corner subpixel refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points for a 3D checkerboard (0,0,0), (1,0,0), ..., (6,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size  # Scale to real-world size

# Arrays to store object points and image points
objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

# Initialize camera
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (1280, 720), "format": "RGB888"})
picam2.configure(config)
picam2.start()

print("Press 's' to save a frame for calibration. Press 'q' to finish and calibrate.")

while True:
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert to BGR for OpenCV

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)  # Store 3D points
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(refined_corners)  # Store 2D points

        cv2.drawChessboardCorners(img, CHECKERBOARD, refined_corners, ret)

    cv2.imshow("Calibration", img)

    key = cv2.waitKey(1)
    if key == ord('s') and ret:
        print("Frame saved for calibration.")
    elif key == ord('q'):
        break

cv2.destroyAllWindows()

# Perform camera calibration
print("Calibrating camera...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

if ret:
    print("\nCamera calibration successful!")
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs)

    # Save the calibration parameters
    np.savez("camera_calibration_data.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print("Calibration data saved as 'camera_calibration_data.npz'")

else:
    print("Camera calibration failed. Try collecting more images.")

cv2.destroyAllWindows()
