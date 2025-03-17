import cv2
import numpy as np
import glob

# === Define Chessboard Size ===
CHESSBOARD_SIZE = (9, 6)  # Adjust based on your printed pattern
SQUARE_SIZE = 0.025  # Size of each square in meters (25mm = 0.025m)

# === Initialize Storage for Calibration Data ===
obj_points = []  # 3D points in real-world space
img_points = []  # 2D points in image plane

# Prepare the 3D object points (assuming z=0)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # Scale to real-world size

# === Initialize Camera ===
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Use the correct index for your ArduCam

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# === Capture Images for Calibration ===
print("Press 'c' to capture an image. Press 'q' when done.")

image_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners, ret)

    cv2.imshow("Camera View", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c') and ret:
        obj_points.append(objp)
        img_points.append(corners)
        image_count += 1
        print(f"Captured {image_count} images for calibration.")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(obj_points) < 10:  # Ensure enough images are captured
    print("Not enough images captured for calibration. Try again.")
    exit()

# === Perform Camera Calibration ===
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

if ret:
    print("\n=== Camera Calibration Successful ===")
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs)

    # Save Calibration Results
    np.save("camera_matrix.npy", camera_matrix)
    np.save("dist_coeffs.npy", dist_coeffs)
    print("Calibration parameters saved.")
else:
    print("Calibration failed.")
