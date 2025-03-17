import cv2


for i in range(5):  # Try indexes 0-4
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    if cap.isOpened():
        print(f"Camera found at index {i}")
        cap.release()