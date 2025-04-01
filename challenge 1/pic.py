import time
from picamera2 import Picamera2
import cv2

# ------------------ CONFIG ------------------
camera_resolution = (1280, 720)
target_altitude = 6  # in meters (unused here but kept for context)

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

# Allow camera to warm up
time.sleep(2)

# ------------------ CAPTURE FUNCTION ------------------
def capture_photo(index=None):
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = f"_{index}" if index is not None else ""
    photo_path = f"hover_photo_{timestamp}{suffix}.jpg"
    cv2.imwrite(photo_path, img)
    print(f"Photo captured and saved at: {photo_path}")

# ------------------ MAIN ------------------
print("Code started")
time.sleep(5)

capture_photo(1)
time.sleep(3)

capture_photo(2)
time.sleep(3)

picam2.stop()
print("Finished.")
picam2.close()  # Close the camera if needed
# picam2.close()
# picam2.close()  # Close the camera if needed