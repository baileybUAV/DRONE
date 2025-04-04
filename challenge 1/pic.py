import time
from picamera2 import Picamera2

# ------------------ CONFIG ------------------
camera_resolution = (1280, 720)
target_altitude = 6  # in meters (unused here but kept for context)

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": camera_resolution, "format": "RGB888"},buffer_count=2)
picam2.configure(config)
picam2.set_controls({
    "AfMode": 1,           # Enable continuous autofocus (if supported)
    "AfTrigger": 0,        # Start autofocus loop
    "AwbEnable": True,     # Auto white balance
    "AeEnable": True,      # Auto exposure
})
picam2.start()

# Allow camera to warm up
time.sleep(2)

# ------------------ CAPTURE FUNCTION ------------------
def capture_photo(index=None):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = f"_{index}" if index is not None else ""
    photo_path = f"photo_{timestamp}{suffix}.jpg"
    picam2.capture_file(photo_path)
    print(f"Photo captured and saved at: {photo_path}")

# ------------------ MAIN ------------------
print("Code started")

capture_photo(1)
time.sleep(3)

capture_photo(2)

picam2.stop()
picam2.close()
print("Finished.")
