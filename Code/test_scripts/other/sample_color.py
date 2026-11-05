from picamera2 import Picamera2
import cv2
import time
import numpy as np

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
picam2.start()
time.sleep(1)

frame = picam2.capture_array()

# Save raw frame
cv2.imwrite("sample_frame.jpg", frame)

# Crop center box. Put the red object in the CENTER of the image.
h, w, _ = frame.shape
cx, cy = w // 2, h // 2
box = frame[cy-40:cy+40, cx-40:cx+40]

hsv = cv2.cvtColor(box, cv2.COLOR_RGB2HSV)

avg = np.mean(hsv.reshape(-1, 3), axis=0)
low = np.percentile(hsv.reshape(-1, 3), 10, axis=0)
high = np.percentile(hsv.reshape(-1, 3), 90, axis=0)

print("Average HSV:", avg)
print("Low HSV:", low)
print("High HSV:", high)

picam2.stop()
