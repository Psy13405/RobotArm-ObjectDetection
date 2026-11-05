from picamera2 import Picamera2
import cv2
import time

# Low HSV values
LOW_H = 39
LOW_S = 10
LOW_V = 76

# High HSV values
HIGH_H = 99
HIGH_S = 150
HIGH_V = 210

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
picam2.start()
time.sleep(1)

while True:
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    mask1 = cv2.inRange(hsv, (0, 140, 100), (8, 255, 255))
    mask2 = cv2.inRange(hsv, (LOW_H, LOW_S, LOW_V), (HIGH_V, HIGH_S, HIGH_V))
    mask = mask1 | mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    debug = frame.copy()

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > 800:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            print(f"cx={cx}, cy={cy}, area={int(area)}")

            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 6, (255, 0, 0), -1)

    cv2.imwrite("debug_frame.jpg", debug)
    cv2.imwrite("debug_mask.jpg", mask)

    time.sleep(0.5)
