from picamera2 import Picamera2
import cv2
import time
import numpy as np

def map_value(value, in_min, in_max, out_min, out_max):
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

time.sleep(1)

lower_orange = np.array([5, 80, 80])
upper_orange = np.array([25, 255, 255])

lower_highlight = np.array([0, 0, 200])
upper_highlight = np.array([30, 60, 255])

while True:
    frame = picam2.capture_array()

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Orange ping pong ball detection
    mask = cv2.inRange(hsv, (5, 80, 80), (25, 255, 255))
    #orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Also catch the bright highlight on the ball
    #highlight_mask = cv2.inRange(hsv, lower_highlight, upper_highlight)
    # Combine both
    #mask = cv2.bitwise_or(orange_mask, highlight_mask) 
    #kernel = np.ones((7, 7), np.uint8)
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fill gaps
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove noise

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > 500:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            print(f"Object center: x={cx}, y={cy}, area={int(area)}")
            target_x = 130
            target_y = map_value(cx, 0, 640, -80, 80)
            target_z = 0

            print(f"IK target: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

    #cv2.imshow("Camera", frame)
    #cv2.imshow("Mask", mask)
    cv2.imwrite("ball_frame.jpg", frame)
    cv2.imwrite("ball_mask.jpg", mask)

    #if cv2.waitKey(1) & 0xFF == ord("q"):
    #    break

picam2.stop()
cv2.destroyAllWindows()
