from picamera2 import Picamera2
import cv2
import time

from src.ik_engine import IKEngine
from src.serial_controller import SerialController

def map_value(value, in_min, in_max, out_min, out_max):
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
picam2.start()
time.sleep(1)

ik = IKEngine()
sc = SerialController("/dev/ttyACM0")
sc.connect()

last_move = 0

try:
    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        mask1 = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 800:
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2

                target_x = 130
                target_y = map_value(cx, 0, 640, 80, -80)
                target_z = 0

                now = time.time()

                # Move at most twice per second
                if now - last_move > 0.5:
                    print(f"Target: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")

                    base, shoulder, elbow = ik.calculate_ik(target_x, target_y, target_z)
                    
                    #clamped servo range
                    base = max(0, min(1000, int(round(base))))
                    shoulder = max(0, min(1000, int(round(shoulder))))
                    elbow = max(0, min(1000, int(round(elbow))))
                    
                    #base = int(round(base))
                    #shoulder = int(round(shoulder))
                    #elbow = int(round(elbow))

                    print(f"Servos: base={base}, shoulder={shoulder}, elbow={elbow}")

                    sc.move_all(base, shoulder, elbow, time_ms=700)
                    last_move = now

except KeyboardInterrupt:
    print("Stopping")

finally:
    picam2.stop()
    sc.disconnect()
