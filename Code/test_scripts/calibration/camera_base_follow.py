from picamera2 import Picamera2
import cv2, time
from src.serial_controller import SerialController

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
picam2.start()
time.sleep(1)

sc = SerialController("/dev/ttyACM0")
sc.connect()

BASE_MIN = 330
BASE_MAX = 500
base = 412          # known safe center
CENTER_X = 320
DEADBAND = 40       # ignore small errors
GAIN = 0.08         # larger = more aggressive

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

                error = cx - CENTER_X

                now = time.time()
                if abs(error) > DEADBAND and now - last_move > 0.25:
                    base = base - int(error * GAIN)
                    base = max(BASE_MIN, min(BASE_MAX, base))

                    print(f"cx={cx}, error={error}, base={base}, area={int(area)}")
                    sc.move_servo(sc.SERVO_IDS["base"], base, 300)

                    last_move = now

except KeyboardInterrupt:
    print("Stopping")

finally:
    sc.move_servo(sc.SERVO_IDS["base"], 412, 1000)
    picam2.stop()
    sc.disconnect()
