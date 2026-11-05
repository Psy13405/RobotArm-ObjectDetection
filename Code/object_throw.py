import cv2
import time
from src.serial_controller import SerialController
from picamera2 import Picamera2

SERIAL_PORT = "/dev/ttyACM0"

BASE_MIN = 140
BASE_CENTER = 515
BASE_MAX = 890

SHOULDER_MIN = 130
SHOULDER_CENTER = 500
SHOULDER_MAX = 870

ELBOW_MIN = 130
ELBOW_CENTER = 510
ELBOW_MAX = 880

AREA_FAR = 1000
AREA_CLOSE = 12000

MIN_AREA = 800
DEADBAND = 6
ELBOW_DEADBAND = 12
MOVE_TIME_MS = 120
MOVE_INTERVAL = 0.05
LOOKAHEAD = 0.15

def map_value(v, in_min, in_max, out_min, out_max):
    return out_min + (v - in_min) * (out_max - out_min) / (in_max - in_min)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
))
picam2.start()
time.sleep(1)

sc = SerialController(SERIAL_PORT)
sc.connect()

sc.move_servo(sc.SERVO_IDS["base"], BASE_CENTER, 1000)
sc.move_servo(sc.SERVO_IDS["shoulder"], SHOULDER_CENTER, 1000)
sc.move_servo(sc.SERVO_IDS["elbow"], ELBOW_CENTER, 1000)
time.sleep(1)

last_base = BASE_CENTER
last_shoulder = SHOULDER_CENTER
last_elbow = ELBOW_CENTER
last_move = 0

last_seen_time = None
last_cx = None
last_cy = None
last_area = None

try:
    while True:
        frame = picam2.capture_array()

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Green object
        #mask = cv2.inRange(hsv, (35, 80, 60), (85, 255, 255))

        # Ping pong ball
        mask = cv2.inRange(hsv, (5, 80, 80), (25, 255, 255))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            continue

        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area < MIN_AREA:
            continue

        x, y, bw, bh = cv2.boundingRect(c)
        cx = x + bw // 2
        cy = y + bh // 2

        now = time.time()

        pred_cx = cx
        pred_cy = cy
        pred_area = area

        if last_seen_time is not None:
            dt = max(now - last_seen_time, 0.001)

            vx = (cx - last_cx) / dt
            vy = (cy - last_cy) / dt
            va = (area - last_area) / dt

            pred_cx = cx + vx * LOOKAHEAD
            pred_cy = cy + vy * LOOKAHEAD
            pred_area = area + va * LOOKAHEAD

        last_cx = cx
        last_cy = cy
        last_area = area
        last_seen_time = now

        pred_cx = clamp(pred_cx, 0, w)
        pred_cy = clamp(pred_cy, 0, h)
        pred_area = clamp(pred_area, AREA_FAR, AREA_CLOSE)

        base = int(round(map_value(pred_cx, 0, w, BASE_MAX, BASE_MIN)))
        shoulder = int(round(map_value(pred_cy, 0, h, SHOULDER_MAX, SHOULDER_MIN)))
        elbow = int(round(map_value(pred_area, AREA_FAR, AREA_CLOSE, ELBOW_MAX, ELBOW_MIN)))

        base = clamp(base, BASE_MIN, BASE_MAX)
        shoulder = clamp(shoulder, SHOULDER_MIN, SHOULDER_MAX)
        elbow = clamp(elbow, ELBOW_MIN, ELBOW_MAX)

        base_changed = abs(base - last_base) > DEADBAND
        shoulder_changed = abs(shoulder - last_shoulder) > DEADBAND
        elbow_changed = abs(elbow - last_elbow) > ELBOW_DEADBAND

        if (base_changed or shoulder_changed or elbow_changed) and now - last_move > MOVE_INTERVAL:
            print(
                f"cx={cx}, cy={cy}, area={int(area)} | "
                f"pred_cx={int(pred_cx)}, pred_cy={int(pred_cy)}, pred_area={int(pred_area)} | "
                f"base={base}, shoulder={shoulder}, elbow={elbow}"
            )

            if base_changed:
                sc.move_servo(sc.SERVO_IDS["base"], base, MOVE_TIME_MS)
                last_base = base

            if shoulder_changed:
                sc.move_servo(sc.SERVO_IDS["shoulder"], shoulder, MOVE_TIME_MS)
                last_shoulder = shoulder

            if elbow_changed:
                sc.move_servo(sc.SERVO_IDS["elbow"], elbow, MOVE_TIME_MS)
                last_elbow = elbow

            last_move = now

except KeyboardInterrupt:
    print("Stopping")

finally:
    sc.move_servo(sc.SERVO_IDS["base"], BASE_CENTER, 1000)
    sc.move_servo(sc.SERVO_IDS["shoulder"], SHOULDER_CENTER, 1000)
    sc.move_servo(sc.SERVO_IDS["elbow"], ELBOW_CENTER, 1000)

    sc.disconnect()
    picam2.stop()
