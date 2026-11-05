import cv2
import time
import RPi.GPIO as GPIO
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

# Tune these after watching printed area values
AREA_FAR = 1000
AREA_CLOSE = 12000

MIN_AREA = 800
DEADBAND = 10
ELBOW_DEADBAND = 20
MOVE_TIME_MS = 300

# ── GPIO gripper servo config ──────────────────────────────────
GRIPPER_PIN = 17       # BCM pin number — adjust to your wiring
PWM_FREQ = 50
MIN_DUTY = 2.5         # duty cycle for 0°
MAX_DUTY = 12.5        # duty cycle for 180°
MAX_ANGLE = 180

# Trigger threshold: "very close" object area
CLOSE_TRIGGER_AREA = AREA_CLOSE * 0.9  # tune as needed

def map_value(v, in_min, in_max, out_min, out_max):
    return out_min + (v - in_min) * (out_max - out_min) / (in_max - in_min)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_to_duty(angle):
    angle = clamp(angle, 0, MAX_ANGLE)
    return MIN_DUTY + (angle / MAX_ANGLE) * (MAX_DUTY - MIN_DUTY)

def gripper_set_angle(pwm, angle, hold_seconds=0.3):
    pwm.ChangeDutyCycle(angle_to_duty(angle))
    time.sleep(hold_seconds)
    pwm.ChangeDutyCycle(0)

# ── GPIO setup ──────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(GRIPPER_PIN, GPIO.OUT)
gripper_pwm = GPIO.PWM(GRIPPER_PIN, PWM_FREQ)
gripper_pwm.start(0)

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
gripper_set_angle(gripper_pwm, 0)  # start open
time.sleep(1)

last_base = BASE_CENTER
last_shoulder = SHOULDER_CENTER
last_elbow = ELBOW_CENTER
last_move = 0
gripper_closed = False

try:
    while True:
        frame = picam2.capture_array()

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green object detection
        mask = cv2.inRange(hsv, (3, 60, 60), (32, 255, 255))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                x, y, bw, bh = cv2.boundingRect(c)
                cx = x + bw // 2
                cy = y + bh // 2

                base = int(round(map_value(cx, 0, w, BASE_MIN, BASE_MAX)))
                shoulder = int(round(map_value(cy, 0, h, SHOULDER_MAX, SHOULDER_MIN)))

                elbow = int(round(map_value(area, AREA_FAR, AREA_CLOSE, ELBOW_MAX, ELBOW_MIN)))

                base = clamp(base, BASE_MIN, BASE_MAX)
                shoulder = clamp(shoulder, SHOULDER_MIN, SHOULDER_MAX)
                elbow = clamp(elbow, ELBOW_MIN, ELBOW_MAX)

                now = time.time()

                base_changed = abs(base - last_base) > DEADBAND
                shoulder_changed = abs(shoulder - last_shoulder) > DEADBAND
                elbow_changed = abs(elbow - last_elbow) > ELBOW_DEADBAND

                if (base_changed or shoulder_changed or elbow_changed) and now - last_move > 0.15:
                    print(
                        f"cx={cx}, cy={cy}, area={int(area)}, "
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

                # ── Gripper trigger based on proximity ──
                if area >= CLOSE_TRIGGER_AREA and not gripper_closed:
                    print(f"Object very close (area={int(area)}) — closing gripper to 180°")
                    gripper_set_angle(gripper_pwm, 180)
                    gripper_closed = True
                elif area < CLOSE_TRIGGER_AREA and gripper_closed:
                    print(f"Object moved away (area={int(area)}) — opening gripper to 0°")
                    gripper_set_angle(gripper_pwm, 0)
                    gripper_closed = False

                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)

        cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)
        cv2.line(frame, (0, h // 2), (w, h // 2), (255, 255, 255), 2)

        cv2.putText(frame, f"Base: {last_base}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(frame, f"Shoulder: {last_shoulder}", (20, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(frame, f"Elbow: {last_elbow}", (20, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(frame, f"Gripper: {'CLOSED' if gripper_closed else 'OPEN'}", (20, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        cv2.imwrite("debug_frame.jpg", frame)
        cv2.imwrite("debug_mask.jpg", mask)

except KeyboardInterrupt:
    print("Stopping")

finally:
    sc.move_servo(sc.SERVO_IDS["base"], BASE_CENTER, 1000)
    sc.move_servo(sc.SERVO_IDS["shoulder"], SHOULDER_CENTER, 1000)
    sc.move_servo(sc.SERVO_IDS["elbow"], ELBOW_CENTER, 1000)

    gripper_set_angle(gripper_pwm, 0)  # release gripper on exit
    gripper_pwm.stop()
    GPIO.cleanup()

    sc.disconnect()
    picam2.stop()
