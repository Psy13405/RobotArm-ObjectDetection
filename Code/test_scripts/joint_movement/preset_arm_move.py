import time
import RPi.GPIO as GPIO

from src.serial_controller import SerialController

SERIAL_PORT = "/dev/ttyACM0"

# Center positions
BASE_CENTER = 515
SHOULDER_CENTER = 500
ELBOW_CENTER = 510

# Target positions
BASE_TARGET = BASE_CENTER
SHOULDER_TARGET = 377
ELBOW_TARGET = 674

MOVE_TIME_MS = 1000
HOLD_SECONDS = 2.0

# GPIO hand servo
HAND_PIN = 17
PWM_FREQ = 50
MIN_DUTY = 2.5
MAX_DUTY = 12.5


def angle_to_duty(angle):
    angle = max(0, min(180, angle))
    return MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY)


def move_hand(pwm, angle, hold_seconds=0.7):
    pwm.ChangeDutyCycle(angle_to_duty(angle))
    time.sleep(hold_seconds)
    pwm.ChangeDutyCycle(0)


def move_arm(sc, base, shoulder, elbow, move_time_ms):
    sc.move_servo(sc.SERVO_IDS["base"], base, move_time_ms)
    sc.move_servo(sc.SERVO_IDS["shoulder"], shoulder, move_time_ms)
    sc.move_servo(sc.SERVO_IDS["elbow"], elbow, move_time_ms)


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HAND_PIN, GPIO.OUT)

    hand_pwm = GPIO.PWM(HAND_PIN, PWM_FREQ)
    hand_pwm.start(0)

    sc = SerialController(SERIAL_PORT)
    sc.connect()

    time.sleep(3.0)

    try:
        print("Moving to center...")
        move_arm(
            sc,
            BASE_CENTER,
            SHOULDER_CENTER,
            ELBOW_CENTER,
            MOVE_TIME_MS,
        )
        move_hand(hand_pwm, 0)
        time.sleep(1.2)

        print("Moving to target position...")
        move_arm(
            sc,
            BASE_TARGET,
            SHOULDER_TARGET,
            ELBOW_TARGET,
            MOVE_TIME_MS,
        )
        time.sleep(1.2)

        print("Moving hand to 180 degrees...")
        move_hand(hand_pwm, 180)

        print(f"Holding target for {HOLD_SECONDS} seconds...")
        time.sleep(HOLD_SECONDS)

        print("Returning to center...")
        move_arm(
            sc,
            BASE_CENTER,
            SHOULDER_CENTER,
            ELBOW_CENTER,
            MOVE_TIME_MS,
        )

        print("Opening hand to 0 degrees...")
        move_hand(hand_pwm, 0)

        print("Done.")

    finally:
        hand_pwm.stop()
        GPIO.cleanup()
        sc.disconnect()


if __name__ == "__main__":
    main()
