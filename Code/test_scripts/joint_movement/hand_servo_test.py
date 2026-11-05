import RPi.GPIO as GPIO
import time

# ── Configuration ──────────────────────────────────────────────
SERVO_PIN   = 17      # BCM pin number (change to your wiring)
PWM_FREQ    = 50      # Hz — standard for servos
MIN_DUTY    = 2.5     # duty cycle for 0°
MAX_DUTY    = 12.5    # duty cycle for 180°
# ──────────────────────────────────────────────────────────────
def angle_to_duty(angle: float) -> float:
    """Convert an angle (0–180°) to a PWM duty cycle."""
    angle = max(0, min(180, angle))   # clamp
    return MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY)
def move_to(pwm: GPIO.PWM, angle: float, hold_seconds: float = 0.5) -> None:
    """Move the servo to *angle* degrees and hold for *hold_seconds*."""
    duty = angle_to_duty(angle)
    pwm.ChangeDutyCycle(duty)
    time.sleep(hold_seconds)
    # Stop sending pulses (reduces jitter while holding position)
    pwm.ChangeDutyCycle(0)
def sweep(pwm: GPIO.PWM, start: float = 0, end: float = 180,
          step: float = 10, delay: float = 0.05) -> None:
    """Sweep the servo from *start* to *end* degrees in *step* increments."""
    angles = (
        list(range(int(start), int(end) + 1, int(step))) +
        list(range(int(end), int(start) - 1, -int(step)))
    )
    for angle in angles:
        pwm.ChangeDutyCycle(angle_to_duty(angle))
        time.sleep(delay)
    pwm.ChangeDutyCycle(0)

def main():
    # Board mode: use BCM (GPIO number) pin numbering
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, PWM_FREQ)
    pwm.start(0)   # start with signal off
    try:
        print("Moving to 0° …")
        move_to(pwm, 0, hold_seconds=1)
        print("Moving to 90° …")
        move_to(pwm, 90, hold_seconds=1)
        print("Moving to 180° …")
        move_to(pwm, 180, hold_seconds=1)
        print("Sweeping 0° → 180° → 0° …")
        sweep(pwm, start=0, end=180, step=5, delay=0.02)
        # Interactive mode
        print("\nEnter an angle (0–180) or 'q' to quit:")
        while True:
            user_input = input("  Angle: ").strip()
            if user_input.lower() == 'q':
                break
            try:
                angle = float(user_input)
                move_to(pwm, angle, hold_seconds=0.8)
                print(f"  → moved to {angle:.1f}°")
            except ValueError:
                print("  Please enter a number between 0 and 180.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up. Goodbye!")
if __name__ == "__main__":
    main()
