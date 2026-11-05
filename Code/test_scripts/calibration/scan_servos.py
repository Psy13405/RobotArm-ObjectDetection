from src.serial_controller import SerialController
import time

sc = SerialController("/dev/ttyACM0")
sc.connect()

for sid in range(1, 7):
    print(f"Testing servo ID {sid}")
    sc.move_servo(sid, 520, 800)
    time.sleep(1)
    sc.move_servo(sid, 480, 800)
    time.sleep(1)

sc.disconnect()
