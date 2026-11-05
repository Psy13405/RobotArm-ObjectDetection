from src.serial_controller import SerialController
import time

sc = SerialController("/dev/ttyACM0")
sc.connect()

for pos in [400, 600, 500]:
    print("Base")
    sc.move_servo(2, pos, 1000)
    time.sleep(2)

for pos in [400, 600, 500]:
    print("Shoulder")
    sc.move_servo(3, pos, 1000)
    time.sleep(2)

for pos in [400, 600, 500]:
    print("Elbow")
    sc.move_servo(1, pos, 1000)
    time.sleep(2)

sc.disconnect()
