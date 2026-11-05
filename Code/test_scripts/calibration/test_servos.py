from src.serial_controller import SerialController
import time

sc = SerialController("/dev/ttyACM0")
sc.connect()

# gentle test positions
sc.move_all(512, 512, 512, time_ms=1000)
time.sleep(2)

sc.move_all(550, 500, 500, time_ms=1000)
time.sleep(2)

sc.move_all(512, 512, 512, time_ms=1000)
time.sleep(1)

sc.disconnect()
