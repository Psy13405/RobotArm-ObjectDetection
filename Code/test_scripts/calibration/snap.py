# snap.py
from picamera2 import Picamera2
import cv2, argparse, time, os
from datetime import datetime

parser = argparse.ArgumentParser()
parser.add_argument('-o', '--output', default='captures')
parser.add_argument('-n', '--name',   default=None)
parser.add_argument('-r', '--res',    default='1920x1080')
args = parser.parse_args()

os.makedirs(args.output, exist_ok=True)

w, h = map(int, args.res.split('x'))

picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration(
    main={"size": (w, h)}
))
picam2.start()
time.sleep(2)  # IMX708 needs ~2s to settle + auto-expose

fname = args.name or datetime.now().strftime("%Y%m%d_%H%M%S")
path  = f"{args.output}/{fname}.jpg"

picam2.capture_file(path)
picam2.stop()
print(f"Saved: {path}")
