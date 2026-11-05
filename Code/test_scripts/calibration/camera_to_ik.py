def map_value(value, in_min, in_max, out_min, out_max):
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

# Camera frame size
CAM_W = 640
CAM_H = 480

# Safe robot workspace values
ROBOT_X = 130       # forward reach
ROBOT_Y_MIN = -80   # left
ROBOT_Y_MAX = 80    # right
ROBOT_Z = 0         # height

# Example camera object center
cx = 320
cy = 240

target_x = ROBOT_X
target_y = map_value(cx, 0, CAM_W, ROBOT_Y_MIN, ROBOT_Y_MAX)
target_z = ROBOT_Z

print(f"Camera: x={cx}, y={cy}")
print(f"Robot target: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")
