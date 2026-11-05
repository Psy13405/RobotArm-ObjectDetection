import serial
import time
import math

# --- HARDWARE SETUP ---
USB_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- PHYSICAL MEASUREMENTS (Crucial for accurate IK) ---
# Measure the distance between the center of the rotating joints in millimeters
L1 =  75.0# Length from Shoulder joint to Elbow joint
L2 = 85.0 # Length from Elbow joint to the tip of the arm

try:
    port = serial.Serial(USB_PORT, BAUD_RATE, timeout=0.1)
    print(f"Port {USB_PORT} opened successfully.")
except Exception as e:
    print(f"Error opening port: {e}")
    exit()

def calculate_ik(x, y, z):
    """Translates 3D spatial coordinates into 0-1000 servo positions."""
    # 1. Base Pan Angle (Looking at the target from above)
    base_rad = math.atan2(y, x)
    
    # 2. 2D Distances
    r = math.hypot(x, y) # Flat distance on the table
    D = math.hypot(r, z) # Diagonal distance from shoulder to target
    
    # Safety Check: Prevent the math from crashing if the target is too far away
    if D > (L1 + L2):
        print(f"  [WARNING] Target ({x},{y},{z}) is out of reach! Stretching to maximum limit.")
        D = L1 + L2 - 0.1 
        
    # 3. Elbow Angle (Law of Cosines)
    elbow_rad = math.acos((L1**2 + L2**2 - D**2) / (2 * L1 * L2))
    
    # 4. Shoulder Angle
    alpha = math.atan2(z, r)
    beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
    shoulder_rad = alpha + beta 
    
    # --- Convert to Servo Positions ---
    base_deg = math.degrees(base_rad)
    shoulder_deg = math.degrees(shoulder_rad)
    elbow_deg = math.degrees(elbow_rad)
    
    # Map degrees to the 0-1000 scale (0.24 degrees per unit)
    base_pos = int(500 - (base_deg / 0.24))
    shoulder_pos = int(500 + (shoulder_deg - 90) / 0.24)
    
    # The elbow often needs to be inverted depending on how you bolted the bracket on.
    # If the elbow bends backwards, change the minus sign to a plus sign here:
    elbow_pos = int(500 - ((180 - elbow_deg) / 0.24)) 
    
    # Clamp to safe motor limits
    base_pos = max(0, min(1000, base_pos))
    shoulder_pos = max(0, min(1000, shoulder_pos))
    elbow_pos = max(0, min(1000, elbow_pos))
    
    return base_pos, shoulder_pos, elbow_pos

def move_servo(servo_id, angle, time_ms):
    """Sends the raw hex packet to a specific servo."""
    length = 7
    cmd = 1 
    angle_lsb = angle & 0xFF
    angle_msb = (angle >> 8) & 0xFF
    time_lsb = time_ms & 0xFF
    time_msb = (time_ms >> 8) & 0xFF
    checksum = (~(servo_id + length + cmd + angle_lsb + angle_msb + time_lsb + time_msb)) & 0xFF
    
    packet = bytearray([0x55, 0x55, servo_id, length, cmd, angle_lsb, angle_msb, time_lsb, time_msb, checksum])
    port.reset_input_buffer()
    port.write(packet)

def reach_for_target(x, y, z, move_time_ms):
    """The Master Function: Calculates IK and moves all joints simultaneously."""
    print(f"\nCommanding arm to coordinate: X:{x}, Y:{y}, Z:{z}")
    
    # Run the math
    base_pos, shoulder_pos, elbow_pos = calculate_ik(x, y, z)
    print(f"  Calculated Servo Angles -> Base: {base_pos}, Shoulder: {shoulder_pos}, Elbow: {elbow_pos}")
    
    # Move the hardware
    move_servo(2, base_pos, move_time_ms)
    move_servo(3, shoulder_pos, move_time_ms)
    move_servo(1, elbow_pos, move_time_ms)
    
    # Pause the script while the arm physically moves
    time.sleep(move_time_ms / 1000.0)

# --- THE 3D MOVEMENT SEQUENCE ---
print("\nInitializing Spatial Control Sequence...")

# # 1. Move to a safe "Home" position (Straight up)
# reach_for_target(0, 0, L1+L2, 1000)
# time.sleep(2)

# # 2. Reach forward and down (like grabbing something off the table)
# reach_for_target(150, 0, 50, 1000)
# time.sleep(2)

# # 3. Sweep to the right, keeping the exact same height and distance
# reach_for_target(100, 100, 50, 1000)
# time.sleep(2)

# # 4. Reach up high and to the left
# reach_for_target(-100, 100, 180, 1000)
# time.sleep(2)

# # 5. Return to safe Home
# reach_for_target(0, 0, L1+L2, 1000)


#reach left
reach_for_target(130, 50, 0, 500)
time.sleep(2)




print("\nSequence Complete!")
# port.close()
