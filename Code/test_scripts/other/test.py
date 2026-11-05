import serial
import time

# --- SETUP ---
# Windows: Keep as 'COM3' (or whatever your Device Manager says)
# Raspberry Pi: Change this to '/dev/ttyUSB0'
USB_PORT = '/dev/cu.usbmodem5B3E1210981'
BAUD_RATE = 115200

try:
    port = serial.Serial(USB_PORT, BAUD_RATE, timeout=0.1)
    print(f"Port {USB_PORT} opened successfully.")
except Exception as e:
    print(f"Error opening port: {e}")
    exit()

def move_servo(servo_id, angle, time_ms):
    """Sends the raw hex packet to a specific servo ID."""
    angle = max(0, min(1000, int(angle)))
    time_ms = max(0, min(30000, int(time_ms)))

    length = 7
    cmd = 1 # 0x01: Move Time Write
    
    angle_lsb = angle & 0xFF
    angle_msb = (angle >> 8) & 0xFF
    time_lsb = time_ms & 0xFF
    time_msb = (time_ms >> 8) & 0xFF
    
    checksum = (~(servo_id + length + cmd + angle_lsb + angle_msb + time_lsb + time_msb)) & 0xFF
    
    packet = bytearray([0x55, 0x55, servo_id, length, cmd, angle_lsb, angle_msb, time_lsb, time_msb, checksum])
    port.reset_input_buffer()
    port.write(packet)

def move_arm(base_pos, shoulder_pos, elbow_pos, move_time):
    """
    Commands all three joints to move simultaneously.
    Positional Range: 0 to 1000 (500 is dead center).
    """
    print(f"Moving to -> Base: {base_pos} | Shoulder: {shoulder_pos} | Elbow: {elbow_pos}")
    
    # Blast the commands out back-to-back
    move_servo(2, base_pos, move_time)
    move_servo(3, shoulder_pos, move_time)
    move_servo(1, elbow_pos, move_time)
    
    # Pause the Python script to let the physical motors finish their transit
    time.sleep(move_time / 1000.0)

def promptarm():
    user_input = input("Enter motor angles (base, shoulder, elbow) and time (ms) separated by commas (e.g., 500,500,500,1000): ")
    base, shoulder, elbow, move_time = map(int, user_input.split(','))
    move_arm(base, shoulder, elbow, move_time)



# time.sleep(1)

for x in range(0):
    # --- THE MOVEMENT CHOREOGRAPHY ---
    print("\nStarting 3-DOF Arm Sequence...")
    # move_arm(500, 0, 500, 300)
    move_arm(500, 500, 500, 300)
    # 1. Wake up and go to HOME position (Stand straight up)
    print("\n1. Snapping to Home Position...")
    move_arm(500, 500, 500, 300)
    time.sleep(0.5)

    # 2. Pan left and reach forward
    print("\n2. Panning right and reaching out...")

    move_arm(800, 200, 200, 500) 
    time.sleep(0.5)

    # 3. Pan right and reach higher
    print("\n3. Panning left and reaching up...")
    move_arm(300, 700, 800, 500)
    time.sleep(0.5)

    # 4. Return to center quickly
    print("\n4. Returning to center...")
    move_arm(500, 500, 500, 700)

print("\nSequence Complete!")

move_arm(500, 500, 500, 300)
