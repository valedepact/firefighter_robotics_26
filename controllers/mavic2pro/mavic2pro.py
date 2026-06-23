from controller import Robot, Keyboard
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# === Devices ===
imu = robot.getDevice("inertial unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

# === Camera ===
camera = robot.getDevice("camera")
camera.enable(timestep)
width = camera.getWidth()
height = camera.getHeight()
print(f"✅ Camera enabled ({width}x{height}) - Double-click camera in Scene Tree to view")

# === Keyboard (for testing) ===
keyboard = Keyboard()
keyboard.enable(timestep)

# === Motors ===
fl_motor = robot.getDevice("front left propeller")
fr_motor = robot.getDevice("front right propeller")
rl_motor = robot.getDevice("rear left propeller")
rr_motor = robot.getDevice("rear right propeller")

for motor in [fl_motor, fr_motor, rl_motor, rr_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# === Parameters ===
TARGET_ALTITUDE = 8.0          # Good patrol altitude
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

# Fire detection parameters
FIRE_LOWER = np.array([0, 50, 200])      # HSV range for fire
FIRE_UPPER = np.array([30, 255, 255])

state = "PATROL"   # PATROL, GOTO_FIRE, EXTINGUISH, RETURN
target_pos = None

print(f"=== Firefighter Drone STARTED - Target Alt: {TARGET_ALTITUDE}m ===")

step = 0

while robot.step(timestep) != -1:
    step += 1
    
    # Read sensors
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_rate, pitch_rate, _ = gyro.getValues()

    # Keyboard override
    key = keyboard.getKey()
    if key == Keyboard.UP:    TARGET_ALTITUDE += 0.05
    if key == Keyboard.DOWN:  TARGET_ALTITUDE = max(1.0, TARGET_ALTITUDE - 0.05)

    # === Fire Detection ===
    if step % 8 == 0:   # process image every ~80ms
        image = camera.getImage()
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, FIRE_LOWER, FIRE_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours and max(cv2.contourArea(c) for c in contours) > 50:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] > 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                
                # Simple bearing estimation
                center_error_x = (cx - width//2) / (width//2)
                
                if state == "PATROL":
                    print("🔥 FIRE DETECTED!")
                    state = "GOTO_FIRE"
                    # Approximate target (will improve later)
                    target_pos = gps.getValues()

    # === State Machine ===
    if state == "PATROL":
        # Simple patrol pattern (you can improve this)
        pass

    elif state == "GOTO_FIRE":
        # Go toward fire (basic version)
        TARGET_ALTITUDE = 6.0
        # Add horizontal movement here later

    # === Attitude Control ===
    vertical_input = K_VERTICAL_P * (TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET)
    
    roll_input  = K_ROLL_P * roll  + roll_rate
    pitch_input = K_PITCH_P * pitch + pitch_rate

    front_left  = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input
    front_right = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input
    rear_left   = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input
    rear_right  = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input

    # Clamp and apply
    for val in [front_left, front_right, rear_left, rear_right]:
        val = max(0.0, min(200.0, val))

    fl_motor.setVelocity(front_left)
    fr_motor.setVelocity(-front_right)
    rl_motor.setVelocity(-rear_left)
    rr_motor.setVelocity(rear_right)

    if step % 50 == 0:
        print(f"State: {state} | alt={altitude:.2f}m | target={TARGET_ALTITUDE:.1f}m")