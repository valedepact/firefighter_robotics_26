from controller import Robot, Keyboard
import cv2
import numpy as np

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Devices
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

camera = robot.getDevice("camera")
camera.enable(timestep)
width, height = camera.getWidth(), camera.getHeight()
print(f"✅ Camera enabled ({width}x{height})")

keyboard = Keyboard()
keyboard.enable(timestep)

# Motors
fl = robot.getDevice("front left propeller")
fr = robot.getDevice("front right propeller")
rl = robot.getDevice("rear left propeller")
rr = robot.getDevice("rear right propeller")

for m in [fl, fr, rl, rr]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# === Stronger Takeoff Parameters ===
TARGET_ALTITUDE = 8.0
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 1.0
K_VERTICAL_P = 4.5        # Stronger vertical gain
K_ROLL_P = 50.0
K_PITCH_P = 35.0
K_YAW_P = 5.0

FIRE_LOWER = np.array([0, 60, 180])
FIRE_UPPER = np.array([35, 255, 255])

state = "TAKEOFF"
step = 0
min_detection_alt = 4.0

print("=== Firefighter Drone STARTED - TAKEOFF MODE ===")

while robot.step(timestep) != -1:
    step += 1
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    # Keyboard override
    key = keyboard.getKey()
    if key == Keyboard.UP:    TARGET_ALTITUDE += 0.2
    if key == Keyboard.DOWN:  TARGET_ALTITUDE = max(1.0, TARGET_ALTITUDE - 0.2)

    # === Fire Detection (only when flying high) ===
    if step % 10 == 0 and altitude > min_detection_alt and state != "TAKEOFF":
        img = np.frombuffer(camera.getImage(), np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, FIRE_LOWER, FIRE_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            area = max(cv2.contourArea(c) for c in contours)
            if area > 120 and state == "PATROL":
                print(f"🔥 FIRE DETECTED! Area={area:.0f}")
                state = "GOTO_FIRE"

    # === State Logic ===
    forward_input = 0.0

    if state == "TAKEOFF":
        TARGET_ALTITUDE = 8.0
        if altitude > 6.0:
            state = "PATROL"
            print("✅ Reached safe altitude → Switching to PATROL")

    elif state == "GOTO_FIRE":
        TARGET_ALTITUDE = 6.0
        forward_input = 9.0

    # === PID Control ===
    vertical_input = K_VERTICAL_P * (TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET)
    
    roll_input  = K_ROLL_P * roll + 1.8 * roll_rate
    pitch_input = K_PITCH_P * pitch + 1.8 * pitch_rate + forward_input
    yaw_input   = K_YAW_P * yaw_rate

    # Motor mixing
    front_left  = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input + yaw_input
    front_right = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input - yaw_input
    rear_left   = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
    rear_right  = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

    # Clamp
    front_left  = max(0.0, min(200.0, front_left))
    front_right = max(0.0, min(200.0, front_right))
    rear_left   = max(0.0, min(200.0, rear_left))
    rear_right  = max(0.0, min(200.0, rear_right))

    fl.setVelocity(front_left)
    fr.setVelocity(-front_right)
    rl.setVelocity(-rear_left)
    rr.setVelocity(rear_right)

    if step % 40 == 0:
        print(f"State: {state} | alt={altitude:.2f}m | target={TARGET_ALTITUDE:.1f}m | pitch={pitch:.3f}")