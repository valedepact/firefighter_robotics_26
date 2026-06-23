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

# Parameters
TARGET_ALTITUDE = 7.0
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.2
K_ROLL_P = 50.0
K_PITCH_P = 30.0
K_YAW_P = 4.0

# Fire detection
FIRE_LOWER = np.array([0, 60, 180])
FIRE_UPPER = np.array([35, 255, 255])

state = "PATROL"
step = 0
fire_detected_time = 0

print("=== Firefighter Drone STARTED ===")

while robot.step(timestep) != -1:
    step += 1
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    # Keyboard altitude adjustment
    key = keyboard.getKey()
    if key == Keyboard.UP:    TARGET_ALTITUDE += 0.1
    if key == Keyboard.DOWN:  TARGET_ALTITUDE = max(1.0, TARGET_ALTITUDE - 0.1)

    # === Fire Detection (every 8 steps) ===
    if step % 8 == 0:
        img = np.frombuffer(camera.getImage(), np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, FIRE_LOWER, FIRE_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 80 and state == "PATROL":
                print(f"🔥 FIRE DETECTED! Area = {area:.0f}")
                state = "GOTO_FIRE"
                fire_detected_time = step

    # === State Machine ===
    forward_input = 0.0
    yaw_input = 0.0

    if state == "GOTO_FIRE":
        TARGET_ALTITUDE = 6.0
        forward_input = 12.0          # move forward
        # Simple yaw towards center of image can be added later

    # === PID Control ===
    vertical_input = K_VERTICAL_P * (TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET)
    
    roll_input  = K_ROLL_P * roll  + 1.5 * roll_rate
    pitch_input = K_PITCH_P * pitch + 1.5 * pitch_rate + forward_input   # forward movement via pitch
    yaw_input   = K_YAW_P * yaw_rate   # basic stabilization

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

    # Status
    if step % 60 == 0:
        print(f"State: {state} | alt={altitude:.2f}m | target={TARGET_ALTITUDE:.1f}m | pitch={pitch:.3f}")