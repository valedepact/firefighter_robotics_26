from controller import Robot, Keyboard

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
print("✅ Camera enabled - Double-click 'camera' in Scene Tree to view live feed")

# === Keyboard (for manual altitude control) ===
keyboard = Keyboard()
keyboard.enable(timestep)

# === Motors ===
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")

for motor in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# === Tuned Parameters ===
TARGET_ALTITUDE = 2.0          # Increased target
K_VERTICAL_THRUST = 68.5
K_VERTICAL_OFFSET = 0.6
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

print(f"=== Mavic2Pro controller STARTED - Target: {TARGET_ALTITUDE}m ===")

step = 0

while robot.step(timestep) != -1:
    step += 1
    
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    # === Keyboard control (Up/Down arrows to change altitude) ===
    key = keyboard.getKey()
    if key == Keyboard.UP:
        TARGET_ALTITUDE += 0.01
    elif key == Keyboard.DOWN:
        TARGET_ALTITUDE = max(0.5, TARGET_ALTITUDE - 0.01)

    # Soft flip protection
    if abs(roll) > 1.8 or abs(pitch) > 1.8:
        for m in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
            m.setVelocity(0.0)
        print(f"FLIPPED! roll={roll:.2f} pitch={pitch:.2f} | Motors stopped")
        continue

    # PID
    vertical_input = K_VERTICAL_P * (TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET)
    
    roll_input  = K_ROLL_P * roll  + roll_rate
    pitch_input = K_PITCH_P * pitch + pitch_rate

    # Correct motor mixing
    front_left  = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input
    front_right = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input
    rear_left   = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input
    rear_right  = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input

    # Clamp
    front_left  = max(0.0, min(200.0, front_left))
    front_right = max(0.0, min(200.0, front_right))
    rear_left   = max(0.0, min(200.0, rear_left))
    rear_right  = max(0.0, min(200.0, rear_right))

    # Apply
    front_left_motor.setVelocity(front_left)
    front_right_motor.setVelocity(-front_right)
    rear_left_motor.setVelocity(-rear_left)
    rear_right_motor.setVelocity(rear_right)

    # Status print every 0.5 seconds
    if step % 50 == 0:
        print(f"alt={altitude:.2f}m  target={TARGET_ALTITUDE:.2f}m  "
              f"roll={roll:.3f}  pitch={pitch:.3f}")