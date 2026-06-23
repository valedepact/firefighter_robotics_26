from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timestep)

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")

motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

K_VERTICAL_THRUST = 68.5  ##with this thrust, the drone lifts
K_VERTICAL_OFFSET = 0.6  #vertical offset where thhe robot targets to stabilize itself 
K_VERTICAL_P = 3.0 #P constant of the vertical PID
K_ROLL_P = 50.0  #P constant of the roll PID
K_PITCH_P = 30.0  # P constant of the pitch PID
TARGET_ALTITUDE = 1.5

step_count = 0
print_interval = max(1, int(1000 / timestep))  # roughly once per second

while robot.step(timestep) != -1:
    roll, pitch, _ = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_rate, pitch_rate, _ = gyro.getValues()

    roll_input = K_ROLL_P * max(-1.0, min(1.0, roll)) + roll_rate
    pitch_input = K_PITCH_P * max(-1.0, min(1.0, pitch)) + pitch_rate

    alt_error = max(-1.0, min(1.0, TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET))
    vertical_input = K_VERTICAL_P * (alt_error ** 3)

    fl = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input      #front left
    fr = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input      #front right
    rl = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input      #REAR LEFT
    rr = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input      #rear right

    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)

    step_count += 1
    if step_count % print_interval == 0:
        print(f"alt={altitude:.3f} roll={roll:.3f} pitch={pitch:.3f} fl={fl:.1f} fr={fr:.1f} rl={rl:.1f} rr={rr:.1f}")