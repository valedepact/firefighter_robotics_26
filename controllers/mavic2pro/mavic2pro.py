from controller import Supervisor, Keyboard
import cv2
import numpy as np
from detection import scan
from flight import FlightController
from navigation import Navigator
from extinguish import Extinguisher

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

fc = FlightController(robot)
nav = Navigator()
ext = Extinguisher(robot)   # once at startup

# === Devices ===
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

camera = robot.getDevice("camera")
camera.enable(timestep)
print(f"✅ Camera enabled ({camera.getWidth()}x{camera.getHeight()})")

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

# Gentle parameters
TARGET_ALTITUDE = 7.0
K_VERTICAL_THRUST = 68.5
K_VERTICAL_P = 2.5
K_ROLL_P = 30.0
K_PITCH_P = 20.0

state = "TAKEOFF"
step = 0
thrust_ramp = 0.0

# Force position and orientation
self_node = robot.getSelf()
if self_node:
    self_node.getField("translation").setSFVec3f([-20, -20, 2.0])
    self_node.getField("rotation").setSFRotation([0, 0, 1, 0])

print("=== CLEAN & GENTLE TAKEOFF ===")

while robot.step(timestep) != -1:
    step += 1
    roll, pitch, yaw = imu.getRollPitchYaw()
    altitude = gps.getValues()[2]
    roll_rate, pitch_rate, yaw_rate = gyro.getValues()

    # Keyboard help
    key = keyboard.getKey()
    if key == Keyboard.UP:    TARGET_ALTITUDE += 0.4
    if key == Keyboard.DOWN:  TARGET_ALTITUDE = max(1.0, TARGET_ALTITUDE - 0.4)

    # Smooth ramp
    if state == "TAKEOFF":
        thrust_ramp = min(thrust_ramp + 0.01, 1.0)
        current_thrust = K_VERTICAL_THRUST * thrust_ramp
    else:
        current_thrust = K_VERTICAL_THRUST
    
    # Update flight controller
    if fc.update(imu, gps, gyro):
        state = "PATROL"


    # State transition
    if state == "TAKEOFF" and altitude > 6.0:
        state = "PATROL"
        print("✅ Takeoff successful → PATROL")

    # Very simple control
    vertical_input = K_VERTICAL_P * (TARGET_ALTITUDE - altitude + 1.2)
    roll_input  = K_ROLL_P * roll + 1.0 * roll_rate
    pitch_input = K_PITCH_P * pitch + 1.0 * pitch_rate
    yaw_input   = 3.0 * yaw_rate

    # Motor mixing
    fl_val = current_thrust + vertical_input - roll_input + pitch_input + yaw_input
    fr_val = current_thrust + vertical_input + roll_input + pitch_input - yaw_input
    rl_val = current_thrust + vertical_input - roll_input - pitch_input + yaw_input
    rr_val = current_thrust + vertical_input + roll_input - pitch_input - yaw_input

    fl.setVelocity(fl_val)
    fr.setVelocity(-fr_val)
    rl.setVelocity(-rl_val)
    rr.setVelocity(rr_val)

    if step % 30 == 0:
        print(f"State: {state} | alt={altitude:.2f}m | roll={roll:.3f} | pitch={pitch:.3f}")


kind, result = scan(camera)
if kind == "fire":
    print(f"Fire at offset ({result['offset_x']:.2f}, {result['offset_y']:.2f})")
    state = "NAVIGATE"

# In PATROL state:
kind, result = scan(camera)
if kind == "fire":
    nav.set_fire_position(*gps.getValues()[:2])
    state = "NAVIGATE"
elif nav.patrol(gps, fc):
    pass  # grid done, keep patrolling

# In NAVIGATE state:
if nav.fly_to_fire(gps, fc, result):
    state = "EXTINGUISH"

# In EXTINGUISH state:
if ext.update(gps, fc, fire_def_name="FIRE_1"):
    ext.reset()
    state = "RETURN"