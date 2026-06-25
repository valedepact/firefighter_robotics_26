"""
spot.py — Main controller with full locomotion, navigation, coordination, extinguishing.
"""
from controller import Supervisor
from gait import Gait
from navigation import Navigator   # Updated version below
# from extinguish import Extinguisher  # Add if separate file
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Devices
camera = robot.getDevice("left head camera")  # or "camera"
camera.enable(timestep)
front_ds = robot.getDevice("front distance sensor")
front_ds.enable(timestep)
coord_receiver = robot.getDevice("coordination receiver")
coord_receiver.enable(timestep)

gait = Gait(robot)
navigator = Navigator(robot, gait)

print("🔥 Spot Firefighter — Full Autonomy Active on Uneven Terrain")

while robot.step(timestep) != -1:
    state = navigator.update()
    
    if robot.getTime() % 5 < timestep / 1000:
        pos = robot.getSelf().getPosition()
        print(f"[{robot.getTime():.1f}s] State: {state} | Pos: ({pos[0]:.1f}, {pos[1]:.1f})")
