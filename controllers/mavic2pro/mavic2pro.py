from controller import Supervisor
from detection import scan
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timestep)
emitter = robot.getDevice("coordination emitter")

print("🚁 Mavic 2 Pro — Scouting + Fire Reporting Active")

t = 0
while robot.step(timestep) != -1:
    t += timestep / 1000.0
    detection_type, info = scan(camera)
    
    if detection_type == "fire" and info["detected"]:
        pos = robot.getSelf().getPosition()
        emitter.send(f"FIRE:{pos[0]:.1f},{pos[1]:.1f}")
        print(f"🚁 FIRE REPORTED @ ({pos[0]:.1f}, {pos[1]:.1f})")
    
    # Simple scouting pattern (lawnmower)
    x = 10 * math.sin(t * 0.2)
    y = t * 0.5 % 30 - 15
    robot.getSelf().getField("translation").setSFVec3f([x, y, 15])
