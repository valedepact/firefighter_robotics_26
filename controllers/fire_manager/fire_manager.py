from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("Fire Manager Started")

while robot.step(timestep) != -1:
    pass