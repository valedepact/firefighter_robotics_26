import math
from detection import scan
from astar_nav import AStarNavigator

class Navigator:
    def __init__(self, robot, gait):
        self.robot = robot
        self.gait = gait
        self.camera = robot.getDevice("camera")
        self.camera.enable(2 * robot.getBasicTimeStep())
        self.front_ds = robot.getDevice("front distance sensor")
        self.front_ds.enable(2 * robot.getBasicTimeStep())
        self.receiver = robot.getDevice("coordination receiver")
        self.receiver.enable(2 * robot.getBasicTimeStep())
        self.state = "PATROL"
        self.waypoints = []
        self.detected_fires = []
        self.last_prioritize = 0
        self.astar = AStarNavigator()

    def update(self):
        # Coordination
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            self.receiver.nextPacket()
            if data.startswith("FIRE:"):
                try:
                    x, y = map(float, data[5:].split(","))
                    self.waypoints.append((x, y))
                    self.state = "NAVIGATE"
                except: pass

        detection_type, info = scan(self.camera)
        ds_value = self.front_ds.getValue()

        # Obstacle Avoidance
        if 0 < ds_value < 3.0:
            self.gait.step(-0.3, 1.2)
            return "AVOID"

        # Multi-fire prioritization
        if detection_type == "fire" and info["detected"]:
            pos = self.robot.getSelf().getPosition()[:2]
            dist = math.hypot(pos[0] - (-5), pos[1] - (-5))  # approx
            threat = info["area"] / (dist + 1)
            self.detected_fires.append((threat, info, pos, self.robot.getTime()))
            self.detected_fires = sorted(self.detected_fires, reverse=True)[:5]

        if self.robot.getTime() - self.last_prioritize > 5:
            self.detected_fires.sort(key=lambda x: x[0], reverse=True)
            self.last_prioritize = self.robot.getTime()

        # State logic
        if self.waypoints or self.detected_fires:
            self.state = "NAVIGATE"

        if self.state == "PATROL":
            self.gait.step(0.55, 0.2 * math.sin(self.robot.getTime() * 0.6))
        elif self.state == "NAVIGATE":
            target = self.waypoints[0] if self.waypoints else self.detected_fires[0][1] if self.detected_fires else (0,0)
            # A* or reactive steering...
            self.gait.step(0.6, 0.0)  # placeholder — extend with full A*
            if len(self.waypoints) > 0 and math.hypot(target[0]-self.robot.getSelf().getPosition()[0], target[1]-self.robot.getSelf().getPosition()[1]) < 2:
                self.waypoints.pop(0)
        elif detection_type == "fire":
            offset_x = info.get("offset_x", 0)
            self.gait.step(0.6, -offset_x * 2.0)
            if info.get("area", 0) > 1200:
                print("🔥 EXTINGUISHING 🔥")

        return self.state
