import math
from detection import scan
from astar_nav import AStarNavigator

class Navigator:
    def __init__(self, robot, gait):
        self.robot = robot
        self.gait = gait
        self.camera = robot.getDevice("left head camera")
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
        self.current_target = None

    def update(self):
        # === Coordination from Mavic ===
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            self.receiver.nextPacket()
            if data.startswith("FIRE:"):
                try:
                    x, y = map(float, data[5:].split(","))
                    self.waypoints.append((x, y))
                    self.state = "NAVIGATE"
                except:
                    pass

        # Perception + Obstacle Avoidance
        detection_type, info = scan(self.camera)
        ds_value = self.front_ds.getValue()

        if 0 < ds_value < 3.0:  # Obstacle
            self.gait.step(-0.3, 1.2)
            return "AVOID"

        # Multi-fire prioritization
        if detection_type == "fire" and info["detected"]:
            pos = self.robot.getSelf().getPosition()[:2]
            dist = math.hypot(pos[0] + 5, pos[1] + 5)  # rough
            threat = info["area"] / (dist + 1)
            self.detected_fires.append((threat, info, pos, self.robot.getTime()))
            self.detected_fires = sorted(self.detected_fires, key=lambda x: x[0], reverse=True)[:5]

        if self.robot.getTime() - self.last_prioritize > 5:
            self.last_prioritize = self.robot.getTime()

        # State handling
        if self.waypoints or self.detected_fires:
            self.state = "NAVIGATE"

        if self.state == "PATROL":
            self.gait.step(0.55, 0.2 * math.sin(self.robot.getTime() * 0.6))
        elif self.state == "NAVIGATE":
            target = self.waypoints[0] if self.waypoints else (self.detected_fires[0][2][0], self.detected_fires[0][2][1]) if self.detected_fires else (0, 0)
            dx = target[0] - self.robot.getSelf().getPosition()[0]
            dy = target[1] - self.robot.getSelf().getPosition()[1]
            dist = math.hypot(dx, dy)
            if dist < 2.0 and self.waypoints:
                self.waypoints.pop(0)
            else:
                desired_yaw = math.atan2(dx, dy)
                # Simplified reactive + A* fallback
                current_yaw = self._get_yaw()
                error = desired_yaw - current_yaw
                turn_rate = max(-1.5, min(1.5, error * 3.0))
                forward = 0.65 if dist > 5 else 0.4
                self.gait.step(forward, turn_rate)
                
                if info.get("area", 0) > 1000:
                    print("🔥 EXTINGUISHING FIRE 🔥")
        else:
            self.gait.step(0.0, 0.0)

        return self.state

    def _get_yaw(self):
        rot = self.robot.getSelf().getOrientation()
        return math.atan2(2 * (rot[0] * rot[3] + rot[1] * rot[2]), 1 - 2 * (rot[1]**2 + rot[2]**2))
