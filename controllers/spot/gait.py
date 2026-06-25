"""
gait.py — Enhanced Open-loop CPG trot gait for Spot with Uneven Terrain Following.

UPGRADES FOR UNEVEN GROUND:
- Terrain height sampling (matches ElevationGrid sine-based bumps)
- Dynamic body Z adjustment so Spot "rides" the terrain instead of clipping/floating
- Slightly increased leg lift amplitude for better clearance on hills
- Optional slope compensation stub (easy to extend with normal vector)

This keeps the reliable supervisor-driven translation (as in original + Cyberbotics reference)
while making movement look natural on bumpy wildfire terrain.

Usage remains identical:
    gait = Gait(robot)
    gait.step(forward_speed, turn_rate)
"""

import math

GAIT_PERIOD           = 80
ROT_AMPLITUDE         = 0.32    # slightly increased for terrain
ELBOW_LIFT_AMPLITUDE  = 0.55    # increased lift for hills/valleys

STEP_DISTANCE_PER_S   = 0.35
TURN_RATE_PER_S       = 1.2

# Terrain parameters (MUST MATCH the ElevationGrid in wildfire.wbt)
TERRAIN_AMPLITUDE     = 0.6
TERRAIN_FREQUENCY     = 0.08
TERRAIN_BASE_Z        = 0.2     # ElevationGrid translation Z
LEG_CLEARANCE         = 0.45    # extra height so feet clear bumps

STAND_POSE = {
    "front left":  (-0.1, 0.0, 0.0),
    "front right": ( 0.1, 0.0, 0.0),
    "rear left":   (-0.1, 0.0, 0.0),
    "rear right":  ( 0.1, 0.0, 0.0),
}

_LEG_PHASE = {
    "front left":  0.0,
    "rear right":  0.0,
    "front right": math.pi,
    "rear left":   math.pi,
}

_LEFT_LEGS  = ("front left", "rear left")
_RIGHT_LEGS = ("front right", "rear right")

_LEG_NAMES = ("front left", "front right", "rear left", "rear right")
_JOINTS    = ("shoulder abduction motor", "shoulder rotation motor", "elbow motor")


class Gait:
    def __init__(self, robot):
        self._motors = {}
        for leg in _LEG_NAMES:
            for joint in _JOINTS:
                name = f"{leg} {joint}"
                self._motors[name] = robot.getDevice(name)

        self._phase = 0.0
        self._robot          = robot
        self._timestep_s      = robot.getBasicTimeStep() / 1000.0
        self._self_node       = robot.getSelf()
        self._translation     = self._self_node.getField("translation")
        self._rotation_field  = self._self_node.getField("rotation")

    def stand_pose(self):
        targets = {}
        for leg, (abd, rot, elbow) in STAND_POSE.items():
            targets[f"{leg} shoulder abduction motor"] = abd
            targets[f"{leg} shoulder rotation motor"]  = rot
            targets[f"{leg} elbow motor"]               = elbow
        return targets

    def hold_pose(self, targets):
        for name, position in targets.items():
            self._motors[name].setPosition(position)

    def step(self, forward_speed, turn_rate=0.0):
        self._phase += (2 * math.pi / GAIT_PERIOD) * forward_speed

        for leg, (abd, _, _) in STAND_POSE.items():
            theta = self._phase + _LEG_PHASE[leg]

            side_scale = 1.0
            if leg in _LEFT_LEGS:
                side_scale = max(0.0, 1.0 - turn_rate)
            elif leg in _RIGHT_LEGS:
                side_scale = max(0.0, 1.0 + turn_rate)

            rotation = ROT_AMPLITUDE * side_scale * math.sin(theta)
            lift     = ELBOW_LIFT_AMPLITUDE * side_scale * max(0.0, math.sin(theta))

            self._motors[f"{leg} shoulder abduction motor"].setPosition(abd)
            self._motors[f"{leg} shoulder rotation motor"].setPosition(rotation)
            self._motors[f"{leg} elbow motor"].setPosition(lift)

        self._advance_body(forward_speed, turn_rate)

    def _get_terrain_height(self, x, y):
        """
        Compute terrain height at (x, y) — MUST MATCH ElevationGrid formula.
        Multi-octave sine for rolling hills + localized bumps.
        """
        h = 0.0
        amp = TERRAIN_AMPLITUDE
        freq = TERRAIN_FREQUENCY
        for _ in range(3):  # 3 octaves
            h += amp * math.sin(x * freq) * math.sin(y * freq * 0.9)
            h += amp * 0.6 * math.sin(x * freq * 1.7 + 1.2) * math.cos(y * freq * 1.3)
            amp *= 0.5
            freq *= 2.0

        # Localized bump zone (match generator)
        if 20 < x < 35 and 25 < y < 40:
            h += 0.4 * math.sin((x-27)*0.8) * math.cos((y-32)*0.7)

        return h

    def _advance_body(self, forward_speed, turn_rate):
        """
        Supervisor-driven movement + terrain following.
        Body Z is dynamically adjusted to ride the terrain.
        """
        axis_angle = self._rotation_field.getSFRotation()
        yaw = axis_angle[3] if axis_angle[2] >= 0 else -axis_angle[3]

        # Apply turn
        yaw += turn_rate * TURN_RATE_PER_S * self._timestep_s
        self._rotation_field.setSFRotation([0, 0, 1, yaw])

        # Forward movement
        distance = forward_speed * STEP_DISTANCE_PER_S * self._timestep_s
        x, y, z = self._translation.getSFVec3f()
        x += distance * math.sin(yaw)
        y += distance * math.cos(yaw)

        # === TERRAIN FOLLOWING ===
        terrain_h = self._get_terrain_height(x, y)
        target_z = TERRAIN_BASE_Z + terrain_h + LEG_CLEARANCE

        # Smooth Z transition (prevents jerky motion)
        z = z * 0.7 + target_z * 0.3

        self._translation.setSFVec3f([x, y, z])

    # Optional future extension: compute local slope for body pitch/roll
    # def _get_terrain_normal(self, x, y): ...
