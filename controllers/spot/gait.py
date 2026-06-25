"""
gait.py — Open-loop CPG (central pattern generator) trot gait for Spot.

Each leg's hip-rotation and elbow(knee) joints are driven by phase-offset
sinusoids around Cyberbotics' own validated standing pose (the exact
values their reference spot_moving_demo.c uses for stand_up()). Diagonal
leg pairs (front-left+rear-right, front-right+rear-left) are 180 degrees
out of phase — the standard trot pattern.

Leg motion alone does not reliably translate the body — without tuned
foot/ground friction or closed-loop balance, an open-loop joint sine can
animate the legs while the torso stays put. To guarantee visible forward
motion (the same approach used by Cyberbotics' own forest-firefighters
reference solution), the gait also directly advances the robot's
`translation` field via the supervisor each step, along the body's
current heading (read from the IMU) and scaled by forward_speed. This is
a supervisor-driven walk, not a physics-driven one — legs are cosmetic,
motion is guaranteed.
"""

import math

GAIT_PERIOD           = 80     # steps per full stride cycle
ROT_AMPLITUDE         = 0.3    # rad — hip swing amplitude at full forward_speed
ELBOW_LIFT_AMPLITUDE  = 0.5    # rad — knee lift added only during each leg's swing half

# How far the body advances per step at forward_speed=1.0, and how fast it
# turns at turn_rate=1.0 — tuned to roughly match the trot's visual cadence.
STEP_DISTANCE_PER_S  = 0.35   # m/s of forward travel at full forward_speed
TURN_RATE_PER_S      = 1.2    # rad/s of yaw change at turn_rate=±1.0

# Validated standing pose (front left, front right, rear left, rear right),
# each (abduction, rotation, elbow) — taken from Cyberbotics' own
# spot_moving_demo.c stand_up() values.
STAND_POSE = {
    "front left":  (-0.1, 0.0, 0.0),
    "front right": ( 0.1, 0.0, 0.0),
    "rear left":   (-0.1, 0.0, 0.0),
    "rear right":  ( 0.1, 0.0, 0.0),
}

# Diagonal pairs 180 degrees out of phase
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
    """
    Usage
    -----
        gait = Gait(robot)             # robot must be a Supervisor
        ...
        gait.step(forward_speed, turn_rate)   # call every simulation step
    """

    def __init__(self, robot):
        self._motors = {}
        for leg in _LEG_NAMES:
            for joint in _JOINTS:
                name = f"{leg} {joint}"
                self._motors[name] = robot.getDevice(name)

        self._phase = 0.0

        # Supervisor handle + translation field for direct body movement.
        self._robot          = robot
        self._timestep_s      = robot.getBasicTimeStep() / 1000.0
        self._self_node       = robot.getSelf()
        self._translation     = self._self_node.getField("translation")

    def stand_pose(self):
        """Target motor positions for the validated standing pose (no gait motion)."""
        targets = {}
        for leg, (abd, rot, elbow) in STAND_POSE.items():
            targets[f"{leg} shoulder abduction motor"] = abd
            targets[f"{leg} shoulder rotation motor"]  = rot
            targets[f"{leg} elbow motor"]               = elbow
        return targets

    def hold_pose(self, targets):
        """Drive all 12 motors directly to the given {motor_name: position} targets."""
        for name, position in targets.items():
            self._motors[name].setPosition(position)

    def step(self, forward_speed, turn_rate=0.0):
        """
        Advance the gait by one simulation step, and physically move the
        body forward along its current heading.

        forward_speed : 0.0 (frozen, standing) .. 1.0 (full cadence)
        turn_rate     : -1.0 (turn left) .. +1.0 (turn right) — biases stride
                        amplitude side-to-side, skid-steer style, and also
                        drives the body's actual yaw rotation.
        """
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

    def _advance_body(self, forward_speed, turn_rate):
        """
        Directly move the robot's translation/rotation fields forward along
        its current heading, the same supervisor-driven approach Cyberbotics'
        forest-firefighters reference solution uses for Spot.
        """
        rotation_field = self._self_node.getField("rotation")
        axis_angle     = rotation_field.getSFRotation()   # (x, y, z, angle)
        yaw            = axis_angle[3] if axis_angle[2] >= 0 else -axis_angle[3]

        # Turn first, so this step's forward motion follows the new heading.
        yaw += turn_rate * TURN_RATE_PER_S * self._timestep_s
        rotation_field.setSFRotation([0, 0, 1, yaw])

        distance = forward_speed * STEP_DISTANCE_PER_S * self._timestep_s
        x, y, z = self._translation.getSFVec3f()
        x += distance * math.sin(yaw)
        y += distance * math.cos(yaw)
        self._translation.setSFVec3f([x, y, z])
