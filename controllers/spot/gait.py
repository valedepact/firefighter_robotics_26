"""
gait.py — Open-loop CPG (central pattern generator) trot gait for Spot.

Each leg's hip-rotation and elbow(knee) joints are driven by phase-offset
sinusoids around Cyberbotics' own validated standing pose (the exact
values their reference spot_moving_demo.c uses for stand_up()). Diagonal
leg pairs (front-left+rear-right, front-right+rear-left) are 180 degrees
out of phase — the standard trot pattern.

This has no closed-loop balance feedback (no IMU-based stabilization) —
it is an open-loop pattern generator, not a full dynamically-balanced gait
controller. Expect to need live tuning in Webots; I cannot run this to
verify stability from here.
"""

import math

GAIT_PERIOD           = 80     # steps per full stride cycle
ROT_AMPLITUDE         = 0.3    # rad — hip swing amplitude at full forward_speed
ELBOW_LIFT_AMPLITUDE  = 0.5    # rad — knee lift added only during each leg's swing half

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
        gait = Gait(robot)
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
        Advance the gait by one simulation step.

        forward_speed : 0.0 (frozen, standing) .. 1.0 (full cadence)
        turn_rate     : -1.0 (turn left) .. +1.0 (turn right) — biases stride
                        amplitude side-to-side, skid-steer style.
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
