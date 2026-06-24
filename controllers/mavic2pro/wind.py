"""
wind.py — Live-controllable wind disturbance for the Mavic 2 Pro.

Applied as a real physical force on the drone's body (via Node.addForce),
not a fake bias in the flight controller — the existing attitude PID has
to genuinely counteract it, same as a real drone would.
"""

import math

from controller import Keyboard

WIND_SPEED_STEP = 0.5    # newtons added/removed per keypress
MAX_WIND_SPEED  = 8.0    # newtons — cap so wind can't overpower the drone entirely
WIND_TURN_STEP  = math.radians(10)  # radians per keypress


class WindController:
    """
    Tracks wind speed/direction and turns it into a world-frame force vector.

    Usage
    -----
        wind = WindController()
        ...
        while robot.step(timestep) != -1:
            wind.update(keyboard)
            drone_body.addForce(wind.force_vector(), False)   # every step
    """

    def __init__(self):
        self.speed     = 0.0   # newtons — starts off, user turns it on
        self.direction = 0.0   # radians, world XY-plane (0 = +X axis)
        print("💨 Wind controls: PAGE UP/DOWN = speed, LEFT/RIGHT = direction")

    def update(self, key):
        """Adjust speed/direction from a single already-read keyboard key."""
        if key == Keyboard.PAGEUP:
            self.speed = min(self.speed + WIND_SPEED_STEP, MAX_WIND_SPEED)
            print(f"💨 Wind speed → {self.speed:.1f} N")
        elif key == Keyboard.PAGEDOWN:
            self.speed = max(self.speed - WIND_SPEED_STEP, 0.0)
            print(f"💨 Wind speed → {self.speed:.1f} N")
        elif key == Keyboard.LEFT:
            self.direction += WIND_TURN_STEP
            print(f"💨 Wind direction → {math.degrees(self.direction):.0f}°")
        elif key == Keyboard.RIGHT:
            self.direction -= WIND_TURN_STEP
            print(f"💨 Wind direction → {math.degrees(self.direction):.0f}°")

    def force_vector(self):
        """World-frame force vector [fx, fy, fz] to apply this step."""
        return [self.speed * math.cos(self.direction),
                self.speed * math.sin(self.direction),
                0.0]
