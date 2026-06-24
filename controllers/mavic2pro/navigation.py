# GPS-based movement, fly-to-target, patrol path

"""
navigation.py — Mavic 2 Pro navigation
Handles: lawnmower patrol grid, GPS-based fly-to-target, camera-based fire tracking,
         and return-to-base.  Works alongside flight.py — never touches motors directly.
"""

import math

# ──────────────────────────────────────────────
#  Arena & base constants  (match wildfire.wbt)
# ──────────────────────────────────────────────
ARENA_MIN_X   = -38.0
ARENA_MAX_X   =  38.0
ARENA_MIN_Y   = -38.0
ARENA_MAX_Y   =  38.0

BASE_X        = -20.0   # drone spawn position
BASE_Y        = -20.0

# Patrol grid — lawnmower rows spaced 10 m apart
PATROL_STEP_Y =  10.0   # gap between patrol rows
PATROL_ROWS   = [
    (ARENA_MIN_X + 2, ARENA_MAX_X - 2, y)
    for y in range(int(ARENA_MIN_Y) + 5, int(ARENA_MAX_Y), int(PATROL_STEP_Y))
]   # list of (x_start, x_end, y) — direction alternates each row

# ──────────────────────────────────────────────
#  Tuning
# ──────────────────────────────────────────────
ARRIVAL_RADIUS      = 2.0   # metres — close enough to a waypoint
FIRE_ARRIVAL_RADIUS = 1.5   # metres — close enough to be above fire
PATROL_SPEED        = 0.10  # pitch/roll command magnitude during patrol
NAV_SPEED           = 0.15  # pitch/roll command magnitude flying to fire
YAW_GAIN            = 0.8   # how aggressively to yaw toward target
CAMERA_YAW_GAIN     = 0.5   # yaw correction from camera offset_x
CAMERA_PITCH_GAIN   = 0.3   # pitch correction from camera offset_y


class Navigator:
    """
    High-level navigation for the Mavic 2 Pro.

    Usage
    -----
        nav = Navigator()

        # In the step loop:
        nav.patrol(gps, fc)           # during PATROL state
        nav.fly_to_fire(gps, fc, detection_result)   # during NAVIGATE state
        nav.return_to_base(gps, fc)   # during RETURN state

    Each method returns True when the goal is reached.
    """

    def __init__(self):
        self._row_index    = 0
        self._going_right  = True   # alternating patrol direction
        self._patrol_phase = "TO_START"   # TO_START | SWEEP | NEXT_ROW
        self._waypoint     = self._first_waypoint()

        # Remembered fire GPS position (set once detection triggers NAVIGATE)
        self.fire_gps = None

        print("Navigator ready — lawnmower patrol initialised")
        print(f"  {len(PATROL_ROWS)} rows × {PATROL_STEP_Y} m spacing")

    # ── Patrol ────────────────────────────────────────────────────────────────

    def patrol(self, gps, fc):
        """
        Fly a lawnmower grid over the arena.
        Returns True when the full grid has been covered (loop back to start).
        Calls fc.set_velocity() to steer.
        """
        pos  = gps.getValues()
        x, y = pos[0], pos[1]
        wx, wy = self._waypoint

        dist = math.hypot(wx - x, wy - y)

        if dist < ARRIVAL_RADIUS:
            reached = self._advance_waypoint()
            if reached:
                print("✅ Full patrol grid covered — restarting")
                self._row_index   = 0
                self._going_right = True
                self._patrol_phase = "TO_START"
                self._waypoint = self._first_waypoint()
                return True
            wx, wy = self._waypoint

        # Steer toward waypoint
        dx = wx - x
        dy = wy - y
        pitch, roll = self._direction_to_pitch_roll(dx, dy, PATROL_SPEED)
        yaw = self._yaw_toward(dx, dy)

        fc.set_velocity(pitch=pitch, roll=roll, yaw=yaw)
        return False

    # ── Fly to fire ───────────────────────────────────────────────────────────

    def fly_to_fire(self, gps, fc, detection_result=None):
        """
        Navigate toward the fire.  Two modes:
          1. Camera-guided  — if detection_result is provided and fire is visible,
                              use pixel offsets to centre the drone above it.
          2. GPS-guided     — use self.fire_gps (set externally when fire first spotted).

        Returns True when the drone is close enough to extinguish.
        """
        pos  = gps.getValues()
        x, y = pos[0], pos[1]

        # ── Mode 1: camera lock-on ──
        if detection_result and detection_result.get("detected"):
            ox = detection_result["offset_x"]   # -1 … +1  (right is +)
            oy = detection_result["offset_y"]   # -1 … +1  (down  is +)

            # If fire fills a large chunk of the frame we are close enough
            if detection_result["area"] > 2000:
                fc.hover()
                print("✅ Fire centred in frame — close enough to extinguish")
                return True

            # Yaw to centre fire horizontally; pitch to close distance
            yaw   = CAMERA_YAW_GAIN   * ox
            pitch = CAMERA_PITCH_GAIN * (1 - oy)   # oy<0 means fire above → fly forward

            fc.set_velocity(pitch=pitch, roll=0.0, yaw=yaw)
            return False

        # ── Mode 2: GPS fly-to ──
        if self.fire_gps is None:
            print("⚠️  fly_to_fire called but fire_gps not set — hovering")
            fc.hover()
            return False

        fx, fy = self.fire_gps
        dx, dy = fx - x, fy - y
        dist   = math.hypot(dx, dy)

        if dist < FIRE_ARRIVAL_RADIUS:
            fc.hover()
            print(f"✅ Arrived at fire GPS ({fx:.1f}, {fy:.1f})")
            return True

        pitch, roll = self._direction_to_pitch_roll(dx, dy, NAV_SPEED)
        yaw = self._yaw_toward(dx, dy)
        fc.set_velocity(pitch=pitch, roll=roll, yaw=yaw)
        return False

    def set_fire_position(self, x, y):
        """Call this from the main loop when fire is first detected via GPS."""
        self.fire_gps = (x, y)
        print(f"🔥 Fire position locked: ({x:.1f}, {y:.1f})")

    # ── Return to base ────────────────────────────────────────────────────────

    def return_to_base(self, gps, fc):
        """
        Fly back to the spawn position.
        Returns True when base is reached.
        """
        pos  = gps.getValues()
        x, y = pos[0], pos[1]
        dx   = BASE_X - x
        dy   = BASE_Y - y
        dist = math.hypot(dx, dy)

        if dist < ARRIVAL_RADIUS:
            fc.hover()
            print("✅ Returned to base")
            return True

        pitch, roll = self._direction_to_pitch_roll(dx, dy, PATROL_SPEED)
        yaw = self._yaw_toward(dx, dy)
        fc.set_velocity(pitch=pitch, roll=roll, yaw=yaw)
        return False

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _first_waypoint(self):
        x_start, _, y = PATROL_ROWS[0]
        return (x_start, y)

    def _advance_waypoint(self):
        """
        Move to the next patrol waypoint.
        Returns True if we just wrapped back to the beginning.
        """
        row = PATROL_ROWS[self._row_index]
        x_start, x_end, y = row

        if self._patrol_phase == "TO_START":
            # Arrived at row start — begin sweep
            self._patrol_phase = "SWEEP"
            target_x = x_end if self._going_right else x_start
            self._waypoint = (target_x, y)
            return False

        if self._patrol_phase == "SWEEP":
            # Finished sweeping this row — move to next row
            self._row_index += 1
            if self._row_index >= len(PATROL_ROWS):
                return True   # full grid done

            self._going_right  = not self._going_right
            self._patrol_phase = "TO_START"
            _, _, next_y = PATROL_ROWS[self._row_index]
            current_x    = self._waypoint[0]
            self._waypoint = (current_x, next_y)
            return False

        return False

    @staticmethod
    def _direction_to_pitch_roll(dx, dy, speed):
        """
        Convert a world-space (dx, dy) vector into (pitch, roll) commands.
        Webots X axis = right, Y axis = forward for the drone at yaw=0.
        speed normalises the magnitude.
        """
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return 0.0, 0.0

        norm_x = dx / dist
        norm_y = dy / dist

        # pitch forward/back maps to Y, roll left/right maps to X
        pitch = -norm_y * speed   # negative because forward pitch tilts nose down
        roll  =  norm_x * speed
        return pitch, roll

    @staticmethod
    def _yaw_toward(dx, dy):
        """Small yaw correction to face the target."""
        angle = math.atan2(dx, dy)   # angle in world frame
        # Return a proportional yaw command clamped to ±1
        return max(-1.0, min(1.0, angle * YAW_GAIN / math.pi))