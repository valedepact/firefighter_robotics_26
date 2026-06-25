# GPS-based ground movement for Spot — lawnmower patrol, walk-to-target,
# reactive steer-around obstacle avoidance.
#
# Ground-adapted from controllers/mavic2pro/navigation.py: steering uses a
# single turn_rate (heading-only) fed to gait.step(forward_speed, turn_rate)
# instead of the drone's pitch/roll/yaw velocity commands, and obstacles are
# steered AROUND rather than climbed over — a legged robot can't fly over a
# tree.

import math

# Same arena/base convention as the drone (controllers/mavic2pro/navigation.py)
ARENA_MIN_X = -38.0
ARENA_MAX_X = 38.0
ARENA_MIN_Y = -38.0
ARENA_MAX_Y = 38.0

BASE_X = -20.0
BASE_Y = -20.0

PATROL_STEP_Y = 10.0   # gap between patrol rows
PATROL_ROWS = [ARENA_MIN_Y + i * PATROL_STEP_Y
               for i in range(int((ARENA_MAX_Y - ARENA_MIN_Y) / PATROL_STEP_Y) + 1)]

WAYPOINT_ARRIVAL_RADIUS = 1.5   # metres — close enough to a patrol waypoint
ARRIVAL_RADIUS           = 1.5   # metres — close enough to base
FIRE_ARRIVAL_RADIUS       = 1.0   # metres — ground robot must get close to the tree trunk

YAW_GAIN            = 0.8    # heading-error → turn_rate proportional gain
CAMERA_YAW_GAIN      = 1.2   # turn_rate gain from camera horizontal offset

OBSTACLE_RANGE   = 3.0   # metres — front distance sensor threshold to trigger avoidance
AVOID_TURN_RATE  = 1.0   # hard turn while avoiding
AVOID_SPEED      = 0.3   # slow crawl while turning to clear an obstacle


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _heading_turn_rate(imu, dx, dy):
    """
    Proportional turn_rate toward world-frame offset (dx, dy), correctly
    accounting for Spot's current yaw (unlike the drone, which mostly
    translates via body-frame pitch/roll and only uses yaw as a small
    drift correction, Spot's only way to move toward a target is to
    actually turn its whole body — so the steering signal must be a
    genuine heading ERROR, not the raw world-frame bearing).

    NOTE: sign convention (which way "turn_rate > 0" turns the body) is
    inferred from Webots' yaw-about-Z convention, not verified against a
    live run. If Spot consistently turns away from targets instead of
    toward them, flip the sign on heading_error below first.
    """
    target_heading = math.atan2(dx, dy)
    yaw = imu.getRollPitchYaw()[2]
    heading_error = math.atan2(math.sin(target_heading - yaw), math.cos(target_heading - yaw))
    return _clamp(heading_error * YAW_GAIN / (math.pi / 2), -1.0, 1.0)


class GroundNavigator:
    """
    High-level ground navigation for Spot.

    Usage
    -----
        nav = GroundNavigator()

        # In the step loop:
        nav.patrol(gps, imu, gait, front_ds)
        nav.fly_to_fire(gps, imu, gait, detection_result, front_ds)
        nav.return_to_base(gps, imu, gait, front_ds)

    Each method returns True when the goal is reached.
    """

    def __init__(self):
        self._row_index    = 0
        self._going_right  = True
        self._patrol_phase = "TO_START"
        self._waypoint      = self._first_waypoint()

        # Remembered fire GPS position (set once detection triggers NAVIGATE)
        self.fire_gps = None

        # True while actively steering around an obstacle
        self._avoiding = False

        print("GroundNavigator ready — lawnmower patrol initialised")
        print(f"  {len(PATROL_ROWS)} rows × {PATROL_STEP_Y} m spacing")

    # ── Obstacle avoidance ───────────────────────────────────────────────────
    def _avoid_obstacles(self, front_ds):
        """
        Returns (forward_speed, turn_rate) override while steering around an
        obstacle, or None if the path ahead is clear.
        """
        if front_ds is None:
            return None
        distance = front_ds.getValue()
        if distance < OBSTACLE_RANGE:
            if not self._avoiding:
                self._avoiding = True
                print(f"🌳 Obstacle ahead ({distance:.1f}m) — steering around it")
            return AVOID_SPEED, AVOID_TURN_RATE
        if self._avoiding:
            self._avoiding = False
            print("✅ Clear of obstacle — resuming heading")
        return None

    # ── Patrol ────────────────────────────────────────────────────────────────
    def _first_waypoint(self):
        return (ARENA_MIN_X, PATROL_ROWS[0])

    def patrol(self, gps, imu, gait, front_ds=None):
        """
        Walk a lawnmower grid over the arena.
        Returns True when the full grid has been covered (loop back to start).
        """
        avoid = self._avoid_obstacles(front_ds)
        if avoid is not None:
            gait.step(*avoid)
            return False

        pos  = gps.getValues()
        x, y = pos[0], pos[1]
        wx, wy = self._waypoint
        dist = math.hypot(wx - x, wy - y)

        if dist < WAYPOINT_ARRIVAL_RADIUS:
            if self._patrol_phase == "TO_START":
                self._patrol_phase = "SWEEP"
                self._waypoint = (ARENA_MAX_X if self._going_right else ARENA_MIN_X,
                                   PATROL_ROWS[self._row_index])
            elif self._patrol_phase == "SWEEP":
                self._row_index += 1
                if self._row_index >= len(PATROL_ROWS):
                    self._row_index   = 0
                    self._going_right = True
                    self._patrol_phase = "TO_START"
                    self._waypoint     = self._first_waypoint()
                    return True
                self._going_right   = not self._going_right
                self._patrol_phase  = "SWEEP"
                self._waypoint = (ARENA_MAX_X if self._going_right else ARENA_MIN_X,
                                   PATROL_ROWS[self._row_index])

        wx, wy = self._waypoint
        dx, dy = wx - x, wy - y
        turn_rate = _heading_turn_rate(imu, dx, dy)
        gait.step(forward_speed=1.0, turn_rate=turn_rate)
        return False

    # ── Navigate to fire ─────────────────────────────────────────────────────
    def set_fire_position(self, x, y):
        self.fire_gps = (x, y)

    def fly_to_fire(self, gps, imu, gait, detection_result=None, front_ds=None):
        """
        Walk toward the fire. Two modes:
          1. Camera lock-on (detection_result given) — visual servoing, no
             obstacle avoidance override (final close approach).
          2. GPS walk-to (no live detection) — heads toward self.fire_gps,
             with reactive steer-around obstacle avoidance.
        Returns True once close enough to extinguish.
        """
        pos  = gps.getValues()
        x, y = pos[0], pos[1]

        # ── Mode 1: camera lock-on ──
        if detection_result is not None and detection_result.get("detected"):
            offset_x = detection_result["offset_x"]
            turn_rate = _clamp(offset_x * CAMERA_YAW_GAIN, -1.0, 1.0)
            gait.step(forward_speed=1.0, turn_rate=turn_rate)

            if self.fire_gps is not None:
                dist = math.hypot(self.fire_gps[0] - x, self.fire_gps[1] - y)
                if dist < FIRE_ARRIVAL_RADIUS:
                    print(f"✅ Fire centred and close ({dist:.1f}m) — close enough to extinguish")
                    return True
            return False

        # ── Mode 2: GPS walk-to ──
        avoid = self._avoid_obstacles(front_ds)
        if avoid is not None:
            gait.step(*avoid)
            return False

        if self.fire_gps is None:
            return False

        fx, fy = self.fire_gps
        dx, dy = fx - x, fy - y
        dist = math.hypot(dx, dy)

        if dist < FIRE_ARRIVAL_RADIUS:
            print(f"✅ Arrived at fire ({fx:.1f}, {fy:.1f})")
            return True

        turn_rate = _heading_turn_rate(imu, dx, dy)
        gait.step(forward_speed=1.0, turn_rate=turn_rate)
        return False

    # ── Return to base ───────────────────────────────────────────────────────
    def return_to_base(self, gps, imu, gait, front_ds=None):
        """
        Walk back to the spawn position.
        Returns True when base is reached.
        """
        avoid = self._avoid_obstacles(front_ds)
        if avoid is not None:
            gait.step(*avoid)
            return False

        pos  = gps.getValues()
        x, y = pos[0], pos[1]
        dx   = BASE_X - x
        dy   = BASE_Y - y
        dist = math.hypot(dx, dy)

        if dist < ARRIVAL_RADIUS:
            return True

        turn_rate = _heading_turn_rate(imu, dx, dy)
        gait.step(forward_speed=1.0, turn_rate=turn_rate)
        return False
