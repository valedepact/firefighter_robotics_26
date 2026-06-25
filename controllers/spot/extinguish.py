# Ground-level water spray logic, positioning Spot next to the fire
"""
extinguish.py — Fire extinguishing logic for Spot (ground-adapted from the
drone's controllers/mavic2pro/extinguish.py).

Strategy:
  1. Spot is already close (within GroundNavigator.FIRE_ARRIVAL_RADIUS) by the
     time this is called — APPROACH creeps the last bit closer and settles.
  2. Once stable, spawn a Water node at ground level next to the fire
     (Supervisor API) — no fall needed, it's already on the ground.
  3. Poll until the water sphere is close enough to the fire node → remove both.
  4. Signal completion so spot.py transitions back to RETURN.

Requires Spot to have  supervisor TRUE  in the .wbt world file.
"""

import math

# ──────────────────────────────────────────────
#  Tuning
# ──────────────────────────────────────────────
APPROACH_STEPS      = 20     # steps creeping closer before stabilising
STABLE_STEPS        = 25     # steps standing still before spraying (let Spot settle)
EXTINGUISH_RADIUS   = 1.2    # metres — water-to-fire distance that counts as a hit
CHECK_INTERVAL      = 5      # check proximity every N steps (not every step)
WATER_FALL_TIMEOUT  = 80     # steps before giving up on a drop (shorter — already on ground)
DROP_COOLDOWN       = 30     # steps between successive drops (for multi-fire)

# Water node VRML template — spawned at ground level next to Spot
_WATER_VRML = """\
Water {{
  translation {x} {y} {z}
  radius 0.3
  name "{name}"
}}
"""


class Extinguisher:
    """
    Manages the full extinguish sequence for one or more fires, ground-level.

    Usage
    -----
        ext = Extinguisher(robot)          # robot must be a Supervisor

        # In the EXTINGUISH state step loop:
        done = ext.update(gps, gait, fire_def_name)

        # done == True  → fire out, return to base
        # done == False → still working
    """

    def __init__(self, robot):
        self._robot        = robot
        self._root         = robot.getRoot()
        self._children     = self._root.getField("children")

        self._phase          = "APPROACH"   # APPROACH | STABILISE | DROP | WAIT | DONE
        self._approach_count = 0
        self._stable_count   = 0
        self._wait_count     = 0
        self._drop_count     = 0          # how many drops fired this session
        self._water_name     = None       # name of the spawned Water node
        self._water_node     = None       # Webots node handle
        self._step           = 0

        print("Extinguisher ready (ground)")

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, gps, gait, fire_def_name="FIRE_1"):
        """
        Call every simulation step while in EXTINGUISH state.

        Parameters
        ----------
        gps           : Webots GPS device (already enabled)
        gait          : Gait instance from gait.py
        fire_def_name : DEF name of the fire node in the world (e.g. "FIRE_1")

        Returns True when the fire is confirmed extinguished.
        """
        self._step += 1
        pos = gps.getValues()
        x, y, z = pos[0], pos[1], pos[2]

        # ── Phase: creep the last bit closer ─────────────────────────────────
        if self._phase == "APPROACH":
            gait.step(forward_speed=0.3, turn_rate=0.0)
            self._approach_count += 1

            if self._approach_count >= APPROACH_STEPS:
                print("✅ Approach complete — stabilising")
                self._phase        = "STABILISE"
                self._stable_count = 0
            return False

        # ── Phase: stand still and let Spot stop wobbling ────────────────────
        if self._phase == "STABILISE":
            gait.hold_pose(gait.stand_pose())
            self._stable_count += 1

            if self._stable_count >= STABLE_STEPS:
                print("✅ Stable — spraying water")
                self._phase = "DROP"
            return False

        # ── Phase: spawn water node ───────────────────────────────────────────
        if self._phase == "DROP":
            self._drop_count += 1
            self._water_name  = f"water_drop_{self._drop_count}"

            # Spawn at ground level, right where Spot is standing
            vrml = _WATER_VRML.format(
                x=round(x, 3),
                y=round(y, 3),
                z=round(max(0.1, z), 3),
                name=self._water_name,
            )
            self._children.importMFNodeFromString(-1, vrml)
            self._water_node = self._robot.getFromDef(self._water_name)

            if self._water_node is None:
                # importMFNodeFromString doesn't set DEF — the new node is
                # appended at the end of children, so grab it by index
                self._water_node = self._children.getMFNode(self._children.getCount() - 1)

            print(f"💧 Water spray #{self._drop_count} at ({x:.1f}, {y:.1f}, {z:.1f})")
            self._wait_count = 0
            self._phase      = "WAIT"
            return False

        # ── Phase: wait and check proximity to fire ──────────────────────────
        if self._phase == "WAIT":
            self._wait_count += 1
            gait.hold_pose(gait.stand_pose())

            # Check every CHECK_INTERVAL steps
            if self._wait_count % CHECK_INTERVAL == 0:
                hit = self._check_hit(fire_def_name)

                if hit:
                    self._remove_fire(fire_def_name)
                    self._remove_water()
                    print(f"🔥➡️💧 Fire '{fire_def_name}' extinguished!")
                    self._phase = "DONE"
                    return True

            # Timeout — try another drop
            if self._wait_count >= WATER_FALL_TIMEOUT:
                print(f"⚠️  Water spray #{self._drop_count} missed — retrying")
                self._remove_water()
                self._phase        = "STABILISE"
                self._stable_count = 0

            return False

        # ── Phase: done (should not normally be reached via update) ───────────
        if self._phase == "DONE":
            return True

        return False

    def reset(self):
        """Call this before starting a new extinguish sequence on a different fire."""
        self._phase          = "APPROACH"
        self._approach_count = 0
        self._stable_count   = 0
        self._wait_count     = 0
        self._water_name     = None
        self._water_node     = None
        print("Extinguisher reset — ready for next fire")

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _check_hit(self, fire_def_name):
        """
        Returns True if the water sphere is within EXTINGUISH_RADIUS of the fire node.
        """
        fire_node = self._robot.getFromDef(fire_def_name)
        if fire_node is None:
            print(f"⚠️  Fire node '{fire_def_name}' not found — assuming extinguished")
            return True

        fire_pos = fire_node.getField("translation").getSFVec3f()

        if self._water_node is not None:
            try:
                water_pos = self._water_node.getField("translation").getSFVec3f()
                dist = math.sqrt(
                    (water_pos[0] - fire_pos[0]) ** 2 +
                    (water_pos[1] - fire_pos[1]) ** 2 +
                    (water_pos[2] - fire_pos[2]) ** 2
                )
                return dist < EXTINGUISH_RADIUS
            except Exception:
                pass   # node may have been removed already

        return False

    def _remove_fire(self, fire_def_name):
        """Remove the fire node from the world using the Supervisor API."""
        fire_node = self._robot.getFromDef(fire_def_name)
        if fire_node:
            fire_node.remove()
            print(f"🗑️  Fire node '{fire_def_name}' removed from world")
        else:
            print(f"⚠️  Could not find '{fire_def_name}' to remove")

    def _remove_water(self):
        """Clean up the water sphere from the world."""
        if self._water_node is not None:
            try:
                self._water_node.remove()
            except Exception:
                pass
            self._water_node = None

        # Fallback: search by name
        if self._water_name:
            node = self._robot.getFromDef(self._water_name)
            if node:
                node.remove()
        self._water_name = None
