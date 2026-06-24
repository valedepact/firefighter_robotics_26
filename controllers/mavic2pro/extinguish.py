# Water drop logic, positioning over fire
"""
extinguish.py — Fire extinguishing logic for the Mavic 2 Pro
Strategy:
  1. Drone descends to DROP_ALTITUDE above the fire.
  2. Once stable, spawn a Water node directly below the drone (Supervisor API).
  3. Poll until the water sphere is close enough to the fire node → remove both.
  4. Signal completion so mavic2pro.py transitions back to PATROL.

Requires the Mavic2Pro to have  supervisor TRUE  in the .wbt world file.
"""

import math

# ──────────────────────────────────────────────
#  Tuning
# ──────────────────────────────────────────────
DROP_ALTITUDE       = 3.5    # metres — descend to this height before dropping
STABLE_STEPS        = 25     # timesteps to hover before dropping (let drone settle)
EXTINGUISH_RADIUS   = 1.8    # metres — water-to-fire distance that counts as a hit
CHECK_INTERVAL      = 5      # check proximity every N steps (not every step)
WATER_FALL_TIMEOUT  = 150    # steps before giving up on a water drop
DROP_COOLDOWN       = 30     # steps between successive drops (for multi-fire)

# Water node VRML template — spawned just below the drone
_WATER_VRML = """\
Water {{
  translation {x} {y} {z}
  radius 0.3
  name "{name}"
}}
"""


class Extinguisher:
    """
    Manages the full extinguish sequence for one or more fires.

    Usage
    -----
        ext = Extinguisher(robot)          # robot must be a Supervisor

        # In the EXTINGUISH state step loop:
        done = ext.update(gps, fc, fire_node_def)

        # done == True  → fire out, return to patrol
        # done == False → still working
    """

    def __init__(self, robot):
        self._robot        = robot
        self._root         = robot.getRoot()
        self._children     = self._root.getField("children")

        self._phase        = "DESCEND"   # DESCEND | STABILISE | DROP | WAIT | DONE
        self._stable_count = 0
        self._wait_count   = 0
        self._drop_count   = 0          # how many drops fired this session
        self._water_name   = None       # name of the spawned Water node
        self._water_node   = None       # Webots node handle
        self._step         = 0

        print("Extinguisher ready")

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, gps, fc, fire_def_name="FIRE_1"):
        """
        Call every simulation step while in EXTINGUISH state.

        Parameters
        ----------
        gps           : Webots GPS device (already enabled)
        fc            : FlightController instance from flight.py
        fire_def_name : DEF name of the fire node in the world (e.g. "FIRE_1")

        Returns True when the fire is confirmed extinguished.
        """
        self._step += 1
        pos = gps.getValues()
        x, y, z = pos[0], pos[1], pos[2]

        # ── Phase: descend to drop altitude ──────────────────────────────────
        if self._phase == "DESCEND":
            fc.hover()
            fc.set_altitude(DROP_ALTITUDE)

            if abs(z - DROP_ALTITUDE) < 0.4:
                print(f"✅ Descended to {z:.2f} m — stabilising")
                self._phase        = "STABILISE"
                self._stable_count = 0
            return False

        # ── Phase: hover and wait for drone to stop wobbling ─────────────────
        if self._phase == "STABILISE":
            fc.hover()
            self._stable_count += 1

            if self._stable_count >= STABLE_STEPS:
                print("✅ Stable — dropping water")
                self._phase = "DROP"
            return False

        # ── Phase: spawn water node ───────────────────────────────────────────
        if self._phase == "DROP":
            self._drop_count += 1
            self._water_name  = f"water_drop_{self._drop_count}"

            # Spawn 0.5 m below the drone so it starts falling immediately
            vrml = _WATER_VRML.format(
                x=round(x, 3),
                y=round(y, 3),
                z=round(z - 0.5, 3),
                name=self._water_name,
            )
            self._children.importMFNodeFromString(-1, vrml)
            self._water_node = self._robot.getFromDef(self._water_name)

            if self._water_node is None:
                # importMFNodeFromString doesn't set DEF — find by name field
                self._water_node = self._robot.getFromProtoDef(self._water_name)

            print(f"💧 Water drop #{self._drop_count} spawned at ({x:.1f}, {y:.1f}, {z:.1f})")
            self._wait_count = 0
            self._phase      = "WAIT"
            return False

        # ── Phase: wait for water to fall and check proximity to fire ─────────
        if self._phase == "WAIT":
            self._wait_count += 1
            fc.hover()

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
                print(f"⚠️  Water drop #{self._drop_count} missed — retrying")
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
        self._phase        = "DESCEND"
        self._stable_count = 0
        self._wait_count   = 0
        self._water_name   = None
        self._water_node   = None
        print("Extinguisher reset — ready for next fire")

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _check_hit(self, fire_def_name):
        """
        Returns True if the water sphere is within EXTINGUISH_RADIUS of the fire node.
        Falls back to True after enough drops if nodes can't be located (failsafe).
        """
        fire_node = self._robot.getFromDef(fire_def_name)
        if fire_node is None:
            print(f"⚠️  Fire node '{fire_def_name}' not found — assuming extinguished")
            return True

        fire_pos = fire_node.getField("translation").getSFVec3f()

        # Try to get water position
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

        # Failsafe: if water node handle is lost, check if drone is above fire
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