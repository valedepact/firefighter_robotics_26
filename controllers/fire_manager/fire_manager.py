"""
fire_manager.py — Supervisor controller for fire lifecycle management.

Responsibilities:
  - Track all active fires in the world
  - Spread fire to nearby trees over time (probabilistic propagation)
  - Spawn new Fire nodes when propagation triggers
  - Remove fires that have been extinguished by the drone
  - Broadcast active fire positions to the drone via Emitter

Communication with drone and Spot:
  The fire manager uses a Webots Emitter to broadcast a CSV string every
  BROADCAST_INTERVAL steps:
      "FIRE_2,3.0,-8.0,1.5|FIRE_3,-8.0,8.0,3.0"   (def,x,y,strength)
  Both mavic2pro.py and spot.py read this with their own Receiver device to
  update their target fire, and use the strength value to decide whether to
  handle a fire alone or call the other robot in (see coordination.py).
"""

import random
import math
from controller import Supervisor

# ──────────────────────────────────────────────
#  Tuning constants
# ──────────────────────────────────────────────
SPREAD_INTERVAL      = 500    # steps between spread checks (~4 s at 8 ms timestep)
SPREAD_RADIUS        = 12.0    # metres — radius a fire can reach at max strength
MIN_SPREAD_RADIUS    = 2.0    # metres — radius a freshly-spawned (weak) fire can reach
SPREAD_PROBABILITY   = 0.6   # chance per eligible tree per spread event, at max strength
MAX_FIRES            = 6      # cap to keep simulation manageable
BROADCAST_INTERVAL   = 60     # steps between emitter broadcasts

# A fire grows stronger the longer it burns unattended. Strength scales both
# how far it can ignite neighbouring trees (toward SPREAD_RADIUS) and how
# likely it is to do so — a fire that just started is barely dangerous.
STRENGTH_INITIAL = 1.0
STRENGTH_MAX     = 5.0
STRENGTH_GROWTH  = 0.5   # added once per _spread() tick

# ──────────────────────────────────────────────
#  Visual animation — flicker + grow + smoke
#  (borrowed from Cyberbotics' own forest-firefighters reference solution)
# ──────────────────────────────────────────────
# Fire.proto stacks 13 pre-textured flame frames (fire_00..fire_12), each
# offset FRAME_OFFSET_Y * frame_index along Y so only one is ever near the
# real translation at a time. Shifting the whole node's Y by
# -FRAME_OFFSET_Y * (anim_count % FLAME_CYCLE) brings a different frame back
# to the actual fire position each tick, producing a flickering animation.
FLAME_CYCLE      = 13        # number of flame texture frames in Fire.proto
FRAME_OFFSET_Y   = 100000    # must match the per-frame Y offset baked into Fire.proto
FLICKER_INTERVAL = 4         # steps between flicker-frame advances (lower = faster flicker)

# Visual scale grows with strength so a fire looks more dangerous as it
# burns longer unattended — independent of the gameplay SPREAD_RADIUS logic.
FIRE_SCALE_MIN = 2.0   # visual scale at STRENGTH_INITIAL
FIRE_SCALE_MAX = 5.0   # visual scale at STRENGTH_MAX

# Smoke spawned alongside each fire, growing for SMOKE_GROW_STEPS steps
# then holding at SMOKE_SCALE_MAX.
SMOKE_GROW_STEPS  = 300
SMOKE_SCALE_START = 0.2
SMOKE_SCALE_MAX   = 2.5

# All tree positions from wildfire.wbt (x, y)
TREE_POSITIONS = [
    ( 3,   0),
    ( 4,  10),
    (-8,   8),
    ( 3,  -8),
    (-5, -14),
    (-2,   8),
    (-9,  -7),
    (-12,  2),
    (-4,  -2),
    (-11,-18),
    (-15, -8),
    (-1, -16),
    (10,   0),   # trees 13/14 share roughly x=10, y=0
]


class FireManager:
    """
    Tracks and propagates fires in the Webots world.
    """

    def __init__(self, robot):
        self._robot    = robot
        self._root     = robot.getRoot()
        self._children = self._root.getField("children")

        # fire_id → {"def": "FIRE_2", "x": 3.0, "y": -8.0, "node": <WebotsNode>}
        self._fires    = {}
        self._next_id  = 2   # FIRE_1 already exists in the world

        # Track which tree positions already have fire
        self._burning_positions = set()

        # Emitter (optional — won't crash if not wired in .wbt)
        self._emitter = None
        try:
            self._emitter = robot.getDevice("emitter")
            if self._emitter:
                print("📡 Emitter found — will broadcast fire positions")
        except Exception:
            pass

        # Register the pre-existing FIRE_1 at origin
        fire1_node = robot.getFromDef("FIRE_1")
        if fire1_node:
            pos = fire1_node.getField("translation").getSFVec3f()
            self._fires["FIRE_1"] = {
                "def":        "FIRE_1",
                "x":          pos[0],
                "y":          pos[1],
                "node":       fire1_node,
                "strength":   STRENGTH_INITIAL,
                "anim_count": 0,
                "base_z":     pos[2],
                "smoke_node": None,
            }
            self._burning_positions.add((round(pos[0]), round(pos[1])))
            self._spawn_smoke(self._fires["FIRE_1"])
            print(f"🔥 FIRE_1 registered at ({pos[0]:.1f}, {pos[1]:.1f})")
        else:
            print("⚠️  FIRE_1 not found in world — check DEF name in .wbt")

        print(f"FireManager ready | spread every {SPREAD_INTERVAL} steps "
              f"| max {MAX_FIRES} fires")

    def step(self, step_count):
        """Call every simulation step."""
        # Check for extinguished fires (nodes removed by drone)
        self._check_extinguished()

        # Flicker + grow every fire, every step (cheap field writes)
        self._animate(step_count)

        # Spread fire on schedule
        if step_count % SPREAD_INTERVAL == 0 and step_count > 0:
            self._spread()

        # Broadcast positions to drone
        if step_count % BROADCAST_INTERVAL == 0:
            self._broadcast()

    # ── Fire propagation ──────────────────────────────────────────────────────

    def _spread(self):
        if len(self._fires) >= MAX_FIRES:
            return

        # Fires grow stronger the longer they burn unattended
        for fire in self._fires.values():
            fire["strength"] = min(STRENGTH_MAX, fire["strength"] + STRENGTH_GROWTH)

        # tree (x, y) → (dist, strength, radius) of the closest threatening fire
        best = {}

        for fire in self._fires.values():
            fx, fy, strength = fire["x"], fire["y"], fire["strength"]
            radius = MIN_SPREAD_RADIUS + (strength / STRENGTH_MAX) * (SPREAD_RADIUS - MIN_SPREAD_RADIUS)

            for tx, ty in TREE_POSITIONS:
                if (tx, ty) in self._burning_positions:
                    continue   # already on fire
                dist = math.hypot(tx - fx, ty - fy)
                if dist <= radius:
                    if (tx, ty) not in best or dist < best[(tx, ty)][0]:
                        best[(tx, ty)] = (dist, strength, radius)

        # Closer trees, and stronger fires, are more likely to ignite them
        for (tx, ty), (dist, strength, radius) in best.items():
            if len(self._fires) >= MAX_FIRES:
                break
            strength_factor = strength / STRENGTH_MAX
            prob = SPREAD_PROBABILITY * strength_factor * (1 - dist / (radius * 1.5))
            if random.random() < prob:
                self._spawn_fire(tx, ty)

    # ── Visual animation ──────────────────────────────────────────────────────

    def _animate(self, step_count):
        """
        Run every step for every active fire:
          - flicker through the 13 baked flame-texture frames
          - grow the fire's visual scale with its strength
          - grow its smoke plume for its first SMOKE_GROW_STEPS steps
        """
        for fire in self._fires.values():
            node = fire["node"]
            if node is None:
                continue

            fire["anim_count"] += 1

            # Flicker: shift Y so a different baked frame lines up at the real spot.
            if fire["anim_count"] % FLICKER_INTERVAL == 0:
                frame = (fire["anim_count"] // FLICKER_INTERVAL) % FLAME_CYCLE
                translation_field = node.getField("translation")
                translation_field.setSFVec3f(
                    [fire["x"], fire["y"] - FRAME_OFFSET_Y * frame, fire["base_z"]]
                )

            # Grow visual scale with strength (separate from gameplay spread radius).
            strength_factor = (fire["strength"] - STRENGTH_INITIAL) / (STRENGTH_MAX - STRENGTH_INITIAL)
            strength_factor = max(0.0, min(1.0, strength_factor))
            scale = FIRE_SCALE_MIN + strength_factor * (FIRE_SCALE_MAX - FIRE_SCALE_MIN)
            node.getField("scale").setSFVec3f([scale, scale, scale])

            # Grow the smoke plume alongside the fire.
            if fire["smoke_node"] is not None and fire["anim_count"] <= SMOKE_GROW_STEPS:
                smoke_progress = fire["anim_count"] / SMOKE_GROW_STEPS
                smoke_scale = SMOKE_SCALE_START + smoke_progress * (SMOKE_SCALE_MAX - SMOKE_SCALE_START)
                fire["smoke_node"].getField("scale").setSFVec3f([smoke_scale, smoke_scale, smoke_scale])

    def _spawn_smoke(self, fire):
        """Add a Smoke node above the given fire and stash it on the fire dict."""
        def_name = f"SMOKE_{fire['def']}"
        vrml = (f'DEF {def_name} Smoke {{ translation {fire["x"]} {fire["y"]} {fire["base_z"]} '
                f'scale {SMOKE_SCALE_START} {SMOKE_SCALE_START} {SMOKE_SCALE_START} }}\n')
        self._children.importMFNodeFromString(-1, vrml)
        fire["smoke_node"] = self._robot.getFromDef(def_name)

    def _spawn_fire(self, x, y):
        """Add a new Fire node to the world at (x, y)."""
        def_name = f"FIRE_{self._next_id}"
        self._next_id += 1

        vrml = f'DEF {def_name} Fire {{ translation {x} {y} 0  scale {FIRE_SCALE_MIN} {FIRE_SCALE_MIN} {FIRE_SCALE_MIN} }}\n'
        self._children.importMFNodeFromString(-1, vrml)

        node = self._robot.getFromDef(def_name)
        fire = {
            "def":        def_name,
            "x":          float(x),
            "y":          float(y),
            "node":       node,
            "strength":   STRENGTH_INITIAL,
            "anim_count": 0,
            "base_z":     0.0,
            "smoke_node": None,
        }
        self._fires[def_name] = fire
        self._spawn_smoke(fire)
        self._burning_positions.add((round(x), round(y)))
        print(f"🔥 Fire spread! {def_name} spawned at ({x}, {y}) "
              f"| total fires: {len(self._fires)}")

    # ── Extinguish detection ──────────────────────────────────────────────────

    def _check_extinguished(self):
        """
        Remove any fire from our registry whose node no longer exists in the world.
        (The drone's Extinguisher calls fire_node.remove() when it hits.)
        """
        to_remove = []
        for def_name, fire in self._fires.items():
            node = self._robot.getFromDef(def_name)
            if node is None:
                to_remove.append(def_name)

        for def_name in to_remove:
            fire = self._fires.pop(def_name)
            pos  = (round(fire["x"]), round(fire["y"]))
            self._burning_positions.discard(pos)
            if fire.get("smoke_node") is not None:
                fire["smoke_node"].remove()
            print(f"💧 {def_name} extinguished — {len(self._fires)} fires remaining")

    # ── Broadcast ─────────────────────────────────────────────────────────────

    def _broadcast(self):
        """
        Send active fire list to the drone and Spot via Emitter.
        Format: "FIRE_1,0.0,0.0,1.0|FIRE_2,3.0,-8.0,2.5" (def,x,y,strength)
        """
        if not self._emitter or not self._fires:
            return

        parts = [
            f"{f['def']},{f['x']:.1f},{f['y']:.1f},{f['strength']:.1f}"
            for f in self._fires.values()
        ]
        message = "|".join(parts)
        self._emitter.send(message.encode("utf-8"))

    def active_fires(self):
        """Return list of (def_name, x, y) for all active fires."""
        return [(f["def"], f["x"], f["y"]) for f in self._fires.values()]


# ──────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

fm = FireManager(robot)
# force ignite a tree on startup
fm._spawn_fire(3, -8)

step = 0

print("=== FIRE MANAGER RUNNING ===")

while robot.step(timestep) != -1:
    step += 1
    fm.step(step)

    if step % 500 == 0:
        fires = fm.active_fires()
        print(f"[step {step}] Active fires: {fires}")
        
