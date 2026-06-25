"""
spot.py — Main controller entry point for the Spot ground robot.
Owns the state machine and wires together gait, detection, navigation, extinguish.

State machine:
    REST → STANDUP → PATROL → NAVIGATE → EXTINGUISH → RETURN → REST → ...
A low battery forces RETURN from any state (overrides an active fire response).

Mirrors controllers/mavic2pro/mavic2pro.py's structure — TAKEOFF is replaced
by STANDUP (driving the 12 leg motors from their default pose to Spot's
validated standing pose before gait control begins).
"""

from controller import Supervisor, Keyboard
from gait        import Gait
from detection   import scan
from navigation  import GroundNavigator
from extinguish  import Extinguisher
from battery     import Battery
import coordination as coord

# ──────────────────────────────────────────────
#  Robot & devices
# ──────────────────────────────────────────────
robot    = Supervisor()
timestep = int(robot.getBasicTimeStep())

imu = robot.getDevice("inertial unit"); imu.enable(timestep)
gps = robot.getDevice("gps");           gps.enable(timestep)

camera = robot.getDevice("left head camera"); camera.enable(timestep)
print(f"✅ Camera enabled ({camera.getWidth()}x{camera.getHeight()})")

keyboard = Keyboard(); keyboard.enable(timestep)

fire_receiver = robot.getDevice("fire receiver"); fire_receiver.enable(timestep)
print("📡 Receiver enabled — listening for fire updates on channel 1")

front_ds = robot.getDevice("front distance sensor")
front_ds.enable(timestep)

coord_emitter  = robot.getDevice("coordination emitter")
coord_receiver = robot.getDevice("coordination receiver"); coord_receiver.enable(timestep)
ROBOT_ID = "spot"

# ──────────────────────────────────────────────
#  Modules
# ──────────────────────────────────────────────
gait    = Gait(robot)
nav     = GroundNavigator()
ext     = Extinguisher(robot)
battery = Battery()

# ──────────────────────────────────────────────
#  State machine
# ──────────────────────────────────────────────
state          = "REST"     # Spot starts parked at base, not walking
step           = 0
current_fire   = "FIRE_1"   # updated by fire_manager broadcasts
known_fires    = []         # list of (def_name, x, y, strength) from fire_manager
last_detection = None       # most recent scan() result, kept for NAVIGATE phase

STANDUP_STEPS = 100   # steps to let the leg motors reach the standing pose
standup_count = 0

# How long to rest before patrolling again even with no fire signal.
PATROL_INTERVAL = 3000   # steps
rest_timer      = 0

# Debounce: require the same detection kind for several consecutive frames
# before acting on it, so a single-frame false positive can't send Spot
# chasing a fire that isn't there.
DETECTION_CONFIRM_FRAMES = 3
detection_streak_kind  = None
detection_streak_count = 0

# Coordination with the drone: fires already claimed by the other robot,
# fires it has called for backup on, and its last known state (for shift
# alternation — see the REST handler below).
claimed_fires      = {}     # fire_def -> robot_id
help_requested_for = set()  # fire_def values the other robot has called HELP on
other_robot_state  = None
STATUS_BROADCAST_INTERVAL = 60   # steps


def _commit_to_fire(fire_def, strength):
    """Broadcast CLAIM (going alone) or HELP (need backup) based on strength."""
    if strength >= coord.HELP_STRENGTH_THRESHOLD:
        coord_emitter.send(coord.help_message(fire_def, ROBOT_ID).encode("utf-8"))
        print(f"📣 {fire_def} is strong (strength {strength:.1f}) — calling the drone for backup")
    else:
        coord_emitter.send(coord.claim_message(fire_def, ROBOT_ID).encode("utf-8"))


print("🔋 Press P to send Spot on patrol immediately (works even while resting)")
print("=== SPOT STARTING ===")

while robot.step(timestep) != -1:
    step += 1

    # ── Read fire_manager broadcasts ──────────────────────────────────────
    while fire_receiver.getQueueLength() > 0:
        msg = fire_receiver.getString()
        fire_receiver.nextPacket()
        try:
            # Format: "FIRE_1,0.0,0.0,1.0|FIRE_2,3.0,-8.0,2.5" (def,x,y,strength)
            entries = [e.split(",") for e in msg.split("|") if e]
            known_fires = [(e[0], float(e[1]), float(e[2]), float(e[3]))
                           for e in entries if len(e) == 4]
            if known_fires:
                current_fire = known_fires[0][0]
        except Exception as err:
            print(f"⚠️  Receiver parse error: {err}")

    # ── Read the drone's coordination broadcasts ───────────────────────────
    while coord_receiver.getQueueLength() > 0:
        msg = coord_receiver.getString()
        coord_receiver.nextPacket()
        parsed = coord.parse_message(msg)
        if parsed is None:
            continue
        if parsed["kind"] == "CLAIM":
            claimed_fires[parsed["fire_def"]] = parsed["robot_id"]
        elif parsed["kind"] == "HELP":
            help_requested_for.add(parsed["fire_def"])
        elif parsed["kind"] == "STATUS":
            other_robot_state = parsed["state"]

    if step % STATUS_BROADCAST_INTERVAL == 0:
        coord_emitter.send(coord.status_message(ROBOT_ID, state).encode("utf-8"))

    # ── Debug keyboard shortcuts ────────────────────────────────────────────
    key = keyboard.getKey()

    # ── Battery ──────────────────────────────────────────────────────────────
    if state == "REST":
        battery.charge()
    else:
        battery.drain()

    # Low battery overrides everything else, including an active fire
    # response — same priority rule as the drone.
    if battery.is_low and state not in ("RETURN", "REST"):
        if state == "EXTINGUISH":
            ext.reset()
        nav.fire_gps = None
        state = "RETURN"
        print(f"🔋 Battery low ({battery.percent:.0f}%) — abandoning mission, returning to charge")

    # ── State machine ───────────────────────────────────────────────────────

    if state == "REST":
        rest_timer += 1
        gait.hold_pose(gait.stand_pose())

        manual_trigger = (key == ord('P'))
        fire_signal    = bool(known_fires) and nav.fire_gps is None
        help_call      = bool(help_requested_for) and nav.fire_gps is None
        interval_done  = (rest_timer >= PATROL_INTERVAL
                           and other_robot_state in (None, "REST"))

        if battery.can_launch and (manual_trigger or fire_signal or help_call or interval_done):
            rest_timer    = 0
            standup_count = 0
            state = "STANDUP"
            trigger = ("manual" if manual_trigger else "help call" if help_call
                       else "fire signal" if fire_signal else "interval")
            print(f"🐕 Leaving REST → STANDUP  (battery: {battery.percent:.0f}%, trigger: {trigger})")

    elif state == "STANDUP":
        gait.hold_pose(gait.stand_pose())
        standup_count += 1
        if standup_count >= STANDUP_STEPS:
            state = "PATROL"
            print("✅ Standing — beginning patrol")

    elif state == "PATROL":
        kind, result = scan(camera)

        if kind in ("fire", "smoke") and kind == detection_streak_kind:
            detection_streak_count += 1
        elif kind in ("fire", "smoke"):
            detection_streak_kind  = kind
            detection_streak_count = 1
        else:
            detection_streak_kind  = None
            detection_streak_count = 0

        confirmed = detection_streak_count >= DETECTION_CONFIRM_FRAMES

        if kind == "fire" and confirmed:
            nav.set_fire_position(*gps.getValues()[:2])
            last_detection = result
            state = "NAVIGATE"
            current_match = next((f for f in known_fires if f[0] == current_fire), None)
            if current_match:
                _commit_to_fire(current_fire, current_match[3])
            print(f"🔥 Fire detected during patrol → NAVIGATE  (target: {current_fire})")

        elif kind == "smoke" and confirmed:
            nav.set_fire_position(*gps.getValues()[:2])
            last_detection = result
            state = "NAVIGATE"
            current_match = next((f for f in known_fires if f[0] == current_fire), None)
            if current_match:
                _commit_to_fire(current_fire, current_match[3])
            print(f"🌫️  Smoke detected → NAVIGATE  (target: {current_fire})")

        else:
            eligible = [f for f in known_fires
                        if f[0] not in claimed_fires
                        or claimed_fires[f[0]] == ROBOT_ID
                        or f[0] in help_requested_for]

            if eligible and nav.fire_gps is None:
                fire_def, fx, fy, strength = eligible[0]
                nav.set_fire_position(fx, fy)
                state = "NAVIGATE"
                _commit_to_fire(fire_def, strength)
                print(f"📡 Fire manager reported {fire_def} at ({fx}, {fy}) → NAVIGATE")
            else:
                nav.patrol(gps, imu, gait, front_ds)

    elif state == "NAVIGATE":
        kind, result = scan(camera)
        if kind in ("fire", "smoke"):
            last_detection = result
        else:
            last_detection = None

        arrived = nav.fly_to_fire(gps, imu, gait, last_detection, front_ds)
        if arrived:
            state = "EXTINGUISH"
            print("➡️  Arrived at fire → EXTINGUISH")

    elif state == "EXTINGUISH":
        done = ext.update(gps, gait, fire_def_name=current_fire)
        if done:
            ext.reset()
            nav.fire_gps = None
            state = "RETURN"
            print("✅ Fire out → RETURN")

    elif state == "RETURN":
        arrived = nav.return_to_base(gps, imu, gait, front_ds)
        if arrived:
            state = "REST"
            print(f"🏠 Back at base → REST  (battery: {battery.percent:.0f}%)")

    # ── Periodic status print ───────────────────────────────────────────────
    if step % 60 == 0:
        pos = gps.getValues()
        print(f"[{step:>6}] state={state:<12} | pos=({pos[0]:.1f}, {pos[1]:.1f}) "
              f"| battery={battery.percent:.0f}% | fires={[f[0] for f in known_fires]}")
