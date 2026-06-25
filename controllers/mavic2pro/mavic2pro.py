"""
mavic2pro.py — Main controller entry point for the Mavic 2 Pro drone.
Owns the state machine and wires together flight, detection, navigation, extinguish.

State machine:
    REST → TAKEOFF → PATROL → NAVIGATE → EXTINGUISH → RETURN → REST → ...
A low battery forces RETURN from any state (overrides an active fire response).
"""

from controller import Supervisor, Keyboard
from flight     import FlightController
from detection  import scan
from navigation import Navigator
from extinguish import Extinguisher
from wind       import WindController
from battery    import Battery

# ──────────────────────────────────────────────
#  Robot & devices
# ──────────────────────────────────────────────
robot    = Supervisor()
timestep = int(robot.getBasicTimeStep())

imu  = robot.getDevice("inertial unit"); imu.enable(timestep)
gps  = robot.getDevice("gps");           gps.enable(timestep)
gyro = robot.getDevice("gyro");          gyro.enable(timestep)

camera = robot.getDevice("camera");      camera.enable(timestep)
print(f"✅ Camera enabled ({camera.getWidth()}x{camera.getHeight()})")

keyboard = Keyboard(); keyboard.enable(timestep)

receiver = robot.getDevice("receiver");  receiver.enable(timestep)
print("📡 Receiver enabled — listening for fire updates on channel 1")

front_ds = robot.getDevice("front distance sensor")
front_ds.enable(timestep)

# ──────────────────────────────────────────────
#  Modules
# ──────────────────────────────────────────────
fc      = FlightController(robot)
nav     = Navigator()
ext     = Extinguisher(robot)
wind    = WindController()
battery = Battery()
drone_body = robot.getSelf()

# ──────────────────────────────────────────────
#  State machine
# ──────────────────────────────────────────────
state          = "REST"     # drone starts parked at base, not flying
step           = 0
current_fire   = "FIRE_1"   # updated by fire_manager broadcasts
known_fires    = []         # list of (def_name, x, y) from fire_manager
last_detection = None       # most recent scan() result, kept for NAVIGATE phase

# How long to rest before patrolling again even with no fire signal.
PATROL_INTERVAL = 3000   # steps
rest_timer      = 0
REST_ALTITUDE   = 1.0    # flight.py's set_altitude() floor — a low hover at base,
                          # not a literal touchdown (see flight.py MIN_ALTITUDE)

# Debounce: require the same detection kind for several consecutive frames
# before acting on it, so a single-frame false positive (sun glare, etc.)
# can't send the drone chasing a fire that isn't there.
DETECTION_CONFIRM_FRAMES = 3
detection_streak_kind  = None
detection_streak_count = 0

print("🔋 Press P to send the drone on patrol immediately (works even while resting)")

print("=== MAVIC 2 PRO STARTING ===")

while robot.step(timestep) != -1:
    step += 1

    # ── Read fire_manager broadcasts ──────────────────────────────────────
    while receiver.getQueueLength() > 0:
        msg = receiver.getString()
        receiver.nextPacket()
        try:
            # Format: "FIRE_1,0.0,0.0|FIRE_2,3.0,-8.0"
            entries = [e.split(",") for e in msg.split("|") if e]
            known_fires = [(e[0], float(e[1]), float(e[2])) for e in entries if len(e) == 3]
            if known_fires:
                # Always target the first fire in the list
                current_fire = known_fires[0][0]
        except Exception as err:
            print(f"⚠️  Receiver parse error: {err}")

    # ── Debug keyboard shortcuts ──────────────────────────────────────────
    key = keyboard.getKey()
    if key == Keyboard.UP:
        fc.set_altitude(fc.target_altitude + 0.5)
    if key == Keyboard.DOWN:
        fc.set_altitude(fc.target_altitude - 0.5)

    # ── Wind — live-controllable physical disturbance force ───────────────
    wind.update(key)
    drone_body.addForce(wind.force_vector(), False)

    # ── Battery ─────────────────────────────────────────────────────────────
    if state == "REST":
        battery.charge()
    else:
        battery.drain()

    # Low battery overrides everything else, including an active fire
    # response — "the choice of going to charge overpowers stopping fire".
    if battery.is_low and state not in ("RETURN", "REST"):
        if state == "EXTINGUISH":
            ext.reset()
        nav.fire_gps = None
        fc.set_altitude(7.0)
        state = "RETURN"
        print(f"🔋 Battery low ({battery.percent:.0f}%) — abandoning mission, returning to charge")

    # ── Always run flight controller (handles motors every step) ──────────
    takeoff_done = fc.update(imu, gps, gyro)
    if takeoff_done and state == "TAKEOFF":
        state = "PATROL"

    # ── State machine ─────────────────────────────────────────────────────

    if state == "REST":
        rest_timer += 1
        fc.hover()
        fc.set_altitude(REST_ALTITUDE)

        manual_trigger = (key == ord('P'))
        fire_signal    = bool(known_fires) and nav.fire_gps is None
        interval_done  = rest_timer >= PATROL_INTERVAL

        if battery.can_launch and (manual_trigger or fire_signal or interval_done):
            rest_timer = 0
            fc.set_altitude(7.0)
            state = "TAKEOFF"
            print(f"🚁 Leaving REST → TAKEOFF  (battery: {battery.percent:.0f}%, "
                  f"trigger: {'manual' if manual_trigger else 'fire signal' if fire_signal else 'interval'})")

    elif state == "TAKEOFF":
        pass   # fc.update() handles everything during takeoff

    elif state == "PATROL":
        kind, result = scan(camera)

        # Debounce — only act once the same kind has been seen for several
        # consecutive frames, to filter single-frame false positives.
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
            print(f"🔥 Fire detected during patrol → NAVIGATE  (target: {current_fire})")

        elif kind == "smoke" and confirmed:
            nav.set_fire_position(*gps.getValues()[:2])
            last_detection = result
            state = "NAVIGATE"
            print(f"🌫️  Smoke detected → NAVIGATE  (target: {current_fire})")

        else:
            # Use known_fires from fire_manager to navigate directly if available
            if known_fires and nav.fire_gps is None:
                fx, fy = known_fires[0][1], known_fires[0][2]
                nav.set_fire_position(fx, fy)
                state = "NAVIGATE"
                print(f"📡 Fire manager reported {current_fire} at ({fx}, {fy}) → NAVIGATE")
            else:
                nav.patrol(gps, fc, front_ds)

    elif state == "NAVIGATE":
        kind, result = scan(camera)
        if kind in ("fire", "smoke"):
            last_detection = result
        else:
            last_detection = None

        arrived = nav.fly_to_fire(gps, fc, last_detection, front_ds)
        if arrived:
            state = "EXTINGUISH"
            fc.set_altitude(3.5)
            print("➡️  Arrived at fire → EXTINGUISH")

    elif state == "EXTINGUISH":
        done = ext.update(gps, fc, fire_def_name=current_fire)
        if done:
            ext.reset()
            nav.fire_gps = None   # clear so patrol doesn't re-navigate there
            fc.set_altitude(7.0)
            state = "RETURN"
            print("✅ Fire out → RETURN")

    elif state == "RETURN":
        arrived = nav.return_to_base(gps, fc, front_ds)
        if arrived:
            state = "REST"
            print(f"🏠 Back at base → REST  (battery: {battery.percent:.0f}%)")

    # ── Periodic status print ─────────────────────────────────────────────
    if step % 60 == 0:
        pos = gps.getValues()
        print(f"[{step:>6}] state={state:<12} | pos=({pos[0]:.1f}, {pos[1]:.1f}) "
              f"| alt={pos[2]:.1f}m | battery={battery.percent:.0f}% "
              f"| fires={[f[0] for f in known_fires]}")