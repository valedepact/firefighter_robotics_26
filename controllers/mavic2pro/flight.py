# Takeoff, hovering, motor mixing, altitude control
"""
flight.py — Mavic 2 Pro flight control
Handles: takeoff, hovering, altitude hold, attitude stabilisation, motor mixing.
mavic2pro.py creates one FlightController instance and calls .update() every step.
"""

# ──────────────────────────────────────────────
#  Tuning constants
# ──────────────────────────────────────────────
K_VERTICAL_THRUST = 68.5   # base thrust to counteract gravity
K_VERTICAL_P      = 2.5    # altitude P-gain
K_ROLL_P          = 30.0   # roll  stabilisation P-gain
K_PITCH_P         = 20.0   # pitch stabilisation P-gain
K_ROLL_RATE       = 1.0    # roll  rate damping
K_PITCH_RATE      = 1.0    # pitch rate damping
K_YAW_RATE        = 3.0    # yaw   rate damping

TAKEOFF_ALTITUDE  = 7.0    # metres — cruise altitude after takeoff
TAKEOFF_THRESHOLD = 6.0    # metres — altitude that marks takeoff complete
THRUST_RAMP_STEP  = 0.01   # how fast thrust builds during takeoff (per timestep)
MIN_ALTITUDE      = 1.0    # safety floor
ALTITUDE_BIAS     = 1.2    # empirical offset to keep the drone level at cruise


class FlightController:
    """
    Wraps all low-level motor maths so mavic2pro.py never touches motor values directly.

    Usage
    -----
        fc = FlightController(robot)
        ...
        while robot.step(timestep) != -1:
            done = fc.update(imu, gps, gyro)
            if done:              # takeoff finished
                state = "PATROL"
            fc.set_velocity(vx=1.0, vy=0.0, yaw_rate=0.0)   # optional
    """

    def __init__(self, robot):
        # Motors
        self._fl = robot.getDevice("front left propeller")
        self._fr = robot.getDevice("front right propeller")
        self._rl = robot.getDevice("rear left propeller")
        self._rr = robot.getDevice("rear right propeller")

        for m in [self._fl, self._fr, self._rl, self._rr]:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

        # State
        self.target_altitude = TAKEOFF_ALTITUDE
        self._thrust_ramp    = 0.0
        self._taking_off     = True

        # External velocity commands (set by navigation.py)
        self._cmd_pitch = 0.0   # forward/back tilt
        self._cmd_roll  = 0.0   # left/right tilt
        self._cmd_yaw   = 0.0   # yaw rate override

        print("FlightController ready — beginning takeoff ramp")

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, imu, gps, gyro):
        """
        Must be called once per simulation step.

        Reads sensors, computes motor outputs, applies them.
        Returns True the first time takeoff is confirmed complete.
        """
        roll, pitch, _ = imu.getRollPitchYaw()
        altitude        = gps.getValues()[2]
        roll_rate, pitch_rate, yaw_rate = gyro.getValues()

        # Ramp thrust smoothly during takeoff
        if self._taking_off:
            self._thrust_ramp = min(self._thrust_ramp + THRUST_RAMP_STEP, 1.0)
            base_thrust = K_VERTICAL_THRUST * self._thrust_ramp
        else:
            base_thrust = K_VERTICAL_THRUST

        # Altitude hold (P controller)
        vertical_input = K_VERTICAL_P * (self.target_altitude - altitude + ALTITUDE_BIAS)

        # Attitude stabilisation — blend sensor + external commands
        roll_input  = K_ROLL_P  * (roll  + self._cmd_roll)  + K_ROLL_RATE  * roll_rate
        pitch_input = K_PITCH_P * (pitch + self._cmd_pitch) + K_PITCH_RATE * pitch_rate
        yaw_input   = K_YAW_RATE * (yaw_rate + self._cmd_yaw)

        # Motor mixing (Mavic 2 Pro sign convention)
        self._fl.setVelocity( (base_thrust + vertical_input - roll_input + pitch_input + yaw_input))
        self._fr.setVelocity(-(base_thrust + vertical_input + roll_input + pitch_input - yaw_input))
        self._rl.setVelocity(-(base_thrust + vertical_input - roll_input - pitch_input + yaw_input))
        self._rr.setVelocity( (base_thrust + vertical_input + roll_input - pitch_input - yaw_input))

        # Check takeoff completion
        if self._taking_off and altitude >= TAKEOFF_THRESHOLD:
            self._taking_off = False
            print(f"✅ Takeoff complete — altitude {altitude:.2f} m → PATROL")
            return True   # signal to main loop: takeoff done

        return False

    def set_velocity(self, pitch=0.0, roll=0.0, yaw=0.0):
        """
        Called by navigation.py to tilt the drone and move it.

        Parameters (all normalised, suggested range -1 … +1)
        ----------
        pitch  : positive = fly forward, negative = fly backward
        roll   : positive = fly right,   negative = fly left
        yaw    : positive = rotate right, negative = rotate left
        """
        self._cmd_pitch = pitch
        self._cmd_roll  = roll
        self._cmd_yaw   = yaw

    def hover(self):
        """Stop all lateral movement — hold position."""
        self.set_velocity(0.0, 0.0, 0.0)

    def set_altitude(self, metres):
        """Change the target cruise altitude (clamped to MIN_ALTITUDE)."""
        self.target_altitude = max(MIN_ALTITUDE, metres)
        print(f"Target altitude → {self.target_altitude:.1f} m")

    def descend_to(self, metres):
        """Alias for set_altitude — used when positioning over a fire."""
        self.set_altitude(metres)

    @property
    def is_taking_off(self):
        return self._taking_off