"""
battery.py — Simple battery model for the Mavic 2 Pro.

Drains while airborne, recharges while resting at base. Self-contained
(no dependency on the rest of the state machine) so a future second
drone can just instantiate its own Battery() rather than needing a
shared/rewritten model.
"""

BATTERY_MAX               = 100.0
BATTERY_DRAIN_PER_STEP    = 0.02   # full charge lasts ~5000 steps (~40s) of continuous flight
BATTERY_CHARGE_PER_STEP   = 0.5    # full recharge takes ~200 steps (~1.6s)
BATTERY_LOW_THRESHOLD     = 25.0   # at/below this, return-to-charge overrides everything else
MIN_BATTERY_TO_LAUNCH     = 30.0   # must have at least this much charge to take off


class Battery:
    def __init__(self):
        self.percent = BATTERY_MAX

    def drain(self, amount=BATTERY_DRAIN_PER_STEP):
        self.percent = max(0.0, self.percent - amount)

    def charge(self, amount=BATTERY_CHARGE_PER_STEP):
        self.percent = min(BATTERY_MAX, self.percent + amount)

    @property
    def is_low(self):
        return self.percent <= BATTERY_LOW_THRESHOLD

    @property
    def can_launch(self):
        return self.percent >= MIN_BATTERY_TO_LAUNCH
