"""
coordination.py — Shared message format for drone/Spot coordination.

Duplicated into both controllers/mavic2pro/ and controllers/spot/ (Webots
runs each controller as its own process in its own folder, so cross-folder
imports aren't practical — duplication is the established pattern already
used for battery.py and detection.py in this project).

Messages are broadcast on the coordination channel (3, distinct from
fire_manager's broadcast channel 1) via each robot's own Emitter/Receiver
pair. Format is plain CSV text, one message per send:

    "CLAIM,FIRE_3,mavic2pro"   — I'm handling this fire alone, don't bother
    "HELP,FIRE_3,mavic2pro"    — this fire is too strong for me alone, join me
    "STATUS,mavic2pro,PATROL"  — my current state (for shift alternation)

A fire's strength (broadcast by fire_manager.py alongside its position) is
the shared decision rule for CLAIM vs HELP — see HELP_STRENGTH_THRESHOLD.
"""

HELP_STRENGTH_THRESHOLD = 3.0   # fire strength at/above which backup is requested


def claim_message(fire_def, robot_id):
    return f"CLAIM,{fire_def},{robot_id}"


def help_message(fire_def, robot_id):
    return f"HELP,{fire_def},{robot_id}"


def status_message(robot_id, state):
    return f"STATUS,{robot_id},{state}"


def parse_message(msg):
    """
    Returns a dict describing the message, or None if malformed.
        {"kind": "CLAIM"|"HELP", "fire_def": str, "robot_id": str}
        {"kind": "STATUS", "robot_id": str, "state": str}
    """
    parts = msg.split(",")
    if len(parts) != 3:
        return None

    kind = parts[0]
    if kind in ("CLAIM", "HELP"):
        return {"kind": kind, "fire_def": parts[1], "robot_id": parts[2]}
    if kind == "STATUS":
        return {"kind": kind, "robot_id": parts[1], "state": parts[2]}
    return None
