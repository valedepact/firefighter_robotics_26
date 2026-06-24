# Camera processing, fire/smoke detection (OpenCV)

import cv2
import numpy as np

# ──────────────────────────────────────────────
#  HSV colour ranges for fire and smoke
#  (tuned for the Webots flame textures)
# ──────────────────────────────────────────────

# Fire: bright orange-red-yellow
FIRE_LOWER = np.array([5,  150, 150], dtype=np.uint8)
FIRE_UPPER = np.array([35, 255, 255], dtype=np.uint8)

# Smoke: desaturated grey/white puffs
SMOKE_LOWER = np.array([0,   0, 160], dtype=np.uint8)
SMOKE_UPPER = np.array([180, 40, 255], dtype=np.uint8)

# Minimum pixel area to count as a real detection (filters noise)
FIRE_MIN_AREA  = 30
SMOKE_MIN_AREA = 60


def _webots_image_to_bgr(camera):
    """
    Convert a Webots camera image (BGRA bytes) to an OpenCV BGR numpy array.
    Returns None if the camera has no image yet.
    """
    raw = camera.getImage()
    if raw is None:
        return None

    w = camera.getWidth()
    h = camera.getHeight()

    # Webots returns BGRA, 1 byte per channel
    img = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))
    bgr = img[:, :, :3]   # drop alpha
    return bgr


def _largest_contour_info(mask):
    """
    Find the largest contour in a binary mask.
    Returns (area, cx, cy) where cx/cy are pixel coordinates of the centroid,
    or (0, None, None) if nothing found.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, None, None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return area, None, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return area, cx, cy


# ──────────────────────────────────────────────
#  Public API
# ──────────────────────────────────────────────

def detect_fire(camera):
    """
    Analyse the camera image for fire.

    Returns a dict:
        {
            "detected": bool,
            "area":     int,          # pixel area of largest blob
            "cx":       int | None,   # pixel column of centroid
            "cy":       int | None,   # pixel row    of centroid
            "offset_x": float | None, # normalised horizontal offset  (-1 left … +1 right)
            "offset_y": float | None, # normalised vertical   offset  (-1 top  … +1 bottom)
        }
    """
    bgr = _webots_image_to_bgr(camera)
    if bgr is None:
        return {"detected": False, "area": 0, "cx": None, "cy": None,
                "offset_x": None, "offset_y": None}

    hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, FIRE_LOWER, FIRE_UPPER)

    # Clean up salt-and-pepper noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    area, cx, cy = _largest_contour_info(mask)

    if area < FIRE_MIN_AREA or cx is None:
        return {"detected": False, "area": 0, "cx": None, "cy": None,
                "offset_x": None, "offset_y": None}

    w = camera.getWidth()
    h = camera.getHeight()
    offset_x = (cx - w / 2) / (w / 2)   # -1 … +1
    offset_y = (cy - h / 2) / (h / 2)   # -1 … +1

    return {
        "detected": True,
        "area":     int(area),
        "cx":       cx,
        "cy":       cy,
        "offset_x": offset_x,
        "offset_y": offset_y,
    }


def detect_smoke(camera):
    """
    Analyse the camera image for smoke (early warning before fire is visible).

    Returns a dict with the same keys as detect_fire().
    """
    bgr = _webots_image_to_bgr(camera)
    if bgr is None:
        return {"detected": False, "area": 0, "cx": None, "cy": None,
                "offset_x": None, "offset_y": None}

    hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, SMOKE_LOWER, SMOKE_UPPER)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    area, cx, cy = _largest_contour_info(mask)

    if area < SMOKE_MIN_AREA or cx is None:
        return {"detected": False, "area": 0, "cx": None, "cy": None,
                "offset_x": None, "offset_y": None}

    w = camera.getWidth()
    h = camera.getHeight()
    offset_x = (cx - w / 2) / (w / 2)
    offset_y = (cy - h / 2) / (h / 2)

    return {
        "detected": True,
        "area":     int(area),
        "cx":       cx,
        "cy":       cy,
        "offset_x": offset_x,
        "offset_y": offset_y,
    }


def scan(camera):
    """
    Convenience wrapper — checks fire first, then smoke.

    Returns ("fire", result_dict), ("smoke", result_dict), or (None, result_dict).
    result_dict always has the same keys so callers can treat them uniformly.
    """
    fire_result = detect_fire(camera)
    if fire_result["detected"]:
        return "fire", fire_result

    smoke_result = detect_smoke(camera)
    if smoke_result["detected"]:
        return "smoke", smoke_result

    empty = {"detected": False, "area": 0, "cx": None, "cy": None,
             "offset_x": None, "offset_y": None}
    return None, empty