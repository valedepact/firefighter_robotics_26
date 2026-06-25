# Camera processing, fire/smoke detection (OpenCV)

import os

import cv2
import numpy as np

# ──────────────────────────────────────────────
#  Reference templates — the actual flame/smoke billboard textures used by
#  the Fire/Smoke PROTOs, so template matching compares against what the
#  camera will really render rather than a synthetic stand-in.
# ──────────────────────────────────────────────
_TEXTURES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "..", "protos", "textures")

TEMPLATE_MATCH_THRESHOLD = 0.45
TEMPLATE_SCALES          = (16, 32, 64, 96)   # pixel sizes tried during matching


def _load_templates(filenames):
    """Load a list of texture files as grayscale templates. Skips any that fail to load."""
    templates = []
    for name in filenames:
        path = os.path.join(_TEXTURES_DIR, name)
        img  = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            templates.append(img)
    return templates


try:
    FIRE_TEMPLATES  = _load_templates(["fire_00.png", "fire_03.png", "fire_06.png", "fire_09.png"])
    SMOKE_TEMPLATES = _load_templates(["smoke.png"])
    if not FIRE_TEMPLATES or not SMOKE_TEMPLATES:
        print("⚠️  Some detection templates failed to load — falling back to colour-only detection")
except Exception as err:
    print(f"⚠️  Could not load detection templates ({err}) — falling back to colour-only detection")
    FIRE_TEMPLATES, SMOKE_TEMPLATES = [], []


def _template_score(gray_roi, templates):
    """
    Best normalised correlation score between gray_roi and any template,
    tried at a few scales. Returns 0.0 if the ROI is empty or too small.
    """
    roi_h, roi_w = gray_roi.shape[:2]
    if roi_h == 0 or roi_w == 0:
        return 0.0

    best = 0.0
    for template in templates:
        for scale in TEMPLATE_SCALES:
            if scale > roi_h or scale > roi_w:
                continue
            resized = cv2.resize(template, (scale, scale))
            result  = cv2.matchTemplate(gray_roi, resized, cv2.TM_CCOEFF_NORMED)
            best    = max(best, float(result.max()))
    return best


# ──────────────────────────────────────────────
#  HSV colour ranges for fire and smoke
#  (tuned for the Webots flame textures)
# ──────────────────────────────────────────────

# Fire: bright orange-red-yellow
FIRE_LOWER = np.array([5,  150, 150], dtype=np.uint8)
FIRE_UPPER = np.array([35, 255, 255], dtype=np.uint8)

# Smoke: desaturated grey/white puffs. Upper V is capped below 255 (not
# blown-out) to exclude bright sky/sun glare, which is also bright and
# desaturated but should not register as smoke.
SMOKE_LOWER = np.array([0,   0, 160], dtype=np.uint8)
SMOKE_UPPER = np.array([180, 40, 230], dtype=np.uint8)

# Minimum pixel area to count as a real detection (filters noise).
# SMOKE_MIN_AREA is higher than FIRE_MIN_AREA because the smoke colour
# range is hue-agnostic and otherwise lets small specular highlights
# (e.g. on terrain) register as a detection.
FIRE_MIN_AREA  = 30
SMOKE_MIN_AREA = 250


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
    Returns (area, cx, cy, bbox) where cx/cy are pixel coordinates of the
    centroid and bbox is (x, y, w, h), or (0, None, None, None) if nothing found.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, None, None, None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    bbox = cv2.boundingRect(largest)   # (x, y, w, h)

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return area, None, None, bbox

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return area, cx, cy, bbox


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

    area, cx, cy, bbox = _largest_contour_info(mask)

    if area < FIRE_MIN_AREA or cx is None:
        return {"detected": False, "area": 0, "cx": None, "cy": None,
                "offset_x": None, "offset_y": None}

    if FIRE_TEMPLATES:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        x, y, bw, bh = bbox
        roi = gray[y:y + bh, x:x + bw]
        if _template_score(roi, FIRE_TEMPLATES) < TEMPLATE_MATCH_THRESHOLD:
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

    area, cx, cy, bbox = _largest_contour_info(mask)

    if area < SMOKE_MIN_AREA or cx is None:
        return {"detected": False, "area": 0, "cx": None, "cy": None,
                "offset_x": None, "offset_y": None}

    if SMOKE_TEMPLATES:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        x, y, bw, bh = bbox
        roi = gray[y:y + bh, x:x + bw]
        if _template_score(roi, SMOKE_TEMPLATES) < TEMPLATE_MATCH_THRESHOLD:
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