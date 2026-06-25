import cv2
import numpy as np

def draw_debug(bgr, detection, state="PATROL", height=0.0):
    if bgr is None:
        return None
    img = bgr.copy()
    if detection.get("detected"):
        cx, cy = detection.get("cx", 0), detection.get("cy", 0)
        cv2.circle(img, (cx, cy), 20, (0, 0, 255), 4)
        cv2.putText(img, f"{state} Area:{detection.get('area',0)}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(img, f"Terrain Z: {height:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return img
