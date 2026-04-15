import cv2
import numpy as np

COLOR_MAP = {
    "banana": [(20, 100, 100), (35, 255, 255)],
    "apple": [(0, 100, 50), (10, 255, 255)],
    "orange": [(10, 150, 100), (25, 255, 255)],
    "lemon": [(25, 100, 100), (35, 255, 255)],
    "lime": [(35, 100, 50), (85, 255, 255)],
    "strawberry": [(0, 120, 50), (10, 255, 255)],
    "watermelon": [(0, 100, 50), (10, 255, 255)],

    "bottle": [(0, 0, 150), (179, 80, 255)],
    "orange bottle": [(10, 150, 100), (25, 255, 255)],
    "red bottle": [(0, 100, 50), (10, 255, 255)],
    "blue bottle": [(100, 100, 50), (130, 255, 255)],
    "green bottle": [(35, 80, 50), (85, 255, 255)],
    "water bottle": [(0, 0, 180), (179, 40, 255)],

    "cup": [(0, 0, 150), (179, 80, 255)],
    "bowl": [(0, 0, 100), (179, 60, 255)],
    "book": [(0, 0, 50), (179, 255, 255)],
    "phone": None,
    "laptop": [(0, 0, 30), (179, 60, 180)],
    "chair": None,
    "bag": None,

    "red shirt": [(0, 100, 50), (10, 255, 255)],
    "blue shirt": [(100, 100, 50), (130, 255, 255)],
    "yellow shirt": [(20, 100, 100), (35, 255, 255)],

    "red": [(0, 100, 50), (10, 255, 255)],
    "orange": [(10, 150, 100), (25, 255, 255)],
    "yellow": [(20, 100, 100), (35, 255, 255)],
    "green": [(35, 80, 50), (85, 255, 255)],
    "blue": [(100, 100, 50), (130, 255, 255)],
    "purple": [(130, 80, 50), (160, 255, 255)],
    "pink": [(160, 80, 100), (179, 255, 255)],
    "white": [(0, 0, 200), (179, 30, 255)],
    "black": [(0, 0, 0), (179, 255, 50)],
}

def get_color_range(task_description: str):
    task_lower = task_description.lower()
    for keyword in sorted(COLOR_MAP.keys(), key=len, reverse=True):
        if keyword in task_lower and COLOR_MAP[keyword] is not None:
            return COLOR_MAP[keyword]
    return None

def color_detected(image_bgr: np.ndarray, hsv_range, min_pixel_ratio=0.01) -> dict:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array(hsv_range[0])
    upper = np.array(hsv_range[1])
    mask = cv2.inRange(hsv, lower, upper)

    total_pixels = image_bgr.shape[0] * image_bgr.shape[1]
    matching_pixels = cv2.countNonZero(mask)
    ratio = matching_pixels / total_pixels

    if ratio < min_pixel_ratio:
        return {
            "detected": False,
            "ratio": round(ratio, 4),
            "direction": None,
            "centroid_x": None,
            "centroid_y": None
        }

    moments = cv2.moments(mask)
    if moments["m00"] > 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
    else:
        cx = image_bgr.shape[1] // 2
        cy = image_bgr.shape[0] // 2

    width = image_bgr.shape[1]
    if cx < width // 3:
        direction = "left"
    elif cx > 2 * width // 3:
        direction = "right"
    else:
        direction = "center"

    return {
        "detected": True,
        "ratio": round(ratio, 4),
        "direction": direction,
        "centroid_x": cx,
        "centroid_y": cy
    }

def add_color_overlay(image_bgr: np.ndarray, mask, color=(0, 255, 0)) -> np.ndarray:
    overlay = image_bgr.copy()
    overlay[mask > 0] = color
    return cv2.addWeighted(image_bgr, 0.7, overlay, 0.3, 0)