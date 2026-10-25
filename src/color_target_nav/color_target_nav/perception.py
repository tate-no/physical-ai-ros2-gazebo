"""HSV segmentation; no GUI or ROS dependencies."""
import math
from dataclasses import dataclass
import cv2
import numpy as np

RANGES = {
    'RED': [((0, 80, 40), (10, 255, 255)), ((170, 80, 40), (179, 255, 255))],
    'BLUE': [((100, 80, 40), (130, 255, 255))],
    'YELLOW': [((20, 80, 40), (40, 255, 255))],
}


@dataclass(frozen=True)
class Detection:
    bearing: float
    low: float
    high: float


class VisionLock:
    """Confirm current color in fresh consecutive images after each scan epoch."""
    def __init__(self, required=3):
        if not isinstance(required, int) or not 2 <= required <= 10:
            raise ValueError('vision_confirm_frames must be an integer from 2 to 10')
        self.required = required
        self.reset(None, None)

    def reset(self, color, epoch):
        self.token = color, epoch
        self.count = 0
        self.detection = None

    def update(self, color, epoch, detection):
        if self.token != (color, epoch):
            self.reset(color, epoch)
        if detection is None:
            self.count = 0
            self.detection = None
            return None
        self.count = min(self.required, self.count + 1)
        self.detection = detection
        return detection if self.count >= self.required else None


def detect_region(frame, color, min_area=80, horizontal_fov=1.57):
    """Camera bearing and angular extent of the largest matching HSV region."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(low), np.array(high))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < min_area:
        return None
    x, _, w, _ = cv2.boundingRect(contour)
    focal = frame.shape[1] / (2 * math.tan(horizontal_fov / 2))
    bearing = math.atan((frame.shape[1] / 2 - x - w / 2) / focal)
    low = math.atan((frame.shape[1] / 2 - x - w) / focal)
    high = math.atan((frame.shape[1] / 2 - x) / focal)
    return Detection(bearing, low, high)


def detect(frame, color, min_area=80, horizontal_fov=1.57):
    """Compatibility helper returning just the bearing."""
    region = detect_region(frame, color, min_area, horizontal_fov)
    return region.bearing if region is not None else None
