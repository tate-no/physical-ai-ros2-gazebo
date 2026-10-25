import unittest
import cv2
import numpy as np
from color_target_nav.perception import detect, detect_region, VisionLock, Detection


class PerceptionTests(unittest.TestCase):
    def test_colors_and_bearing_signs(self):
        for color, bgr in [('RED', (0, 0, 255)), ('BLUE', (255, 0, 0)), ('YELLOW', (0, 255, 255))]:
            frame = np.zeros((240, 320, 3), np.uint8)
            frame[50:110, 30:90] = bgr
            self.assertGreater(detect(frame, color), 0)
            self.assertLess(detect(frame[:, ::-1], color), 0)
            self.assertIsNone(detect(np.zeros_like(frame), color))

    def test_red_hue_wrap_and_noise(self):
        hsv = np.zeros((100, 100, 3), np.uint8)
        hsv[20:60, 20:60] = (178, 255, 255)
        self.assertIsNotNone(detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 'RED'))
        hsv[:] = 0
        hsv[5:7, 5:7] = (0, 255, 255)
        self.assertIsNone(detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 'RED'))

    def test_detection_bounds_and_fresh_frames_after_target_or_recovery_change(self):
        frame = np.zeros((240, 320, 3), np.uint8)
        frame[70:140, 140:180] = (255, 0, 0)
        detection = detect_region(frame, 'BLUE')
        self.assertLess(detection.low, detection.bearing)
        self.assertGreater(detection.high, detection.bearing)
        lock = VisionLock()
        for _ in range(2):
            self.assertIsNone(lock.update('BLUE', 0, detection))
        self.assertIsNotNone(lock.update('BLUE', 0, detection))
        # The same color after avoiding an obstacle must be observed anew.
        self.assertIsNone(lock.update('BLUE', 1, detection))
        self.assertEqual(lock.count, 1)
        self.assertIsNone(lock.update('BLUE', 1, None))
        self.assertEqual(lock.count, 0)
        self.assertIsNone(lock.update('YELLOW', 2, Detection(0, -.1, .1)))

    def test_vision_confirmation_configuration_validation(self):
        for required in (0, 1, 11, 2.5):
            with self.assertRaises(ValueError):
                VisionLock(required)
