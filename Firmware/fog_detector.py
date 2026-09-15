#!/usr/bin/env python3
"""
==============================================================
 FogGuard — Vision / Fog Detection Module
 Runs on the Raspberry Pi. Imported by fogguard_gateway.py
==============================================================

WHAT THIS DOES
  1. Grabs frames from the camera (thermal preferred, RGB works)
  2. Estimates atmospheric visibility from the frame itself,
     using the Dark Channel Prior (He et al.) to recover the
     transmission map, then mapping it to metres via a
     site-calibrated curve
  3. Applies a dust-vs-fog discriminator tuned for iron-ore sites
  4. Optionally runs a YOLO detector for vehicles/people ahead
  5. Returns a 0-5 severity estimate plus a confidence score

WHY DARK CHANNEL PRIOR
  It gives a per-pixel transmission estimate without needing a
  trained model, so the system still works before site-specific
  training data exists. Once real Bailadila fog/dust images are
  collected, swap in the fine-tuned CNN (see load_cnn()).

INSTALL
  pip3 install opencv-python numpy
  # optional, for object detection:
  pip3 install ultralytics
==============================================================
"""

import time
import logging
import numpy as np
import cv2

log = logging.getLogger("fogguard.vision")

# ---- Dust-vs-fog discrimination -----------------------------
# Iron-ore dust is strongly red/brown shifted (high R, low B).
# Water fog is near-neutral grey. We use the R-B channel ratio
# to tell them apart, because they need different responses:
# dust clears faster and is more localised than monsoon fog.
DUST_RB_RATIO_THRESHOLD = 1.25

# ---- Severity thresholds (metres) — must match ESP32 --------
SEVERITY_BANDS = [
    (150.0, 0),
    (100.0, 1),
    (60.0,  2),
    (30.0,  3),
    (15.0,  4),
    (0.0,   5),
]


def visibility_to_severity(vis_m: float) -> int:
    for threshold, sev in SEVERITY_BANDS:
        if vis_m >= threshold:
            return sev
    return 5


class FogDetector:
    """Camera-based fog/dust severity estimator."""

    # ---- SITE CALIBRATION CONSTANTS -------------------------
    # Fit these at the mine (see estimate_visibility_m docstring).
    VIS_CLEAR_M    = 260.0   # visibility reported at haze_index = 0
    HAZE_EXPONENT  = 1.5     # curve steepness as haze rises

    def __init__(self, camera_index=0, width=640, height=480,
                 use_yolo=False, yolo_weights="yolov8n.pt"):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None

        self.use_yolo = use_yolo
        self.yolo = None
        self.yolo_weights = yolo_weights

        self._smoothed_vis = self.VIS_CLEAR_M

    # ---------------------------------------------------------
    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            log.error("Cannot open camera %s", self.camera_index)
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if self.use_yolo:
            self._load_yolo()
        log.info("Camera opened (%dx%d)", self.width, self.height)
        return True

    def _load_yolo(self):
        try:
            from ultralytics import YOLO
            self.yolo = YOLO(self.yolo_weights)
            log.info("YOLO loaded: %s", self.yolo_weights)
        except Exception as e:
            log.warning("YOLO unavailable (%s) — detection disabled", e)
            self.yolo = None

    def close(self):
        if self.cap:
            self.cap.release()

    # ---------------------------------------------------------
    # Core visibility estimation
    # ---------------------------------------------------------
    @staticmethod
    def dark_channel(img: np.ndarray, patch: int = 15) -> np.ndarray:
        """Minimum across colour channels, then a min-filter."""
        min_ch = np.min(img, axis=2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (patch, patch))
        return cv2.erode(min_ch, kernel)

    @staticmethod
    def estimate_atmospheric_light(img: np.ndarray,
                                   dark: np.ndarray) -> np.ndarray:
        """Brightest 0.1% of dark-channel pixels approximate the airlight."""
        h, w = dark.shape
        n = max(1, int(h * w * 0.001))
        idx = np.argpartition(dark.ravel(), -n)[-n:]
        flat = img.reshape(-1, 3)
        return flat[idx].max(axis=0).astype(np.float64)

    def estimate_haze_index(self, frame: np.ndarray) -> float:
        """
        Returns a haze index in [0, 1]:  0 = perfectly clear, 1 = opaque.

        Derived from the Dark Channel Prior transmission estimate.
        We deliberately stop at a *relative* index rather than claiming
        absolute metres from the physics alone, because Koschmieder's law
        needs a known scene depth that an uncalibrated camera cannot supply.
        The index is converted to metres by a site-calibrated curve below.
        """
        img = frame.astype(np.float64) / 255.0

        dark = self.dark_channel(img)
        A = np.maximum(self.estimate_atmospheric_light(img, dark), 1e-3)

        omega = 0.95
        t = 1.0 - omega * self.dark_channel(img / A)
        t = np.clip(t, 0.02, 1.0)

        # Upper half of frame = distant scene, least affected by road surface
        upper = t[: t.shape[0] // 2, :]
        t_med = float(np.median(upper))

        return float(np.clip(1.0 - t_med, 0.0, 1.0))

    def estimate_visibility_m(self, frame: np.ndarray):
        """
        Returns (visibility_metres, confidence, is_dust).

        CALIBRATION NOTE
        ----------------
        visibility = VIS_CLEAR_M * (1 - haze_index) ** HAZE_EXPONENT

        VIS_CLEAR_M and HAZE_EXPONENT are the two site-calibration
        constants. To calibrate at Bailadila: park the vehicle facing a
        haul road with markers at known distances (25/50/100/200 m),
        record the haze index at several fog levels, note the furthest
        visible marker each time, then fit these two constants.
        Ship the fitted values below.
        """
        h = self.estimate_haze_index(frame)

        visibility = self.VIS_CLEAR_M * ((1.0 - h) ** self.HAZE_EXPONENT)
        visibility = float(np.clip(visibility, 3.0, self.VIS_CLEAR_M))

        # Confidence: low when the frame is too dark, blown out, or flat
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = float(np.mean(gray)) / 255.0
        contrast = float(np.std(gray)) / 128.0
        confidence = float(np.clip(
            0.55 * min(contrast, 1.0) + 0.45 * (1.0 - abs(brightness - 0.5) * 2),
            0.0, 1.0))

        is_dust = self._is_dust(frame)

        # Temporal smoothing so one bad frame cannot slam the brakes
        self._smoothed_vis = 0.8 * self._smoothed_vis + 0.2 * visibility
        return self._smoothed_vis, confidence, is_dust

    @staticmethod
    def _is_dust(frame: np.ndarray) -> bool:
        """
        Iron-ore dust is red/brown shifted; water fog is neutral grey.
        A high R:B ratio therefore indicates airborne ore dust.
        """
        b, g, r = cv2.split(frame.astype(np.float32))
        mean_r = float(np.mean(r)) + 1e-6
        mean_b = float(np.mean(b)) + 1e-6
        return (mean_r / mean_b) > DUST_RB_RATIO_THRESHOLD

    # ---------------------------------------------------------
    # Dehazing — used to feed the detector a clearer image
    # ---------------------------------------------------------
    def dehaze(self, frame: np.ndarray) -> np.ndarray:
        img = frame.astype(np.float64) / 255.0
        dark = self.dark_channel(img)
        A = np.maximum(self.estimate_atmospheric_light(img, dark), 1e-3)

        t = 1.0 - 0.95 * self.dark_channel(img / A)
        t = cv2.GaussianBlur(t, (41, 41), 0)
        t = np.clip(t, 0.15, 1.0)[:, :, None]

        out = (img - A) / t + A
        return np.clip(out * 255.0, 0, 255).astype(np.uint8)

    # ---------------------------------------------------------
    # Object detection (optional)
    # ---------------------------------------------------------
    def detect_objects(self, frame: np.ndarray):
        if self.yolo is None:
            return []
        try:
            results = self.yolo(frame, verbose=False)[0]
            out = []
            # Only report classes that matter on a haul road
            keep = {"person", "car", "truck", "bus", "motorcycle", "bicycle"}
            for box in results.boxes:
                cls_name = results.names[int(box.cls[0])]
                if cls_name not in keep:
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                out.append({
                    "label": cls_name,
                    "conf": round(float(box.conf[0]), 2),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    # crude distance proxy from bbox height
                    "approx_m": round(self._bbox_distance(y2 - y1), 1),
                })
            return out
        except Exception as e:
            log.warning("Detection failed: %s", e)
            return []

    @staticmethod
    def _bbox_distance(box_height_px: float) -> float:
        """
        Very rough pinhole estimate: distance ~ (real_h * focal) / pixel_h.
        Calibrate REAL_H/FOCAL on site; this is only an indicative figure
        shown to the driver, never used for braking decisions.
        """
        REAL_H_M = 3.0        # typical dumper height
        FOCAL_PX = 700.0      # calibrate per camera
        if box_height_px <= 1:
            return 999.0
        return (REAL_H_M * FOCAL_PX) / box_height_px

    # ---------------------------------------------------------
    # Main per-frame entry point
    # ---------------------------------------------------------
    def process_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            return None

        vis_m, conf, is_dust = self.estimate_visibility_m(frame)
        severity = visibility_to_severity(vis_m)

        detections = []
        if severity >= 2 and self.yolo is not None:
            # Dehaze first so the detector sees a clearer image
            detections = self.detect_objects(self.dehaze(frame))
        elif self.yolo is not None:
            detections = self.detect_objects(frame)

        return {
            "visibility_m": round(vis_m, 1),
            "severity": severity,
            "confidence": round(conf, 2),
            "is_dust": is_dust,
            "detections": detections,
            "ts": time.time(),
        }


# ==============================================================
# Standalone test:  python3 fog_detector.py
# ==============================================================
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    det = FogDetector(camera_index=0, use_yolo=False)
    if not det.open():
        raise SystemExit("Camera not available")

    print("Running — press Ctrl+C to stop")
    try:
        while True:
            r = det.process_frame()
            if r:
                print(f"visibility={r['visibility_m']:>6.1f} m  "
                      f"severity={r['severity']}  "
                      f"conf={r['confidence']:.2f}  "
                      f"dust={r['is_dust']}  "
                      f"objects={len(r['detections'])}")
            time.sleep(1)
    except KeyboardInterrupt:
        det.close()
        print("\nStopped.")
