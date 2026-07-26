"""Camera + YOLO detection worker.

Ported from ros2/src/robot_pkg/pkg/vision.py. Runs in a background thread and
publishes detections into a lock-protected list the planner reads directly
(replaces the ROS topic). Uses CAP_DSHOW on Windows / CAP_V4L2 on Linux.
"""
import logging
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

logger = logging.getLogger("vision")


class VisionWorker:
    def __init__(self, camera_index, model_path, config):
        self.camera_index = camera_index
        self.model_path = str(model_path)
        self.frame_width = int(config.get("frame_width", 0)) or None
        self.frame_height = int(config.get("frame_height", 0)) or None
        # Camera->robot mapping + workspace ROI from camera_calibrate.py: a
        # homography (pixel -> robot mm, handles the tilted-table perspective)
        # and the ROI polygon (detections outside it are dropped). Detections are
        # emitted in mm (matching kinematics/planner).
        self._load_calibration(config)
        self.conf_threshold = float(config.get("conf_threshold", 0.7))
        freq = float(config.get("stream_frequency", 24)) or 10.0
        self.period = 1.0 / freq
        self.show_window = bool(config.get("show_window", False))

        self._lock = threading.Lock()
        self._detections = []
        self._last_detection_time = 0.0
        self._last_jpeg = None  # latest annotated frame, JPEG bytes (for the web dashboard)
        self._stop_evt = threading.Event()
        self._thread = None
        self.model = None
        self.cap = None

    def start(self):
        from ultralytics import YOLO  # deferred: heavy import, only needed if vision is used

        logger.info("Loading model: %s", self.model_path)
        self.model = YOLO(self.model_path)

        api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
        self.cap = cv2.VideoCapture(self.camera_index, api)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera index {self.camera_index} could not be opened")
        if self.frame_width and self.frame_height:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        logger.info("Vision online on camera index %s (%gx%g)", self.camera_index, actual_w, actual_h)
        self._thread = threading.Thread(target=self._loop, daemon=True, name="vision")
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self.cap:
            self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()

    def get_detections(self):
        with self._lock:
            return list(self._detections)

    def clear_detections(self):
        with self._lock:
            self._detections = []

    def get_frame_jpeg(self):
        """Latest annotated frame as JPEG bytes, or None before the first frame."""
        with self._lock:
            return self._last_jpeg

    def has_recent_detection(self, max_age=2.0):
        with self._lock:
            if not self._detections:
                return False
            return (time.time() - self._last_detection_time) < max_age

    def _loop(self):
        next_tick = time.monotonic()
        while not self._stop_evt.is_set():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            try:
                self._process_frame(frame)
            except Exception:
                logger.exception("frame processing failed")
            next_tick += self.period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    def _load_calibration(self, config):
        """Load the homography + ROI written by camera_calibrate.py."""
        calib_file = config.get("calibration_file", "camera_calibration.yaml")
        calib_path = Path(__file__).resolve().parent / calib_file
        if not calib_path.exists():
            raise RuntimeError(
                f"calibration file {calib_path} not found -- run camera_calibrate.py "
                "with a checkerboard first to generate it")
        data = yaml.safe_load(calib_path.read_text())
        # Resolution the homography/ROI pixels are expressed in. Frames captured at
        # a different resolution are rescaled to this before the homography applies
        # (see set_frame_size): the camera resamples the same field of view, so a
        # resolution change is a pure per-axis pixel scaling, not a new calibration.
        self._calib_w = int(data.get("image_width", 0)) or None
        self._calib_h = int(data.get("image_height", 0)) or None
        self._H = np.array(data["homography_px_to_mm"], dtype=np.float64)
        self._roi_mm = np.array(data["roi_polygon_mm"], dtype=np.float32)
        self._roi_px_calib = np.array(data["roi_polygon_px"], dtype=np.float64)
        # Effective (current-frame-resolution) transform + ROI; defaults to the
        # calibration resolution until a frame of a known size arrives.
        self.set_frame_size(self._calib_w, self._calib_h)

    def set_frame_size(self, frame_w, frame_h):
        """Adapt the calibrated homography + ROI (expressed in calibration-resolution
        pixels) to frames of a different resolution.

        The camera does not anamorphically stretch its field of view across
        resolutions: it keeps square pixels and, when the requested aspect ratio
        differs from the calibration frame's, centre-crops the longer axis (the
        other axis' field of view is preserved). So a current-frame pixel maps to a
        calibration pixel by a uniform scale plus a centred crop offset -- not an
        independent per-axis scale. Recomputes the effective pixel->mm homography
        (and its inverse) and the ROI polygon in the current frame's pixels. Call
        once per frame with the frame's actual dimensions, before pixel_to_robot /
        _origin_pixel / drawing the ROI."""
        cw, ch = self._calib_w, self._calib_h
        if cw and ch and frame_w and frame_h:
            if frame_w * ch <= cw * frame_h:   # frame relatively narrower -> width cropped
                s = ch / frame_h               # calib px per frame px (uniform)
                x_off = (cw - frame_w * s) / 2.0
                y_off = 0.0
            else:                              # frame relatively wider -> height cropped
                s = cw / frame_w
                x_off = 0.0
                y_off = (ch - frame_h * s) / 2.0
        else:
            s, x_off, y_off = 1.0, 0.0, 0.0
        self._frame_w, self._frame_h = frame_w, frame_h
        # current-frame px -> calibration px (uniform scale + centred crop offset)
        frame_to_calib = np.array([[s,   0.0, x_off],
                                   [0.0, s,   y_off],
                                   [0.0, 0.0, 1.0]])
        self._H_eff = self._H @ frame_to_calib
        self._H_eff_inv = np.linalg.inv(self._H_eff)
        # ROI polygon (calibration px) expressed in current-frame px for drawing
        self._roi_px = ((self._roi_px_calib - [x_off, y_off]) / s).astype(np.int32)

    def pixel_to_robot(self, cx_px, cy_px):
        """Map a pixel (cx, cy) in the current frame's resolution to robot
        (x_mm, y_mm) via the calibrated homography (see set_frame_size)."""
        v = self._H_eff @ np.array([cx_px, cy_px, 1.0])
        return float(v[0] / v[2]), float(v[1] / v[2])

    def _in_roi(self, x_mm, y_mm):
        """True if a robot-frame point falls inside the workspace ROI polygon."""
        return cv2.pointPolygonTest(self._roi_mm, (float(x_mm), float(y_mm)), False) >= 0

    def _origin_pixel(self):
        """Pixel where robot (0, 0) projects to (for the on-frame origin marker),
        in the current frame's resolution."""
        v = self._H_eff_inv @ np.array([0.0, 0.0, 1.0])
        return int(round(v[0] / v[2])), int(round(v[1] / v[2]))

    def _process_frame(self, frame):
        self.set_frame_size(frame.shape[1], frame.shape[0])
        results = self.model(frame, conf=self.conf_threshold, verbose=False)

        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id = int(box.cls[0])
                name = self.model.names[cls_id]

                cx_px = (x1 + x2) // 2
                cy_px = (y1 + y2) // 2

                obj_x_mm, obj_y_mm = self.pixel_to_robot(cx_px, cy_px)
                if not self._in_roi(obj_x_mm, obj_y_mm):
                    continue  # outside the calibrated workspace -- ignore

                detections.append({"name": name, "x": float(obj_x_mm), "y": float(obj_y_mm), "z": 0.0})

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.polylines(frame, [self._roi_px], True, (0, 255, 255), 2)  # ROI outline
        origin_px_x, origin_px_y = self._origin_pixel()
        cv2.circle(frame, (origin_px_x, origin_px_y), 8, (0, 0, 255), -1)
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

        with self._lock:
            self._detections = detections
            self._last_detection_time = time.time()
            if ok:
                self._last_jpeg = jpeg.tobytes()

        if self.show_window:
            cv2.imshow("Robot Vision", frame)
            cv2.waitKey(1)
