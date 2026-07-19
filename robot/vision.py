"""Camera + YOLO detection worker.

Ported from ros2/src/robot_pkg/pkg/vision.py. Runs in a background thread and
publishes detections into a lock-protected list the planner reads directly
(replaces the ROS topic). Uses CAP_DSHOW on Windows / CAP_V4L2 on Linux.
"""
import cmath
import logging
import math
import sys
import threading
import time

import cv2

logger = logging.getLogger("vision")


class VisionWorker:
    def __init__(self, camera_index, model_path, config):
        self.camera_index = camera_index
        self.model_path = str(model_path)
        self.frame_width = int(config.get("frame_width", 0)) or None
        self.frame_height = int(config.get("frame_height", 0)) or None
        # Camera->robot 2D transform (mm), calibrated by camera_calibrate.py:
        #   r = a*p + b,  a = (1/pixels_per_mm) * exp(i*rotate),  p = cx +/- i*cy
        # Detections are emitted in mm (matching kinematics/planner).
        px_per_mm = float(config.get("pixels_per_mm", 1.0)) or 1.0
        rotate = math.radians(float(config.get("rotate", 0.0)))
        self.z_flip = bool(config.get("z_flip", False))
        self._a = (1.0 / px_per_mm) * cmath.exp(1j * rotate)
        self._b = complex(float(config.get("x_0_mm", 0.0)), float(config.get("y_0_mm", 0.0)))
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

    def pixel_to_robot(self, cx_px, cy_px):
        """Map a pixel (cx, cy) to robot (x_mm, y_mm) via the calibrated transform."""
        p = complex(cx_px, -cy_px) if self.z_flip else complex(cx_px, cy_px)
        r = self._a * p + self._b
        return r.real, r.imag

    def _origin_pixel(self):
        """Pixel where robot (0, 0) projects to (for the on-frame origin marker)."""
        p = -self._b / self._a  # r = 0 => a*p + b = 0
        return int(round(p.real)), int(round(-p.imag if self.z_flip else p.imag))

    def _process_frame(self, frame):
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

                detections.append({"name": name, "x": float(obj_x_mm), "y": float(obj_y_mm), "z": 0.0})

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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
