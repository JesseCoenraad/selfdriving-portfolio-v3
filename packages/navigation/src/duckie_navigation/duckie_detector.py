import math
import numpy as np

try:
    import onnxruntime as ort
    _ONNX_OK = True
except ImportError:
    _ONNX_OK = False


class DuckieDetector:
    """
    Runs best.onnx (YOLO-format) to detect duckies in a BGR camera image.

    Supports YOLOv5 and YOLOv8 output formats, detected automatically
    from the output tensor shape.

    Usage:
        detector = DuckieDetector("/path/to/best.onnx")
        boxes    = detector.detect(bgr_image)        # all detections
        blocked  = detector.duckie_in_path(bgr_image)  # True if duck ahead
    """

    CONF_THRESHOLD = 0.45
    IOU_THRESHOLD  = 0.45

    def __init__(self, model_path: str):
        if not _ONNX_OK:
            raise ImportError(
                "onnxruntime not found. Add 'onnxruntime' to dependencies-py3.txt."
            )
        self._session = ort.InferenceSession(
            model_path, providers=["CPUExecutionProvider"]
        )
        inp = self._session.get_inputs()[0]
        self._input_name = inp.name
        # Input shape is (1, 3, H, W); fall back to 640x640 if dynamic
        shape = inp.shape
        self._input_h = int(shape[2]) if isinstance(shape[2], int) else 640
        self._input_w = int(shape[3]) if isinstance(shape[3], int) else 640

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(self, image_bgr: np.ndarray) -> list:
        """
        Run inference on a BGR image (OpenCV format).

        Returns a list of (x1, y1, x2, y2, confidence) tuples in original
        pixel coordinates, sorted by confidence descending.
        """
        orig_h, orig_w = image_bgr.shape[:2]
        blob    = self._preprocess(image_bgr)
        outputs = self._session.run(None, {self._input_name: blob})
        return self._postprocess(outputs, orig_w, orig_h)

    def duckie_in_path(self, image_bgr: np.ndarray) -> bool:
        """
        Returns True when a duckie is detected in the trigger zone:
        lower-centre of the image (road directly ahead of the robot).
        """
        return self.get_path_duckie(image_bgr) is not None

    def get_path_duckie(self, image_bgr: np.ndarray):
        """
        Returns the most dangerous duckie as (cx_norm, cy_norm) where both
        values are normalised to [-0.5, 0.5] relative to the image centre,
        or None if no duckie is found in the lower portion of the image.

        cx_norm > 0 → duckie is on the right side  → steer left to avoid
        cx_norm < 0 → duckie is on the left side   → steer right to avoid
        """
        boxes = self.detect(image_bgr)
        if not boxes:
            return None

        h, w = image_bgr.shape[:2]
        # Consider any duckie in the lower two-thirds of the image
        # (wider than the trigger zone to keep tracking during avoidance)
        avoidance_zone_y = h * 0.35

        candidates = []
        for x1, y1, x2, y2, conf in boxes:
            cy = (y1 + y2) / 2.0
            if cy > avoidance_zone_y:
                cx = (x1 + x2) / 2.0
                # Priority: lower in image = closer = more dangerous
                candidates.append((cy, cx, conf))

        if not candidates:
            return None

        # Most dangerous: lowest in image (highest cy value)
        candidates.sort(key=lambda c: c[0], reverse=True)
        _, cx, _ = candidates[0]

        cx_norm = (cx - w / 2.0) / w   # [-0.5, 0.5]
        cy_norm = (candidates[0][0] - h / 2.0) / h

        return cx_norm, cy_norm

    # ------------------------------------------------------------------
    # Pre / post processing
    # ------------------------------------------------------------------

    def _preprocess(self, image_bgr: np.ndarray) -> np.ndarray:
        import cv2
        img = cv2.resize(image_bgr, (self._input_w, self._input_h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        return img.transpose(2, 0, 1)[np.newaxis, ...]   # NCHW

    def _postprocess(self, outputs, orig_w: int, orig_h: int) -> list:
        raw = outputs[0][0]   # remove batch dim → (features, anchors) or (anchors, features)

        # Normalise to (anchors, features)
        # YOLOv8: (5+, num_anchors) e.g. (5, 8400) — features < anchors
        # YOLOv5: (num_anchors, 5+)  e.g. (25200, 6) — anchors > features
        if raw.ndim == 2 and raw.shape[0] < raw.shape[1]:
            raw = raw.T

        scale_x = orig_w / self._input_w
        scale_y = orig_h / self._input_h
        boxes   = []

        for row in raw:
            cx, cy, w, h = float(row[0]), float(row[1]), float(row[2]), float(row[3])
            rest = row[4:]

            if len(rest) == 1:
                # YOLOv8 single-class: one score directly
                conf = float(rest[0])
            elif len(rest) == 2:
                # YOLOv5 single-class: obj_conf × class_conf
                conf = float(rest[0]) * float(rest[1])
            else:
                # Multi-class: obj_conf × max(class_scores) — or just max if v8
                conf = float(rest[0]) * float(np.max(rest[1:])) if len(rest) > 1 else float(rest[0])

            if conf < self.CONF_THRESHOLD:
                continue

            x1 = (cx - w / 2.0) * scale_x
            y1 = (cy - h / 2.0) * scale_y
            x2 = (cx + w / 2.0) * scale_x
            y2 = (cy + h / 2.0) * scale_y
            boxes.append((x1, y1, x2, y2, conf))

        return _nms(boxes, self.IOU_THRESHOLD)


# ------------------------------------------------------------------
# NMS helpers
# ------------------------------------------------------------------

def _nms(boxes: list, iou_thresh: float) -> list:
    if not boxes:
        return []
    boxes = sorted(boxes, key=lambda b: b[4], reverse=True)
    kept  = []
    while boxes:
        best = boxes.pop(0)
        kept.append(best)
        boxes = [b for b in boxes if _iou(best, b) < iou_thresh]
    return kept


def _iou(a: tuple, b: tuple) -> float:
    ix1 = max(a[0], b[0]);  iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]);  iy2 = min(a[3], b[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (area_a + area_b - inter + 1e-9)