from __future__ import annotations

import math
from typing import Any

from rhw_model_scheduler.contracts import TASK_TYPE_GAUGE, InferenceError, InferencePayload, parse_task_type
from rhw_model_scheduler.yolo_registry import YoloModelRegistry


DEFAULT_ANGLE_RES = 720
DEFAULT_INNER_RADIUS_RATIO = 0.65
DEFAULT_OUTER_RADIUS_RATIO = 0.95
DEFAULT_POINTER_MASK_WIDTH = 1
TIP_KEYPOINT_CONF_THRES = 0.5
END_KEYPOINT_CONF_THRES = 0.5


class GaugeTaskAdapter:
    def __init__(self, registry: YoloModelRegistry | None = None) -> None:
        self.registry = registry or YoloModelRegistry()
        self.angle_res = DEFAULT_ANGLE_RES
        self.inner_r_ratio = DEFAULT_INNER_RADIUS_RATIO
        self.outer_r_ratio = DEFAULT_OUTER_RADIUS_RATIO
        self.ptr_mask_w = DEFAULT_POINTER_MASK_WIDTH
        self.tip_keypoint_conf_thres = TIP_KEYPOINT_CONF_THRES
        self.end_keypoint_conf_thres = END_KEYPOINT_CONF_THRES

    def warmup(self, task_type: str, *, image_size: int = 640) -> dict[str, Any]:
        normalized_task = parse_task_type(task_type)
        if normalized_task != TASK_TYPE_GAUGE:
            raise ValueError("GaugeTaskAdapter only supports gauge tasks")
        status = self.registry.get_model_status(normalized_task)
        model = self.registry.get_model(normalized_task)
        try:
            import numpy as np

            dummy = np.zeros((int(image_size), int(image_size), 3), dtype=np.uint8)
            model.predict(dummy, conf=0.25, iou=0.45, max_det=2, verbose=False)
            warmup_error = None
        except Exception as exc:
            warmup_error = str(exc)
        status.update(self.registry.get_model_status(normalized_task))
        status["warmup_error"] = warmup_error
        return status

    def run(
        self,
        task_type: str,
        image_path: str,
        conf: float = 0.25,
        iou: float = 0.45,
        max_det: int = 100,
        device: str | None = None,
    ) -> InferencePayload:
        normalized_task = parse_task_type(task_type)
        if normalized_task != TASK_TYPE_GAUGE:
            raise ValueError("GaugeTaskAdapter only supports gauge tasks")

        errors: list[InferenceError] = []
        model = self.registry.get_model(normalized_task)
        model_status = self.registry.get_model_status(normalized_task)
        model_path = str(model_status.get("resolved_model_path") or self.registry.get_model_path(normalized_task))

        try:
            import cv2

            image = cv2.imread(str(image_path))
            if image is None:
                raise ValueError(f"cannot read frame image: {image_path}")
        except Exception as exc:
            errors.append({"error_category": "frame_read_error", "stage": "input", "message": str(exc)})
            return self._build_payload(
                model_path=model_path,
                model_status=model_status,
                items=[],
                errors=errors,
                detected=False,
                no_detection_reason="frame_read_error",
            )

        try:
            predict_kwargs: dict[str, Any] = {
                "conf": float(conf),
                "iou": float(iou),
                "max_det": max(2, int(max_det or 0)),
                "verbose": False,
            }
            if device is not None:
                predict_kwargs["device"] = device
            result = model.predict(source=image, **predict_kwargs)[0]
        except Exception as exc:
            errors.append({"error_category": "predict_error", "stage": "inference", "message": str(exc)})
            return self._build_payload(
                model_path=model_path,
                model_status=model_status,
                items=[],
                errors=errors,
                detected=False,
                no_detection_reason="predict_error",
            )

        try:
            item, no_detection_reason = self._process_result(result, image)
        except Exception as exc:
            errors.append({"error_category": "postprocess_error", "stage": "adapter", "message": str(exc)})
            return self._build_payload(
                model_path=model_path,
                model_status=model_status,
                items=[],
                errors=errors,
                detected=False,
                no_detection_reason="postprocess_error",
            )

        items = [item] if item is not None else []
        return self._build_payload(
            model_path=model_path,
            model_status=model_status,
            items=items,
            errors=errors,
            detected=item is not None,
            no_detection_reason=no_detection_reason,
        )

    def _process_result(self, result: Any, image: Any) -> tuple[dict[str, Any] | None, str]:
        import numpy as np

        boxes_obj = getattr(result, "boxes", None)
        if boxes_obj is None:
            return None, "no_boxes"

        class_ids = _to_numpy(getattr(boxes_obj, "cls", None)).reshape(-1)
        confidences = _to_numpy(getattr(boxes_obj, "conf", None)).reshape(-1)
        boxes = _to_numpy(getattr(boxes_obj, "xyxy", None))
        if len(class_ids) == 0 or boxes.size == 0:
            return None, "no_boxes"

        keypoints_obj = getattr(result, "keypoints", None)
        if keypoints_obj is None:
            return None, "missing_keypoints"
        keypoints = _to_numpy(getattr(keypoints_obj, "data", None))
        if keypoints.size == 0:
            return None, "missing_keypoints"

        meter_idx = _select_detection_index(class_ids, confidences, 0)
        pointer_idx = _select_detection_index(class_ids, confidences, 1)
        if meter_idx is None:
            return None, "missing_meter"
        if pointer_idx is None or pointer_idx >= len(keypoints):
            return None, "missing_pointer"

        meter_box = [float(value) for value in boxes[meter_idx][:4]]
        pointer_box = [float(value) for value in boxes[pointer_idx][:4]]
        pointer_kpts = keypoints[pointer_idx]
        if len(pointer_kpts) < 3:
            return None, "missing_pointer_keypoints"

        center_pt = pointer_kpts[0]
        tip_pt = pointer_kpts[1]
        end_pt = pointer_kpts[2]
        tip_conf = float(tip_pt[2])
        end_conf = float(end_pt[2])
        if tip_conf >= self.tip_keypoint_conf_thres:
            angle = self._calc_angle(center_pt[:2], tip_pt[:2])
            angle_src = "tip"
            pointer_confidence = tip_conf
        elif end_conf >= self.end_keypoint_conf_thres:
            angle = self._calc_reverse_angle(center_pt[:2], end_pt[:2])
            angle_src = "reverse_end"
            pointer_confidence = end_conf
        else:
            return None, "low_keypoint_confidence"

        height, width = image.shape[:2]
        mx1, my1, mx2, my2 = meter_box
        meter_cx = (mx1 + mx2) / 2.0
        meter_cy = (my1 + my2) / 2.0
        meter_rx = (mx2 - mx1) / 2.0 * 0.9
        meter_ry = (my2 - my1) / 2.0 * 0.9
        if meter_rx <= 0 or meter_ry <= 0:
            return None, "invalid_meter_geometry"

        meter_r_max = max(meter_rx, meter_ry)
        roi_size = int(meter_r_max * 2.2)
        if roi_size <= 0:
            return None, "invalid_meter_geometry"
        x_start = max(0, int(meter_cx - roi_size / 2.0))
        y_start = max(0, int(meter_cy - roi_size / 2.0))
        x_end = min(width, x_start + roi_size)
        y_end = min(height, y_start + roi_size)
        roi = image[y_start:y_end, x_start:x_end]
        if roi.size == 0:
            return None, "empty_meter_roi"

        roi_cx = int(meter_cx - x_start)
        roi_cy = int(meter_cy - y_start)
        unwrapped = self._polar_unwrap(roi, roi_cx, roi_cy, meter_rx, meter_ry)
        if unwrapped.size == 0:
            return None, "empty_unwrapped_roi"

        ptr_mask = self._create_pointer_mask(angle)
        color_zones = self._detect_color_zones(unwrapped, ptr_mask)
        pressure_status, zone_confidence = self._match_zone(angle, color_zones)
        final_confidence = pointer_confidence * zone_confidence

        keypoint_payload = {
            "center": _pack_keypoint(center_pt),
            "tip": _pack_keypoint(tip_pt),
            "end": _pack_keypoint(end_pt),
        }
        item = {
            "pressure_status": pressure_status,
            "confidence": float(final_confidence),
            "angle_deg": float(angle),
            "angle_source": angle_src,
            "meter_bbox_xyxy": meter_box,
            "pointer_bbox_xyxy": pointer_box,
            "keypoints": keypoint_payload,
            "color_zones": [_pack_color_zone(zone) for zone in color_zones],
        }
        return item, ""

    def _polar_unwrap(self, roi: Any, cx: int, cy: int, rx: float, ry: float) -> Any:
        import cv2
        import numpy as np

        r_res = int(max(rx, ry) * (self.outer_r_ratio - self.inner_r_ratio))
        if r_res <= 0:
            return np.zeros((0, 0, 3), dtype=np.uint8)
        angles = np.linspace(0, 2 * np.pi, self.angle_res, endpoint=False)
        ratios = np.linspace(self.inner_r_ratio, self.outer_r_ratio, r_res)
        r_grid, a_grid = np.meshgrid(ratios, angles)
        map_x = (cx + (rx * r_grid) * np.cos(a_grid)).astype(np.float32)
        map_y = (cy + (ry * r_grid) * np.sin(a_grid)).astype(np.float32)
        return cv2.remap(
            roi,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )

    def _create_pointer_mask(self, pointer_angle: float) -> Any:
        import numpy as np

        mask = np.zeros(self.angle_res, dtype=np.uint8)
        center_row = int(pointer_angle * self.angle_res / 360.0) % self.angle_res
        for offset in range(-self.ptr_mask_w // 2, self.ptr_mask_w // 2 + 1):
            mask[(center_row + offset) % self.angle_res] = 1
        return mask

    def _detect_color_zones(self, unwrapped: Any, ptr_mask: Any) -> list[dict[str, Any]]:
        import cv2
        import numpy as np

        color_ranges = {
            "green": [(np.array([35, 40, 40]), np.array([85, 255, 255]))],
            "yellow": [(np.array([20, 40, 40]), np.array([35, 255, 255]))],
            "red": [
                (np.array([0, 40, 40]), np.array([10, 255, 255])),
                (np.array([170, 40, 40]), np.array([180, 255, 255])),
            ],
            "blue": [(np.array([85, 80, 80]), np.array([100, 255, 255]))],
        }
        hsv = cv2.cvtColor(unwrapped, cv2.COLOR_BGR2HSV)
        zones: list[dict[str, Any]] = []
        for color_name, ranges in color_ranges.items():
            mask = np.zeros((self.angle_res, unwrapped.shape[1]), dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            for row_idx in range(self.angle_res):
                if ptr_mask[row_idx]:
                    mask[row_idx, :] = 0
            projection = self._smooth(np.sum(mask > 0, axis=1))
            for peak in self._find_peaks(projection):
                start_angle = peak["start"] * 360.0 / self.angle_res
                end_angle = peak["end"] * 360.0 / self.angle_res
                zones.append(
                    {
                        "color": color_name,
                        "start": start_angle,
                        "end": end_angle,
                        "center": (start_angle + end_angle) / 2.0,
                        "strength": peak["height"],
                    }
                )
        zones.sort(key=lambda item: item["center"])
        return zones

    @staticmethod
    def _smooth(data: Any, window: int = 8) -> Any:
        import numpy as np

        return np.convolve(data, np.ones(window) / window, mode="same")

    @staticmethod
    def _find_peaks(projection: Any, min_height_ratio: float = 0.2, min_width: int = 10) -> list[dict[str, Any]]:
        import numpy as np

        max_val = float(np.max(projection))
        if max_val == 0.0:
            return []
        peaks: list[dict[str, Any]] = []
        row_idx = 0
        min_height = max_val * min_height_ratio
        while row_idx < len(projection):
            if projection[row_idx] >= min_height:
                start = row_idx
                peak_val = float(projection[row_idx])
                while row_idx < len(projection) and projection[row_idx] >= min_height * 0.5:
                    peak_val = max(peak_val, float(projection[row_idx]))
                    row_idx += 1
                end = row_idx - 1
                if end - start >= min_width:
                    peaks.append({"start": start, "end": end, "height": peak_val, "width": end - start})
            else:
                row_idx += 1
        return peaks

    @staticmethod
    def _match_zone(angle: float, zones: list[dict[str, Any]]) -> tuple[str, float]:
        best = "unknown"
        confidence = 0.0
        dist_to_boundary = float("inf")
        matched_zone: dict[str, Any] | None = None
        for zone in zones:
            start = float(zone["start"])
            end = float(zone["end"])
            in_zone = start <= angle <= end if start <= end else angle >= start or angle <= end
            if in_zone:
                d1 = min(abs(angle - start), abs(angle - start + 360.0), abs(angle - start - 360.0))
                d2 = min(abs(angle - end), abs(angle - end + 360.0), abs(angle - end - 360.0))
                dist_to_boundary = min(d1, d2)
                best = str(zone["color"])
                matched_zone = zone
                break
        if best != "unknown" and matched_zone is not None:
            if dist_to_boundary <= 0.0:
                confidence = 0.3
            elif dist_to_boundary < 5.0:
                confidence = 0.3 + (dist_to_boundary / 5.0) * 0.5
            else:
                start = float(matched_zone["start"])
                end = float(matched_zone["end"])
                half_width = (end - start) / 2.0 if start <= end else (end + 360.0 - start) / 2.0
                if half_width > 5.0:
                    safe_ratio = min((dist_to_boundary - 5.0) / (half_width - 5.0), 1.0)
                    confidence = 0.8 + safe_ratio * 0.15
                else:
                    confidence = 0.85
        return best, confidence

    @staticmethod
    def _calc_angle(center: Any, tip: Any) -> float:
        dx = float(tip[0]) - float(center[0])
        dy = float(tip[1]) - float(center[1])
        angle_deg = math.degrees(math.atan2(dy, dx))
        return angle_deg + 360.0 if angle_deg < 0 else angle_deg

    @staticmethod
    def _calc_reverse_angle(center: Any, tail: Any) -> float:
        dx = float(center[0]) - float(tail[0])
        dy = float(center[1]) - float(tail[1])
        angle_deg = math.degrees(math.atan2(dy, dx))
        return angle_deg + 360.0 if angle_deg < 0 else angle_deg

    def _build_payload(
        self,
        *,
        model_path: str,
        model_status: dict[str, Any],
        items: list[dict[str, Any]],
        errors: list[InferenceError],
        detected: bool,
        no_detection_reason: str,
    ) -> InferencePayload:
        return {
            "task_type": TASK_TYPE_GAUGE,
            "items": items,
            "errors": errors,
            "meta": {
                "model_path": model_path,
                "backend": model_status.get("backend"),
                "coordinate_system": "pixel_xyxy_angle_degrees",
                "model_status": model_status,
                "gauge_detected": bool(detected),
                "gauge_no_detection_reason": no_detection_reason,
            },
        }


def _to_numpy(value: Any) -> Any:
    import numpy as np

    if value is None:
        return np.asarray([])
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def _select_detection_index(class_ids: Any, confidences: Any, class_id: int) -> int | None:
    best_idx: int | None = None
    best_confidence = float("-inf")
    for idx, raw_class_id in enumerate(class_ids):
        if int(raw_class_id) != class_id:
            continue
        confidence = float(confidences[idx]) if idx < len(confidences) else 0.0
        if confidence > best_confidence:
            best_confidence = confidence
            best_idx = idx
    return best_idx


def _pack_keypoint(point: Any) -> dict[str, float]:
    return {"x": float(point[0]), "y": float(point[1]), "confidence": float(point[2])}


def _pack_color_zone(zone: dict[str, Any]) -> dict[str, Any]:
    return {
        "color": str(zone["color"]),
        "start": float(zone["start"]),
        "end": float(zone["end"]),
        "center": float(zone["center"]),
        "strength": float(zone["strength"]),
    }
