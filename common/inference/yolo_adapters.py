from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from common.inference.contracts import (
    TASK_TYPE_DET,
    TASK_TYPE_KPT,
    TASK_TYPE_SEG,
    InferenceError,
    InferencePayload,
    parse_task_type,
)
from common.inference.yolo_registry import YoloModelRegistry


_COORDINATE_SYSTEM_BY_TASK = {
    TASK_TYPE_DET: "pixel_xyxy",
    TASK_TYPE_KPT: "normalized",
    TASK_TYPE_SEG: "normalized",
}


def _as_list(value: Any) -> list[Any]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    if value is None:
        return []
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        return list(value)
    return [value]


def _as_float_list(values: Any, expected_len: int | None = None) -> list[float]:
    numeric = [float(v) for v in _as_list(values)]
    if expected_len is not None and len(numeric) != expected_len:
        raise ValueError(f"expected length {expected_len}, got {len(numeric)}")
    return numeric


def _as_scalar(value: Any) -> Any:
    if hasattr(value, "item"):
        return value.item()
    values = _as_list(value)
    if len(values) == 1:
        return values[0]
    return value


def _resolve_class_name(names: Any, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, Sequence):
        if 0 <= class_id < len(names):
            return str(names[class_id])
    return str(class_id)


class YoloTaskAdapter:
    def __init__(self, registry: YoloModelRegistry | None = None) -> None:
        self.registry = registry or YoloModelRegistry()

    def run(self, task_type: str, image_path: str, conf: float = 0.25, iou: float = 0.45, max_det: int = 100) -> InferencePayload:
        normalized_task = parse_task_type(task_type)
        errors: list[InferenceError] = []
        items: list[dict[str, Any]] = []

        model = self.registry.get_model(normalized_task)
        model_path = self.registry.get_model_path(normalized_task)
        try:
            results = model.predict(
                image_path,
                conf=conf,
                iou=iou,
                max_det=max_det,
                verbose=False,
            )
        except Exception as exc:
            errors.append(
                {
                    "error_category": "predict_error",
                    "stage": "inference",
                    "message": str(exc),
                }
            )
            return self._build_payload(
                task_type=normalized_task,
                model_path=model_path,
                items=items,
                errors=errors,
            )

        for result in _as_list(results):
            if normalized_task == TASK_TYPE_DET:
                self._append_det_items(result=result, items=items, errors=errors)
            elif normalized_task == TASK_TYPE_KPT:
                self._append_kpt_items(result=result, items=items, errors=errors)
            else:
                self._append_seg_items(result=result, items=items, errors=errors)

        return self._build_payload(
            task_type=normalized_task,
            model_path=model_path,
            items=items,
            errors=errors,
        )

    def _append_det_items(
        self,
        *,
        result: Any,
        items: list[dict[str, Any]],
        errors: list[InferenceError],
    ) -> None:
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return

        class_ids = _as_list(getattr(boxes, "cls", []))
        confidences = _as_list(getattr(boxes, "conf", []))
        bboxes = _as_list(getattr(boxes, "xyxy", []))

        for idx in range(len(class_ids)):
            try:
                class_id = int(_as_scalar(class_ids[idx]))
                confidence = float(_as_scalar(confidences[idx]))
                bbox_xyxy = _as_float_list(bboxes[idx], expected_len=4)
                items.append(
                    {
                        "class_id": class_id,
                        "class_name": _resolve_class_name(getattr(result, "names", None), class_id),
                        "confidence": confidence,
                        "bbox_xyxy": bbox_xyxy,
                    }
                )
            except Exception as exc:
                errors.append(
                    {
                        "error_category": "postprocess_error",
                        "stage": "adapter",
                        "message": str(exc),
                    }
                )

    def _append_kpt_items(
        self,
        *,
        result: Any,
        items: list[dict[str, Any]],
        errors: list[InferenceError],
    ) -> None:
        keypoints = getattr(result, "keypoints", None)
        if keypoints is None:
            return

        coords_by_item = _as_list(getattr(keypoints, "xyn", []))
        conf_by_item = _as_list(getattr(keypoints, "conf", []))
        boxes = getattr(result, "boxes", None)
        class_ids = _as_list(getattr(boxes, "cls", []))
        item_confidences = _as_list(getattr(boxes, "conf", []))

        for idx, coords in enumerate(coords_by_item):
            try:
                class_id = int(_as_scalar(class_ids[idx])) if idx < len(class_ids) else -1
                confidence = float(_as_scalar(item_confidences[idx])) if idx < len(item_confidences) else 0.0
                point_confs = _as_list(conf_by_item[idx]) if idx < len(conf_by_item) else []
                points: list[dict[str, float]] = []
                for point_idx, xy in enumerate(_as_list(coords)):
                    x_value, y_value = _as_float_list(xy, expected_len=2)
                    point_conf = float(_as_scalar(point_confs[point_idx])) if point_idx < len(point_confs) else 0.0
                    points.append({"x": x_value, "y": y_value, "confidence": point_conf})

                items.append(
                    {
                        "class_id": class_id,
                        "class_name": _resolve_class_name(getattr(result, "names", None), class_id),
                        "confidence": confidence,
                        "keypoints": points,
                    }
                )
            except Exception as exc:
                errors.append(
                    {
                        "error_category": "postprocess_error",
                        "stage": "adapter",
                        "message": str(exc),
                    }
                )

    def _append_seg_items(
        self,
        *,
        result: Any,
        items: list[dict[str, Any]],
        errors: list[InferenceError],
    ) -> None:
        masks = getattr(result, "masks", None)
        if masks is None:
            return

        masks_by_item = _as_list(getattr(masks, "xyn", []))
        boxes = getattr(result, "boxes", None)
        class_ids = _as_list(getattr(boxes, "cls", []))
        confidences = _as_list(getattr(boxes, "conf", []))

        for idx, mask_item in enumerate(masks_by_item):
            try:
                class_id = int(_as_scalar(class_ids[idx])) if idx < len(class_ids) else -1
                confidence = float(_as_scalar(confidences[idx])) if idx < len(confidences) else 0.0
                contours = self._normalize_contours(mask_item)
                items.append(
                    {
                        "class_id": class_id,
                        "class_name": _resolve_class_name(getattr(result, "names", None), class_id),
                        "confidence": confidence,
                        "contours": contours,
                    }
                )
            except Exception as exc:
                errors.append(
                    {
                        "error_category": "postprocess_error",
                        "stage": "adapter",
                        "message": str(exc),
                    }
                )

    def _normalize_contours(self, mask_item: Any) -> list[list[list[float]]]:
        points_or_contours = _as_list(mask_item)
        if not points_or_contours:
            return []

        first = _as_list(points_or_contours[0])
        if len(first) == 2 and all(isinstance(v, (int, float)) for v in first):
            return [[_as_float_list(point, expected_len=2) for point in points_or_contours]]

        contours: list[list[list[float]]] = []
        for contour in points_or_contours:
            contours.append([_as_float_list(point, expected_len=2) for point in _as_list(contour)])
        return contours

    def _build_payload(
        self,
        *,
        task_type: str,
        model_path: str,
        items: list[dict[str, Any]],
        errors: list[InferenceError],
    ) -> InferencePayload:
        return {
            "task_type": task_type,
            "items": items,
            "meta": {
                "model_path": model_path,
                "coordinate_system": _COORDINATE_SYSTEM_BY_TASK[task_type],
                "item_count": len(items),
                "error_count": len(errors),
            },
            "errors": errors,
        }
