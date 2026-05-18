from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable


def polygon_area(points: Iterable[tuple[float, float]]) -> float:
    pts = list(points)
    if len(pts) < 3:
        return 0.0
    area = 0.0
    for idx, (x1, y1) in enumerate(pts):
        x2, y2 = pts[(idx + 1) % len(pts)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def parse_roi_polygon(value: str | None, width: int, height: int):
    import numpy as np

    if not value:
        return None
    raw = value.strip()
    if Path(raw).exists():
        raw = Path(raw).read_text(encoding="utf-8")
    if raw.startswith("["):
        points = json.loads(raw)
    else:
        points = []
        for item in raw.split(";"):
            if not item.strip():
                continue
            x_text, y_text = item.split(",", 1)
            points.append([float(x_text), float(y_text)])
    if len(points) < 3:
        raise ValueError("roi_polygon must contain at least three points")
    arr = np.array(points, dtype=np.float32)
    if float(arr.max()) <= 1.0:
        arr[:, 0] *= width
        arr[:, 1] *= height
    return arr.astype(np.int32)


def rust_level(rust_ratio: float) -> str:
    if rust_ratio < 0.02:
        return "normal/slight"
    if rust_ratio < 0.10:
        return "mild"
    if rust_ratio < 0.25:
        return "moderate"
    return "severe"


def mask_area(mask, roi_mask=None) -> int:
    binary = mask.astype(bool)
    if roi_mask is not None:
        binary &= roi_mask.astype(bool)
    return int(binary.sum())


def _clip_pixel_point(x: float, y: float, width: int, height: int) -> list[int]:
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("contour point must contain finite numeric coordinates")
    clipped_x = min(max(float(x), 0.0), 1.0)
    clipped_y = min(max(float(y), 0.0), 1.0)
    return [
        min(max(int(round(clipped_x * width)), 0), max(width - 1, 0)),
        min(max(int(round(clipped_y * height)), 0), max(height - 1, 0)),
    ]


def _contour_to_pixel_polygon(contour: Any, width: int, height: int) -> list[list[int]]:
    points = list(contour or [])
    if len(points) < 3:
        raise ValueError("contour must contain at least three points")
    polygon: list[list[int]] = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("contour point must be a two-value sequence")
        polygon.append(_clip_pixel_point(float(point[0]), float(point[1]), width, height))
    if polygon_area((float(x), float(y)) for x, y in polygon) <= 0.0:
        raise ValueError("contour area must be positive")
    return polygon


def _roi_mask(width: int, height: int, roi_polygon=None, roi_mask=None):
    import cv2
    import numpy as np

    if roi_mask is not None:
        mask = np.asarray(roi_mask).astype(np.uint8)
        if mask.shape[:2] != (height, width):
            raise ValueError("roi_mask shape must match image height and width")
        return mask
    if roi_polygon is None:
        return None
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = np.asarray(roi_polygon, dtype=np.int32)
    if polygon.reshape(-1, 2).shape[0] < 3:
        raise ValueError("roi_polygon must contain at least three points")
    cv2.fillPoly(mask, [polygon.reshape(-1, 2)], 1)
    return mask


def _instance_mask(polygons: list[list[list[int]]], width: int, height: int):
    import cv2
    import numpy as np

    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.asarray(polygon, dtype=np.int32) for polygon in polygons], 1)
    return mask


def postprocess_segmentation_items(
    items: Iterable[dict[str, Any]],
    *,
    width: int,
    height: int,
    min_area: int = 0,
    roi_polygon=None,
    roi_mask=None,
) -> dict[str, Any]:
    import numpy as np

    roi = _roi_mask(width, height, roi_polygon=roi_polygon, roi_mask=roi_mask)
    valid_area = mask_area(roi) if roi is not None else int(width * height)
    roi_bool = roi.astype(bool) if roi is not None else None
    union_mask = np.zeros((height, width), dtype=bool)
    instances: list[dict[str, Any]] = []

    for item in items:
        valid_polygons: list[list[list[int]]] = []
        for contour in item.get("contours") or []:
            try:
                valid_polygons.append(_contour_to_pixel_polygon(contour, width, height))
            except (TypeError, ValueError):
                continue
        if not valid_polygons:
            continue

        instance_mask = _instance_mask(valid_polygons, width, height).astype(bool)
        if roi_bool is not None:
            instance_mask &= roi_bool
        instance_area = int(instance_mask.sum())
        if instance_area < min_area:
            continue
        union_mask |= instance_mask

        all_points = [point for polygon in valid_polygons for point in polygon]
        xs = [point[0] for point in all_points]
        ys = [point[1] for point in all_points]
        main_polygon = max(valid_polygons, key=lambda polygon: polygon_area((float(x), float(y)) for x, y in polygon))
        class_name = item.get("class_name", item.get("class", item.get("class_id", "rust")))
        instances.append(
            {
                "class": str(class_name),
                "confidence": float(item.get("confidence", 0.0) or 0.0),
                "bbox": [min(xs), min(ys), max(xs), max(ys)],
                "area": int(instance_area),
                "polygon": main_polygon,
            }
        )

    rust_area = int(union_mask.sum())
    rust_ratio = float(rust_area / valid_area) if valid_area else 0.0
    return {
        "rust_count": len(instances),
        "rust_area": rust_area,
        "valid_area": int(valid_area),
        "rust_ratio": rust_ratio,
        "rust_level": rust_level(rust_ratio),
        "instances": instances,
    }


def build_rust_result(
    *,
    image_name: str,
    model_path: str,
    items: Iterable[dict[str, Any]],
    width: int,
    height: int,
    min_area: int = 0,
    roi_polygon=None,
    roi_mask=None,
    adapter_meta: dict[str, Any] | None = None,
    errors: list[dict[str, Any]] | None = None,
    processing_seconds: float = 0.0,
    task_type: str = "seg",
) -> dict[str, Any]:
    metrics = postprocess_segmentation_items(
        items,
        width=width,
        height=height,
        min_area=min_area,
        roi_polygon=roi_polygon,
        roi_mask=roi_mask,
    )
    return {
        "image_name": image_name,
        "model_path": str(model_path),
        "task_type": task_type,
        "rust_count": metrics["rust_count"],
        "rust_area": metrics["rust_area"],
        "valid_area": metrics["valid_area"],
        "rust_ratio": metrics["rust_ratio"],
        "rust_level": metrics["rust_level"],
        "instances": metrics["instances"],
        "meta": adapter_meta or {},
        "errors": errors or [],
        "processing_seconds": processing_seconds,
    }
