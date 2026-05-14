#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
from typing import Any

from rhw_model_scheduler.scheduler_core import ModelTaskSchedulerCore
from rhw_model_scheduler.workspace import find_workspace_root


def _resolve_workspace_path(workspace_root: Path, path: str | Path) -> Path:
    raw = Path(path)
    if raw.is_absolute():
        return raw.resolve()
    return (workspace_root / raw).resolve()


def _sample_indices(total_frames: int, sample_count: int) -> list[int]:
    if total_frames <= 0:
        return []
    if sample_count <= 0 or sample_count >= total_frames:
        return list(range(total_frames))
    if sample_count == 1:
        return [0]
    # Some H.264 files report a final frame that cannot be seek-read reliably.
    # Avoid the tail-most frame for fixed-size deployment benchmarks.
    max_index = max(total_frames - 2, 0)
    values = {round(idx * max_index / (sample_count - 1)) for idx in range(sample_count)}
    return sorted(int(value) for value in values)


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    rank = (len(ordered) - 1) * percentile
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return float(ordered[lower] * (1.0 - weight) + ordered[upper] * weight)


def _stats(values: list[float], error_count: int) -> dict[str, Any]:
    return {
        "count": len(values),
        "mean_ms": float(statistics.fmean(values)) if values else 0.0,
        "p50_ms": _percentile(values, 0.50),
        "p90_ms": _percentile(values, 0.90),
        "p95_ms": _percentile(values, 0.95),
        "min_ms": float(min(values)) if values else 0.0,
        "max_ms": float(max(values)) if values else 0.0,
        "error_count": int(error_count),
    }


def _candidate_frame_indices(target: int, total_frames: int, *, radius: int = 25) -> list[int]:
    max_index = max(total_frames - 2, 0) if total_frames > 1 else max(total_frames - 1, 0)
    candidates = [target]
    for delta in range(1, radius + 1):
        candidates.extend([target + delta, target - delta])
    return [idx for idx in candidates if 0 <= idx <= max_index]


def _extract_frames(video_path: Path, frame_indices: list[int], frame_dir: Path) -> list[tuple[int, Path]]:
    import cv2

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {video_path}")
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    frame_dir.mkdir(parents=True, exist_ok=True)
    frame_infos: list[tuple[int, Path]] = []
    used_source_indices: set[int] = set()
    try:
        for output_idx, frame_idx in enumerate(frame_indices):
            selected_frame_idx: int | None = None
            selected_frame = None
            for candidate_idx in _candidate_frame_indices(int(frame_idx), total_frames):
                if candidate_idx in used_source_indices:
                    continue
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(candidate_idx))
                ok, frame = cap.read()
                if ok and frame is not None:
                    selected_frame_idx = candidate_idx
                    selected_frame = frame
                    break
            if selected_frame_idx is None or selected_frame is None:
                continue
            used_source_indices.add(selected_frame_idx)
            frame_path = frame_dir / f"frame_{output_idx:04d}_src_{selected_frame_idx:06d}.jpg"
            if not cv2.imwrite(str(frame_path), selected_frame):
                raise RuntimeError(f"failed to write frame: {frame_path}")
            frame_infos.append((selected_frame_idx, frame_path))
    finally:
        cap.release()
    return frame_infos


def _video_frame_count(video_path: Path) -> int:
    import cv2

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open video: {video_path}")
    try:
        return int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    finally:
        cap.release()


def _write_summary_csv(path: Path, task_summaries: dict[str, dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["task_name", "count", "mean_ms", "p50_ms", "p90_ms", "p95_ms", "min_ms", "max_ms", "error_count"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for task_name, summary in task_summaries.items():
            row = {"task_name": task_name}
            row.update({field: summary.get(field) for field in fields if field != "task_name"})
            writer.writerow(row)


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark scheduler model latency on a recorded video.")
    parser.add_argument("--workspace-root", default="")
    parser.add_argument("--video", default="sample_data/20260428_142826.mp4")
    parser.add_argument("--sample-count", type=int, default=100)
    parser.add_argument("--tasks", nargs="+", default=[], help="Task names to benchmark. Defaults to all manifest tasks.")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=100)
    parser.add_argument("--device", default="")
    parser.add_argument("--output-root", default="runtime/latency_benchmark")
    parser.add_argument("--no-warmup", action="store_true")
    args = parser.parse_args()

    workspace_root = Path(args.workspace_root).resolve() if args.workspace_root else find_workspace_root()
    video_path = _resolve_workspace_path(workspace_root, args.video)
    total_frames = _video_frame_count(video_path)
    frame_indices = _sample_indices(total_frames, int(args.sample_count))

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = _resolve_workspace_path(workspace_root, args.output_root) / run_id
    frame_dir = run_dir / "frames"
    result_root = run_dir / "scheduler_results"
    frame_infos = _extract_frames(video_path, frame_indices, frame_dir)
    frame_paths = [path for _, path in frame_infos]
    if not frame_infos:
        raise RuntimeError(f"no frames extracted from video: {video_path}")

    core = ModelTaskSchedulerCore(workspace_root=workspace_root, output_root=result_root)
    selected_tasks = args.tasks or list(core.specs.keys())
    params = {"device": args.device.strip()} if args.device.strip() else {}
    params_json = json.dumps(params) if params else ""
    warmup: dict[str, Any] = {}
    if not args.no_warmup:
        for task_name in selected_tasks:
            payload = core.run_task(
                request_id=f"warmup-{task_name}",
                task_name=task_name,
                frame_path=frame_paths[0],
                conf=args.conf,
                iou=args.iou,
                max_det=args.max_det,
                params_json=params_json,
            )
            warmup[task_name] = {
                "ok": bool(payload.get("ok")),
                "latency_ms": float((payload.get("data") or {}).get("latency_ms") or 0.0),
                "message": str(payload.get("message") or ""),
            }

    task_records: dict[str, list[dict[str, Any]]] = {task_name: [] for task_name in selected_tasks}
    task_latencies: dict[str, list[float]] = {task_name: [] for task_name in selected_tasks}
    task_error_counts: dict[str, int] = {task_name: 0 for task_name in selected_tasks}

    for task_name in selected_tasks:
        for frame_idx, frame_path in frame_infos:
            request_id = f"{task_name}-{frame_path.stem}"
            payload = core.run_task(
                request_id=request_id,
                task_name=task_name,
                frame_path=frame_path,
                conf=args.conf,
                iou=args.iou,
                max_det=args.max_det,
                params_json=params_json,
            )
            data = payload.get("data") or {}
            latency_ms = float(data.get("latency_ms") or 0.0)
            ok = bool(payload.get("ok"))
            error_count = int(data.get("error_count") or 0)
            if not ok or error_count:
                task_error_counts[task_name] += max(1, error_count)
            else:
                task_latencies[task_name].append(latency_ms)
            task_records[task_name].append(
                {
                    "task_name": task_name,
                    "source_frame_index": int(frame_idx),
                    "frame_path": str(frame_path),
                    "ok": ok,
                    "latency_ms": latency_ms,
                    "item_count": int(data.get("item_count") or 0),
                    "error_count": error_count,
                    "error_category": str(data.get("error_category") or ""),
                    "result_json_path": str(data.get("result_json_path") or ""),
                }
            )

    task_summaries = {
        task_name: _stats(task_latencies[task_name], task_error_counts[task_name])
        for task_name in selected_tasks
    }
    summary = {
        "run_id": run_id,
        "workspace_root": str(workspace_root),
        "video_path": str(video_path),
        "total_video_frames": total_frames,
        "sample_count_requested": int(args.sample_count),
        "sample_count_actual": len(frame_paths),
        "tasks_requested": selected_tasks,
        "warmup_excluded_from_stats": not args.no_warmup,
        "warmup": warmup,
        "tasks": task_summaries,
        "records": task_records,
    }

    summary_json = run_dir / "summary.json"
    summary_csv = run_dir / "summary.csv"
    summary_json.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_summary_csv(summary_csv, task_summaries)
    print(json.dumps({"summary_json": str(summary_json), "summary_csv": str(summary_csv), "tasks": task_summaries}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
