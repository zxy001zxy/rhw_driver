#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from rhw_model_scheduler.scheduler_core import load_model_manifest
from rhw_model_scheduler.workspace import find_workspace_root


def _engine_path_for(model_path: Path) -> Path:
    if model_path.suffix.lower() == ".engine":
        return model_path
    return model_path.with_suffix(".engine")


def _export_engine(
    *,
    task_type: str,
    model_path: Path,
    imgsz: int,
    device: str,
    half: bool,
    dynamic: bool,
    force: bool,
) -> dict[str, Any]:
    engine_path = _engine_path_for(model_path)
    if engine_path.is_file() and not force:
        return {
            "model_path": str(model_path),
            "engine_path": str(engine_path),
            "status": "exists",
        }

    from ultralytics import YOLO

    yolo_task = {"det": "detect", "kpt": "pose", "seg": "segment", "gauge": "pose"}[task_type]
    model = YOLO(str(model_path), task=yolo_task)
    exported = model.export(
        format="engine",
        imgsz=int(imgsz),
        device=str(device),
        half=bool(half),
        dynamic=bool(dynamic),
        simplify=False,
        verbose=False,
    )
    exported_path = Path(str(exported or engine_path)).resolve()
    if not exported_path.is_file() and engine_path.is_file():
        exported_path = engine_path.resolve()
    if not exported_path.is_file():
        raise RuntimeError(f"TensorRT export did not produce an engine file for {model_path}")
    return {
        "model_path": str(model_path),
        "engine_path": str(exported_path),
        "status": "exported",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Export current scheduler YOLO models to TensorRT engines.")
    parser.add_argument("--workspace-root", default="")
    parser.add_argument("--manifest", default="models/current/manifest.json")
    parser.add_argument("--tasks", nargs="+", default=[])
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--fp32", action="store_true", help="Use FP32 instead of the default FP16 export.")
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    workspace_root = Path(args.workspace_root).resolve() if args.workspace_root else find_workspace_root()
    specs = load_model_manifest(args.manifest, workspace_root=workspace_root)
    selected_tasks = args.tasks or list(specs.keys())
    results: dict[str, Any] = {}
    for task_name in selected_tasks:
        spec = specs.get(task_name)
        if spec is None:
            raise ValueError(f"unknown task_name: {task_name}")
        results[task_name] = _export_engine(
            task_type=spec.task_type,
            model_path=Path(spec.model_path),
            imgsz=args.imgsz,
            device=args.device,
            half=not args.fp32,
            dynamic=args.dynamic,
            force=args.force,
        )
    print(json.dumps(results, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
