from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rhw_model_scheduler.gauge_adapter import GaugeTaskAdapter
from rhw_model_scheduler.scheduler_core import ModelTaskSchedulerCore, load_model_manifest, ResourceSnapshot
from rhw_model_scheduler.latency_benchmark import _candidate_frame_indices, _sample_indices
from rhw_model_scheduler.yolo_registry import YoloModelRegistry


class FakeAdapter:
    def __init__(self, task_type: str) -> None:
        self.task_type = task_type
        self.warmed = False

    def warmup(self, task_type: str):
        self.warmed = True
        return {"task_type": task_type, "backend": "fake", "warmup_error": None}

    def run(self, task_type: str, image_path: str, conf: float, iou: float, max_det: int, device=None):
        return {
            "task_type": task_type,
            "items": [{"class_id": 1, "confidence": 0.9}],
            "errors": [],
            "meta": {
                "backend": "fake",
                "model_path": "fake.pt",
                "device": device,
            },
        }


class FakeGaugeModel:
    def __init__(self, result) -> None:
        self.result = result
        self.names = {0: "meter", 1: "pointer"}

    def predict(self, source, **kwargs):
        return [self.result]


class FakeGaugeRegistry:
    def __init__(self, result) -> None:
        self.model = FakeGaugeModel(result)

    def get_model(self, task_type: str):
        return self.model

    def get_model_status(self, task_type: str):
        return {
            "task_type": task_type,
            "backend": "fake",
            "resolved_model_path": "models/current/colormeter_gauge.engine",
            "class_names": [{"class_id": 0, "class_name": "meter"}, {"class_id": 1, "class_name": "pointer"}],
        }

    def get_model_path(self, task_type: str):
        return "models/current/colormeter_gauge.pt"


def _fake_pose_result(*, detected: bool):
    if not detected:
        return SimpleNamespace(
            boxes=SimpleNamespace(
                cls=np.asarray([], dtype=float),
                conf=np.asarray([], dtype=float),
                xyxy=np.asarray([], dtype=float),
            ),
            keypoints=SimpleNamespace(data=np.asarray([], dtype=float)),
        )
    return SimpleNamespace(
        boxes=SimpleNamespace(
            cls=np.asarray([0, 1], dtype=float),
            conf=np.asarray([0.9, 0.8], dtype=float),
            xyxy=np.asarray([[10, 10, 110, 110], [50, 50, 100, 70]], dtype=float),
        ),
        keypoints=SimpleNamespace(
            data=np.asarray(
                [
                    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                    [[60, 60, 0.95], [100, 60, 0.92], [35, 60, 0.88]],
                ],
                dtype=float,
            )
        ),
    )


class SchedulerCoreTest(unittest.TestCase):
    def _workspace(self):
        temp = tempfile.TemporaryDirectory()
        root = Path(temp.name)
        model_dir = root / "models" / "current"
        model_dir.mkdir(parents=True)
        (model_dir / "det.pt").write_bytes(b"fake")
        (model_dir / "kpt.pt").write_bytes(b"fake")
        (model_dir / "seg.pt").write_bytes(b"fake")
        (model_dir / "gauge.pt").write_bytes(b"fake")
        manifest = {
            "models": [
                {"task": "fire_equipment_detection", "task_type": "det", "current_path": "models/current/det.pt"},
                {"task": "front_panel_pose", "task_type": "kpt", "current_path": "models/current/kpt.pt"},
                {"task": "rust_segmentation", "task_type": "seg", "current_path": "models/current/seg.pt"},
                {"task": "colormeter_gauge", "task_type": "gauge", "current_path": "models/current/gauge.pt"},
            ]
        }
        (model_dir / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        frame = root / "frame.jpg"
        frame.write_bytes(b"fake image")
        self.addCleanup(temp.cleanup)
        return root, frame

    def test_manifest_loads_four_tasks_without_onnx(self) -> None:
        root, _ = self._workspace()
        specs = load_model_manifest(workspace_root=root)
        self.assertEqual(
            sorted(specs),
            ["colormeter_gauge", "fire_equipment_detection", "front_panel_pose", "rust_segmentation"],
        )
        self.assertEqual({spec.task_type for spec in specs.values()}, {"det", "kpt", "seg", "gauge"})

    def test_run_task_writes_summary(self) -> None:
        root, frame = self._workspace()

        def factory(spec):
            return FakeAdapter(spec.task_type)

        core = ModelTaskSchedulerCore(
            workspace_root=root,
            adapter_factory=factory,
            resource_probe=lambda: ResourceSnapshot(system_available_mb=9999.0, gpu_free_mb=9999.0),
        )
        payload = core.run_task(
            request_id="req-1",
            task_name="fire_equipment_detection",
            frame_path=frame,
            params_json='{"device": "cpu"}',
        )
        self.assertTrue(payload["ok"])
        data = payload["data"]
        self.assertEqual(data["task_name"], "fire_equipment_detection")
        self.assertEqual(data["task_type"], "det")
        self.assertEqual(data["item_count"], 1)
        self.assertTrue((root / data["result_json_path"]).is_file())

    def test_unknown_task_returns_invalid_params(self) -> None:
        root, frame = self._workspace()
        core = ModelTaskSchedulerCore(workspace_root=root, adapter_factory=lambda spec: FakeAdapter(spec.task_type))
        payload = core.run_task(request_id="req-2", task_name="missing", frame_path=frame)
        self.assertFalse(payload["ok"])
        self.assertEqual(payload["code"], "INVALID_PARAMS")

    def test_sample_indices_avoid_unstable_tail_frame(self) -> None:
        indices = _sample_indices(total_frames=1415, sample_count=100)
        self.assertEqual(len(indices), 100)
        self.assertEqual(indices[0], 0)
        self.assertLess(indices[-1], 1414)

    def test_candidate_frame_indices_search_nearby_frames(self) -> None:
        candidates = _candidate_frame_indices(target=50, total_frames=100, radius=2)
        self.assertEqual(candidates[:5], [50, 51, 49, 52, 48])

    def test_tensorrt_engine_is_preferred_for_all_task_types(self) -> None:
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        model_paths = {}
        for task_type in ("det", "kpt", "seg", "gauge"):
            model_path = root / f"{task_type}.pt"
            engine_path = root / f"{task_type}.engine"
            model_path.write_bytes(b"pt")
            engine_path.write_bytes(b"engine")
            model_paths[task_type] = str(model_path)
        registry = YoloModelRegistry(model_paths=model_paths)
        for task_type in ("det", "kpt", "seg", "gauge"):
            self.assertTrue(registry.get_model_path(task_type).endswith(f"{task_type}.engine"))

    def test_gauge_adapter_no_detection_returns_ok_empty_payload(self) -> None:
        frame = self._write_test_frame()
        adapter = GaugeTaskAdapter(FakeGaugeRegistry(_fake_pose_result(detected=False)))
        payload = adapter.run("gauge", str(frame))
        self.assertEqual(payload["task_type"], "gauge")
        self.assertEqual(payload["items"], [])
        self.assertEqual(payload["errors"], [])
        self.assertFalse(payload["meta"]["gauge_detected"])
        self.assertEqual(payload["meta"]["gauge_no_detection_reason"], "no_boxes")

    def test_gauge_adapter_detected_payload_shape(self) -> None:
        frame = self._write_test_frame()
        adapter = GaugeTaskAdapter(FakeGaugeRegistry(_fake_pose_result(detected=True)))
        payload = adapter.run("gauge", str(frame))
        self.assertTrue(payload["meta"]["gauge_detected"])
        self.assertEqual(len(payload["items"]), 1)
        item = payload["items"][0]
        self.assertIn("pressure_status", item)
        self.assertAlmostEqual(item["angle_deg"], 0.0)
        self.assertEqual(item["angle_source"], "tip")
        self.assertEqual(len(item["meter_bbox_xyxy"]), 4)
        self.assertEqual(len(item["pointer_bbox_xyxy"]), 4)
        self.assertIn("center", item["keypoints"])
        self.assertIsInstance(item["color_zones"], list)

    def _write_test_frame(self) -> Path:
        import cv2

        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        path = Path(temp.name) / "frame.jpg"
        image = np.zeros((120, 120, 3), dtype=np.uint8)
        image[:, :] = (0, 180, 0)
        self.assertTrue(cv2.imwrite(str(path), image))
        return path


if __name__ == "__main__":
    unittest.main()
