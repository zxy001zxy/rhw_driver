#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="ROS2 smoke client for rhw_model_scheduler_node.")
    parser.add_argument("--task-name", default="fire_equipment_detection")
    parser.add_argument("--service", default="/rhw/model/task/run")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--params-json", default="")
    args = parser.parse_args()

    import rclpy
    from rhw_msgs.srv import ModelTaskRun

    rclpy.init()
    node = rclpy.create_node("rhw_model_scheduler_smoke")
    client = node.create_client(ModelTaskRun, args.service)

    deadline = time.time() + args.timeout_sec
    while not client.wait_for_service(timeout_sec=0.5):
        if time.time() > deadline:
            node.destroy_node()
            rclpy.shutdown()
            raise TimeoutError(f"service not available: {args.service}")

    request = ModelTaskRun.Request()
    request.request_id = f"smoke-{int(time.time())}"
    request.task_name = args.task_name
    request.conf = args.conf
    request.iou = 0.45
    request.max_det = 100
    request.wait_for_frame_timeout_sec = 3.0
    request.max_frame_age_sec = 2.0
    request.params_json = args.params_json

    future = client.call_async(request)
    while rclpy.ok() and not future.done():
        if time.time() > deadline:
            node.destroy_node()
            rclpy.shutdown()
            raise TimeoutError("model scheduler service call timed out")
        rclpy.spin_once(node, timeout_sec=0.1)

    response = future.result()
    payload = {
        "ok": bool(response.ok),
        "code": response.code,
        "message": response.message,
        "request_id": response.request_id,
        "task_name": response.task_name,
        "task_type": response.task_type,
        "item_count": int(response.item_count),
        "error_count": int(response.error_count),
        "latency_ms": float(response.latency_ms),
        "error_category": response.error_category,
        "result_json_path": response.result_json_path,
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if response.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
