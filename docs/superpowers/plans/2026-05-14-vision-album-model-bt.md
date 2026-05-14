# Vision Album Model BT Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a TYPE_VISION waypoint run capture, synchronous HTTPS album upload, and model inference as three separate behavior-tree steps.

**Architecture:** `rhw_task_scheduler` stays responsible for orchestration only. `CaptureImage` writes capture metadata to the blackboard, `UploadInspectionAlbum` calls a ROS service on `inspection_reporter_node`, and `RunModelTask` calls `/rhw/model/task/run`; platform HTTPS details remain inside `rhw_udp_mqtt_bridge`.

**Tech Stack:** ROS 2 Humble, `rclpy`, `py_trees`, `rosidl`, `requests`, existing `rhw_msgs` service/message package.

---

## File Structure

- Commit hygiene for this dirty workspace: before each commit, run `git diff --` with the paths named under that task's **Files** list. If an existing file already contains unrelated pre-existing changes, stage only this task's hunks with `git add -p` followed by the exact path from the **Files** list. New files can be staged with `git add` followed by their exact path.
- Create: `rhw_msgs/srv/InspectionAlbumUpload.srv`  
  Defines the synchronous album upload request/response used by the behavior tree.
- Modify: `rhw_msgs/CMakeLists.txt`  
  Adds `srv/InspectionAlbumUpload.srv` and existing `srv/ModelTaskRun.srv` to `srv_files`.
- Modify: `rhw_msgs/msg/WaypointTask.msg`  
  Updates the TYPE_VISION example so `inference_type` is a real model `task_name`.
- Modify: `rhw_udp_mqtt_bridge/rhw_udp_mqtt_bridge/inspection_reporter_node.py`  
  Adds `/inspection/album_report/upload` service and refactors HTTPS upload into a result-returning helper.
- Modify: `rhw_udp_mqtt_bridge/config/udp_mqtt_bridge.yaml`  
  Adds `album_upload_service`.
- Create: `rhw_task_scheduler/rhw_task_scheduler/bt_actions/vision_actions.py`  
  Contains `UploadInspectionAlbum` and `RunModelTask` behavior-tree leaves.
- Modify: `rhw_task_scheduler/rhw_task_scheduler/bt_actions/ptz_actions.py`  
  Stores capture URL and file size on the blackboard; stops triggering upload from the capture node.
- Modify: `rhw_task_scheduler/rhw_task_scheduler/mission_bt_node.py`  
  Declares parameters, registers blackboard keys, clears per-waypoint capture/model state, and inserts new behavior nodes.
- Modify: `rhw_task_scheduler/config/task_scheduler.yaml`  
  Adds album upload and model service names/timeouts.
- Modify: `rhw_task_scheduler/rhw_task_scheduler/mission_test_mocks.py`  
  Adds mock album upload and model task services and includes `inference_type` in the mock vision waypoint.
- Modify: `rhw_task_scheduler/launch/mission_test.launch.py`  
  Adds real/mock toggles and service remapping parameters for album upload and model task run.
- Modify: `rhw_task_scheduler/README.md`, `rhw_udp_mqtt_bridge/README.md`, `rhw_model_scheduler_ws/README.md`  
  Documents the new flow and test commands.

---

### Task 1: Add ROS Interfaces

**Files:**
- Create: `rhw_msgs/srv/InspectionAlbumUpload.srv`
- Modify: `rhw_msgs/CMakeLists.txt`
- Modify: `rhw_msgs/msg/WaypointTask.msg`

- [ ] **Step 1: Create the album upload service interface**

Create `rhw_msgs/srv/InspectionAlbumUpload.srv` with exactly:

```text
# 服务: /inspection/album_report/upload
# 同步上传巡检相册/抓拍结果，供行为树判定视觉点成败

# 平台任务 ID；手动测试可由任务调度节点生成
string task_id

# 抓拍点位 ID
string point_id

# 抓拍点位名称
string point_name

# 本地图片文件路径
string image_path

# 云台/摄像机抓拍接口返回的图片 URL
string capture_url

# 图片文件大小，单位字节
uint32 file_size

---

# true 表示 HTTPS 上报已完成且平台响应成功
bool ok

# 机器可读结果码，例如 OK、DISABLED、CONFIG_ERROR、POST_FAILED
string code

# 人可读结果说明
string message

# 外层 HTTPS 请求 traceId
string trace_id

# HTTP 状态码；未发起 HTTP 请求时为 0
int32 http_status

# 平台响应正文，最多由上报节点截断到安全长度
string response_body
```

- [ ] **Step 2: Register both service interfaces in `rhw_msgs/CMakeLists.txt`**

In the `set(srv_files` block, add these entries near the existing mission and capture services:

```cmake
  "srv/InspectionAlbumUpload.srv"
  "srv/ModelTaskRun.srv"
```

Keep the existing `srv/CaptureImage.srv`, `srv/StartMission.srv`, `srv/StopMission.srv`, and `srv/PauseMission.srv` entries unchanged.

- [ ] **Step 3: Update the vision waypoint example**

In `rhw_msgs/msg/WaypointTask.msg`, replace the TYPE_VISION example with:

```text
# 示例（TYPE_VISION）:
#   {"azimuth": 180.0, "elevation": 0.0, "channel": 1,
#    "zoom": 6.5, "azimuth_speed": 50, "elevation_speed": 50,
#    "inference_type": "fire_equipment_detection"}
```

- [ ] **Step 4: Build the interface package**

Run:

```bash
colcon build --packages-select rhw_msgs
```

Expected: build succeeds and generates `rhw_msgs/srv/InspectionAlbumUpload` and `rhw_msgs/srv/ModelTaskRun`.

- [ ] **Step 5: Inspect generated interfaces**

Run:

```bash
source install/setup.bash
ros2 interface show rhw_msgs/srv/InspectionAlbumUpload
ros2 interface show rhw_msgs/srv/ModelTaskRun
```

Expected: both commands print the request and response fields.

- [ ] **Step 6: Commit interfaces**

Run:

```bash
git add rhw_msgs/srv/InspectionAlbumUpload.srv rhw_msgs/CMakeLists.txt rhw_msgs/msg/WaypointTask.msg
git commit -m "✨ feat: add inspection album upload interface"
```

---

### Task 2: Add Synchronous Upload Service to Inspection Reporter

**Files:**
- Modify: `rhw_udp_mqtt_bridge/rhw_udp_mqtt_bridge/inspection_reporter_node.py`
- Modify: `rhw_udp_mqtt_bridge/config/udp_mqtt_bridge.yaml`

- [ ] **Step 1: Import the new service type**

In `inspection_reporter_node.py`, change the import section from:

```python
from rhw_msgs.msg import InspectionAlbumReport
```

to:

```python
from rhw_msgs.msg import InspectionAlbumReport
from rhw_msgs.srv import InspectionAlbumUpload
```

- [ ] **Step 2: Declare and read the service name parameter**

In `_declare_parameters()`, add:

```python
self.declare_parameter('album_upload_service', '/inspection/album_report/upload')
```

In `_read_parameters()`, add:

```python
self._album_upload_service = str(self.get_parameter('album_upload_service').value)
```

- [ ] **Step 3: Create the service server**

After `self.create_subscription(...)` in `__init__`, add:

```python
self.create_service(
    InspectionAlbumUpload,
    self._album_upload_service,
    self._handle_album_upload,
)
```

Update the startup log to include:

```python
f'upload_service={self._album_upload_service} '
```

- [ ] **Step 4: Add request conversion helper**

Add this method to `InspectionReporterNode`:

```python
def _album_upload_request_to_msg(
    self,
    request: InspectionAlbumUpload.Request,
) -> InspectionAlbumReport:
    msg = InspectionAlbumReport()
    msg.task_id = str(request.task_id)
    msg.point_id = str(request.point_id)
    msg.point_name = str(request.point_name or request.point_id)
    msg.image_path = str(request.image_path)
    msg.capture_url = str(request.capture_url)
    msg.file_size = max(0, min(int(request.file_size), 4294967295))
    return msg
```

- [ ] **Step 5: Add a result-returning upload helper**

Replace the topic callback body with a call to a new helper. The helper should be:

```python
def _report_album(self, msg: InspectionAlbumReport) -> dict[str, Any]:
    if not self._enabled:
        return {
            'ok': False,
            'code': 'DISABLED',
            'message': 'inspection reporter is disabled',
            'trace_id': '',
            'http_status': 0,
            'response_body': '',
        }
    if not self._album_report_url:
        return {
            'ok': False,
            'code': 'CONFIG_ERROR',
            'message': 'album_report_url is empty',
            'trace_id': '',
            'http_status': 0,
            'response_body': '',
        }

    try:
        payload = self._build_album_payload(msg)
    except Exception as exc:
        return {
            'ok': False,
            'code': 'PAYLOAD_ERROR',
            'message': f'build album report payload failed: {exc}',
            'trace_id': '',
            'http_status': 0,
            'response_body': '',
        }

    result = self._post_with_retries(payload, msg)
    result['trace_id'] = str(payload.get('traceId', ''))
    return result
```

- [ ] **Step 6: Update the topic callback to use the helper**

Set `_on_album_report()` to:

```python
def _on_album_report(self, msg: InspectionAlbumReport) -> None:
    result = self._report_album(msg)
    if result['ok']:
        return
    self.get_logger().error(
        'Album report topic upload failed: '
        f'task_id={msg.task_id} point_id={msg.point_id} '
        f'code={result["code"]} message={result["message"]}'
    )
```

- [ ] **Step 7: Add the service handler**

Add this method:

```python
def _handle_album_upload(
    self,
    request: InspectionAlbumUpload.Request,
    response: InspectionAlbumUpload.Response,
) -> InspectionAlbumUpload.Response:
    msg = self._album_upload_request_to_msg(request)
    result = self._report_album(msg)
    response.ok = bool(result['ok'])
    response.code = str(result['code'])
    response.message = str(result['message'])
    response.trace_id = str(result.get('trace_id', ''))
    response.http_status = int(result.get('http_status', 0))
    response.response_body = str(result.get('response_body', ''))[:2000]
    return response
```

- [ ] **Step 8: Make `_post_with_retries()` return a result**

Change `_post_with_retries()` signature to:

```python
def _post_with_retries(
    self,
    payload: dict[str, Any],
    msg: InspectionAlbumReport,
) -> dict[str, Any]:
```

Inside the success branch, return:

```python
return {
    'ok': True,
    'code': 'OK',
    'message': detail,
    'trace_id': str(payload.get('traceId', '')),
    'http_status': response.status_code,
    'response_body': response.text[:2000],
}
```

Inside the non-success branch, keep track of the last failure:

```python
last_result = {
    'ok': False,
    'code': 'POST_FAILED',
    'message': detail,
    'trace_id': str(payload.get('traceId', '')),
    'http_status': response.status_code,
    'response_body': response.text[:2000],
}
```

Inside the exception branch, set:

```python
last_result = {
    'ok': False,
    'code': 'POST_EXCEPTION',
    'message': str(exc),
    'trace_id': str(payload.get('traceId', '')),
    'http_status': 0,
    'response_body': '',
}
```

At the end of the method, after the exhausted-retries log, return:

```python
return last_result
```

Initialize `last_result` before the loop:

```python
last_result = {
    'ok': False,
    'code': 'POST_NOT_ATTEMPTED',
    'message': 'post was not attempted',
    'trace_id': str(payload.get('traceId', '')),
    'http_status': 0,
    'response_body': '',
}
```

- [ ] **Step 9: Add config parameter**

In `rhw_udp_mqtt_bridge/config/udp_mqtt_bridge.yaml`, under `inspection_reporter_node.ros__parameters`, add:

```yaml
    album_upload_service: "/inspection/album_report/upload"
```

- [ ] **Step 10: Run syntax check**

Run:

```bash
python3 -m compileall rhw_udp_mqtt_bridge
```

Expected: compile completes without syntax errors.

- [ ] **Step 11: Commit reporter service**

Run:

```bash
git add rhw_udp_mqtt_bridge/rhw_udp_mqtt_bridge/inspection_reporter_node.py rhw_udp_mqtt_bridge/config/udp_mqtt_bridge.yaml
git commit -m "✨ feat: expose inspection album upload service"
```

---

### Task 3: Store Capture Metadata on the Blackboard

**Files:**
- Modify: `rhw_task_scheduler/rhw_task_scheduler/bt_actions/ptz_actions.py`
- Modify: `rhw_task_scheduler/rhw_task_scheduler/mission_bt_node.py`

- [ ] **Step 1: Register additional blackboard keys in `CaptureImage`**

In `CaptureImage.__init__()`, after the existing `/last_capture_path` registration, add:

```python
self._bb.register_key(key='/last_capture_url', access=py_trees.common.Access.WRITE)
self._bb.register_key(key='/last_capture_file_size', access=py_trees.common.Access.WRITE)
```

- [ ] **Step 2: Write metadata on capture success**

In `CaptureImage.update()`, inside `if result.result == 1:`, replace:

```python
self._bb.set('/last_capture_path', result.file_path)
self._node.get_logger().info(f'Capture saved: {result.file_path}')
if hasattr(self._node, 'publish_inspection_album_report'):
    self._node.publish_inspection_album_report(wp, result)
return py_trees.common.Status.SUCCESS
```

with:

```python
self._bb.set('/last_capture_path', str(result.file_path))
self._bb.set('/last_capture_url', str(result.capture_url))
self._bb.set('/last_capture_file_size', int(result.file_size))
self._node.get_logger().info(f'Capture saved: {result.file_path}')
return py_trees.common.Status.SUCCESS
```

- [ ] **Step 3: Update the docstring**

Change the `CaptureImage` docstring blackboard section to:

```python
    写入 Blackboard:
        /last_capture_path       — str (抓拍文件路径)
        /last_capture_url        — str (设备抓拍 URL)
        /last_capture_file_size  — int (图片大小，字节)
```

- [ ] **Step 4: Register keys in `MissionBtNode`**

In `mission_bt_node.py`, after:

```python
self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.WRITE)
```

add:

```python
self._bb.register_key(key='/last_capture_url', access=py_trees.common.Access.WRITE)
self._bb.register_key(key='/last_capture_file_size', access=py_trees.common.Access.WRITE)
self._bb.register_key(key='/last_model_result_json_path', access=py_trees.common.Access.WRITE)
```

After:

```python
self._bb.set('/last_capture_path', '')
```

add:

```python
self._bb.set('/last_capture_url', '')
self._bb.set('/last_capture_file_size', 0)
self._bb.set('/last_model_result_json_path', '')
```

- [ ] **Step 5: Clear keys per waypoint**

In `_setup_current_waypoint()`, after:

```python
self._bb.set('/last_capture_path', '')
```

add:

```python
self._bb.set('/last_capture_url', '')
self._bb.set('/last_capture_file_size', 0)
self._bb.set('/last_model_result_json_path', '')
```

- [ ] **Step 6: Run syntax check**

Run:

```bash
python3 -m compileall rhw_task_scheduler
```

Expected: compile completes without syntax errors.

- [ ] **Step 7: Commit blackboard capture metadata**

Run:

```bash
git add rhw_task_scheduler/rhw_task_scheduler/bt_actions/ptz_actions.py rhw_task_scheduler/rhw_task_scheduler/mission_bt_node.py
git commit -m "♻️ refactor: keep capture metadata on blackboard"
```

---

### Task 4: Add Vision Behavior Actions

**Files:**
- Create: `rhw_task_scheduler/rhw_task_scheduler/bt_actions/vision_actions.py`

- [ ] **Step 1: Create `vision_actions.py` with imports and constants**

Create the file with:

```python
"""vision_actions — 视觉点抓拍后的行为树叶节点。"""
from __future__ import annotations

import time

import py_trees
from rclpy.node import Node

from rhw_msgs.srv import InspectionAlbumUpload, ModelTaskRun
from rhw_task_scheduler.bt_utils import parse_task_params
from rhw_task_scheduler.service_audit import ServiceAuditPublisher
```

- [ ] **Step 2: Add `UploadInspectionAlbum` class**

Append this class:

```python
class UploadInspectionAlbum(py_trees.behaviour.Behaviour):
    """调用 /inspection/album_report/upload，同步判定图片上报结果。"""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_path', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_url', access=py_trees.common.Access.READ)
        self._bb.register_key(key='/last_capture_file_size', access=py_trees.common.Access.READ)

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        self._service_name = str(
            self._node.get_parameter('inspection_album_upload_service').value
        )
        self._timeout_sec = max(
            float(self._node.get_parameter('album_upload_timeout_sec').value),
            0.1,
        )
        self._client = self._node.create_client(InspectionAlbumUpload, self._service_name)
        self._future = None
        self._req_time: float | None = None
        self._deadline: float | None = None

    def initialise(self) -> None:
        self._future = None
        self._req_time = None
        self._deadline = time.monotonic() + self._timeout_sec

    def _build_request(self) -> InspectionAlbumUpload.Request | None:
        wp = self._bb.get('/current_waypoint') or {}
        image_path = str(self._bb.get('/last_capture_path') or '')
        if not image_path:
            self._node.get_logger().warning('Album upload skipped: last_capture_path is empty')
            return None

        req = InspectionAlbumUpload.Request()
        req.task_id = str(getattr(self._node, '_current_task_id', '') or '')
        req.point_id = str(wp.get('waypoint_id', ''))
        req.point_name = str(wp.get('label') or req.point_id)
        req.image_path = image_path
        req.capture_url = str(self._bb.get('/last_capture_url') or '')
        req.file_size = max(0, min(int(self._bb.get('/last_capture_file_size') or 0), 4294967295))
        return req

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            if not self._future.done():
                if self._deadline is not None and time.monotonic() > self._deadline:
                    self._node.get_logger().warning('Album upload service timeout')
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING

            duration = (time.time() - self._req_time) * 1000 if self._req_time else None
            try:
                result = self._future.result()
            except Exception as exc:
                self._audit.publish(
                    service=self._service_name,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'Album upload exception: {exc}')
                return py_trees.common.Status.FAILURE

            self._audit.publish(
                service=self._service_name,
                role='client',
                phase='response',
                response=result,
                success=bool(result.ok),
                duration_ms=duration,
            )
            if bool(result.ok):
                self._node.get_logger().info(
                    f'Album upload succeeded: trace_id={result.trace_id}'
                )
                return py_trees.common.Status.SUCCESS
            self._node.get_logger().warning(
                f'Album upload failed: code={result.code} message={result.message}'
            )
            return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            if self._deadline is not None and time.monotonic() > self._deadline:
                self._node.get_logger().warning('Album upload service not ready before timeout')
                return py_trees.common.Status.FAILURE
            self._node.get_logger().warning('Album upload service not ready')
            return py_trees.common.Status.RUNNING

        req = self._build_request()
        if req is None:
            return py_trees.common.Status.FAILURE

        self._req_time = time.time()
        self._audit.publish(
            service=self._service_name,
            role='client',
            phase='request',
            request=req,
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
```

- [ ] **Step 3: Add `RunModelTask` class**

Append this class:

```python
class RunModelTask(py_trees.behaviour.Behaviour):
    """调用 /rhw/model/task/run，根据 task_params.inference_type 执行模型任务。"""

    def __init__(self, name: str, node: Node, **kwargs):
        super().__init__(name, **kwargs)
        self._node = node
        self._bb = self.attach_blackboard_client()
        self._bb.register_key(key='/current_waypoint', access=py_trees.common.Access.READ)
        self._bb.register_key(
            key='/last_model_result_json_path',
            access=py_trees.common.Access.WRITE,
        )

        if hasattr(self._node, '_service_audit'):
            self._audit = self._node._service_audit
        else:
            self._audit = ServiceAuditPublisher(self._node)

        self._service_name = str(self._node.get_parameter('model_task_run_service').value)
        self._timeout_sec = max(
            float(self._node.get_parameter('model_task_timeout_sec').value),
            0.1,
        )
        self._client = self._node.create_client(ModelTaskRun, self._service_name)
        self._future = None
        self._req_time: float | None = None
        self._deadline: float | None = None

    def initialise(self) -> None:
        self._future = None
        self._req_time = None
        self._deadline = time.monotonic() + self._timeout_sec

    def _build_request(self) -> ModelTaskRun.Request | None:
        wp = self._bb.get('/current_waypoint') or {}
        params = parse_task_params(wp)
        task_name = str(params.get('inference_type', '')).strip()
        if not task_name:
            self._node.get_logger().warning(
                'Model task skipped: task_params.inference_type is empty'
            )
            return None

        waypoint_id = str(wp.get('waypoint_id', 'waypoint'))
        task_id = str(getattr(self._node, '_current_task_id', '') or 'mission')

        req = ModelTaskRun.Request()
        req.request_id = f'{task_id}-{waypoint_id}-{time.time_ns()}'
        req.task_name = task_name
        req.conf = 0.25
        req.iou = 0.45
        req.max_det = 100
        req.wait_for_frame_timeout_sec = 3.0
        req.max_frame_age_sec = 2.0
        req.params_json = ''
        return req

    def update(self) -> py_trees.common.Status:
        if self._future is not None:
            if not self._future.done():
                if self._deadline is not None and time.monotonic() > self._deadline:
                    self._node.get_logger().warning('Model task service timeout')
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING

            duration = (time.time() - self._req_time) * 1000 if self._req_time else None
            try:
                result = self._future.result()
            except Exception as exc:
                self._audit.publish(
                    service=self._service_name,
                    role='client',
                    phase='response',
                    success=False,
                    duration_ms=duration,
                    details={'error': str(exc)},
                )
                self._node.get_logger().error(f'Model task exception: {exc}')
                return py_trees.common.Status.FAILURE

            self._audit.publish(
                service=self._service_name,
                role='client',
                phase='response',
                response=result,
                success=bool(result.ok),
                duration_ms=duration,
            )
            if bool(result.ok):
                self._bb.set('/last_model_result_json_path', str(result.result_json_path))
                self._node.get_logger().info(
                    'Model task succeeded: '
                    f'task_name={result.task_name} items={result.item_count} '
                    f'result={result.result_json_path}'
                )
                return py_trees.common.Status.SUCCESS
            self._node.get_logger().warning(
                f'Model task failed: code={result.code} message={result.message}'
            )
            return py_trees.common.Status.FAILURE

        if not self._client.service_is_ready():
            if self._deadline is not None and time.monotonic() > self._deadline:
                self._node.get_logger().warning('Model task service not ready before timeout')
                return py_trees.common.Status.FAILURE
            self._node.get_logger().warning('Model task service not ready')
            return py_trees.common.Status.RUNNING

        req = self._build_request()
        if req is None:
            return py_trees.common.Status.FAILURE

        self._req_time = time.time()
        self._audit.publish(
            service=self._service_name,
            role='client',
            phase='request',
            request=req,
        )
        self._future = self._client.call_async(req)
        return py_trees.common.Status.RUNNING
```

- [ ] **Step 4: Run syntax check**

Run:

```bash
python3 -m compileall rhw_task_scheduler
```

Expected: compile completes without syntax errors.

- [ ] **Step 5: Commit vision actions**

Run:

```bash
git add rhw_task_scheduler/rhw_task_scheduler/bt_actions/vision_actions.py
git commit -m "✨ feat: add vision upload and model BT actions"
```

---

### Task 5: Insert Upload and Model Nodes into Mission BT

**Files:**
- Modify: `rhw_task_scheduler/rhw_task_scheduler/mission_bt_node.py`
- Modify: `rhw_task_scheduler/config/task_scheduler.yaml`

- [ ] **Step 1: Import new actions**

In `mission_bt_node.py`, add:

```python
from rhw_task_scheduler.bt_actions.vision_actions import (
    RunModelTask,
    UploadInspectionAlbum,
)
```

- [ ] **Step 2: Declare new node parameters**

In `_declare_parameters()`, after `inspection_album_report_topic`, add:

```python
self.declare_parameter('inspection_album_upload_service', '/inspection/album_report/upload')
self.declare_parameter('album_upload_timeout_sec', 30.0)
self.declare_parameter('model_task_run_service', '/rhw/model/task/run')
self.declare_parameter('model_task_timeout_sec', 60.0)
```

- [ ] **Step 3: Insert actions after capture**

In `_build_waypoint_tree()`, replace:

```python
vision_seq.add_child(CaptureImage('CaptureImage', node=self))
task_selector.add_child(vision_seq)
```

with:

```python
vision_seq.add_child(CaptureImage('CaptureImage', node=self))
vision_seq.add_child(UploadInspectionAlbum('UploadInspectionAlbum', node=self))
vision_seq.add_child(RunModelTask('RunModelTask', node=self))
task_selector.add_child(vision_seq)
```

- [ ] **Step 4: Update YAML config**

In `rhw_task_scheduler/config/task_scheduler.yaml`, under mission node topic settings, add:

```yaml
    inspection_album_upload_service: "/inspection/album_report/upload"
    album_upload_timeout_sec: 30.0
    model_task_run_service: "/rhw/model/task/run"
    model_task_timeout_sec: 60.0
```

- [ ] **Step 5: Run syntax check**

Run:

```bash
python3 -m compileall rhw_task_scheduler
```

Expected: compile completes without syntax errors.

- [ ] **Step 6: Commit BT integration**

Run:

```bash
git add rhw_task_scheduler/rhw_task_scheduler/mission_bt_node.py rhw_task_scheduler/config/task_scheduler.yaml
git commit -m "✨ feat: run album upload and model tasks in BT"
```

---

### Task 6: Add Mission Test Mocks for Upload and Model Services

**Files:**
- Modify: `rhw_task_scheduler/rhw_task_scheduler/mission_test_mocks.py`
- Modify: `rhw_task_scheduler/launch/mission_test.launch.py`

- [ ] **Step 1: Import service types**

In `mission_test_mocks.py`, change:

```python
from rhw_msgs.srv import CaptureImage, GetWaypoints, PtzAbsoluteMove, Recharge
```

to:

```python
from rhw_msgs.srv import (
    CaptureImage,
    GetWaypoints,
    InspectionAlbumUpload,
    ModelTaskRun,
    PtzAbsoluteMove,
    Recharge,
)
```

- [ ] **Step 2: Add mock parameters**

In `_declare_parameters()`, add:

```python
self.declare_parameter('use_real_album_upload', False)
self.declare_parameter('use_real_model_task', False)
self.declare_parameter('album_upload_service', '/test/inspection/album_report/upload')
self.declare_parameter('model_task_run_service', '/test/rhw/model/task/run')
self.declare_parameter('album_upload_result', True)
self.declare_parameter('model_task_result', True)
```

In `_read_parameters()`, add:

```python
self._use_real_album_upload = bool(self.get_parameter('use_real_album_upload').value)
self._use_real_model_task = bool(self.get_parameter('use_real_model_task').value)
self._album_upload_service = str(self.get_parameter('album_upload_service').value)
self._model_task_run_service = str(self.get_parameter('model_task_run_service').value)
self._album_upload_result = bool(self.get_parameter('album_upload_result').value)
self._model_task_result = bool(self.get_parameter('model_task_result').value)
```

- [ ] **Step 3: Add service member fields**

In `__init__`, after `self._ptz_capture_srv = None`, add:

```python
self._album_upload_srv = None
self._model_task_srv = None
```

- [ ] **Step 4: Create mock services**

In `_setup_interfaces()`, after the PTZ mock block, add:

```python
if not self._use_real_album_upload:
    self._album_upload_srv = self.create_service(
        InspectionAlbumUpload,
        self._album_upload_service,
        self._handle_album_upload,
        callback_group=self._callback_group,
    )

if not self._use_real_model_task:
    self._model_task_srv = self.create_service(
        ModelTaskRun,
        self._model_task_run_service,
        self._handle_model_task_run,
        callback_group=self._callback_group,
    )
```

Update the mock configuration log to append:

```python
f'album_upload={not self._use_real_album_upload} '
f'model_task={not self._use_real_model_task} '
```

- [ ] **Step 5: Add `inference_type` to the default vision waypoint**

In `_default_waypoints()`, inside the `vision_001` `task_params` dictionary, add:

```python
'inference_type': 'fire_equipment_detection',
```

- [ ] **Step 6: Add mock upload handler**

Add this method before `_handle_recharge()`:

```python
def _handle_album_upload(
    self,
    request: InspectionAlbumUpload.Request,
    response: InspectionAlbumUpload.Response,
) -> InspectionAlbumUpload.Response:
    started_at = time.monotonic()
    self._audit.publish(
        service=self._album_upload_service,
        role='server',
        phase='request',
        request=request,
    )

    response.ok = bool(self._album_upload_result)
    response.code = 'OK' if response.ok else 'MOCK_UPLOAD_FAILED'
    response.message = 'mock album upload ok' if response.ok else 'mock album upload failed'
    response.trace_id = f'mock-{time.time_ns()}'
    response.http_status = 200 if response.ok else 500
    response.response_body = '{"code":0}' if response.ok else '{"code":500}'

    self._audit.publish(
        service=self._album_upload_service,
        role='server',
        phase='response',
        request=request,
        response=response,
        success=response.ok,
        duration_ms=(time.monotonic() - started_at) * 1000.0,
    )
    self.get_logger().info(
        f'InspectionAlbumUpload result={response.ok} image={request.image_path}'
    )
    return response
```

- [ ] **Step 7: Add mock model handler**

Add this method after `_handle_album_upload()`:

```python
def _handle_model_task_run(
    self,
    request: ModelTaskRun.Request,
    response: ModelTaskRun.Response,
) -> ModelTaskRun.Response:
    started_at = time.monotonic()
    self._audit.publish(
        service=self._model_task_run_service,
        role='server',
        phase='request',
        request=request,
    )

    response.ok = bool(self._model_task_result)
    response.code = 'OK' if response.ok else 'MOCK_MODEL_FAILED'
    response.message = 'mock model task ok' if response.ok else 'mock model task failed'
    response.request_id = str(request.request_id)
    response.task_name = str(request.task_name)
    response.task_type = 'mock'
    response.model_path = '/tmp/mock_model.engine'
    response.backend = 'mock'
    response.frame_path = '/tmp/mock_frame.jpg'
    response.result_json_path = f'/tmp/mock_model_result_{request.request_id}.json'
    response.item_count = 1 if response.ok else 0
    response.error_count = 0 if response.ok else 1
    response.latency_ms = 12.3
    response.error_category = '' if response.ok else 'mock_error'
    response.detail_json = '{"items":[{"label":"mock","score":0.99}]}' if response.ok else '{}'

    self._audit.publish(
        service=self._model_task_run_service,
        role='server',
        phase='response',
        request=request,
        response=response,
        success=response.ok,
        duration_ms=(time.monotonic() - started_at) * 1000.0,
    )
    self.get_logger().info(
        f'ModelTaskRun result={response.ok} task_name={request.task_name}'
    )
    return response
```

- [ ] **Step 8: Add launch arguments**

In `mission_test.launch.py`, read these booleans in `launch_setup()`:

```python
use_real_album_upload = _as_bool(
    LaunchConfiguration('use_real_album_upload').perform(context)
)
use_real_model_task = _as_bool(
    LaunchConfiguration('use_real_model_task').perform(context)
)
album_upload_result = _as_bool(
    LaunchConfiguration('album_upload_result').perform(context)
)
model_task_result = _as_bool(
    LaunchConfiguration('model_task_result').perform(context)
)
```

Pass them to `mock_node` parameters:

```python
'use_real_album_upload': use_real_album_upload,
'use_real_model_task': use_real_model_task,
'album_upload_result': album_upload_result,
'model_task_result': model_task_result,
```

Add mission node service remaps in the inline parameter dict:

```python
'inspection_album_upload_service': _service_name(
    '/inspection/album_report/upload',
    '/test/inspection/album_report/upload',
    use_real_album_upload,
),
'model_task_run_service': _service_name(
    '/rhw/model/task/run',
    '/test/rhw/model/task/run',
    use_real_model_task,
),
```

Add launch arguments in `generate_launch_description()`:

```python
DeclareLaunchArgument(
    'use_real_album_upload',
    default_value='false',
    description='Use the real inspection album upload service',
),
DeclareLaunchArgument(
    'use_real_model_task',
    default_value='false',
    description='Use the real model task run service',
),
DeclareLaunchArgument(
    'album_upload_result',
    default_value='true',
    description='Mock album upload success or failure',
),
DeclareLaunchArgument(
    'model_task_result',
    default_value='true',
    description='Mock model task success or failure',
),
```

- [ ] **Step 9: Run syntax check**

Run:

```bash
python3 -m compileall rhw_task_scheduler
```

Expected: compile completes without syntax errors.

- [ ] **Step 10: Commit mocks**

Run:

```bash
git add rhw_task_scheduler/rhw_task_scheduler/mission_test_mocks.py rhw_task_scheduler/launch/mission_test.launch.py
git commit -m "🧪 test: mock vision upload and model services"
```

---

### Task 7: Update Documentation

**Files:**
- Modify: `rhw_task_scheduler/README.md`
- Modify: `rhw_udp_mqtt_bridge/README.md`
- Modify: `rhw_model_scheduler_ws/README.md`

- [ ] **Step 1: Update task scheduler README vision flow**

In `rhw_task_scheduler/README.md`, update the TYPE_VISION description to include:

```markdown
`TYPE_VISION=2` 视觉点完整流程：

```text
NavigateToGoal
-> WaitAfterNavArrived
-> PtzAbsoluteMove
-> WaitPtzStable
-> WaitAfterPtzArrived
-> CaptureImage
-> UploadInspectionAlbum
-> RunModelTask
```

`task_params.inference_type` 需要填写模型调度服务的完整 `task_name`，例如：

```json
{
  "azimuth": 180.0,
  "elevation": 0.0,
  "zoom": 6.5,
  "channel": 1,
  "inference_type": "fire_equipment_detection"
}
```
```

- [ ] **Step 2: Add mission test command**

In `rhw_task_scheduler/README.md`, add:

```markdown
全 mock 测试视觉点完整流程：

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=false \
  use_real_navigation:=false \
  use_real_ptz:=false \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  bt_viewer:=true
```

只使用真实云台，上传和模型仍使用 mock：

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=true \
  use_real_navigation:=false \
  use_real_ptz:=true \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  bt_viewer:=true
```
```

- [ ] **Step 3: Update UDP MQTT bridge README**

In `rhw_udp_mqtt_bridge/README.md`, add:

```markdown
### 同步相册上传 Service

行为树通过 `/inspection/album_report/upload` 同步调用 HTTPS 相册上报。该 service 成功时返回 `ok=true`，失败时返回 `ok=false`，任务调度会把当前视觉点判定为失败。

```bash
ros2 service call /inspection/album_report/upload rhw_msgs/srv/InspectionAlbumUpload "{
  task_id: 'task-test',
  point_id: '001',
  point_name: '测试点位',
  image_path: '/tmp/test.jpg',
  capture_url: '',
  file_size: 12345
}"
```
```

- [ ] **Step 4: Update model scheduler README**

In `rhw_model_scheduler_ws/README.md`, near the `/rhw/model/task/run` example, add:

```markdown
任务调度行为树会把视觉点 `task_params.inference_type` 直接作为本服务的 `task_name`。点位部署时请填写完整任务名，例如 `fire_equipment_detection`、`front_panel_pose`、`rust_segmentation` 或 `colormeter_gauge`。
```

- [ ] **Step 5: Commit docs**

Run:

```bash
git add rhw_task_scheduler/README.md rhw_udp_mqtt_bridge/README.md rhw_model_scheduler_ws/README.md
git commit -m "📝 docs: document vision upload and model flow"
```

---

### Task 8: Verify Build and Mission Flow

**Files:**
- No source edits unless verification finds a concrete defect.

- [ ] **Step 1: Build affected packages**

Run:

```bash
colcon build --packages-select rhw_msgs rhw_task_scheduler rhw_udp_mqtt_bridge
```

Expected: all three packages build successfully.

- [ ] **Step 2: Run Python compile checks**

Run:

```bash
python3 -m compileall rhw_task_scheduler rhw_udp_mqtt_bridge
```

Expected: both packages compile without syntax errors.

- [ ] **Step 3: Source the workspace**

Run:

```bash
source install/setup.bash
```

Expected: command exits with status 0.

- [ ] **Step 4: Check service interfaces**

Run:

```bash
ros2 interface show rhw_msgs/srv/InspectionAlbumUpload
ros2 interface show rhw_msgs/srv/ModelTaskRun
```

Expected: both interfaces print fields matching Task 1.

- [ ] **Step 5: Start full mock mission test**

Run:

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=false \
  use_real_navigation:=false \
  use_real_ptz:=false \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  bt_viewer:=true
```

Expected: launch starts `mission_test_mocks`, `mission_bt_node`, and optionally `bt_web_viewer`.

- [ ] **Step 6: Start a mission from a second terminal**

Run:

```bash
source install/setup.bash
ros2 service call /mission/start rhw_msgs/srv/StartMission "{
  map_name: 'factory_map',
  waypoint_ids: ['vision_001']
}"
```

Expected response:

```text
result=1
message='Mission started ...'
```

Expected logs include:

```text
Capture saved:
Album upload succeeded:
Model task succeeded:
Waypoint completed: vision_001
```

- [ ] **Step 7: Verify upload failure fails the vision point**

Run:

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=false \
  use_real_navigation:=false \
  use_real_ptz:=false \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  album_upload_result:=false \
  model_task_result:=true
```

From a second terminal, call `/mission/start` for `vision_001`.

Expected logs include:

```text
Album upload failed:
Waypoint failed: vision_001
```

- [ ] **Step 8: Verify model failure fails the vision point**

Run:

```bash
ros2 launch rhw_task_scheduler mission_test.launch.py \
  use_real_waypoints:=false \
  use_real_navigation:=false \
  use_real_ptz:=false \
  use_real_album_upload:=false \
  use_real_model_task:=false \
  album_upload_result:=true \
  model_task_result:=false
```

From a second terminal, call `/mission/start` for `vision_001`.

Expected logs include:

```text
Model task failed:
Waypoint failed: vision_001
```

- [ ] **Step 9: Review git diff**

Run:

```bash
git status --short
git diff --stat
```

Expected: only intended files from this plan are modified or committed. Existing unrelated worktree changes may still appear; do not revert them.

- [ ] **Step 10: Finish verification**

If verification found a source defect, return to the task that owns that file, make the smallest correction there, rerun that task's verification command, and use that task's commit command. Do not create a verification-only commit.

If no source fixes were needed, leave the repository as-is.
