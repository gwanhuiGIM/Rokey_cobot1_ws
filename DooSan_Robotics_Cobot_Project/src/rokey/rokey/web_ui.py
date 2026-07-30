"""FastAPI coffee-system UI, stage-test page, and local administrator page."""

from __future__ import annotations

import asyncio
import json
import threading
from collections import deque
from contextlib import asynccontextmanager
from typing import Any, Optional

import rclpy
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    from .monitor_pjt.system_monitor import SystemMonitor
except ImportError:  # Support direct execution from the source directory.
    from monitor_pjt.system_monitor import SystemMonitor

STATUS_TOPIC = "/coffee_system/status"
CONTROL_TOPIC = "/coffee_system/control"
ADMIN_STATUS_TOPIC = "/system_monitor/status"
ADMIN_LOG_TOPIC = "/system_monitor/log"
ADMIN_CMD_TOPIC = "/system_monitor/cmd"

TEST_STAGE_NAMES = {
    "full_sequence": "전체 공정",
    "bean_drop": "원두 투입",
    "grinder": "그라인더",
    "dripper_in": "필터 투입",
    "spiral_pour": "스파이럴 드립",
    "final_drip": "최종 드립",
    "gripper_open": "그리퍼 열기",
    "gripper_close": "그리퍼 닫기",
}
TEST_GRIND_TURNS = {3, 5, 7, 10}
SPEED_MIN_PERCENT = 10
SPEED_MAX_PERCENT = 100
TEST_GRIP_OPEN_MODES = {
    "spoon_cup": "스푼·컵 열기",
    "jar": "병 열기",
    "handle": "손잡이 열기",
}

DEFAULT_STATE: dict[str, Any] = {
    "phase": "WAITING_CONTROLLER",
    "screen": 1,
    "progress": 0,
    "title": "원두 선택 또는 단계 테스트를 시작해 주세요",
    "message": "DI 13~16 또는 단계 테스트를 선택할 때까지 시간 제한 없이 기다립니다.",
    "busy": False,
    "waiting_physical_button": True,
    "waiting_external_force": False,
    "selection_timeout_sec": None,
    "external_force_timeout_sec": 10.0,
    "wait_remaining_sec": None,
    "force_threshold_n": 4.0,
    "force_delta_n": 0.0,
    "force_peak_n": 0.0,
    "selected_bean": "",
    "selected_button": None,
    "selected_grind": "",
    "selected_grind_button": None,
    "grind_turns": 0,
    "grind_current_turns": 0.0,
    "spiral_stage": "대기",
    "spiral_progress": 0.0,
    "spiral_radius_mm": 44.0,
    "spiral_revolutions": 5.0,
    "spiral_duration_sec": 15.0,
    "spiral_j6_delta_deg": -60.0,
    "final_drip_stage": "대기",
    "final_drip_progress": 0.0,
    "final_pour_j6_delta_deg": 45.0,
    "final_pour_tcp_name": "mug",
    "final_pour_restore_tcp_name": "GripperDA_v1",
    "final_pour_linear_vel_mm_s": 80.0,
    "final_pour_angular_vel_deg_s": 25.0,
    "operation_speed_percent": 100,
    "speed_service_ready": False,
    "speed_update_pending": False,
    "speed_update_error": "",
    "speed_previous_percent": 100,
    "speed_applied_percent": 100,
    "speed_effective_variables": {},
    "speed_change_log": [],
    "test_result": "",
    "test_command_error": "",
    "test_mode": False,
    "test_stage_id": "",
    "test_stage_name": "",
    "grip_failure": False,
    "equipment_error": False,
    "failed_stage_id": "",
    "failed_stage_name": "",
    "failed_grip_task": "",
    "recovery_kind": "",
    "recovery_state": "",
    "grip_diagnostics": {},
    "gripper_signal_online": None,
    "recovery_countdown_sec": 0.0,
    "error": "",
    "connected": False,
    "control_connected": False,
}


class CoffeeWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("coffee_webui_bridge")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        admin_qos = QoSProfile(depth=10)
        admin_qos.reliability = ReliabilityPolicy.RELIABLE
        admin_qos.durability = DurabilityPolicy.VOLATILE
        self._lock = threading.RLock()
        self._state = dict(DEFAULT_STATE)
        self._admin_state: dict[str, Any] = {}
        self._admin_logs: deque[str] = deque(maxlen=300)
        self.create_subscription(String, STATUS_TOPIC, self._callback, qos)
        self.create_subscription(
            String, ADMIN_STATUS_TOPIC, self._admin_callback, admin_qos)
        self.create_subscription(
            String, ADMIN_LOG_TOPIC, self._log_callback, admin_qos)
        self._admin_cmd = self.create_publisher(
            String, ADMIN_CMD_TOPIC, admin_qos)
        self._coffee_cmd = self.create_publisher(
            String, CONTROL_TOPIC, admin_qos)

    def _callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state.update(payload)

    def state(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._state)
        result["connected"] = self.count_publishers(STATUS_TOPIC) > 0
        result["control_connected"] = (
            self.count_subscribers(CONTROL_TOPIC) > 0
        )
        return result

    def _admin_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                with self._lock:
                    self._admin_state = payload
        except json.JSONDecodeError:
            return

    def _log_callback(self, msg: String) -> None:
        with self._lock:
            self._admin_logs.append(msg.data)

    def admin_state(self) -> dict[str, Any]:
        with self._lock:
            state = dict(self._admin_state)
            state["logs"] = list(self._admin_logs)
        try:
            state["connected"] = (
                rclpy.ok() and self.count_publishers(ADMIN_STATUS_TOPIC) > 0)
        except Exception:
            state["connected"] = False
        return state

    def admin_command(self, payload: dict[str, Any]) -> None:
        self._admin_cmd.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def coffee_command(self, payload: dict[str, Any]) -> None:
        self._coffee_cmd.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))


bridge: Optional[CoffeeWebBridge] = None
monitor: Optional[SystemMonitor] = None
executor: Optional[MultiThreadedExecutor] = None
spin_thread: Optional[threading.Thread] = None


def _spin_nodes(active_executor: MultiThreadedExecutor) -> None:
    try:
        active_executor.spin()
    except ExternalShutdownException:
        pass


@asynccontextmanager
async def lifespan(_: FastAPI):
    global bridge, monitor, executor, spin_thread
    rclpy.init(args=None)
    bridge = CoffeeWebBridge()
    monitor = SystemMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    executor.add_node(monitor)
    spin_thread = threading.Thread(
        target=_spin_nodes, args=(executor,), daemon=True)
    spin_thread.start()
    try:
        yield
    finally:
        active_bridge = bridge
        active_monitor = monitor
        active_executor = executor
        bridge = None
        monitor = None
        executor = None
        if active_executor is not None:
            active_executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if active_bridge is not None:
            active_bridge.destroy_node()
        if active_monitor is not None:
            active_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


app = FastAPI(title="ROKEY Coffee System", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse(HTML)


@app.get("/api/state")
async def api_state() -> dict[str, Any]:
    return bridge.state() if bridge is not None else dict(DEFAULT_STATE)




def _require_local(request: Request) -> None:
    client_host = request.client.host if request.client else ""
    if client_host not in {"127.0.0.1", "::1"}:
        raise HTTPException(
            status_code=403,
            detail="관리자 모드는 로봇 PC에서만 실행할 수 있습니다.",
        )


@app.get("/test", response_class=HTMLResponse)
async def test_page(request: Request) -> HTMLResponse:
    _require_local(request)
    return HTMLResponse(TEST_HTML)


@app.post("/api/test/start")
async def test_start(request: Request) -> dict[str, Any]:
    _require_local(request)
    payload = await request.json()
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="JSON 객체가 필요합니다.")

    stage = str(payload.get("stage", "")).strip()
    if stage not in TEST_STAGE_NAMES:
        raise HTTPException(status_code=400, detail="허용되지 않은 테스트 단계입니다.")

    try:
        grind_turns = int(payload.get("grind_turns", 3))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="분쇄 회전 수가 올바르지 않습니다.")
    if grind_turns not in TEST_GRIND_TURNS:
        raise HTTPException(
            status_code=400,
            detail="분쇄 회전 수는 3, 5, 7, 10 중 하나여야 합니다.",
        )

    gripper_open_mode = str(
        payload.get("gripper_open_mode", "spoon_cup")
    ).strip()
    if gripper_open_mode not in TEST_GRIP_OPEN_MODES:
        raise HTTPException(
            status_code=400,
            detail="허용되지 않은 그리퍼 열기 프리셋입니다.",
        )

    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS bridge가 준비되지 않았습니다.")
    if bool(bridge.state().get("busy", False)):
        raise HTTPException(
            status_code=409,
            detail="현재 테스트 또는 전체 공정이 실행 중입니다.",
        )
    if bridge.count_subscribers(CONTROL_TOPIC) <= 0:
        raise HTTPException(
            status_code=503,
            detail="커피 제어 노드가 단계 테스트 명령을 수신할 준비가 되지 않았습니다.",
        )

    command = {
        "cmd": "start_test",
        "stage": stage,
        "grind_turns": grind_turns,
        "gripper_open_mode": gripper_open_mode,
    }
    bridge.coffee_command(command)
    return {
        "ok": True,
        "stage": stage,
        "stage_name": TEST_STAGE_NAMES[stage],
        "grind_turns": grind_turns,
        "gripper_open_mode": gripper_open_mode,
        "gripper_open_mode_name": TEST_GRIP_OPEN_MODES[gripper_open_mode],
    }


@app.post("/api/control/speed")
async def set_operation_speed(request: Request) -> dict[str, Any]:
    payload = await request.json()
    if not isinstance(payload, dict):
        raise HTTPException(status_code=400, detail="JSON 객체가 필요합니다.")
    try:
        speed_percent = int(round(float(payload.get("speed_percent"))))
    except (TypeError, ValueError):
        raise HTTPException(status_code=400, detail="속도 값이 올바르지 않습니다.")
    if not SPEED_MIN_PERCENT <= speed_percent <= SPEED_MAX_PERCENT:
        raise HTTPException(
            status_code=400,
            detail=(
                f"전체 공정 속도는 {SPEED_MIN_PERCENT}~"
                f"{SPEED_MAX_PERCENT}% 범위여야 합니다."
            ),
        )
    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS bridge가 준비되지 않았습니다.")
    if bridge.count_subscribers(CONTROL_TOPIC) <= 0:
        raise HTTPException(
            status_code=503,
            detail="커피 제어 노드가 속도 명령을 수신할 준비가 되지 않았습니다.",
        )
    bridge.coffee_command({
        "cmd": "set_speed",
        "speed_percent": speed_percent,
    })
    return {"ok": True, "speed_percent": speed_percent}


@app.get("/admin", response_class=HTMLResponse)
async def admin_page(request: Request) -> HTMLResponse:
    _require_local(request)
    return HTMLResponse(ADMIN_HTML)


@app.get("/api/admin/state")
async def admin_state(request: Request) -> dict[str, Any]:
    _require_local(request)
    return bridge.admin_state() if bridge is not None else {"connected": False}


@app.post("/api/admin/cmd")
async def admin_command(request: Request) -> dict[str, bool]:
    _require_local(request)
    payload = await request.json()
    command = payload.get("cmd") if isinstance(payload, dict) else None
    allowed = {
        "estop", "stop", "start", "set_control_enabled", "set_mode",
        "move_task", "move_joint6", "gripper",
    }
    if command not in allowed:
        raise HTTPException(status_code=400, detail="허용되지 않은 명령입니다.")
    if bridge is None:
        raise HTTPException(status_code=503, detail="ROS bridge가 준비되지 않았습니다.")
    bridge.admin_command(payload)
    return {"ok": True}


@app.websocket("/ws")
async def websocket_status(websocket: WebSocket) -> None:
    await websocket.accept()
    previous = ""
    try:
        while True:
            state = bridge.state() if bridge is not None else dict(DEFAULT_STATE)
            encoded = json.dumps(state, ensure_ascii=False, sort_keys=True)
            if encoded != previous:
                await websocket.send_json(state)
                previous = encoded
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        return


HTML = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROKEY Coffee System</title>
<style>
:root{
  --canvas:#f2f0eb;
  --ceramic:#edebe9;
  --surface:#ffffff;
  --surface-soft:#f9f9f9;
  --text:rgba(0,0,0,.87);
  --text-soft:rgba(0,0,0,.58);
  --green:#006241;
  --green-accent:#00754a;
  --house:#1e3932;
  --uplift:#2b5148;
  --mint:#d4e9e2;
  --red:#c82014;
  --line:rgba(0,0,0,.10);
  --card-shadow:0 0 .5px rgba(0,0,0,.14),0 1px 1px rgba(0,0,0,.24);
  --nav-shadow:0 1px 3px rgba(0,0,0,.10),0 2px 2px rgba(0,0,0,.06),0 0 2px rgba(0,0,0,.07);
}
*{box-sizing:border-box}
html{font-size:16px}
body{
  margin:0;
  min-height:100vh;
  background:var(--canvas);
  color:var(--text);
  font-family:"Helvetica Neue",Pretendard,"Noto Sans KR",Helvetica,Arial,sans-serif;
  letter-spacing:-.01em;
}
.shell{
  width:min(1240px,calc(100% - 48px));
  margin:0 auto;
  padding:24px 0 48px;
}
.top{
  min-height:84px;
  display:flex;
  justify-content:space-between;
  align-items:center;
  gap:24px;
  margin-bottom:24px;
  padding:16px 24px;
  background:var(--surface);
  border-radius:12px;
  box-shadow:var(--nav-shadow);
}
.top>div:first-child{position:relative;padding-left:54px}
.top>div:first-child::before{
  content:"R";
  position:absolute;
  left:0;
  top:50%;
  width:40px;
  height:40px;
  display:grid;
  place-items:center;
  transform:translateY(-50%);
  border-radius:50%;
  background:var(--green);
  color:#fff;
  font-size:18px;
  font-weight:700;
  letter-spacing:-.03em;
}
.brand{
  color:var(--house);
  font-size:19px;
  line-height:1.2;
  font-weight:700;
  letter-spacing:.08em;
}
.sub{margin-top:4px;color:var(--text-soft);font-size:13px;line-height:1.4}
.conn{
  position:relative;
  flex:0 0 auto;
  min-height:40px;
  display:inline-flex;
  align-items:center;
  gap:8px;
  padding:8px 16px;
  border:1px solid rgba(0,0,0,.30);
  border-radius:50px;
  background:transparent;
  color:var(--text-soft);
  font-size:14px;
  font-weight:600;
  transition:background-color .2s ease,border-color .2s ease,color .2s ease,transform .2s ease;
}
.conn::before{content:"";width:8px;height:8px;border-radius:50%;background:#8b8b8b}
.conn.on{border-color:var(--green-accent);background:var(--green-accent);color:#fff}
.conn.on::before{background:#fff;box-shadow:0 0 0 3px rgba(255,255,255,.22)}

.top-actions{display:flex;align-items:center;gap:10px;flex:0 0 auto}
.admin-btn,.test-btn{
  min-height:40px;
  padding:8px 18px;
  border:1px solid var(--green-accent);
  border-radius:50px;
  background:var(--green-accent);
  color:#fff;
  font:inherit;
  font-size:14px;
  font-weight:600;
  letter-spacing:-.01em;
  cursor:pointer;
  transition:all .2s ease;
}
.admin-btn:hover,.test-btn:hover{background:var(--green);border-color:var(--green)}
.admin-btn:active,.test-btn:active{transform:scale(.95)}
.admin-btn:disabled,.test-btn:disabled{cursor:wait;opacity:.68}
.test-btn{background:#fff;color:var(--green-accent)}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 328px;gap:24px;align-items:stretch}
.panel{
  background:var(--surface);
  border-radius:12px;
  box-shadow:var(--card-shadow);
}
.main{
  position:relative;
  min-height:700px;
  overflow:hidden;
  padding:48px;
}
.main::before{
  content:"";
  position:absolute;
  inset:0 0 auto 0;
  height:8px;
  background:var(--green);
}
.side{
  padding:28px 24px;
  display:flex;
  flex-direction:column;
  background:var(--house);
  color:#fff;
}
.screen{display:none;animation:screen-in .28s ease both}
.screen.active{display:block}
@keyframes screen-in{from{opacity:0;transform:translateY(6px)}to{opacity:1;transform:none}}
.eyebrow{
  color:var(--green);
  font-size:12px;
  line-height:1.5;
  font-weight:700;
  letter-spacing:.15em;
}
.side>.eyebrow{color:rgba(255,255,255,.70)}
h1{
  max-width:760px;
  margin:24px 0 16px;
  color:var(--green);
  font-size:clamp(42px,5.4vw,68px);
  line-height:1.08;
  font-weight:600;
  letter-spacing:-.035em;
}
.lead{
  max-width:720px;
  margin:0;
  color:var(--text-soft);
  font-size:17px;
  line-height:1.75;
}
.notice,.wait{
  padding:18px 20px;
  border-radius:12px;
  font-size:15px;
  line-height:1.65;
}
.notice{
  margin:32px 0 24px;
  background:var(--mint);
  color:var(--house);
  font-weight:600;
}
.beans{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:16px}
.bean{
  position:relative;
  min-height:150px;
  padding:24px;
  overflow:hidden;
  border:1px solid transparent;
  border-radius:12px;
  background:var(--surface);
  box-shadow:var(--card-shadow);
  transition:border-color .2s ease,box-shadow .2s ease,transform .2s ease,background-color .2s ease;
}
.bean::after{
  content:"";
  position:absolute;
  left:0;
  right:0;
  bottom:0;
  height:5px;
  background:var(--ceramic);
  transition:background-color .2s ease;
}
.bean.selected{
  border-color:var(--green-accent);
  background:#fff;
  box-shadow:0 0 0 3px rgba(0,117,74,.12),var(--card-shadow);
  transform:translateY(-2px);
}
.bean.selected::after{background:var(--green-accent)}
.di{
  position:absolute;
  right:16px;
  top:16px;
  padding:6px 12px;
  border:1px solid var(--green-accent);
  border-radius:50px;
  color:var(--green-accent);
  background:#fff;
  font-size:12px;
  line-height:1.2;
  font-weight:700;
  letter-spacing:.03em;
}
.bean.selected .di{background:var(--green-accent);color:#fff}
.bean-name{
  max-width:78%;
  margin-top:42px;
  color:var(--house);
  font-size:20px;
  line-height:1.35;
  font-weight:600;
}
.bean-note{margin-top:8px;color:var(--text-soft);font-size:14px;line-height:1.55}
.process{min-height:570px;display:grid;place-items:center;text-align:center}
.process>div{width:min(100%,680px)}
.icon{
  width:104px;
  height:104px;
  margin:0 auto;
  display:grid;
  place-items:center;
  border-radius:50%;
  background:var(--mint);
  color:var(--green);
  font-size:54px;
  line-height:1;
  box-shadow:0 0 6px rgba(0,0,0,.10),0 8px 12px rgba(0,0,0,.08);
}
#s8 .icon{background:var(--green-accent);color:#fff}
#s9 .icon{background:rgba(200,32,20,.08);color:var(--red)}
#s6 .icon,#s7 .icon{animation:pour-sway 2.2s ease-in-out infinite}
.new-order-button{
  width:min(100%,420px);
  min-height:76px;
  margin:30px auto 0;
  padding:14px 24px;
  display:flex;
  justify-content:center;
  align-items:center;
  gap:16px;
  border:0;
  border-radius:999px;
  background:var(--green-accent);
  color:#fff;
  font:inherit;
  box-shadow:var(--card-shadow);
  cursor:default;
  opacity:1;
}
.new-order-button:disabled{color:#fff;background:var(--green-accent);opacity:1}
.new-order-di{padding:7px 12px;border:1px solid rgba(255,255,255,.65);border-radius:999px;font-size:13px;font-weight:800;letter-spacing:.04em}
.new-order-label{font-size:20px;font-weight:700}
@keyframes pour-sway{
  0%,100%{transform:rotate(-4deg)}
  50%{transform:rotate(8deg)}
}
.spiral-progress-card{
  width:min(100%,620px);
  margin:30px auto 0;
  padding:26px 28px;
  border:1px solid rgba(0,117,74,.16);
  border-radius:16px;
  background:var(--surface-soft);
  box-shadow:var(--card-shadow);
  text-align:left;
}
.spiral-progress-top{
  display:flex;
  justify-content:space-between;
  align-items:flex-end;
  gap:20px;
}
.spiral-progress-label{
  color:var(--green);
  font-size:12px;
  font-weight:700;
  letter-spacing:.12em;
}
.spiral-percent{
  color:var(--green-accent);
  font-size:38px;
  line-height:1;
  font-weight:700;
  letter-spacing:-.04em;
}
.spiral-stage{
  margin-top:18px;
  color:var(--house);
  font-size:24px;
  line-height:1.35;
  font-weight:700;
}
.spiral-track{
  height:14px;
  margin-top:20px;
  overflow:hidden;
  border-radius:50px;
  background:rgba(0,98,65,.12);
}
.spiral-bar{
  height:100%;
  width:0;
  border-radius:inherit;
  background:var(--green-accent);
  transition:width .28s ease;
}
.spiral-specs{
  display:grid;
  grid-template-columns:repeat(4,minmax(0,1fr));
  gap:10px;
  margin-top:18px;
}
.spiral-spec{
  padding:12px;
  border-radius:10px;
  background:#fff;
  border:1px solid var(--line);
}
.spiral-spec span{
  display:block;
  color:var(--text-soft);
  font-size:11px;
  line-height:1.4;
}
.spiral-spec strong{
  display:block;
  margin-top:4px;
  color:var(--house);
  font-size:15px;
  line-height:1.3;
}
.final-progress-card{
  width:min(100%,620px);
  margin:30px auto 0;
  padding:26px 28px;
  border:1px solid rgba(0,117,74,.16);
  border-radius:16px;
  background:var(--surface-soft);
  box-shadow:var(--card-shadow);
  text-align:left;
}
.final-progress-top{
  display:flex;
  justify-content:space-between;
  align-items:flex-end;
  gap:20px;
}
.final-progress-label{
  color:var(--green);
  font-size:12px;
  font-weight:700;
  letter-spacing:.12em;
}
.final-percent{
  color:var(--green-accent);
  font-size:38px;
  line-height:1;
  font-weight:700;
  letter-spacing:-.04em;
}
.final-stage{
  margin-top:18px;
  color:var(--house);
  font-size:24px;
  line-height:1.35;
  font-weight:700;
}
.final-track{
  height:14px;
  margin-top:20px;
  overflow:hidden;
  border-radius:50px;
  background:rgba(0,98,65,.12);
}
.final-bar{
  height:100%;
  width:0;
  border-radius:inherit;
  background:var(--green-accent);
  transition:width .28s ease;
}
.final-specs{
  display:grid;
  grid-template-columns:repeat(4,minmax(0,1fr));
  gap:10px;
  margin-top:18px;
}
.final-spec{
  padding:12px;
  border-radius:10px;
  background:#fff;
  border:1px solid var(--line);
}
.final-spec span{
  display:block;
  color:var(--text-soft);
  font-size:11px;
  line-height:1.4;
}
.final-spec strong{
  display:block;
  margin-top:4px;
  color:var(--house);
  font-size:15px;
  line-height:1.3;
}
.title{
  margin:28px 0 12px;
  color:var(--house);
  font-size:36px;
  line-height:1.25;
  font-weight:600;
  letter-spacing:-.025em;
}
.message{max-width:620px;margin:0 auto;color:var(--text-soft);font-size:17px;line-height:1.75;white-space:pre-line}
.wait{
  display:none;
  margin:28px auto 0;
  background:var(--house);
  color:#fff;
  font-weight:600;
  text-align:left;
  box-shadow:var(--card-shadow);
}
.wait.show{display:block}
.force-line{
  display:flex;
  justify-content:space-between;
  align-items:center;
  gap:18px;
  margin-top:16px;
  padding-top:16px;
  border-top:1px solid rgba(255,255,255,.18);
}
.force-value{color:#fff;font-size:19px;font-weight:700;white-space:nowrap}
.grind-progress-card{
  width:min(100%,560px);
  margin:30px auto 0;
  padding:26px 28px;
  border:1px solid rgba(0,117,74,.16);
  border-radius:16px;
  background:var(--surface-soft);
  box-shadow:var(--card-shadow);
  text-align:left;
}
.grind-progress-top{display:flex;justify-content:space-between;align-items:flex-end;gap:20px}
.grind-progress-label{color:var(--green);font-size:12px;font-weight:700;letter-spacing:.12em}
.grind-percent{color:var(--green-accent);font-size:38px;line-height:1;font-weight:700;letter-spacing:-.04em}
.grind-turn-display{display:flex;align-items:baseline;gap:8px;margin-top:18px;color:var(--house)}
.grind-current{font-size:54px;line-height:1;font-weight:700;letter-spacing:-.05em}
.grind-divider{color:rgba(0,0,0,.28);font-size:30px;font-weight:400}
.grind-total{font-size:30px;line-height:1;font-weight:600}
.grind-unit{margin-left:2px;color:var(--text-soft);font-size:16px;font-style:normal;font-weight:600}
.grind-track{height:14px;margin-top:22px;overflow:hidden;border-radius:50px;background:rgba(0,98,65,.12)}
.grind-bar{height:100%;width:0;border-radius:inherit;background:var(--green-accent);transition:width .28s ease}
.grind-progress-foot{display:flex;justify-content:space-between;gap:18px;margin-top:12px;color:var(--text-soft);font-size:13px;line-height:1.45}
.grind-progress-foot strong{color:var(--house);font-weight:600}
#s4 .icon{animation:grinder-spin 2.4s linear infinite}
@keyframes grinder-spin{to{transform:rotate(360deg)}}
.steps{display:grid;gap:8px;margin-top:20px}
.step{
  display:grid;
  grid-template-columns:40px minmax(0,1fr);
  gap:12px;
  align-items:center;
  min-height:64px;
  padding:10px 12px;
  border-radius:12px;
  color:rgba(255,255,255,.70);
  transition:background-color .2s ease,color .2s ease,transform .2s ease;
}
.step.active{background:#fff;color:var(--house);transform:translateX(3px)}
.step.done{color:#fff}
.num{
  width:40px;
  height:40px;
  display:grid;
  place-items:center;
  border:1px solid rgba(255,255,255,.24);
  border-radius:50%;
  background:rgba(255,255,255,.08);
  color:#fff;
  font-size:14px;
  font-weight:700;
}
.step.active .num{border-color:var(--green-accent);background:var(--green-accent);color:#fff}
.step.done .num{border-color:var(--green-accent);background:var(--green-accent);color:#fff}
.name{font-size:14px;line-height:1.35;font-weight:600}
.desc{margin-top:3px;font-size:12px;line-height:1.35;opacity:.72}
.status{margin-top:auto;padding-top:24px;border-top:1px solid rgba(255,255,255,.18)}
.row{display:flex;justify-content:space-between;align-items:flex-start;gap:16px;margin-top:13px}
.label{color:rgba(255,255,255,.64);font-size:12px;line-height:1.45}
.value{max-width:58%;color:#fff;font-size:13px;line-height:1.45;font-weight:600;text-align:right;overflow-wrap:anywhere}
.track{height:8px;margin:14px 0 4px;overflow:hidden;border-radius:50px;background:rgba(255,255,255,.14)}
.speed-control{margin-top:20px;padding:16px;border:1px solid rgba(255,255,255,.18);border-radius:12px;background:rgba(255,255,255,.08)}
.speed-control-head{display:flex;justify-content:space-between;align-items:center;gap:12px;color:#fff;font-size:13px;font-weight:700}
.speed-control input[type=range]{width:100%;margin:14px 0 8px;accent-color:var(--mint);cursor:pointer}
.speed-control-note{min-height:18px;color:rgba(255,255,255,.68);font-size:11px;line-height:1.5}
.speed-change-log{max-height:210px;margin:12px 0 0;padding:12px;overflow:auto;border-radius:8px;background:rgba(0,0,0,.22);color:rgba(255,255,255,.82);font:11px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre-wrap;word-break:break-word}
.bar{height:100%;width:0;border-radius:inherit;background:var(--mint);transition:width .3s ease}
.error{
  display:none;
  margin-top:20px;
  padding:16px 18px;
  border:1px solid rgba(200,32,20,.24);
  border-radius:12px;
  background:rgba(200,32,20,.05);
  color:var(--red);
  font-size:13px;
  line-height:1.55;
  font-weight:600;
}
.error.show{display:block}

#s10 .icon,#s11 .icon{background:rgba(200,32,20,.08);color:var(--red)}
.recovery-card{
  width:min(100%,650px);
  margin:28px auto 0;
  padding:24px;
  border:1px solid rgba(200,32,20,.22);
  border-radius:16px;
  background:#fff8f7;
  text-align:left;
}
.recovery-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-bottom:18px}
.recovery-item{padding:14px;border-radius:10px;background:#fff}
.recovery-item span{display:block;color:var(--text-soft);font-size:12px;font-weight:700}
.recovery-item strong{display:block;margin-top:6px;color:var(--house);font-size:17px}
.recovery-actions{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.recovery-action{padding:16px;border-radius:12px;background:var(--house);color:#fff}
.recovery-action strong{display:block;font-size:18px}
.recovery-action span{display:block;margin-top:6px;color:rgba(255,255,255,.75);font-size:13px;line-height:1.5}
.equipment-note{margin-top:16px;padding:16px;border-radius:12px;background:#fff;color:var(--red);font-size:14px;line-height:1.65;font-weight:700}

@media(max-width:980px){
  .shell{width:min(100% - 32px,920px);padding-top:16px}
  .layout{grid-template-columns:1fr}
  .side{order:-1}
  .steps{grid-template-columns:repeat(6,minmax(0,1fr));gap:6px}
  .step{grid-template-columns:1fr;gap:7px;min-height:104px;padding:12px 6px;text-align:center}
  .step.active{transform:translateY(-2px)}
  .num{margin:0 auto}
  .desc{display:none}
  .status{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:0 20px;margin-top:22px}
  .status .track{grid-column:1/-1}
  .status .row{padding-top:10px;border-top:1px solid rgba(255,255,255,.10)}
  .main{min-height:660px}
}
@media(max-width:680px){
  .shell{width:calc(100% - 24px);padding-bottom:24px}
  .top{min-height:auto;align-items:flex-start;padding:16px}
  .top>div:first-child{padding-left:48px}
  .top>div:first-child::before{width:36px;height:36px}
  .brand{font-size:16px}
  .sub{font-size:12px}
  .conn{min-height:36px;padding:7px 12px;font-size:12px}
  .main{min-height:620px;padding:36px 24px}
  h1{font-size:clamp(36px,11vw,50px);margin-top:20px}
  .lead{font-size:16px}
  .notice{margin-top:24px}
  .beans{grid-template-columns:1fr}
  .bean{min-height:136px;padding:20px}
  .process{min-height:500px}
  .icon{width:88px;height:88px;font-size:46px}
  .title{font-size:30px}
  .message{font-size:16px}
  .side{padding:24px 18px}
  .status{grid-template-columns:repeat(2,minmax(0,1fr))}
}
@media(max-width:480px){
  .top{display:grid;grid-template-columns:1fr}
  .conn{justify-self:start}
  .steps{grid-template-columns:repeat(6,72px);overflow-x:auto;padding-bottom:6px;scrollbar-width:thin}
  .status{grid-template-columns:1fr}
  .main{padding:32px 18px}
  .force-line{align-items:flex-start;flex-direction:column;gap:8px}
  .grind-progress-card{padding:22px 20px}
  .grind-current{font-size:46px}
  .grind-percent{font-size:32px}
  .grind-progress-foot{align-items:flex-start;flex-direction:column;gap:4px}
  .spiral-specs,.final-specs{grid-template-columns:repeat(2,minmax(0,1fr))}
  .bean-name{max-width:100%;padding-right:58px;font-size:18px}
}
@media(prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.01ms!important;animation-iteration-count:1!important;transition-duration:.01ms!important;scroll-behavior:auto!important}
}
</style>
</head>
<body>
<div class="shell">
<header class="top"><div><div class="brand">ROKEY BREW LAB</div><div class="sub">M0609 Physical Button Coffee System</div></div><div class="top-actions"><div id="conn" class="conn">ROS 연결 대기</div><button id="test-page" class="test-btn" type="button">단계 테스트</button><button id="admin" class="admin-btn" type="button">관리자 모드</button></div></header>
<div class="layout">
<main class="panel main">
<section id="s1" class="screen active"><div class="eyebrow">STEP 01 · PHYSICAL SELECTION</div><h1>물리 버튼으로<br>원두를 선택해 주세요.</h1><p class="lead">DI 물리 버튼은 전체 공정을 시작하고, 상단의 단계 테스트 페이지에서는 원하는 공정을 직접 실행합니다.</p><div class="notice">DI 13~16 중 하나를 누르세요. 선택할 때까지 시간 제한 없이 기다립니다.</div><div class="beans">
<article class="bean" data-button="13"><span class="di">DI 13</span><div class="bean-name">에티오피아 예가체프</div><div class="bean-note">재스민 · 레몬 · 밝은 산미</div></article>
<article class="bean" data-button="14"><span class="di">DI 14</span><div class="bean-name">콜롬비아 수프리모</div><div class="bean-note">카라멜 · 견과 · 균형감</div></article>
<article class="bean" data-button="15"><span class="di">DI 15</span><div class="bean-name">브라질 산토스</div><div class="bean-note">초콜릿 · 아몬드 · 낮은 산미</div></article>
<article class="bean" data-button="16"><span class="di">DI 16</span><div class="bean-name">과테말라 안티구아</div><div class="bean-note">코코아 · 스파이스 · 긴 여운</div></article>
</div><div id="error" class="error"></div></section>
<section id="s2" class="screen"><div class="eyebrow">STEP 02 · BEAN LOADING</div><div class="process"><div><div class="icon">🥄</div><div id="t2" class="title"></div><div id="m2" class="message"></div></div></div></section>

<section id="s3" class="screen">
<div class="eyebrow">STEP 03 · GRIND SIZE SELECTION</div>
<h1>원하는 분쇄 굵기를<br>선택해 주세요.</h1>
<p class="lead">물리 버튼 DI 13~16으로 분쇄 굵기를 선택합니다. 1회전은 그라인더 손잡이 360° 회전입니다.</p>
<div class="notice">분쇄 굵기에 따라 그라인더 회전 수가 달라집니다. 10초 동안 입력이 없으면 DI 13 굵게(3회전)를 자동 선택합니다.</div>
<div class="beans">
<article class="bean grind-card" data-grind-button="13"><span class="di">DI 13</span><div class="bean-name">굵게 분쇄</div><div class="bean-note">3회전 · 굵은 입자</div></article>
<article class="bean grind-card" data-grind-button="14"><span class="di">DI 14</span><div class="bean-name">중간 굵게 분쇄</div><div class="bean-note">5회전 · 중간 굵은 입자</div></article>
<article class="bean grind-card" data-grind-button="15"><span class="di">DI 15</span><div class="bean-name">중간 곱게 분쇄</div><div class="bean-note">7회전 · 중간 고운 입자</div></article>
<article class="bean grind-card" data-grind-button="16"><span class="di">DI 16</span><div class="bean-name">곱게 분쇄</div><div class="bean-note">10회전 · 고운 입자</div></article>
</div>
</section>

<section id="s4" class="screen">
<div class="eyebrow">STEP 04 · GRINDING</div>
<div class="process"><div>
<div class="icon">⚙</div>
<div id="t4" class="title">원두를 갈고 있습니다</div>
<div id="m4" class="message">선택한 분쇄 굵기에 맞춰 그라인더를 회전하고 있습니다.</div>
<div class="grind-progress-card">
  <div class="grind-progress-top">
    <span class="grind-progress-label">GRINDING PROGRESS</span>
    <strong id="grind-percent" class="grind-percent">0%</strong>
  </div>
  <div class="grind-turn-display" aria-live="polite">
    <strong id="grind-current" class="grind-current">0</strong>
    <span class="grind-divider">/</span>
    <strong id="grind-total" class="grind-total">0</strong>
    <em class="grind-unit">회전</em>
  </div>
  <div id="grind-track" class="grind-track" role="progressbar" aria-label="그라인더 회전 진행률" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0">
    <div id="grind-bar" class="grind-bar"></div>
  </div>
  <div class="grind-progress-foot">
    <span>현재 회전량 / 전체 회전량</span>
    <strong id="grind-detail">0 / 0회전</strong>
  </div>
</div>
</div></div>
</section>
<section id="s5" class="screen"><div class="eyebrow">STEP 05 · FILTER LOADING</div><div class="process"><div><div class="icon">☕</div><div id="t5" class="title"></div><div id="m5" class="message"></div><div id="wait" class="wait">병 바닥을 가볍게 쳐주세요.<br>외력이 감지되면 진행하며, 10초 동안 감지되지 않아도 자동으로 다음 단계로 진행합니다.<div class="force-line"><span>현재 외력 변화량</span><strong id="force-now" class="force-value">0.0 / 4.0 N</strong></div></div></div></div></section>

<section id="s6" class="screen">
<div class="eyebrow">STEP 06 · SPIRAL POUR</div>
<div class="process"><div>
<div class="icon">♨</div>
<div id="t6" class="title">스파이럴 드립을 준비하는 중</div>
<div id="m6" class="message">주전자를 파지하고 보상 내향 스파이럴 경로를 수행합니다.</div>
<div class="spiral-progress-card">
  <div class="spiral-progress-top">
    <span class="spiral-progress-label">SPIRAL POUR PROGRESS</span>
    <strong id="spiral-percent" class="spiral-percent">0%</strong>
  </div>
  <div id="spiral-stage" class="spiral-stage">대기</div>
  <div id="spiral-track" class="spiral-track" role="progressbar" aria-label="스파이럴 드립 진행률" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0">
    <div id="spiral-bar" class="spiral-bar"></div>
  </div>
  <div class="spiral-specs">
    <div class="spiral-spec"><span>최대 반경</span><strong id="spiral-radius">44 mm</strong></div>
    <div class="spiral-spec"><span>회전 수</span><strong id="spiral-revolutions">5 회전</strong></div>
    <div class="spiral-spec"><span>동작 시간</span><strong id="spiral-duration">15 s</strong></div>
    <div class="spiral-spec"><span>기울임</span><strong id="spiral-j6">-60°</strong></div>
  </div>
</div>
</div></div>
</section>

<section id="s7" class="screen">
<div class="eyebrow">STEP 07 · FINAL DRIP</div>
<div class="process"><div>
<div class="icon">♨</div>
<div id="t7" class="title">최종 드립을 준비하는 중</div>
<div id="m7" class="message">필터 홀더와 물컵을 배치한 뒤 주둥이 위치 보상 제어로 물을 붓습니다.</div>
<div class="final-progress-card">
  <div class="final-progress-top">
    <span class="final-progress-label">FINAL DRIP PROGRESS</span>
    <strong id="final-percent" class="final-percent">0%</strong>
  </div>
  <div id="final-stage" class="final-stage">대기</div>
  <div id="final-track" class="final-track" role="progressbar" aria-label="최종 드립 진행률" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0">
    <div id="final-bar" class="final-bar"></div>
  </div>
  <div class="final-specs">
    <div class="final-spec"><span>활성 TCP</span><strong id="final-tcp">mug</strong></div>
    <div class="final-spec"><span>Joint6 등가 회전</span><strong id="final-j6">+45°</strong></div>
    <div class="final-spec"><span>Cartesian 속도</span><strong id="final-speed">80 / 25</strong></div>
    <div class="final-spec"><span>복원 TCP</span><strong id="final-restore-tcp">GripperDA_v1</strong></div>
  </div>
</div>
</div></div>
</section>

<section id="s8" class="screen"><div class="eyebrow">COMPLETE</div><div class="process"><div><div class="icon">✓</div><div id="t8" class="title"></div><div id="m8" class="message"></div><button id="new-order-button" class="new-order-button" type="button" disabled><span class="new-order-di">DI 13</span><span class="new-order-label">커피 주문하기</span></button></div></div></section>
<section id="s9" class="screen"><div class="eyebrow">ROBOT ERROR</div><div class="process"><div><div class="icon">!</div><div id="t9" class="title"></div><div id="m9" class="message"></div></div></div></section>
<section id="s10" class="screen" role="alertdialog" aria-modal="true"><div class="eyebrow">GRIP FAILURE RECOVERY</div><div class="process"><div><div class="icon">!</div><div id="t10" class="title">그리퍼 파지 실패</div><div id="m10" class="message"></div>
<div class="recovery-card">
  <div class="recovery-grid">
    <div class="recovery-item"><span>기억된 작업 단계</span><strong id="failed-stage">-</strong></div>
    <div class="recovery-item"><span>실패한 파지 작업</span><strong id="failed-task">-</strong></div>
  </div>
  <div class="recovery-actions">
    <div class="recovery-action"><strong>DI 13 · 관리자 호출</strong><span>누르면 화면에 “관리자가 호출되었습니다”가 표시됩니다.</span></div>
    <div class="recovery-action"><strong>DI 14 · 단계 재시작</strong><span>관리자가 환경을 단계 시작 상태로 정리한 뒤 누릅니다.</span></div>
  </div>
</div></div></div></section>
<section id="s11" class="screen" role="alertdialog" aria-modal="true"><div class="eyebrow">EQUIPMENT ERROR</div><div class="process"><div><div class="icon">!</div><div id="t11" class="title">장비 오류: 그리퍼 상태 이상</div><div id="m11" class="message"></div>
<div class="recovery-card">
  <div class="recovery-grid">
    <div class="recovery-item"><span>기억된 작업 단계</span><strong id="signal-failed-stage">-</strong></div>
    <div class="recovery-item"><span>그리퍼 신호 상태</span><strong id="gripper-signal-state">끊김</strong></div>
    <div class="recovery-item"><span>재시작 상태</span><strong id="signal-countdown">신호 대기</strong></div>
    <div class="recovery-item"><span>복구 방식</span><strong>신호 복구 후 단계 처음부터</strong></div>
  </div>
  <div class="equipment-note">그리퍼 정상 신호가 확인되면 DI 13을 눌러주세요. 버튼 입력 후 3초 카운트다운 동안 정상 상태가 유지되어야 기억한 단계를 처음부터 다시 실행합니다.</div>
</div></div></div></section>
</main>
<aside class="panel side"><div class="eyebrow">PROCESS TIMELINE</div><div class="steps">
<div class="step active" data-step="1"><div class="num">1</div><div><div class="name">원두 선택</div><div class="desc">DI 13~16</div></div></div>
<div class="step" data-step="2"><div class="num">2</div><div><div class="name">원두 투입</div><div class="desc">스푼 작업</div></div></div>
<div class="step" data-step="3"><div class="num">3</div><div><div class="name">분쇄 굵기 선택</div><div class="desc">3·5·7·10회전</div></div></div>
<div class="step" data-step="4"><div class="num">4</div><div><div class="name">그라인딩</div><div class="desc">선택 회전 수 적용</div></div></div>
<div class="step" data-step="5"><div class="num">5</div><div><div class="name">필터 투입</div><div class="desc">병 확인</div></div></div>
<div class="step" data-step="6"><div class="num">6</div><div><div class="name">스파이럴 드립</div><div class="desc">주전자 보상 제어</div></div></div>
<div class="step" data-step="7"><div class="num">7</div><div><div class="name">최종 드립</div><div class="desc">필터 홀더 · 물컵 · 물 붓기</div></div></div>
</div><div class="status">
<div class="row"><span class="label">상태</span><span id="phase" class="value">WAITING</span></div>
<div class="track"><div id="bar" class="bar"></div></div>
<div class="row"><span class="label">진행률</span><span id="progress" class="value">0%</span></div>
<div class="row"><span class="label">원두</span><span id="bean" class="value">선택 전</span></div>
<div class="row"><span class="label">원두 선택 버튼</span><span id="button" class="value">-</span></div>
<div class="row"><span class="label">분쇄 굵기</span><span id="grind" class="value">선택 전</span></div>
<div class="row"><span class="label">분쇄 선택 버튼</span><span id="grind-button" class="value">-</span></div>
<div class="row"><span class="label">전체 회전량</span><span id="turns" class="value">-</span></div>
<div class="row"><span class="label">그라인딩 진행</span><span id="grind-side-progress" class="value">0 / 0회전 · 0%</span></div>
<div class="row"><span class="label">스파이럴 단계</span><span id="spiral-side-stage" class="value">대기</span></div>
<div class="row"><span class="label">스파이럴 진행</span><span id="spiral-side-progress" class="value">0%</span></div>
<div class="row"><span class="label">최종 드립 단계</span><span id="final-side-stage" class="value">대기</span></div>
<div class="row"><span class="label">최종 드립 진행</span><span id="final-side-progress" class="value">0%</span></div>
<div class="row"><span class="label">실행 모드</span><span id="run-mode" class="value">정상 공정</span></div>
<div class="row"><span class="label">오류 복구 상태</span><span id="recovery-side" class="value">정상</span></div>
<div class="row"><span class="label">현재 외력 변화</span><span id="force" class="value">0.0 N</span></div>
<div class="row"><span class="label">외력 감지 기준</span><span id="force-threshold" class="value">4.0 N</span></div>
<div class="speed-control">
  <div class="speed-control-head"><span>전체 공정 실시간 속도</span><strong id="global-speed-value">100%</strong></div>
  <input id="global-speed" type="range" min="10" max="100" step="1" value="100" aria-label="전체 공정 속도">
  <div id="global-speed-note" class="speed-control-note">기본 모션 속도에 적용되는 작동 속도 비율입니다.</div>
  <pre id="global-speed-log" class="speed-change-log">속도 변경 로그 대기</pre>
</div>
</div></aside>
</div></div>
<script>
const $=id=>document.getElementById(id);
const clamp=(value,min,max)=>Math.min(max,Math.max(min,value));
let globalSpeedDragging=false;
let globalSpeedTimer=null;
async function sendGlobalSpeed(value){
  const speed=clamp(Math.round(Number(value)||100),10,100);
  $('global-speed-value').textContent=speed+'%';
  $('global-speed-note').textContent='속도 변경 명령 전송 중...';
  try{
    const response=await fetch('/api/control/speed',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({speed_percent:speed})
    });
    const data=await response.json();
    if(!response.ok) throw new Error(data.detail||'속도 변경 실패');
    $('global-speed-note').textContent='전체 공정 속도 '+data.speed_percent+'% 적용 요청 완료';
  }catch(error){
    $('global-speed-note').textContent=error.message;
  }
}
function formatTurns(value){
  const numeric=Number(value);
  if(!Number.isFinite(numeric)) return '0';
  return Number.isInteger(numeric)?String(numeric):numeric.toFixed(1).replace(/\.0$/,'');
}
function show(n,failedStage){
  [1,2,3,4,5,6,7,8,9,10,11].forEach(x=>$('s'+x).classList.toggle('active',x===n));
  const complete=n===8;
  const recoveryStep={
    bean_drop:2,
    grinder:4,
    dripper_in:5,
    spiral_pour:6,
    final_drip:7
  }[failedStage]||7;
  const current=(n===10||n===11)?recoveryStep:(n===9?7:Math.min(n,7));
  document.querySelectorAll('.step').forEach(e=>{
    const x=Number(e.dataset.step);
    e.classList.toggle('active',!complete&&x===current);
    e.classList.toggle('done',complete||x<current);
  });
}
function apply(s){
  const c=!!s.connected;
  $('conn').classList.toggle('on',c);
  $('conn').textContent=c?'ROS 연결됨':'ROS 연결 대기';

  const n=Math.max(1,Math.min(11,Number(s.screen||1)));
  const p=Math.max(0,Math.min(100,Number(s.progress||0)));
  show(n,s.failed_stage_id||'');

  $('phase').textContent=s.phase||'WAITING';
  $('bar').style.width=p+'%';
  $('progress').textContent=p+'%';
  $('bean').textContent=s.selected_bean||'선택 전';
  $('button').textContent=s.selected_button?'DI '+s.selected_button:'-';
  $('grind').textContent=s.selected_grind||'선택 전';
  $('grind-button').textContent=s.selected_grind_button?'DI '+s.selected_grind_button:'-';

  const totalTurns=Math.max(0,Number(s.grind_turns||0));
  let currentTurns=Number(s.grind_current_turns);
  const aliasCurrentTurns=[s.current_grind_turns,s.grind_completed_turns]
    .map(Number)
    .find(value=>Number.isFinite(value)&&value>0);
  if((!Number.isFinite(currentTurns)||currentTurns===0) && aliasCurrentTurns!==undefined){
    currentTurns=aliasCurrentTurns;
  }
  if(!Number.isFinite(currentTurns)) currentTurns=0;
  currentTurns=Math.max(0,currentTurns);
  if(totalTurns>0) currentTurns=Math.min(currentTurns,totalTurns);

  const calculatedPercent=totalTurns>0?(currentTurns/totalTurns)*100:0;
  const publishedPercent=Number(s.grind_progress);
  let grindPercent=calculatedPercent;
  if(currentTurns===0 && Number.isFinite(publishedPercent)){
    grindPercent=publishedPercent;
  }
  grindPercent=clamp(grindPercent,0,100);
  if(totalTurns>0 && currentTurns===0 && grindPercent>0){
    currentTurns=totalTurns*(grindPercent/100);
  }

  const currentText=formatTurns(currentTurns);
  const totalText=totalTurns>0?formatTurns(totalTurns):'0';
  const percentText=Math.round(grindPercent)+'%';

  $('turns').textContent=totalTurns>0?totalText+'회전':'-';
  $('grind-current').textContent=currentText;
  $('grind-total').textContent=totalText;
  $('grind-percent').textContent=percentText;
  $('grind-detail').textContent=currentText+' / '+totalText+'회전';
  $('grind-bar').style.width=grindPercent+'%';
  $('grind-track').setAttribute('aria-valuenow',String(Math.round(grindPercent)));
  $('grind-side-progress').textContent=currentText+' / '+totalText+'회전 · '+percentText;

  const spiralProgress=clamp(Number(s.spiral_progress||0),0,100);
  const spiralPercentText=Math.round(spiralProgress)+'%';
  const spiralStage=s.spiral_stage||'대기';
  $('spiral-percent').textContent=spiralPercentText;
  $('spiral-stage').textContent=spiralStage;
  $('spiral-bar').style.width=spiralProgress+'%';
  $('spiral-track').setAttribute('aria-valuenow',String(Math.round(spiralProgress)));
  $('spiral-side-stage').textContent=spiralStage;
  $('spiral-side-progress').textContent=spiralPercentText;
  $('spiral-radius').textContent=Number(s.spiral_radius_mm||44).toFixed(0)+' mm';
  $('spiral-revolutions').textContent=formatTurns(s.spiral_revolutions||5)+' 회전';
  $('spiral-duration').textContent=Number(s.spiral_duration_sec||15).toFixed(0)+' s';
  $('spiral-j6').textContent=Number(s.spiral_j6_delta_deg||-60).toFixed(0)+'°';

  const finalProgress=clamp(Number(s.final_drip_progress||0),0,100);
  const finalPercentText=Math.round(finalProgress)+'%';
  const finalStage=s.final_drip_stage||'대기';
  $('final-percent').textContent=finalPercentText;
  $('final-stage').textContent=finalStage;
  $('final-bar').style.width=finalProgress+'%';
  $('final-track').setAttribute('aria-valuenow',String(Math.round(finalProgress)));
  $('final-side-stage').textContent=finalStage;
  $('final-side-progress').textContent=finalPercentText;
  $('final-tcp').textContent=s.final_pour_tcp_name||'mug';
  const finalJ6=Number(s.final_pour_j6_delta_deg||45);
  $('final-j6').textContent=(finalJ6>=0?'+':'')+finalJ6.toFixed(0)+'°';
  const finalLin=Number(s.final_pour_linear_vel_mm_s||80);
  const finalRot=Number(s.final_pour_angular_vel_deg_s||25);
  $('final-speed').textContent=finalLin.toFixed(0)+' mm/s · '+finalRot.toFixed(0)+' deg/s';
  $('final-restore-tcp').textContent=s.final_pour_restore_tcp_name||'GripperDA_v1';

  $('run-mode').textContent=s.test_mode
    ? '단계 테스트 · '+(s.test_stage_name||'-')
    : '정상 전체 공정';
  const recoveryLabels={
    WAIT_ADMIN_CALL:'DI 13 관리자 호출 대기',
    ADMIN_CALLED:'관리자 호출 완료 · DI 14 대기',
    WAIT_MOTION_STOP:'모션 정지 확인 중',
    WAIT_GRIPPER_SIGNAL:'그리퍼 신호 복구 대기',
    SIGNAL_STABILIZING:'그리퍼 신호 안정화 중',
    WAIT_RESTART_CONFIRMATION:'신호 정상 · DI 13 입력 대기',
    RESTART_COUNTDOWN:'3초 후 단계 재시작',
    RESTARTING:'기억된 단계 재시작'
  };
  $('recovery-side').textContent=(s.grip_failure||s.equipment_error)
    ? (recoveryLabels[s.recovery_state]||(s.equipment_error?'장비 오류':'파지 실패'))
    : '정상';
  $('failed-stage').textContent=s.failed_stage_name||'-';
  $('failed-task').textContent=s.failed_grip_task||'-';
  $('signal-failed-stage').textContent=s.failed_stage_name||'-';
  $('gripper-signal-state').textContent=s.gripper_signal_online?'정상 확인':'오류 또는 신호 끊김';
  const recoveryCountdown=Number(s.recovery_countdown_sec||0);
  $('signal-countdown').textContent=s.recovery_state==='WAIT_RESTART_CONFIRMATION'
    ? 'DI 13 입력 대기'
    : (s.recovery_state==='RESTART_COUNTDOWN'
      ? recoveryCountdown.toFixed(1)+'초'
      : (s.recovery_state==='RESTARTING'?'재시작 중':'신호 대기'));

  const forceValue=Number(s.force_delta_n||0);
  const forceThreshold=Number(s.force_threshold_n||4);
  $('force').textContent=forceValue.toFixed(1)+' N';
  $('force-threshold').textContent=forceThreshold.toFixed(1)+' N';
  $('force-now').textContent=forceValue.toFixed(1)+' / '+forceThreshold.toFixed(1)+' N';

  const operationSpeed=clamp(Number(s.operation_speed_percent||100),10,100);
  if(!globalSpeedDragging){
    $('global-speed').value=String(Math.round(operationSpeed));
    $('global-speed-value').textContent=Math.round(operationSpeed)+'%';
  }
  if(s.speed_update_error){
    $('global-speed-note').textContent=s.speed_update_error;
  }else if(s.speed_update_pending){
    $('global-speed-note').textContent='속도 변경 적용 중...';
  }else if(s.speed_service_ready){
    $('global-speed-note').textContent='현재 작동 속도 '+Math.round(operationSpeed)+'%';
  }
  const speedLog=Array.isArray(s.speed_change_log)?s.speed_change_log:[];
  $('global-speed-log').textContent=speedLog.length
    ? speedLog.join('\n')
    : '속도 변경 로그 대기';

  [2,3,4,5,6,7,8,9,10,11].forEach(x=>{
    const title=$('t'+x);
    const message=$('m'+x);
    if(x===4 && n===4){
      if(title) title.textContent=s.grind_title||'원두를 갈고 있습니다';
      if(message) message.textContent=s.grind_message||'선택한 분쇄 굵기에 맞춰 그라인더를 회전하고 있습니다.';
      return;
    }
    if(title) title.textContent=s.title||'';
    if(message) message.textContent=s.message||'';
  });

  $('wait').classList.toggle('show',!!s.waiting_external_force&&n===5);

  document.querySelectorAll('.bean[data-button]').forEach(e=>{
    e.classList.toggle(
      'selected',
      Number(e.dataset.button)===Number(s.selected_button)
    );
  });

  document.querySelectorAll('.grind-card').forEach(e=>{
    e.classList.toggle(
      'selected',
      Number(e.dataset.grindButton)===Number(s.selected_grind_button)
    );
  });

  const er=s.error||'';
  $('error').classList.toggle('show',!!er);
  $('error').textContent=er?'오류 상세: '+er:'';
}
function connect(){const ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);ws.onmessage=e=>apply(JSON.parse(e.data));ws.onclose=()=>setTimeout(connect,1200);ws.onerror=()=>ws.close()}
$('global-speed').addEventListener('pointerdown',()=>{globalSpeedDragging=true});
$('global-speed').addEventListener('pointerup',()=>{globalSpeedDragging=false;sendGlobalSpeed($('global-speed').value)});
$('global-speed').addEventListener('input',()=>{
  $('global-speed-value').textContent=$('global-speed').value+'%';
  clearTimeout(globalSpeedTimer);
  globalSpeedTimer=setTimeout(()=>sendGlobalSpeed($('global-speed').value),140);
});
$('global-speed').addEventListener('change',()=>sendGlobalSpeed($('global-speed').value));
$('test-page').addEventListener('click',()=>window.open('/test','rokey-stage-test'));
$('admin').addEventListener('click',()=>window.open('/admin','rokey-admin'));
fetch('/api/state').then(r=>r.json()).then(apply).catch(()=>{});connect();
</script>
</body>
</html>"""


TEST_HTML = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROKEY 단계 테스트</title>
<style>
:root{--canvas:#f2f0eb;--surface:#fff;--green:#006241;--accent:#00754a;--house:#1e3932;--mint:#d4e9e2;--red:#c82014;--text:#222;--soft:#6b6b6b;--line:rgba(0,0,0,.1)}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:var(--canvas);color:var(--text);font-family:"Helvetica Neue",Pretendard,"Noto Sans KR",Arial,sans-serif}
.shell{width:min(1160px,calc(100% - 32px));margin:0 auto;padding:24px 0 48px}
.top{display:flex;justify-content:space-between;align-items:center;gap:20px;padding:20px 24px;border-radius:14px;background:var(--house);color:#fff}
.brand{font-size:21px;font-weight:700;letter-spacing:.06em}.sub{margin-top:5px;color:rgba(255,255,255,.7);font-size:13px}
.conn{padding:9px 14px;border-radius:999px;background:rgba(255,255,255,.12);font-size:13px;font-weight:700}.conn.on{background:var(--accent)}
.warning{margin:20px 0;padding:18px 20px;border-left:5px solid var(--red);border-radius:10px;background:#fff8f7;line-height:1.65}
.toolbar{display:flex;align-items:center;gap:12px;margin:20px 0;padding:16px 18px;border-radius:12px;background:var(--surface)}
.toolbar label{font-size:14px;font-weight:700}.toolbar select{min-height:42px;padding:0 14px;border:1px solid var(--line);border-radius:9px;background:#fff;font:inherit}
.speed-tool{display:grid;grid-template-columns:auto minmax(180px,1fr) 58px;align-items:center;gap:10px;flex:1;min-width:280px;padding-left:12px;border-left:1px solid var(--line)}
.speed-tool input[type=range]{width:100%;accent-color:var(--accent);cursor:pointer}.speed-value{color:var(--accent);font-weight:800;text-align:right}.speed-note{grid-column:2/4;color:var(--soft);font-size:12px;line-height:1.35}
.test-speed-log{margin:14px 0 0;padding:14px;overflow:auto;border-radius:10px;background:#102c25;color:#dff6ec;font:12px/1.6 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;white-space:pre-wrap;word-break:break-word}
.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}
.card{display:flex;min-height:220px;flex-direction:column;padding:22px;border-radius:14px;background:var(--surface);box-shadow:0 1px 2px rgba(0,0,0,.16)}
.code{color:var(--accent);font-size:12px;font-weight:800;letter-spacing:.12em}.card h2{margin:14px 0 8px;color:var(--house);font-size:24px}.card p{margin:0;color:var(--soft);font-size:14px;line-height:1.65}
.card button{margin-top:auto;min-height:46px;border:0;border-radius:999px;background:var(--accent);color:#fff;font:inherit;font-weight:700;cursor:pointer}.card button:hover{background:var(--green)}.card button:disabled{cursor:not-allowed;opacity:.45}
.state{margin-top:20px;padding:20px;border-radius:14px;background:var(--house);color:#fff}.state-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}
.item{padding:14px;border-radius:10px;background:rgba(255,255,255,.08)}.item span{display:block;color:rgba(255,255,255,.65);font-size:12px}.item strong{display:block;margin-top:6px;font-size:15px;word-break:break-word}
.result{min-height:24px;margin-top:16px;font-size:14px;font-weight:700}.result.error{color:#ffd3ce}.result.ok{color:#b8f2da}
@media(max-width:900px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.state-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:600px){.top{align-items:flex-start;flex-direction:column}.grid{grid-template-columns:1fr}.toolbar{align-items:flex-start;flex-direction:column}.state-grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="shell">
<header class="top"><div><div class="brand">ROKEY STAGE TEST</div><div class="sub">M0609 실제 로봇 공정 단독 실행</div></div><div id="conn" class="conn">제어 노드 연결 대기</div></header>
<div class="warning"><strong>실제 로봇이 즉시 움직입니다.</strong><br>선택한 단계의 모든 물체를 해당 단계 시작 위치에 배치하고 작업 공간을 비운 뒤 실행하십시오. 각 테스트가 끝나면 프로그램은 종료되지 않고 다시 테스트 대기 상태로 돌아갑니다.</div>
<div class="toolbar">
<label for="turns">그라인더 테스트 회전 수</label><select id="turns"><option value="3">3회전 · 굵게</option><option value="5">5회전 · 중간 굵게</option><option value="7">7회전 · 중간 곱게</option><option value="10">10회전 · 곱게</option></select>
<label for="gripper-open-mode">그리퍼 열기 프리셋</label><select id="gripper-open-mode"><option value="spoon_cup">스푼·컵 열기 · DO1 OFF / DO2 OFF</option><option value="jar">병 열기 · DO1 OFF / DO2 ON</option><option value="handle">손잡이 열기 · DO1 ON / DO2 ON</option></select>
<div class="speed-tool"><label for="test-speed">전체 공정 속도</label><input id="test-speed" type="range" min="10" max="100" step="1" value="100"><strong id="test-speed-value" class="speed-value">100%</strong><div id="test-speed-note" class="speed-note">실행 중에도 변경할 수 있습니다.</div></div>
</div>
<div class="grid">
<article class="card"><div class="code">FULL SEQUENCE</div><h2>전체 공정</h2><p>원두 투입부터 최종 드립까지 정상 순서로 실행합니다. 분쇄 회전 수는 위 선택값을 사용합니다.</p><button data-stage="full_sequence">전체 공정 시작</button></article>
<article class="card"><div class="code">BEAN DROP</div><h2>원두 투입</h2><p>스푼 접근, 파지 확인, 원두 투입, 스푼 반환 동작만 실행합니다.</p><button data-stage="bean_drop">원두 투입 시작</button></article>
<article class="card"><div class="code">GRINDER</div><h2>그라인더</h2><p>손잡이 파지 확인 후 선택한 회전 수만큼 분쇄 동작을 실행합니다.</p><button data-stage="grinder">그라인더 시작</button></article>
<article class="card"><div class="code">DRIPPER IN</div><h2>필터 투입</h2><p>분쇄 원두 병을 파지하고 필터 투입 및 외력 확인 과정을 실행합니다.</p><button data-stage="dripper_in">필터 투입 시작</button></article>
<article class="card"><div class="code">SPIRAL POUR</div><h2>스파이럴 드립</h2><p>주전자 파지부터 보상 내향 스파이럴, 주전자 반환까지 실행합니다.</p><button data-stage="spiral_pour">스파이럴 시작</button></article>
<article class="card"><div class="code">FINAL DRIP</div><h2>최종 드립</h2><p>필터 홀더와 물컵 파지, mug TCP 고정 물 붓기와 반대 J6 복귀를 실행합니다.</p><button data-stage="final_drip">최종 드립 시작</button></article>
<article class="card"><div class="code">GRIPPER OPEN</div><h2>그리퍼 열기</h2><p>위에서 선택한 DO 프리셋으로 RG2를 열고 0.5초 동안 정착을 기다립니다. 로봇 관절은 움직이지 않습니다.</p><button data-stage="gripper_open">그리퍼 열기</button></article>
<article class="card"><div class="code">GRIPPER CLOSE</div><h2>그리퍼 닫기</h2><p>DO1 ON / DO2 OFF 조합으로 RG2를 닫고 0.5초 동안 정착을 기다립니다. 파지 성공 판정 없이 입출력 동작만 시험합니다.</p><button data-stage="gripper_close">그리퍼 닫기</button></article>
</div>
<section class="state"><div class="state-grid">
<div class="item"><span>현재 phase</span><strong id="phase">-</strong></div>
<div class="item"><span>현재 화면</span><strong id="screen">-</strong></div>
<div class="item"><span>테스트 단계</span><strong id="stage">-</strong></div>
<div class="item"><span>실행 상태</span><strong id="recovery">테스트 대기</strong></div>
</div><div id="result" class="result"></div><pre id="test-speed-log" class="test-speed-log">속도 변경 로그 대기</pre></section>
</div>
<script>
const $=id=>document.getElementById(id);
let latest={};
let testSpeedDragging=false;
let testSpeedTimer=null;
async function setTestSpeed(value){
  const speed=Math.min(100,Math.max(10,Math.round(Number(value)||100)));
  $('test-speed-value').textContent=speed+'%';
  $('test-speed-note').textContent='속도 변경 명령 전송 중...';
  try{
    const response=await fetch('/api/control/speed',{
      method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({speed_percent:speed})
    });
    const data=await response.json();
    if(!response.ok) throw new Error(data.detail||'속도 변경 실패');
    $('test-speed-note').textContent='전체 공정 속도 '+data.speed_percent+'% 적용 요청 완료';
  }catch(error){
    $('test-speed-note').textContent=error.message;
  }
}
function setButtons(){
  const connected=!!latest.control_connected;
  const busy=!!latest.busy;
  document.querySelectorAll('button[data-stage]').forEach(button=>{
    button.disabled=!connected||busy;
  });
}
function render(s){
  latest=s||{};
  const connected=!!latest.control_connected;
  $('conn').classList.toggle('on',connected);
  $('conn').textContent=connected?'제어 노드 연결됨':'제어 노드 연결 대기';
  $('phase').textContent=latest.phase||'-';
  $('screen').textContent=String(latest.screen||'-');
  $('stage').textContent=latest.test_stage_name||'선택 전';
  const phase=latest.phase||'';
  $('recovery').textContent=phase==='TEST_DONE'
    ? '완료 · 대기 상태 전환 중'
    : (phase==='TEST_ERROR'?'오류 · 대기 상태 전환 중':(latest.busy?'실행 중':'다음 테스트 대기'));
  const speed=Math.min(100,Math.max(10,Number(latest.operation_speed_percent||100)));
  if(!testSpeedDragging){
    $('test-speed').value=String(Math.round(speed));
    $('test-speed-value').textContent=Math.round(speed)+'%';
  }
  if(latest.speed_update_error){
    $('test-speed-note').textContent=latest.speed_update_error;
  }else if(latest.speed_update_pending){
    $('test-speed-note').textContent='속도 변경 적용 중...';
  }else if(latest.speed_service_ready){
    $('test-speed-note').textContent='현재 작동 속도 '+Math.round(speed)+'%';
  }
  const speedLog=Array.isArray(latest.speed_change_log)?latest.speed_change_log:[];
  $('test-speed-log').textContent=speedLog.length
    ? speedLog.join('\n')
    : '속도 변경 로그 대기';
  if(latest.test_command_error){
    $('result').className='result error';
    $('result').textContent=latest.test_command_error;
  }
  setButtons();
}
async function poll(){
  try{render(await (await fetch('/api/state')).json())}catch(e){}
  setTimeout(poll,250);
}
async function startStage(stage){
  const result=$('result');
  result.className='result';
  result.textContent='명령 전송 중...';
  try{
    const response=await fetch('/api/test/start',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({
        stage,
        grind_turns:Number($('turns').value),
        gripper_open_mode:$('gripper-open-mode').value
      })
    });
    const data=await response.json();
    if(!response.ok) throw new Error(data.detail||'명령 전송 실패');
    result.className='result ok';
    const detail=stage==='gripper_open'?' · '+data.gripper_open_mode_name:'';
    result.textContent=data.stage_name+detail+' 테스트 명령을 전송했습니다.';
  }catch(error){
    result.className='result error';
    result.textContent=error.message;
  }
}
$('test-speed').addEventListener('pointerdown',()=>{testSpeedDragging=true});
$('test-speed').addEventListener('pointerup',()=>{testSpeedDragging=false;setTestSpeed($('test-speed').value)});
$('test-speed').addEventListener('input',()=>{
  $('test-speed-value').textContent=$('test-speed').value+'%';
  clearTimeout(testSpeedTimer);
  testSpeedTimer=setTimeout(()=>setTestSpeed($('test-speed').value),140);
});
$('test-speed').addEventListener('change',()=>setTestSpeed($('test-speed').value));
document.querySelectorAll('button[data-stage]').forEach(button=>{
  button.addEventListener('click',()=>startStage(button.dataset.stage));
});
poll();
</script>
</body>
</html>"""


ADMIN_HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ROKEY Brew Lab · Web Administrator</title>
<style>
:root{--cream:#f2f0eb;--ceramic:#edebe9;--white:#fff;--green:#006241;--accent:#00754a;--house:#1e3932;--mint:#d4e9e2;--red:#c82014;--gold:#cba258;--text:#222;--soft:#6b6b6b}
*{box-sizing:border-box}html,body{margin:0;min-height:100%;font-family:"Helvetica Neue","Noto Sans KR",Arial,sans-serif;color:var(--text);background:var(--cream)}
body{padding:14px}.app{height:calc(100vh - 28px);display:grid;grid-template-rows:58px 50px 128px 360px minmax(180px,1fr);gap:10px;max-width:1900px;margin:auto}
.bar,.hero,.card{border-radius:12px}.bar{display:flex;align-items:center;gap:18px;padding:0 22px;background:var(--house);color:#fff}.brand{font-size:20px;font-weight:800;letter-spacing:.08em}.sub{color:#ffffffb3;font-size:13px}.spacer{flex:1}.badge,.pill{border-radius:50px;padding:8px 16px;font-weight:700}.badge{background:var(--accent)}
.hero{display:flex;align-items:center;justify-content:center;background:var(--green);color:#fff;font-size:20px;font-weight:700}
.card{min-width:0;background:var(--white);padding:14px;border:1px solid #e1ded8;box-shadow:0 1px 1px #00000018}.title{margin:0 0 10px;color:var(--green);font-size:14px;font-weight:800}.process-card{padding:9px 14px}.process{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:6px}.chip{padding:5px 4px;border-radius:10px;text-align:center;background:var(--ceramic);color:var(--soft);font-size:13px;line-height:18px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.chip.on{background:var(--accent);color:#fff}.progress{height:12px;margin:6px 0;border-radius:7px;background:var(--ceramic);overflow:hidden}.progress>div{height:100%;background:var(--accent);width:0}.process-head{display:flex;gap:10px;align-items:center;min-width:0;line-height:18px}.process-head strong{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.process-head span{margin-left:auto;color:var(--soft)}
.workspace{display:grid;grid-template-columns:38% minmax(0,62%);gap:10px;min-height:0}.left{display:grid;grid-template-rows:1fr 1fr;gap:10px;min-width:0}.left-top{display:grid;grid-template-columns:1fr 1.2fr;gap:10px;min-width:0}.kv{display:grid;grid-template-columns:auto minmax(0,1fr);gap:8px 12px;align-items:center}.kv b,.mono{white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.green{color:var(--green)}.led{display:inline-block;width:10px;height:10px;border-radius:50%;background:#777;margin-right:6px}.led.on{background:var(--accent)}
.joints{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.joint{display:grid;grid-template-columns:24px minmax(40px,1fr) 58px;gap:6px;align-items:center}.track{height:14px;border-radius:8px;background:var(--ceramic);overflow:hidden}.track i{display:block;width:50%;height:100%;background:var(--accent)}.tcp{margin-top:12px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.right{display:grid;grid-template-rows:minmax(0,1fr) 88px;gap:10px;min-width:0}.control-card{display:grid;grid-template-rows:auto minmax(0,1fr);min-height:0}.control-grid{display:grid;grid-template-columns:.95fr 1.15fr 1.4fr 1.1fr;gap:8px;min-height:0}.control-section{min-width:0;min-height:0;padding:10px;border:1px solid #dedad3;border-radius:12px}.control-section h3{margin:0 0 9px;color:var(--green);font-size:14px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.btns{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px}.btns.three{grid-template-columns:repeat(3,minmax(0,1fr))}.safety button,.tool button{padding:2px 4px;font-size:12px}.safety .estop{font-size:17px}
button,input{font:inherit}button{min-width:0;min-height:30px;border:1px solid var(--accent);border-radius:50px;background:#fff;color:var(--accent);font-weight:700;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;cursor:pointer}button:hover{background:var(--mint)}button:active{transform:scale(.95)}button:disabled{border-color:#d6d2ca;background:var(--ceramic);color:#999;cursor:not-allowed}.estop{width:100%;height:42px;border:0;background:var(--red);color:#fff;font-size:17px}.field{display:flex;align-items:center;gap:6px;margin:7px 0;min-width:0}.field label{flex:1;white-space:nowrap}.field input{width:76px;min-width:0;padding:5px;border:1px solid #d6d2ca;border-radius:8px;text-align:right}.offsets{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:4px}.offsets input{width:100%}.enable{display:flex;align-items:center;gap:7px;margin:7px 0;font-weight:700;white-space:nowrap;overflow:hidden}.io{display:grid;grid-template-columns:auto 1fr;gap:7px 12px;align-items:center}.dots{display:flex;justify-content:flex-end;gap:7px;overflow:hidden}.dot{width:13px;height:13px;flex:0 0 13px;border-radius:50%;background:#5b5b5b}.dot.on{background:var(--accent)}
.logs{display:grid;grid-template-rows:auto minmax(0,1fr);min-height:0}.log-head{display:flex;align-items:center;margin-bottom:8px}.log{margin:0;min-height:0;overflow:auto;border-radius:10px;padding:12px;background:var(--house);color:#fff;font:13px/1.55 monospace;white-space:pre-wrap}
@media(max-width:1400px){body{overflow:auto}.app{height:auto;grid-template-rows:auto}.workspace{grid-template-columns:1fr}.control-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.logs{min-height:280px}}
</style></head><body><div class="app">
<header class="bar"><div class="brand">ROKEY BREW LAB</div><div class="sub">M0609 · WEB SYSTEM MONITOR & ROBOT CONTROL</div><div class="spacer"></div><div class="badge">ADMINISTRATOR</div></header>
<div id="hero" class="hero">ROS 연결 대기</div>
<section class="card process-card"><div class="process-head"><strong id="step">공정 보고 없음</strong><span id="pct">0%</span></div><div class="progress"><div id="pbar"></div></div><div id="chips" class="process"></div></section>
<main class="workspace"><div class="left"><div class="left-top">
<section class="card"><h2 class="title">1. 로봇 상태</h2><div class="kv"><span>상태</span><b id="state" class="green">--</b><span>운전 모드</span><b id="mode">--</b><span>속도 모드</span><b id="speedmode">--</b><span>TCP 속도</span><b id="speed">--</b></div></section>
<section class="card"><h2 class="title">2. 통신 상태</h2><div class="kv"><span>ROS</span><b><i id="rosled" class="led"></i><span id="link">--</span></b><span>서비스</span><b id="service">--</b><span>RG2</span><b id="gripper">--</b></div></section>
</div><section class="card"><h2 class="title">3. 로봇 위치</h2><div id="joints" class="joints"></div><div id="tcp" class="tcp mono">TCP --</div></section></div>
<div class="right"><section class="card control-card"><h2 class="title">4. 로봇 제어</h2><div class="control-grid">
<div class="control-section safety"><h3>안전 / 모드</h3><button class="estop" onclick="cmd({cmd:'estop'})">E-STOP</button><label class="enable"><input id="enable" type="checkbox">제어 활성화</label><div class="btns"><button data-motion onclick="cmd({cmd:'start'})">START</button><button data-motion onclick="cmd({cmd:'stop'})">STOP</button><button data-motion onclick="cmd({cmd:'set_mode',mode:'MANUAL'})">MANUAL</button><button data-motion onclick="cmd({cmd:'set_mode',mode:'AUTO'})">AUTO</button></div></div>
<div class="control-section"><h3>XYZ 상대 이동 / DR_BASE</h3><div class="field"><label>이동 간격</label><input id="move" type="number" value="1" min=".1" max="100" step=".1"></div><div class="btns three" id="xyz"></div><div class="field"><label>선속도</label><input id="vel" type="number" value="20" min="1" max="300"></div></div>
<div class="control-section"><h3>TCP 회전 / TOOL offset pivot</h3><div class="field"><label>회전 간격</label><input id="rot" type="number" value="5" min=".1" max="45" step=".1"></div><div class="btns three" id="abc"></div><div class="offsets"><input id="ox" type="number" value="0" title="Offset X"><input id="oy" type="number" value="0" title="Offset Y"><input id="oz" type="number" value="0" title="Offset Z"></div></div>
<div class="control-section tool"><h3>J6 / RG2</h3><div class="field"><label>J6 간격</label><input id="j6" type="number" value="10" min=".1" max="90"></div><div class="btns"><button data-motion onclick="joint6(-1)">J6 −</button><button data-motion onclick="joint6(1)">J6 +</button><button data-motion onclick="cmd({cmd:'gripper',command:'o'})">그리퍼 열기</button><button data-motion onclick="cmd({cmd:'gripper',command:'c'})">그리퍼 닫기</button></div></div>
</div></section><section class="card"><h2 class="title">5. IO / 그리퍼</h2><div class="io"><span>DI</span><div id="di" class="dots"></div><span>DO</span><div id="do" class="dots"></div></div></section></div></main>
<section class="card logs"><div class="log-head"><h2 class="title" style="margin:0">6. 이벤트 로그</h2></div><pre id="log" class="log"></pre></section>
</div><script>
const $=id=>document.getElementById(id),steps=['초기화','홈 위치 이동','대기 · DI13','원두 집기','그라인더에 붓기','뚜껑 닫힘 · DI14','그라인딩','드리퍼 이동','물컵 그립','핸드드립 진행','물컵 원위치','1 사이클 완료'];
$('chips').innerHTML=steps.map((x,i)=>`<div class="chip" data-i="${i}">${i+1}. ${x}</div>`).join('');
function buttons(id,names,start){$(id).innerHTML=[1,-1].flatMap(s=>names.map((n,i)=>`<button data-motion onclick="move(${start+i},${s})">${n}${s>0?'+':'−'}</button>`)).join('')}
buttons('xyz',['X','Y','Z'],0);buttons('abc',['A','B','C'],3);
async function cmd(p){await fetch('/api/admin/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)})}
function move(axis,sign){const rot=axis>2;cmd({cmd:'move_task',axis,distance:sign*Number($(rot?'rot':'move').value),linear_speed:Number($('vel').value),tcp_offset:[+$('ox').value,+$('oy').value,+$('oz').value]})}
function joint6(sign){cmd({cmd:'move_joint6',delta:sign*Number($('j6').value)})}
$('enable').onchange=e=>{document.querySelectorAll('[data-motion]').forEach(b=>b.disabled=!e.target.checked);cmd({cmd:'set_control_enabled',enabled:e.target.checked})};$('enable').onchange({target:$('enable')});
function dots(id,a){$(id).innerHTML=Array.from({length:16},(_,i)=>`<i class="dot ${a&&a[i]?'on':''}"></i>`).join('')}
function render(s){const ok=s.connected,idx=Number(s.step_index??-1);$('hero').textContent=ok?`${s.fault?'FAULT':'정상'} — ${s.robot_state_str||'UNKNOWN'}`:'ROS 연결 대기';$('hero').style.background=s.fault?'var(--red)':ok?'var(--green)':'var(--gold)';$('step').textContent=(s.step_label||'공정 보고 없음')+(s.step_status?` [${s.step_status}]`:'');$('pct').textContent=Math.round(s.task_progress||0)+'%';$('pbar').style.width=(s.task_progress||0)+'%';document.querySelectorAll('.chip').forEach((c,i)=>c.classList.toggle('on',i===idx));$('state').textContent=s.robot_state_str||'--';$('mode').textContent=s.robot_mode||'--';$('speedmode').textContent=s.speed_mode||'--';$('speed').textContent=`${Number(s.linear_speed||0).toFixed(1)} mm/s`;$('rosled').classList.toggle('on',ok);$('link').textContent=s.link_state||'--';$('service').textContent=s.service_ready?'연결됨':'미연결';$('gripper').textContent=s.gripper_connected?`${Number(s.gripper_width_mm||0).toFixed(1)} mm`:'미연결';const j=s.joint_pos_deg||[];$('joints').innerHTML=Array.from({length:6},(_,i)=>`<div class="joint"><b>J${i+1}</b><div class="track"><i></i></div><span>${Number(j[i]||0).toFixed(2)}°</span></div>`).join('');$('tcp').textContent='TCP  '+(s.tcp_posx||[]).map(v=>Number(v).toFixed(1)).join('   ');dots('di',s.di);dots('do',s.do);$('log').textContent=(s.logs||[]).join('\n');$('log').scrollTop=$('log').scrollHeight}
async function poll(){try{render(await (await fetch('/api/admin/state')).json())}catch(e){}setTimeout(poll,200)}poll();
</script></body></html>"""


def main() -> None:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()
