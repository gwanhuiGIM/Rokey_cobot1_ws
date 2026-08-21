#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FastAPI coffee-system UI and local administrator robot-control bridge."""

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
    from .system_monitor import SystemMonitor
except ImportError:  # Support direct execution from this source directory.
    from system_monitor import SystemMonitor

STATUS_TOPIC = "/coffee_system/status"
ADMIN_STATUS_TOPIC = "/system_monitor/status"
ADMIN_LOG_TOPIC = "/system_monitor/log"
ADMIN_CMD_TOPIC = "/system_monitor/cmd"

DEFAULT_STATE: dict[str, Any] = {
    "phase": "WAITING_CONTROLLER",
    "screen": 1,
    "progress": 0,
    "title": "물리 버튼으로 원두를 선택해 주세요",
    "message": "DI 13~16 중 하나를 누르면 커피 시스템이 시작됩니다.",
    "busy": False,
    "waiting_physical_button": True,
    "waiting_external_force": False,
    "force_threshold_n": 10.0,
    "force_delta_n": 0.0,
    "force_peak_n": 0.0,
    "selected_bean": "",
    "selected_button": None,
    "selected_grind": "",
    "selected_grind_button": None,
    "grind_turns": 0,
    "grind_current_turns": 0.0,
    "error": "",
    "connected": False,
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
        try:
            result["connected"] = (
                rclpy.ok() and self.count_publishers(STATUS_TOPIC) > 0)
        except Exception:
            result["connected"] = False
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
    try:
        while True:
            state = bridge.state() if bridge is not None else dict(DEFAULT_STATE)
            # Always send at 5 Hz so a closed browser is detected promptly.
            await websocket.send_json(state)
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
.admin-btn{
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
.admin-btn:hover{background:var(--green);border-color:var(--green)}
.admin-btn:active{transform:scale(.95)}
.admin-btn:disabled{cursor:wait;opacity:.68}
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
#s6 .icon{background:var(--green-accent);color:#fff}
#s7 .icon{background:rgba(200,32,20,.08);color:var(--red)}
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
@media(max-width:980px){
  .shell{width:min(100% - 32px,920px);padding-top:16px}
  .layout{grid-template-columns:1fr}
  .side{order:-1}
  .steps{grid-template-columns:repeat(5,minmax(0,1fr));gap:6px}
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
  .steps{grid-template-columns:repeat(5,72px);overflow-x:auto;padding-bottom:6px;scrollbar-width:thin}
  .status{grid-template-columns:1fr}
  .main{padding:32px 18px}
  .force-line{align-items:flex-start;flex-direction:column;gap:8px}
  .grind-progress-card{padding:22px 20px}
  .grind-current{font-size:46px}
  .grind-percent{font-size:32px}
  .grind-progress-foot{align-items:flex-start;flex-direction:column;gap:4px}
  .bean-name{max-width:100%;padding-right:58px;font-size:18px}
}
@media(prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.01ms!important;animation-iteration-count:1!important;transition-duration:.01ms!important;scroll-behavior:auto!important}
}
</style>
</head>
<body>
<div class="shell">
<header class="top"><div><div class="brand">ROKEY BREW LAB</div><div class="sub">M0609 Physical Button Coffee System</div></div><div class="top-actions"><div id="conn" class="conn">ROS 연결 대기</div><button id="admin" class="admin-btn" type="button">관리자 모드</button></div></header>
<div class="layout">
<main class="panel main">
<section id="s1" class="screen active"><div class="eyebrow">STEP 01 · PHYSICAL SELECTION</div><h1>물리 버튼으로<br>원두를 선택해 주세요.</h1><p class="lead">웹 화면은 상태 표시 전용입니다. 화면을 클릭해도 로봇 명령을 전송하지 않습니다.</p><div class="notice">DI 13~16 중 하나를 누르면 해당 원두가 선택되고 커피 시스템이 시작됩니다.</div><div class="beans">
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
<div class="notice">분쇄 굵기에 따라 그라인더 회전 수가 달라집니다.</div>
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
<section id="s5" class="screen"><div class="eyebrow">STEP 05 · FILTER LOADING</div><div class="process"><div><div class="icon">☕</div><div id="t5" class="title"></div><div id="m5" class="message"></div><div id="wait" class="wait">병 바닥을 가볍게 쳐주세요.<br>10 N 이상의 외력이 감지되면 자동으로 다음 단계로 진행합니다.<div class="force-line"><span>현재 외력 변화량</span><strong id="force-now" class="force-value">0.0 / 10.0 N</strong></div></div></div></div></section>
<section id="s6" class="screen"><div class="eyebrow">COMPLETE</div><div class="process"><div><div class="icon">✓</div><div id="t6" class="title"></div><div id="m6" class="message"></div></div></div></section>
<section id="s7" class="screen"><div class="eyebrow">ROBOT ERROR</div><div class="process"><div><div class="icon">!</div><div id="t7" class="title"></div><div id="m7" class="message"></div></div></div></section>
</main>
<aside class="panel side"><div class="eyebrow">PROCESS TIMELINE</div><div class="steps">
<div class="step active" data-step="1"><div class="num">1</div><div><div class="name">원두 선택</div><div class="desc">DI 13~16</div></div></div>
<div class="step" data-step="2"><div class="num">2</div><div><div class="name">원두 투입</div><div class="desc">스푼 작업</div></div></div>
<div class="step" data-step="3"><div class="num">3</div><div><div class="name">분쇄 굵기 선택</div><div class="desc">3·5·7·10회전</div></div></div>
<div class="step" data-step="4"><div class="num">4</div><div><div class="name">그라인딩</div><div class="desc">선택 회전 수 적용</div></div></div>
<div class="step" data-step="5"><div class="num">5</div><div><div class="name">필터 투입</div><div class="desc">병 확인</div></div></div>
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
<div class="row"><span class="label">현재 외력 변화</span><span id="force" class="value">0.0 N</span></div>
<div class="row"><span class="label">외력 감지 기준</span><span id="force-threshold" class="value">10.0 N</span></div>
</div></aside>
</div></div>
<script>
const $=id=>document.getElementById(id);
const clamp=(value,min,max)=>Math.min(max,Math.max(min,value));
function formatTurns(value){
  const numeric=Number(value);
  if(!Number.isFinite(numeric)) return '0';
  return Number.isInteger(numeric)?String(numeric):numeric.toFixed(1).replace(/\.0$/,'');
}
function show(n){
  [1,2,3,4,5,6,7].forEach(x=>$('s'+x).classList.toggle('active',x===n));
  const complete=n===6;
  const current=n===7?5:Math.min(n,5);
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

  const n=Math.max(1,Math.min(7,Number(s.screen||1)));
  const p=Math.max(0,Math.min(100,Number(s.progress||0)));
  show(n);

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

  const forceValue=Number(s.force_delta_n||0);
  const forceThreshold=Number(s.force_threshold_n||10);
  $('force').textContent=forceValue.toFixed(1)+' N';
  $('force-threshold').textContent=forceThreshold.toFixed(1)+' N';
  $('force-now').textContent=forceValue.toFixed(1)+' / '+forceThreshold.toFixed(1)+' N';

  [2,3,4,5,6,7].forEach(x=>{
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
$('admin').addEventListener('click',()=>window.open('/admin','rokey-admin'));
fetch('/api/state').then(r=>r.json()).then(apply).catch(()=>{});connect();
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
