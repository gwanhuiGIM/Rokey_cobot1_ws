#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only FastAPI Web UI for the M0609 physical-button coffee system."""

from __future__ import annotations

import asyncio
import json
import threading
from contextlib import asynccontextmanager
from typing import Any, Optional

import rclpy
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

STATUS_TOPIC = "/coffee_system/status"

DEFAULT_STATE: dict[str, Any] = {
    "phase": "WAITING_CONTROLLER",
    "screen": 1,
    "progress": 0,
    "title": "물리 버튼으로 원두를 선택해 주세요",
    "message": "DI 13~16 중 하나를 누르면 커피 시스템이 시작됩니다.",
    "busy": False,
    "waiting_physical_button": True,
    "selected_bean": "",
    "selected_button": None,
    "error": "",
    "connected": False,
}


class CoffeeWebBridge(Node):
    def __init__(self) -> None:
        super().__init__("coffee_webui_bridge")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._lock = threading.RLock()
        self._state = dict(DEFAULT_STATE)
        self.create_subscription(String, STATUS_TOPIC, self._callback, qos)

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
        return result


bridge: Optional[CoffeeWebBridge] = None
spin_thread: Optional[threading.Thread] = None


@asynccontextmanager
async def lifespan(_: FastAPI):
    global bridge, spin_thread
    rclpy.init(args=None)
    bridge = CoffeeWebBridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()
    try:
        yield
    finally:
        if bridge is not None:
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)


app = FastAPI(title="ROKEY Coffee System", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def index() -> HTMLResponse:
    return HTMLResponse(HTML)


@app.get("/api/state")
async def api_state() -> dict[str, Any]:
    return bridge.state() if bridge is not None else dict(DEFAULT_STATE)


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
:root{--bg:#eee6dc;--paper:#fffaf5;--ink:#221b16;--muted:#786b61;--coffee:#704229;--accent:#d58a46;--green:#3d7655;--red:#a7423b;--line:rgba(72,47,31,.14)}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:radial-gradient(circle at 12% 8%,rgba(213,138,70,.2),transparent 30rem),var(--bg);color:var(--ink);font-family:Pretendard,"Noto Sans KR",system-ui,sans-serif}
.shell{width:min(1120px,calc(100% - 24px));margin:auto;padding:24px 0 40px}.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:18px}.brand{font-size:20px;font-weight:950}.sub{font-size:12px;color:var(--muted)}.conn{padding:9px 13px;border:1px solid var(--line);border-radius:999px;background:#ffffffa8;color:var(--muted);font-size:13px;font-weight:800}.conn.on{color:var(--green)}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px}.panel{background:#fffaf5e8;border:1px solid #ffffffb8;border-radius:28px;box-shadow:0 22px 65px #3a241824}.main{min-height:670px;padding:34px}.side{padding:23px;display:flex;flex-direction:column}.screen{display:none}.screen.active{display:block}.eyebrow{font-size:12px;font-weight:950;letter-spacing:.12em;color:var(--coffee)}h1{font-size:clamp(35px,5vw,58px);line-height:1.04;letter-spacing:-.055em;margin:15px 0 12px}.lead{color:var(--muted);line-height:1.7}.notice,.wait{padding:18px;border:1px solid #d58a4660;border-radius:18px;background:#fff2df;font-weight:900;line-height:1.65}.notice{margin:24px 0}.beans{display:grid;grid-template-columns:1fr 1fr;gap:14px}.bean{position:relative;min-height:140px;padding:20px;border:1px solid var(--line);border-radius:21px;background:#ffffffa8}.bean.selected{border-color:var(--coffee);box-shadow:0 0 0 4px #7042291c;background:white}.di{position:absolute;right:14px;top:14px;padding:6px 9px;border-radius:999px;background:#70422915;color:var(--coffee);font-size:11px;font-weight:950}.bean-name{font-size:18px;font-weight:950;margin-top:34px}.bean-note{font-size:13px;color:var(--muted);margin-top:7px}.process{min-height:530px;display:grid;place-items:center;text-align:center}.icon{font-size:78px}.title{font-size:31px;font-weight:950;margin:20px 0 10px}.message{max-width:600px;color:var(--muted);line-height:1.75}.wait{display:none;margin-top:22px}.wait.show{display:block}.steps{display:grid;gap:8px;margin-top:16px}.step{display:grid;grid-template-columns:36px 1fr;gap:11px;align-items:center;padding:12px;border-radius:15px;color:var(--muted)}.step.active{background:#70422912;color:var(--ink)}.step.done .num{background:var(--green);color:white}.num{width:36px;height:36px;border-radius:12px;display:grid;place-items:center;background:#70422912;font-weight:950}.step.active .num{background:var(--coffee);color:white}.name{font-size:14px;font-weight:900}.desc{font-size:11px;margin-top:3px}.status{margin-top:auto;padding-top:20px;border-top:1px solid var(--line)}.row{display:flex;justify-content:space-between;gap:12px;margin-top:12px}.label{font-size:12px;color:var(--muted)}.value{font-size:13px;font-weight:900;text-align:right}.track{height:9px;margin:11px 0;border-radius:999px;background:#3a241817;overflow:hidden}.bar{height:100%;width:0;background:linear-gradient(90deg,var(--coffee),var(--accent));transition:.3s}.error{display:none;margin-top:18px;padding:15px;border-radius:15px;background:#a7423b16;color:var(--red);font-size:12px}.error.show{display:block}
@media(max-width:850px){.layout{grid-template-columns:1fr}.side{order:-1}.steps{grid-template-columns:repeat(4,1fr)}.step{grid-template-columns:1fr;text-align:center}.num{margin:auto}.desc{display:none}.status{margin-top:18px}}@media(max-width:600px){.beans{grid-template-columns:1fr}.main{padding:24px 18px}}
</style>
</head>
<body>
<div class="shell">
<header class="top"><div><div class="brand">ROKEY BREW LAB</div><div class="sub">M0609 Physical Button Coffee System</div></div><div id="conn" class="conn">ROS 연결 대기</div></header>
<div class="layout">
<main class="panel main">
<section id="s1" class="screen active"><div class="eyebrow">STEP 01 · PHYSICAL SELECTION</div><h1>물리 버튼으로<br>원두를 선택해 주세요.</h1><p class="lead">웹 화면은 상태 표시 전용입니다. 화면을 클릭해도 로봇 명령을 전송하지 않습니다.</p><div class="notice">DI 13~16 중 하나를 누르면 해당 원두가 선택되고 커피 시스템이 시작됩니다.</div><div class="beans">
<article class="bean" data-button="13"><span class="di">DI 13</span><div class="bean-name">에티오피아 예가체프</div><div class="bean-note">재스민 · 레몬 · 밝은 산미</div></article>
<article class="bean" data-button="14"><span class="di">DI 14</span><div class="bean-name">콜롬비아 수프리모</div><div class="bean-note">카라멜 · 견과 · 균형감</div></article>
<article class="bean" data-button="15"><span class="di">DI 15</span><div class="bean-name">브라질 산토스</div><div class="bean-note">초콜릿 · 아몬드 · 낮은 산미</div></article>
<article class="bean" data-button="16"><span class="di">DI 16</span><div class="bean-name">과테말라 안티구아</div><div class="bean-note">코코아 · 스파이스 · 긴 여운</div></article>
</div><div id="error" class="error"></div></section>
<section id="s2" class="screen"><div class="eyebrow">STEP 02 · BEAN LOADING</div><div class="process"><div><div class="icon">🥄</div><div id="t2" class="title"></div><div id="m2" class="message"></div></div></div></section>
<section id="s3" class="screen"><div class="eyebrow">STEP 03 · GRINDING</div><div class="process"><div><div class="icon">⚙</div><div id="t3" class="title"></div><div id="m3" class="message"></div></div></div></section>
<section id="s4" class="screen"><div class="eyebrow">STEP 04 · FILTER LOADING</div><div class="process"><div><div class="icon">☕</div><div id="t4" class="title"></div><div id="m4" class="message"></div><div id="wait" class="wait">병에 가루가 남아있으면 병 바닥을 살짝 쳐주세요.<br>확인 후 DI 13~16 중 아무 버튼이나 눌러주세요.</div></div></div></section>
<section id="s5" class="screen"><div class="eyebrow">COMPLETE</div><div class="process"><div><div class="icon">✓</div><div id="t5" class="title"></div><div id="m5" class="message"></div></div></div></section>
<section id="s6" class="screen"><div class="eyebrow">ROBOT ERROR</div><div class="process"><div><div class="icon">!</div><div id="t6" class="title"></div><div id="m6" class="message"></div></div></div></section>
</main>
<aside class="panel side"><div class="eyebrow">PROCESS TIMELINE</div><div class="steps">
<div class="step active" data-step="1"><div class="num">1</div><div><div class="name">원두 선택</div><div class="desc">DI 13~16</div></div></div>
<div class="step" data-step="2"><div class="num">2</div><div><div class="name">원두 투입</div><div class="desc">스푼 작업</div></div></div>
<div class="step" data-step="3"><div class="num">3</div><div><div class="name">그라인딩</div><div class="desc">핸들 회전</div></div></div>
<div class="step" data-step="4"><div class="num">4</div><div><div class="name">필터 투입</div><div class="desc">병 확인</div></div></div>
</div><div class="status"><div class="row"><span class="label">상태</span><span id="phase" class="value">WAITING</span></div><div class="track"><div id="bar" class="bar"></div></div><div class="row"><span class="label">진행률</span><span id="progress" class="value">0%</span></div><div class="row"><span class="label">원두</span><span id="bean" class="value">선택 전</span></div><div class="row"><span class="label">선택 버튼</span><span id="button" class="value">-</span></div></div></aside>
</div></div>
<script>
const $=id=>document.getElementById(id);
function show(n){[1,2,3,4,5,6].forEach(x=>$('s'+x).classList.toggle('active',x===n));document.querySelectorAll('.step').forEach(e=>{const x=Number(e.dataset.step),c=Math.min(n,4);e.classList.toggle('active',x===c);e.classList.toggle('done',x<c)})}
function apply(s){const c=!!s.connected;$('conn').classList.toggle('on',c);$('conn').textContent=c?'ROS 연결됨':'ROS 연결 대기';const n=Math.max(1,Math.min(6,Number(s.screen||1))),p=Math.max(0,Math.min(100,Number(s.progress||0)));show(n);$('phase').textContent=s.phase||'WAITING';$('bar').style.width=p+'%';$('progress').textContent=p+'%';$('bean').textContent=s.selected_bean||'선택 전';$('button').textContent=s.selected_button?'DI '+s.selected_button:'-';[2,3,4,5,6].forEach(x=>{$('t'+x).textContent=s.title||'';$('m'+x).textContent=s.message||''});$('wait').classList.toggle('show',!!s.waiting_physical_button&&n===4);document.querySelectorAll('.bean').forEach(e=>e.classList.toggle('selected',Number(e.dataset.button)===Number(s.selected_button)));const er=s.error||'';$('error').classList.toggle('show',!!er);$('error').textContent=er?'오류 상세: '+er:''}
function connect(){const ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);ws.onmessage=e=>apply(JSON.parse(e.data));ws.onclose=()=>setTimeout(connect,1200);ws.onerror=()=>ws.close()}
fetch('/api/state').then(r=>r.json()).then(apply).catch(()=>{});connect();
</script>
</body>
</html>"""


def main() -> None:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()