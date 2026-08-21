# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """Read-only FastAPI Web UI for physical-button selection and force confirmation."""

# from __future__ import annotations

# import asyncio
# import json
# import threading
# from contextlib import asynccontextmanager
# from typing import Any, Optional

# import rclpy
# from fastapi import FastAPI, WebSocket, WebSocketDisconnect
# from fastapi.responses import HTMLResponse
# from rclpy.node import Node
# from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# from std_msgs.msg import String

# STATUS_TOPIC = "/coffee_system/status"

# DEFAULT_STATE: dict[str, Any] = {
#     "phase": "WAITING_CONTROLLER",
#     "screen": 1,
#     "progress": 0,
#     "title": "물리 버튼으로 원두를 선택해 주세요",
#     "message": "DI 13~16 중 하나를 누르면 커피 시스템이 시작됩니다.",
#     "busy": False,
#     "waiting_physical_button": True,
#     "waiting_external_force": False,
#     "force_threshold_n": 10.0,
#     "force_delta_n": 0.0,
#     "force_peak_n": 0.0,
#     "selected_bean": "",
#     "selected_button": None,
#     "selected_grind": "",
#     "selected_grind_button": None,
#     "grind_turns": 0,
#     "error": "",
#     "connected": False,
# }


# class CoffeeWebBridge(Node):
#     def __init__(self) -> None:
#         super().__init__("coffee_webui_bridge")
#         qos = QoSProfile(depth=10)
#         qos.reliability = ReliabilityPolicy.RELIABLE
#         qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
#         self._lock = threading.RLock()
#         self._state = dict(DEFAULT_STATE)
#         self.create_subscription(String, STATUS_TOPIC, self._callback, qos)

#     def _callback(self, msg: String) -> None:
#         try:
#             payload = json.loads(msg.data)
#             if not isinstance(payload, dict):
#                 return
#         except json.JSONDecodeError:
#             return
#         with self._lock:
#             self._state.update(payload)

#     def state(self) -> dict[str, Any]:
#         with self._lock:
#             result = dict(self._state)
#         result["connected"] = self.count_publishers(STATUS_TOPIC) > 0
#         return result


# bridge: Optional[CoffeeWebBridge] = None
# spin_thread: Optional[threading.Thread] = None


# @asynccontextmanager
# async def lifespan(_: FastAPI):
#     global bridge, spin_thread
#     rclpy.init(args=None)
#     bridge = CoffeeWebBridge()
#     spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
#     spin_thread.start()
#     try:
#         yield
#     finally:
#         if bridge is not None:
#             bridge.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()
#         if spin_thread is not None:
#             spin_thread.join(timeout=2.0)


# app = FastAPI(title="ROKEY Coffee System", lifespan=lifespan)


# @app.get("/", response_class=HTMLResponse)
# async def index() -> HTMLResponse:
#     return HTMLResponse(HTML)


# @app.get("/api/state")
# async def api_state() -> dict[str, Any]:
#     return bridge.state() if bridge is not None else dict(DEFAULT_STATE)


# @app.websocket("/ws")
# async def websocket_status(websocket: WebSocket) -> None:
#     await websocket.accept()
#     previous = ""
#     try:
#         while True:
#             state = bridge.state() if bridge is not None else dict(DEFAULT_STATE)
#             encoded = json.dumps(state, ensure_ascii=False, sort_keys=True)
#             if encoded != previous:
#                 await websocket.send_json(state)
#                 previous = encoded
#             await asyncio.sleep(0.2)
#     except WebSocketDisconnect:
#         return


# HTML = r"""<!doctype html>
# <html lang="ko">
# <head>
# <meta charset="utf-8">
# <meta name="viewport" content="width=device-width,initial-scale=1">
# <title>ROKEY Coffee System</title>
# <style>
# :root{--bg:#eee6dc;--paper:#fffaf5;--ink:#221b16;--muted:#786b61;--coffee:#704229;--accent:#d58a46;--green:#3d7655;--red:#a7423b;--line:rgba(72,47,31,.14)}
# *{box-sizing:border-box}body{margin:0;min-height:100vh;background:radial-gradient(circle at 12% 8%,rgba(213,138,70,.2),transparent 30rem),var(--bg);color:var(--ink);font-family:Pretendard,"Noto Sans KR",system-ui,sans-serif}
# .shell{width:min(1120px,calc(100% - 24px));margin:auto;padding:24px 0 40px}.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:18px}.brand{font-size:20px;font-weight:950}.sub{font-size:12px;color:var(--muted)}.conn{padding:9px 13px;border:1px solid var(--line);border-radius:999px;background:#ffffffa8;color:var(--muted);font-size:13px;font-weight:800}.conn.on{color:var(--green)}
# .layout{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px}.panel{background:#fffaf5e8;border:1px solid #ffffffb8;border-radius:28px;box-shadow:0 22px 65px #3a241824}.main{min-height:670px;padding:34px}.side{padding:23px;display:flex;flex-direction:column}.screen{display:none}.screen.active{display:block}.eyebrow{font-size:12px;font-weight:950;letter-spacing:.12em;color:var(--coffee)}h1{font-size:clamp(35px,5vw,58px);line-height:1.04;letter-spacing:-.055em;margin:15px 0 12px}.lead{color:var(--muted);line-height:1.7}.notice,.wait{padding:18px;border:1px solid #d58a4660;border-radius:18px;background:#fff2df;font-weight:900;line-height:1.65}.notice{margin:24px 0}.beans{display:grid;grid-template-columns:1fr 1fr;gap:14px}.bean{position:relative;min-height:140px;padding:20px;border:1px solid var(--line);border-radius:21px;background:#ffffffa8}.bean.selected{border-color:var(--coffee);box-shadow:0 0 0 4px #7042291c;background:white}.di{position:absolute;right:14px;top:14px;padding:6px 9px;border-radius:999px;background:#70422915;color:var(--coffee);font-size:11px;font-weight:950}.bean-name{font-size:18px;font-weight:950;margin-top:34px}.bean-note{font-size:13px;color:var(--muted);margin-top:7px}.process{min-height:530px;display:grid;place-items:center;text-align:center}.icon{font-size:78px}.title{font-size:31px;font-weight:950;margin:20px 0 10px}.message{max-width:600px;color:var(--muted);line-height:1.75}.wait{display:none;margin-top:22px}.wait.show{display:block}.force-line{display:flex;justify-content:space-between;gap:18px;margin-top:13px;padding-top:13px;border-top:1px solid #d58a4660}.force-value{font-size:18px;color:var(--coffee)}.steps{display:grid;gap:8px;margin-top:16px}.step{display:grid;grid-template-columns:36px 1fr;gap:11px;align-items:center;padding:12px;border-radius:15px;color:var(--muted)}.step.active{background:#70422912;color:var(--ink)}.step.done .num{background:var(--green);color:white}.num{width:36px;height:36px;border-radius:12px;display:grid;place-items:center;background:#70422912;font-weight:950}.step.active .num{background:var(--coffee);color:white}.name{font-size:14px;font-weight:900}.desc{font-size:11px;margin-top:3px}.status{margin-top:auto;padding-top:20px;border-top:1px solid var(--line)}.row{display:flex;justify-content:space-between;gap:12px;margin-top:12px}.label{font-size:12px;color:var(--muted)}.value{font-size:13px;font-weight:900;text-align:right}.track{height:9px;margin:11px 0;border-radius:999px;background:#3a241817;overflow:hidden}.bar{height:100%;width:0;background:linear-gradient(90deg,var(--coffee),var(--accent));transition:.3s}.error{display:none;margin-top:18px;padding:15px;border-radius:15px;background:#a7423b16;color:var(--red);font-size:12px}.error.show{display:block}
# @media(max-width:850px){.layout{grid-template-columns:1fr}.side{order:-1}.steps{grid-template-columns:repeat(5,1fr)}.step{grid-template-columns:1fr;text-align:center}.num{margin:auto}.desc{display:none}.status{margin-top:18px}}@media(max-width:600px){.beans{grid-template-columns:1fr}.main{padding:24px 18px}}
# </style>
# </head>
# <body>
# <div class="shell">
# <header class="top"><div><div class="brand">ROKEY BREW LAB</div><div class="sub">M0609 Physical Button Coffee System</div></div><div id="conn" class="conn">ROS 연결 대기</div></header>
# <div class="layout">
# <main class="panel main">
# <section id="s1" class="screen active"><div class="eyebrow">STEP 01 · PHYSICAL SELECTION</div><h1>물리 버튼으로<br>원두를 선택해 주세요.</h1><p class="lead">웹 화면은 상태 표시 전용입니다. 화면을 클릭해도 로봇 명령을 전송하지 않습니다.</p><div class="notice">DI 13~16 중 하나를 누르면 해당 원두가 선택되고 커피 시스템이 시작됩니다.</div><div class="beans">
# <article class="bean" data-button="13"><span class="di">DI 13</span><div class="bean-name">에티오피아 예가체프</div><div class="bean-note">재스민 · 레몬 · 밝은 산미</div></article>
# <article class="bean" data-button="14"><span class="di">DI 14</span><div class="bean-name">콜롬비아 수프리모</div><div class="bean-note">카라멜 · 견과 · 균형감</div></article>
# <article class="bean" data-button="15"><span class="di">DI 15</span><div class="bean-name">브라질 산토스</div><div class="bean-note">초콜릿 · 아몬드 · 낮은 산미</div></article>
# <article class="bean" data-button="16"><span class="di">DI 16</span><div class="bean-name">과테말라 안티구아</div><div class="bean-note">코코아 · 스파이스 · 긴 여운</div></article>
# </div><div id="error" class="error"></div></section>
# <section id="s2" class="screen"><div class="eyebrow">STEP 02 · BEAN LOADING</div><div class="process"><div><div class="icon">🥄</div><div id="t2" class="title"></div><div id="m2" class="message"></div></div></div></section>

# <section id="s3" class="screen">
# <div class="eyebrow">STEP 03 · GRIND SIZE SELECTION</div>
# <h1>원하는 분쇄 굵기를<br>선택해 주세요.</h1>
# <p class="lead">물리 버튼 DI 13~16으로 분쇄 굵기를 선택합니다. 1회전은 그라인더 손잡이 360° 회전입니다.</p>
# <div class="notice">분쇄 굵기에 따라 그라인더 회전 수가 달라집니다.</div>
# <div class="beans">
# <article class="bean grind-card" data-grind-button="13"><span class="di">DI 13</span><div class="bean-name">굵게 분쇄</div><div class="bean-note">3회전 · 굵은 입자</div></article>
# <article class="bean grind-card" data-grind-button="14"><span class="di">DI 14</span><div class="bean-name">중간 굵게 분쇄</div><div class="bean-note">5회전 · 중간 굵은 입자</div></article>
# <article class="bean grind-card" data-grind-button="15"><span class="di">DI 15</span><div class="bean-name">중간 곱게 분쇄</div><div class="bean-note">7회전 · 중간 고운 입자</div></article>
# <article class="bean grind-card" data-grind-button="16"><span class="di">DI 16</span><div class="bean-name">곱게 분쇄</div><div class="bean-note">10회전 · 고운 입자</div></article>
# </div>
# </section>

# <section id="s4" class="screen"><div class="eyebrow">STEP 04 · GRINDING</div><div class="process"><div><div class="icon">⚙</div><div id="t4" class="title"></div><div id="m4" class="message"></div></div></div></section>
# <section id="s5" class="screen"><div class="eyebrow">STEP 05 · FILTER LOADING</div><div class="process"><div><div class="icon">☕</div><div id="t5" class="title"></div><div id="m5" class="message"></div><div id="wait" class="wait">병 바닥을 가볍게 쳐주세요.<br>10 N 이상의 외력이 감지되면 자동으로 다음 단계로 진행합니다.<div class="force-line"><span>현재 외력 변화량</span><strong id="force-now" class="force-value">0.0 / 10.0 N</strong></div></div></div></div></section>
# <section id="s6" class="screen"><div class="eyebrow">COMPLETE</div><div class="process"><div><div class="icon">✓</div><div id="t6" class="title"></div><div id="m6" class="message"></div></div></div></section>
# <section id="s7" class="screen"><div class="eyebrow">ROBOT ERROR</div><div class="process"><div><div class="icon">!</div><div id="t7" class="title"></div><div id="m7" class="message"></div></div></div></section>
# </main>
# <aside class="panel side"><div class="eyebrow">PROCESS TIMELINE</div><div class="steps">
# <div class="step active" data-step="1"><div class="num">1</div><div><div class="name">원두 선택</div><div class="desc">DI 13~16</div></div></div>
# <div class="step" data-step="2"><div class="num">2</div><div><div class="name">원두 투입</div><div class="desc">스푼 작업</div></div></div>
# <div class="step" data-step="3"><div class="num">3</div><div><div class="name">분쇄 굵기 선택</div><div class="desc">3·5·7·10회전</div></div></div>
# <div class="step" data-step="4"><div class="num">4</div><div><div class="name">그라인딩</div><div class="desc">선택 회전 수 적용</div></div></div>
# <div class="step" data-step="5"><div class="num">5</div><div><div class="name">필터 투입</div><div class="desc">병 확인</div></div></div>
# </div><div class="status">
# <div class="row"><span class="label">상태</span><span id="phase" class="value">WAITING</span></div>
# <div class="track"><div id="bar" class="bar"></div></div>
# <div class="row"><span class="label">진행률</span><span id="progress" class="value">0%</span></div>
# <div class="row"><span class="label">원두</span><span id="bean" class="value">선택 전</span></div>
# <div class="row"><span class="label">원두 선택 버튼</span><span id="button" class="value">-</span></div>
# <div class="row"><span class="label">분쇄 굵기</span><span id="grind" class="value">선택 전</span></div>
# <div class="row"><span class="label">분쇄 선택 버튼</span><span id="grind-button" class="value">-</span></div>
# <div class="row"><span class="label">그라인더 회전</span><span id="turns" class="value">-</span></div>
# <div class="row"><span class="label">현재 외력 변화</span><span id="force" class="value">0.0 N</span></div>
# <div class="row"><span class="label">외력 감지 기준</span><span id="force-threshold" class="value">10.0 N</span></div>
# </div></aside>
# </div></div>
# <script>
# const $=id=>document.getElementById(id);
# function show(n){
#   [1,2,3,4,5,6,7].forEach(x=>$('s'+x).classList.toggle('active',x===n));
#   const complete=n===6;
#   const current=n===7?5:Math.min(n,5);
#   document.querySelectorAll('.step').forEach(e=>{
#     const x=Number(e.dataset.step);
#     e.classList.toggle('active',!complete&&x===current);
#     e.classList.toggle('done',complete||x<current);
#   });
# }
# function apply(s){
#   const c=!!s.connected;
#   $('conn').classList.toggle('on',c);
#   $('conn').textContent=c?'ROS 연결됨':'ROS 연결 대기';

#   const n=Math.max(1,Math.min(7,Number(s.screen||1)));
#   const p=Math.max(0,Math.min(100,Number(s.progress||0)));
#   show(n);

#   $('phase').textContent=s.phase||'WAITING';
#   $('bar').style.width=p+'%';
#   $('progress').textContent=p+'%';
#   $('bean').textContent=s.selected_bean||'선택 전';
#   $('button').textContent=s.selected_button?'DI '+s.selected_button:'-';
#   $('grind').textContent=s.selected_grind||'선택 전';
#   $('grind-button').textContent=s.selected_grind_button?'DI '+s.selected_grind_button:'-';
#   $('turns').textContent=Number(s.grind_turns||0)>0?s.grind_turns+'회전':'-';
#   const forceValue=Number(s.force_delta_n||0);
#   const forceThreshold=Number(s.force_threshold_n||10);
#   $('force').textContent=forceValue.toFixed(1)+' N';
#   $('force-threshold').textContent=forceThreshold.toFixed(1)+' N';
#   $('force-now').textContent=forceValue.toFixed(1)+' / '+forceThreshold.toFixed(1)+' N';

#   [2,3,4,5,6,7].forEach(x=>{
#     const title=$('t'+x);
#     const message=$('m'+x);
#     if(title) title.textContent=s.title||'';
#     if(message) message.textContent=s.message||'';
#   });

#   $('wait').classList.toggle('show',!!s.waiting_external_force&&n===5);

#   document.querySelectorAll('.bean[data-button]').forEach(e=>{
#     e.classList.toggle(
#       'selected',
#       Number(e.dataset.button)===Number(s.selected_button)
#     );
#   });

#   document.querySelectorAll('.grind-card').forEach(e=>{
#     e.classList.toggle(
#       'selected',
#       Number(e.dataset.grindButton)===Number(s.selected_grind_button)
#     );
#   });

#   const er=s.error||'';
#   $('error').classList.toggle('show',!!er);
#   $('error').textContent=er?'오류 상세: '+er:'';
# }
# function connect(){const ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`);ws.onmessage=e=>apply(JSON.parse(e.data));ws.onclose=()=>setTimeout(connect,1200);ws.onerror=()=>ws.close()}
# fetch('/api/state').then(r=>r.json()).then(apply).catch(()=>{});connect();
# </script>
# </body>
# </html>"""


# def main() -> None:
#     import uvicorn
#     uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only FastAPI Web UI for physical-button selection and force confirmation."""

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
    "waiting_external_force": False,
    "force_threshold_n": 10.0,
    "force_delta_n": 0.0,
    "force_peak_n": 0.0,
    "selected_bean": "",
    "selected_button": None,
    "selected_grind": "",
    "selected_grind_button": None,
    "grind_turns": 0,
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
  .bean-name{max-width:100%;padding-right:58px;font-size:18px}
}
@media(prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.01ms!important;animation-iteration-count:1!important;transition-duration:.01ms!important;scroll-behavior:auto!important}
}
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

<section id="s4" class="screen"><div class="eyebrow">STEP 04 · GRINDING</div><div class="process"><div><div class="icon">⚙</div><div id="t4" class="title"></div><div id="m4" class="message"></div></div></div></section>
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
<div class="row"><span class="label">그라인더 회전</span><span id="turns" class="value">-</span></div>
<div class="row"><span class="label">현재 외력 변화</span><span id="force" class="value">0.0 N</span></div>
<div class="row"><span class="label">외력 감지 기준</span><span id="force-threshold" class="value">10.0 N</span></div>
</div></aside>
</div></div>
<script>
const $=id=>document.getElementById(id);
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
  $('turns').textContent=Number(s.grind_turns||0)>0?s.grind_turns+'회전':'-';
  const forceValue=Number(s.force_delta_n||0);
  const forceThreshold=Number(s.force_threshold_n||10);
  $('force').textContent=forceValue.toFixed(1)+' N';
  $('force-threshold').textContent=forceThreshold.toFixed(1)+' N';
  $('force-now').textContent=forceValue.toFixed(1)+' / '+forceThreshold.toFixed(1)+' N';

  [2,3,4,5,6,7].forEach(x=>{
    const title=$('t'+x);
    const message=$('m'+x);
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
fetch('/api/state').then(r=>r.json()).then(apply).catch(()=>{});connect();
</script>
</body>
</html>"""


def main() -> None:
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()