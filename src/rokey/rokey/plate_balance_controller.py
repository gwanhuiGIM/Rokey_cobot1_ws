#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plate_balance_controller.py

plate_monitor_v2.py가 STATUS_TOPIC으로 발행하는 물체 위치 추정값
(estimated_y_mm / estimated_z_mm)을 받아, 손목 조인트 J5 / J6만 돌려서 판을
살짝 기울여 물체를 중앙으로 되돌리는 PD 제어 노드.

joint-space(servoj) 제어를 쓰는 이유
====================================
처음엔 task-space(servol, Cartesian 자세) 제어로 만들었는데, 그건 로봇의
역기구학(IK) 솔버가 "위치는 고정, 자세만 살짝 회전"을 만족시키기 위해 J1~J6
전체를 필요한 만큼 알아서 움직인다 — 즉 판을 살짝 기울이려 해도 실제로는
어깨/팔꿈치까지 같이 움찔거릴 수 있다. 그래서 J5, J6 두 조인트만 직접
움직이는 joint-space 제어(servoj)로 바꿨다. 이러면 J1~J4는 절대 안 움직이고,
정확히 손목 2개 조인트만 돈다.

주의: J6는 로봇 설계에 따라 "툴 플랜지 자전축"(그 축 자체로 회전, 판이
안 기울고 제자리에서 빙글빙글 돌기만 함)인 경우가 흔하다. 만약 J6를 돌렸는데
판이 안 기울고 돌기만 하면, 아래 TILT_JOINT_INDEX_J6를 3(J4)으로 바꿔서
J5+J4 조합으로 다시 시도할 것.

plate_monitor_v2.py는 여전히 "로봇 모션 명령을 전혀 보내지 않는" 순수 관찰
도구로 남겨두고, 실제 모션 명령은 이 노드에서만 보낸다. 두 노드는 완전히
독립적으로 시작/종료할 수 있다 (이 노드만 Ctrl+C 하면 모션 명령이 즉시 멈춘다).

핵심 안전장치
============
- DRY_RUN=True(기본값): 실제 모션 명령을 보내지 않고 계산된 목표 조인트각만 로그로 찍는다.
- watchdog: STATUS_TIMEOUT_SEC 안에 새 STATUS가 안 오면 즉시 제어 정지.
- 게이팅: object_present / pose_stable / position_model_ready 가 모두 True일 때만 동작,
  아니면 틸트를 서서히 0(중립)으로 되돌린다.
- deadband: 오차가 DEADBAND_MM 이내면 틸트를 걸지 않는다(떨림 방지).
- 절대 틸트 한계(MAX_TILT_DEG)와 틱당 변화량 한계(MAX_TILT_STEP_DEG)를 둘 다 강제한다.
- 미분(속도) 추정값 클램프 + 강한 EMA 스무딩으로 노이즈 증폭을 억제한다.
- J5/J6 회전 속도 상한을 GUI에서 별도로(더 위험한 값이라 눈에 띄게) 조정할 수 있다.
- 예외 발생 시 즉시 제어 정지.

★★★ 실제 로봇에 명령을 보내기 전 반드시 확인할 것 ★★★
====================================================
1. DRY_RUN=True 상태로 먼저 돌려서 로그에 찍히는 목표 조인트각(target posj)과
   틸트 각도가 말이 되는지 확인할 것.
2. J5, J6이 실제로 어느 방향으로 판을 기울이는지(부호), Y/Z 오차와 반대 방향으로
   기울여야 공이 중앙으로 굴러온다는 전제가 실제로 맞는지, 로봇을 저속/티치
   모드에서 눈으로 보며 확인할 것. 방향이 반대면 SIGN_J5 / SIGN_J6 를 뒤집을 것.
   J6가 판을 안 기울이고 제자리에서 돌기만 하면 TILT_JOINT_INDEX_J6를 바꿀 것.
3. 사람이 펜던트 비상정지 버튼에 손을 올린 채로 첫 실제 테스트를 할 것.
4. MAX_TILT_DEG, MAX_TILT_STEP_DEG, J5/J6 회전 속도 상한(GUI의 빨간 슬라이더)을
   매우 보수적인 값에서 시작해서 점진적으로만 올릴 것. 실기에서 너무 빠르다고
   느껴지면 그 슬라이더부터 낮출 것 — "기본값 복원" 버튼으로 즉시 되돌릴 수 있다.
5. servoj()/get_current_posj() 시그니처는 DSR_ROBOT2.py(vendored, dsr_common2)를
   기준으로 작성했다. 로봇 컨트롤러/펌웨어 버전이 다르면 반드시 재확인할 것.
====================================================
"""

from __future__ import annotations

import json
import math
import queue
import time
import traceback
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import DR_init
from std_msgs.msg import String

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError as error:
    raise RuntimeError(
        "Tkinter가 설치되지 않았습니다. "
        "Ubuntu에서 'sudo apt install python3-tk'를 실행하십시오."
    ) from error


# =============================================================================
# 사용자 설정
# =============================================================================

ROBOT_ID = "dsr01"       # plate_monitor_v2.py의 ROBOT_ID와 반드시 같아야 함
ROBOT_MODEL = "m0609"
NODE_NAME = "plate_balance_controller"

STATUS_TOPIC = f"/{ROBOT_ID}/plate_position/status"   # plate_monitor_v2.py가 발행
ENABLE_TOPIC = f"/{ROBOT_ID}/plate_balance/enable"     # String: "start" | "stop"

# --- 안전 스위치 ------------------------------------------------------------
# True인 동안은 실제 모션 명령(servoj)을 보내지 않고 로그만 남긴다.
# joint-space 제어로 바꾼 직후라 안전하게 True로 시작한다. 검증 후에만 False로.
DRY_RUN = True

# --- 제어 주기 / watchdog ---------------------------------------------------
CONTROL_HZ = 50.0
CONTROL_PERIOD_SEC = 1.0 / CONTROL_HZ
STATUS_TIMEOUT_SEC = 0.5  # 이 시간 안에 새 STATUS 메시지가 안 오면 즉시 정지

# --- 안전 한계 (보수적 기본값, 실기 검증 후 조정) ---------------------------
MAX_TILT_DEG = 2.0          # 기준 자세 대비 최대 절대 틸트 각도
MAX_TILT_STEP_DEG = 0.15    # 제어 tick 한 번에 허용하는 최대 틸트 변화량
DEADBAND_MM = 3.0           # 이 안쪽이면 오차를 0으로 간주 (떨림 방지)

# posj = [q1, q2, q3, q4, q5, q6] (0-indexed) 중 두 관절만 움직인다.
# estimated_y_mm 오차 -> J5, estimated_z_mm 오차 -> J6 로 매핑했다.
# J6가 툴 자전축이라 판이 안 기울고 돌기만 하면 3(J4)으로 바꿔서 재시도할 것.
TILT_JOINT_INDEX_J5 = 4
TILT_JOINT_INDEX_J6 = 5

# servoj에 매번 넘기는 J5/J6 회전 속도/가속도 상한 (deg/s, deg/s^2).
# 다른 조인트(J1~J4)는 목표가 항상 기준값 그대로라 실제로는 안 움직이지만,
# API 시그니처상 vel/acc는 스칼라 하나로 주면 6개 조인트에 동일하게 적용된다.
# 예전엔 기본값이 10.0deg/s였는데, 실기 테스트에서 너무 빠르게 느껴져
# 5.0 -> 3.0deg/s로 더 낮췄고, 슬라이더 상한도 예전 기본값(10.0)을 넘지 못하게 막았다.
JOINT_ACC_TO_VEL_RATIO = 2.0     # 가속도 = 이 배수 * 각속도 (기존 20:10, 40:20 비율 유지)
JOINT_TILT_VEL_DEG_S = 3.0       # 각속도 기본값 (기존 10.0 -> 3.0으로 하향)

# --- PD 게인 (mm 오차 -> deg 틸트) ------------------------------------------
KP_DEG_PER_MM = 0.030
KD_DEG_PER_MM_S = 0.005      # 기존 0.010 -> 0.005로 하향 (노이즈 미분 증폭 완화)
VELOCITY_EMA_ALPHA = 0.15    # 기존 0.3 -> 0.15로 강화 (미분 스무딩을 더 세게)
MAX_RAW_VELOCITY_MM_S = 150.0  # 이보다 큰 순간 미분값은 센서 노이즈로 간주하고 클램프

# 실제로 어느 방향으로 돌려야 공이 중앙으로 오는지는 하드웨어에서 검증 후
# 아래 부호를 뒤집어 맞출 것.
SIGN_J5 = 1.0
SIGN_J6 = 1.0

GUI_POLL_MS = 100

# GUI 슬라이더로 노출할 파라미터:
# (속성명, 표시 라벨, 최소, 최대, 기본값, 표시 포맷, 설명)
# 최소/최대는 "이 정도까지는 실기에서 시도해볼 만한" 보수적인 범위로 잡았다.
# 범위 자체를 늘리고 싶으면 여기 min/max를 바꿀 것 (슬라이더 하한/상한이 곧 안전
# 한계이므로, 늘릴 때는 특히 신중하게).
TUNABLE_PARAM_SPECS = [
    (
        "max_tilt_deg",
        "최대 틸트 한계",
        0.0,
        5.0,
        MAX_TILT_DEG,
        "{:.2f} deg",
        "기준 자세 대비 J5/J6이 벗어날 수 있는 최대 각도. 이보다 크게는 "
        "절대 안 기울어짐 (제일 바깥쪽 안전판).",
    ),
    (
        "max_tilt_step_deg",
        "틱당 변화량 한계",
        0.0,
        0.5,
        MAX_TILT_STEP_DEG,
        "{:.3f} deg",
        "제어 주기(1/15초)마다 목표각이 바뀔 수 있는 최대 폭. 클수록 반응은 "
        "빠르지만 그만큼 급격하게(빠르게) 움직임.",
    ),
    (
        "deadband_mm",
        "데드밴드",
        0.0,
        10.0,
        DEADBAND_MM,
        "{:.1f} mm",
        "추정 오차가 이 값보다 작으면 0으로 간주하고 안 움직임. 작을수록 "
        "민감하게 반응하지만 노이즈에도 그만큼 더 떪.",
    ),
    (
        "kp_deg_per_mm",
        "Kp (비례 게인)",
        0.0,
        0.10,
        KP_DEG_PER_MM,
        "{:.4f}",
        "위치 오차 1mm당 얼마나 기울일지. 클수록 빨리 반응하지만 너무 크면 "
        "중앙을 지나쳐 진동(오버슈트)할 수 있음.",
    ),
    (
        "kd_deg_per_mm_s",
        "Kd (미분 게인)",
        0.0,
        0.02,
        KD_DEG_PER_MM_S,
        "{:.4f}",
        "물체 이동 속도에 반응해 진동을 억제하는 제동 역할. 너무 크면 "
        "위치 신호의 노이즈까지 증폭해서 오히려 더 떨림.",
    ),
]

# 로봇 속도 상한 슬라이더 — 위 5개와 성격이 달라(직접 물리적 속도) GUI에서 별도로,
# 더 눈에 띄게 그린다. 상한(10.0)은 예전 기본값을 넘지 않도록 고정했다.
SPEED_PARAM_SPEC = (
    "joint_tilt_vel_deg_s",
    "J5/J6 회전 속도 상한",
    0.5,
    10.0,
    JOINT_TILT_VEL_DEG_S,
    "{:.2f} deg/s",
    "J5, J6이 실제로 회전하는 물리적 속도 자체. 다른 게인과 달리 이 숫자가 "
    "곧 로봇 속도라 가장 직접적으로 위험함 — 낮을수록 안전.",
)

TUNABLE_PARAM_NAMES = {spec[0] for spec in TUNABLE_PARAM_SPECS} | {
    SPEED_PARAM_SPEC[0]
}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =============================================================================
# 보조 함수
# =============================================================================

def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def to_float_list(value: Any) -> List[float]:
    if value is None:
        raise RuntimeError("로봇 API가 None을 반환했습니다.")

    if isinstance(value, (list, tuple)) and len(value) == 2:
        first = value[0]
        if hasattr(first, "__iter__") and not isinstance(first, (str, bytes)):
            first_values = list(first)
            if len(first_values) >= 6:
                value = first_values

    if isinstance(value, np.ndarray):
        return [float(item) for item in value.reshape(-1).tolist()]

    if hasattr(value, "__iter__") and not isinstance(value, (str, bytes)):
        return [float(item) for item in list(value)]

    return [float(value)]


def to_fixed_array(value: Any, length: int) -> np.ndarray:
    values = to_float_list(value)

    if len(values) < length:
        raise RuntimeError(
            f"{length}개 값이 필요하지만 {len(values)}개가 반환됐습니다: {values}"
        )

    return np.asarray(values[:length], dtype=float)


# =============================================================================
# 컨트롤러 백엔드
# =============================================================================

class PlateBalanceBackend:
    def __init__(
        self,
        node,
        dsr_module,
        command_queue: queue.Queue,
        ui_queue: queue.Queue,
    ) -> None:
        self.node = node
        self.dsr = dsr_module
        self.command_queue = command_queue
        self.ui_queue = ui_queue

        self.status_subscriber = node.create_subscription(
            String,
            STATUS_TOPIC,
            self.on_status,
            10,
        )
        self.enable_subscriber = node.create_subscription(
            String,
            ENABLE_TOPIC,
            self.on_enable_command,
            10,
        )

        self.running = True
        self.enabled = False

        # GUI 버튼으로 실시간 전환 가능. DRY_RUN(모듈 상수)은 초기값일 뿐이고
        # 이후엔 이 속성이 유일한 출처다. 제어가 켜져 있는 동안엔 전환을 막는다
        # (handle_command의 "set_dry_run"에서 체크).
        self.dry_run = DRY_RUN

        self.last_status: Optional[Dict[str, Any]] = None
        self.last_status_monotonic: Optional[float] = None

        # CONTROL_HZ가 STATUS_TOPIC 실제 갱신 속도보다 높을 수 있어서(예: 제어
        # 100Hz vs 관찰 노드 실측 30~100Hz), 같은 STATUS를 여러 틱에 걸쳐
        # 재사용하게 된다. status_sequence로 "진짜 새 데이터가 왔는지"를 구분해서,
        # 새 데이터가 없는 틱엔 미분(속도) 추정을 다시 계산하지 않는다 —
        # 안 그러면 dt를 잘못 나눠서 속도가 튀거나 0으로 오염된다.
        self.status_sequence = 0
        self.last_processed_status_sequence = -1

        self.reference_posj: Optional[List[float]] = None
        self.current_tilt_j5 = 0.0
        self.current_tilt_j6 = 0.0
        self.target_j5 = 0.0
        self.target_j6 = 0.0

        # 안전 상태 전환 로그/표시용 (매 틱마다 찍지 않고 상태가 바뀔 때만 기록)
        self.was_gating_ok: Optional[bool] = None
        self.was_saturated = False

        self.prev_y_mm: Optional[float] = None
        self.prev_z_mm: Optional[float] = None
        self.prev_time: Optional[float] = None
        self.velocity_y = 0.0
        self.velocity_z = 0.0

        # GUI 슬라이더로 실시간 조정 가능한 파라미터. 모듈 상수는 초기값일 뿐이고,
        # 이후로는 이 인스턴스 속성이 유일한 출처(source of truth)다.
        self.max_tilt_deg = MAX_TILT_DEG
        self.max_tilt_step_deg = MAX_TILT_STEP_DEG
        self.deadband_mm = DEADBAND_MM
        self.kp_deg_per_mm = KP_DEG_PER_MM
        self.kd_deg_per_mm_s = KD_DEG_PER_MM_S

        # J5/J6 회전 속도 상한 (별도 슬라이더). 가속도는 속도에 비례해서 자동으로
        # 따라간다 (기존 20:10, 40:20 비율 유지) -> 슬라이더 하나만 만지면 됨.
        self.joint_tilt_vel_deg_s = JOINT_TILT_VEL_DEG_S
        self.joint_tilt_acc_deg_s2 = (
            JOINT_TILT_VEL_DEG_S * JOINT_ACC_TO_VEL_RATIO
        )

    # -------------------------------------------------------------------------
    # UI 전달
    # -------------------------------------------------------------------------

    def send_ui(self, event: str, **payload: Any) -> None:
        self.ui_queue.put({"event": event, **payload})

    def send_log(self, message: str) -> None:
        self.send_ui("log", message=message)

    # -------------------------------------------------------------------------
    # ROS 콜백 (spin_once와 같은 스레드에서 실행됨 -> 락 불필요)
    # -------------------------------------------------------------------------

    def on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        self.last_status = payload
        self.last_status_monotonic = time.monotonic()
        self.status_sequence += 1

    def on_enable_command(self, message: String) -> None:
        command = message.data.strip().lower()

        if command == "start":
            self.command_queue.put(("start", None))
        elif command in ("stop", "disable"):
            self.command_queue.put(("stop", None))

    # -------------------------------------------------------------------------
    # 활성/비활성
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        if self.enabled:
            return

        current_posj = to_fixed_array(self.dsr.get_current_posj(), 6)

        self.reference_posj = current_posj.tolist()
        self.current_tilt_j5 = 0.0
        self.current_tilt_j6 = 0.0
        self.target_j5 = 0.0
        self.target_j6 = 0.0
        self.prev_y_mm = None
        self.prev_z_mm = None
        self.prev_time = None
        self.velocity_y = 0.0
        self.velocity_z = 0.0
        self.was_gating_ok = None
        self.was_saturated = False
        self.last_processed_status_sequence = -1
        self.enabled = True

        mode_text = "DRY_RUN (모션 없음)" if self.dry_run else "⚠ 실제 모션 전송"
        self.send_log(
            f"제어 시작 [{mode_text}] · 기준 조인트각="
            + ", ".join(f"{v:.2f}" for v in self.reference_posj)
        )
        self.send_ui("enabled_state", enabled=True)

    def disable(self) -> None:
        was_enabled = self.enabled
        self.enabled = False

        if was_enabled:
            self.send_log("제어 정지.")
            self.send_ui("enabled_state", enabled=False)

    # -------------------------------------------------------------------------
    # 명령 처리
    # -------------------------------------------------------------------------

    def handle_command(self, command) -> None:
        name, payload = command

        if name == "start":
            try:
                self.enable()
            except Exception as error:
                self.send_log(f"[오류] 제어 시작 실패: {error}")
                self.send_ui(
                    "error",
                    message=str(error),
                    traceback=traceback.format_exc(),
                )

        elif name == "stop":
            self.disable()

        elif name == "set_param":
            param_name = payload["name"]
            if param_name not in TUNABLE_PARAM_NAMES:
                return
            value = float(payload["value"])
            setattr(self, param_name, value)

            if param_name == "joint_tilt_vel_deg_s":
                self.joint_tilt_acc_deg_s2 = value * JOINT_ACC_TO_VEL_RATIO
                self.send_log(
                    f"J5/J6 회전 속도 상한 = {value:.2f} deg/s "
                    f"(가속도 {self.joint_tilt_acc_deg_s2:.2f} deg/s^2)"
                )
            else:
                self.send_log(f"{param_name} = {value:.4f} 로 변경")

        elif name == "set_dry_run":
            if self.enabled:
                self.send_log(
                    "[거부] 제어가 켜져 있는 동안엔 DRY_RUN을 전환할 수 "
                    "없습니다. 먼저 정지하십시오."
                )
                self.send_ui("dry_run_state", dry_run=self.dry_run)
                return

            self.dry_run = bool(payload)
            self.send_log(
                "DRY_RUN 켜짐 (모션 없음)"
                if self.dry_run
                else "⚠ DRY_RUN 꺼짐 — 이제부터 실제 모션 명령을 보냅니다"
            )
            self.send_ui("dry_run_state", dry_run=self.dry_run)

        elif name == "shutdown":
            self.disable()
            self.running = False

    # -------------------------------------------------------------------------
    # 제어 루프
    # -------------------------------------------------------------------------

    def step_toward(self, target_j5: float, target_j6: float) -> None:
        next_j5 = self.current_tilt_j5 + clamp(
            target_j5 - self.current_tilt_j5,
            self.max_tilt_step_deg,
        )
        next_j6 = self.current_tilt_j6 + clamp(
            target_j6 - self.current_tilt_j6,
            self.max_tilt_step_deg,
        )
        next_j5 = clamp(next_j5, self.max_tilt_deg)
        next_j6 = clamp(next_j6, self.max_tilt_deg)

        if self.reference_posj is None:
            return

        target_posj = list(self.reference_posj)
        target_posj[TILT_JOINT_INDEX_J5] = (
            self.reference_posj[TILT_JOINT_INDEX_J5] + next_j5
        )
        target_posj[TILT_JOINT_INDEX_J6] = (
            self.reference_posj[TILT_JOINT_INDEX_J6] + next_j6
        )

        if self.dry_run:
            self.send_log(
                f"[DRY_RUN] J5={next_j5:+.3f} J6={next_j6:+.3f} deg -> "
                "posj=["
                + ", ".join(f"{v:.2f}" for v in target_posj)
                + "]"
            )
        else:
            try:
                self.dsr.servoj(
                    target_posj,
                    vel=self.joint_tilt_vel_deg_s,
                    acc=self.joint_tilt_acc_deg_s2,
                    mode=self.dsr.DR_SERVO_OVERRIDE,
                )
            except Exception as error:
                self.send_log(f"[오류] servoj 실패: {error} -> 제어 정지")
                self.send_ui(
                    "error",
                    message=str(error),
                    traceback=traceback.format_exc(),
                )
                self.disable()
                return

        self.current_tilt_j5 = next_j5
        self.current_tilt_j6 = next_j6

    def control_tick(self, now: float) -> None:
        if not self.enabled:
            return

        if (
            self.last_status_monotonic is None
            or now - self.last_status_monotonic > STATUS_TIMEOUT_SEC
        ):
            self.send_log("[안전정지] STATUS 신호 끊김 -> 제어 정지")
            self.disable()
            return

        status = self.last_status
        if status is None:
            return

        object_present = bool(status.get("object_present"))
        pose_stable = bool(status.get("pose_stable"))
        model_ready = bool(status.get("position_model_ready"))
        gating_ok = object_present and pose_stable and model_ready

        if self.was_gating_ok is not None and gating_ok != self.was_gating_ok:
            if gating_ok:
                self.send_log("게이팅 조건 충족 -> 추종 재개")
            else:
                self.send_log(
                    "[주의] 게이팅 조건 미충족(물체/자세/모델 중 하나) "
                    "-> 중립 복귀 모드로 전환"
                )
        self.was_gating_ok = gating_ok

        if not gating_ok:
            self.step_toward(0.0, 0.0)
            self.send_ui(
                "tick_status",
                object_present=object_present,
                pose_stable=pose_stable,
                model_ready=model_ready,
                estimated_y_mm=status.get("estimated_y_mm"),
                estimated_z_mm=status.get("estimated_z_mm"),
                tilt_j5=self.current_tilt_j5,
                tilt_j6=self.current_tilt_j6,
                gating_ok=False,
                saturated=False,
                note="게이팅 조건 미충족 -> 중립 복귀 중",
            )
            return

        estimated_y = status.get("estimated_y_mm")
        estimated_z = status.get("estimated_z_mm")

        # CONTROL_HZ가 STATUS_TOPIC 실제 갱신 속도보다 높으면 같은 status를
        # 여러 틱에서 재사용하게 된다. 새 데이터가 아닐 땐 미분(속도)/PD를
        # 다시 계산하지 않고, 마지막으로 계산해둔 target_j5/j6만 그대로
        # 유지한다 — step_toward는 매 틱 계속 불러서 그 목표로 부드럽게
        # 접근은 계속한다 (CONTROL_HZ를 올리는 원래 목적).
        is_new_status = self.status_sequence != self.last_processed_status_sequence
        self.last_processed_status_sequence = self.status_sequence

        if (
            is_new_status
            and isinstance(estimated_y, (int, float))
            and isinstance(estimated_z, (int, float))
            and math.isfinite(estimated_y)
            and math.isfinite(estimated_z)
        ):
            if self.prev_time is not None and self.prev_y_mm is not None:
                dt = now - self.prev_time
                if 1.0e-3 < dt < 1.0:
                    # 순간 미분값이 비정상적으로 크면(=estimated_y/z_mm의 센서
                    # 노이즈 스파이크일 가능성이 높음) 실제 속도로 보지 않고
                    # 클램프한다. 그래야 노이즈 한 번 튄 걸로 Kd 항이 크게
                    # 흔들리지 않는다.
                    raw_vy = clamp(
                        (estimated_y - self.prev_y_mm) / dt,
                        MAX_RAW_VELOCITY_MM_S,
                    )
                    raw_vz = clamp(
                        (estimated_z - self.prev_z_mm) / dt,
                        MAX_RAW_VELOCITY_MM_S,
                    )
                    self.velocity_y = (
                        VELOCITY_EMA_ALPHA * raw_vy
                        + (1.0 - VELOCITY_EMA_ALPHA) * self.velocity_y
                    )
                    self.velocity_z = (
                        VELOCITY_EMA_ALPHA * raw_vz
                        + (1.0 - VELOCITY_EMA_ALPHA) * self.velocity_z
                    )

            self.prev_y_mm = estimated_y
            self.prev_z_mm = estimated_z
            self.prev_time = now

            error_y = estimated_y if abs(estimated_y) > self.deadband_mm else 0.0
            error_z = estimated_z if abs(estimated_z) > self.deadband_mm else 0.0

            target_j5 = SIGN_J5 * (
                self.kp_deg_per_mm * error_y
                + self.kd_deg_per_mm_s * self.velocity_y
            )
            target_j6 = SIGN_J6 * (
                self.kp_deg_per_mm * error_z
                + self.kd_deg_per_mm_s * self.velocity_z
            )

            self.target_j5 = clamp(target_j5, self.max_tilt_deg)
            self.target_j6 = clamp(target_j6, self.max_tilt_deg)

        self.step_toward(self.target_j5, self.target_j6)

        saturation_margin_deg = 0.01
        is_saturated = (
            abs(self.current_tilt_j5) >= self.max_tilt_deg - saturation_margin_deg
            or abs(self.current_tilt_j6) >= self.max_tilt_deg - saturation_margin_deg
        )
        if is_saturated != self.was_saturated:
            if is_saturated:
                self.send_log(
                    "[주의] J5/J6 틸트가 최대 한계(max_tilt_deg)에 도달 "
                    "-> 목표를 다 못 따라가는 중"
                )
            else:
                self.send_log("틸트 한계 포화 해제 -> 정상 추종")
        self.was_saturated = is_saturated

        self.send_ui(
            "tick_status",
            object_present=object_present,
            pose_stable=pose_stable,
            model_ready=model_ready,
            estimated_y_mm=estimated_y,
            estimated_z_mm=estimated_z,
            tilt_j5=self.current_tilt_j5,
            tilt_j6=self.current_tilt_j6,
            gating_ok=True,
            saturated=is_saturated,
            note="",
        )

    # -------------------------------------------------------------------------
    # 메인 루프
    # -------------------------------------------------------------------------

    # 단일 스레드 구조:
    # 예전엔 이 백엔드를 별도 스레드에서 while 루프로 돌렸는데, 그러면 rclpy를
    # 스핀하는 스레드와 Tkinter(Tcl) 메인 스레드가 공존해서 "Tcl_Release
    # couldn't find reference" 크래시(Tcl은 싱글스레드 전용)가 간헐적으로 났다.
    # 이제 스레드를 없애고, 아래 tick()을 Tk 이벤트 루프의 after() 콜백으로
    # 메인 스레드에서 주기 호출한다(main() 참고). 제어 루프의 유일한 로봇
    # 명령인 servoj는 토픽 publish라 비블로킹이므로 Tk 루프를 막지 않는다.
    def start(self) -> None:
        mode_text = "DRY_RUN (모션 없음)" if self.dry_run else "⚠ 실제 모션 전송 모드"
        self.send_log(f"컨트롤러 백엔드 시작. [{mode_text}]")

    def tick(self) -> None:
        loop_start = time.monotonic()

        rclpy.spin_once(self.node, timeout_sec=0.0)

        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                break
            self.handle_command(command)

        try:
            self.control_tick(loop_start)
        except Exception as error:
            self.send_log(f"[오류] 제어 루프 예외: {error} -> 제어 정지")
            self.send_ui(
                "error",
                message=str(error),
                traceback=traceback.format_exc(),
            )
            self.disable()

    def stop(self) -> None:
        self.disable()
        self.send_ui("backend_stopped")


# =============================================================================
# GUI 색상 팔레트 (plate_monitor_v2.py와 동일한 다크 톤으로 통일)
# =============================================================================

THEME_BG = "#0f172a"
THEME_PANEL = "#111827"
THEME_CARD = "#1e293b"
THEME_CARD_ALT = "#243043"
THEME_BORDER = "#334155"
THEME_TEXT = "#e2e8f0"
THEME_TEXT_MUTED = "#94a3b8"
THEME_ACCENT = "#38bdf8"
THEME_ACCENT_ACTIVE = "#0ea5e9"
THEME_SUCCESS = "#10b981"
THEME_WARNING = "#f59e0b"
THEME_DANGER = "#ef4444"
THEME_DANGER_DARK = "#7f1d1d"
THEME_DANGER_DARK_ACTIVE = "#991b1b"


# =============================================================================
# GUI
# =============================================================================

class PlateBalanceGUI:
    def __init__(
        self,
        root: tk.Tk,
        command_queue: queue.Queue,
        ui_queue: queue.Queue,
    ) -> None:
        self.root = root
        self.command_queue = command_queue
        self.ui_queue = ui_queue
        self.enabled = False

        self.latest_tilt_j5 = 0.0
        self.latest_tilt_j6 = 0.0

        self.last_gating_ok: Optional[bool] = None
        self.last_saturated = False

        # 백엔드의 self.dry_run과 동일한 값을 미러링. 실제 출처는 백엔드고,
        # 여기선 "dry_run_state" 이벤트로 받은 값만 반영한다.
        self.dry_run = DRY_RUN

        # on_close() 이후 root.destroy() 예약 시간(200ms) 안에 poll_ui_queue가
        # 한 번 더 스스로를 재예약하면, destroy 이후 시점에 실행돼 이미 파괴된
        # Tk 위젯을 건드리다 "Tcl_Release couldn't find reference" 같은 크래시가
        # 날 수 있다. 이 플래그로 종료 시작과 동시에 재예약을 끊는다.
        self.closing = False

        # closing 플래그는 콜백의 파이썬 로직만 막을 뿐, 이미 Tcl 이벤트 큐에
        # 등록된 after() 타이머 자체는 취소하지 않는다. destroy() 시점에 그
        # 타이머가 아직 남아있으면, 해제된 Tcl 커맨드 참조를 찾으려다
        # "Tcl_Release couldn't find reference" 크래시로 이어질 수 있으므로
        # job id를 기억해뒀다가 destroy 직전에 명시적으로 after_cancel한다.
        self.poll_after_id: Optional[str] = None
        self.drive_after_id: Optional[str] = None

        self.root.title("판 밸런스 컨트롤러")
        self.root.geometry("980x640")
        self.root.minsize(820, 520)

        self.build_styles()
        self.build_layout()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.poll_after_id = self.root.after(GUI_POLL_MS, self.poll_ui_queue)

    def build_styles(self) -> None:
        self.root.configure(background=THEME_BG)

        style = ttk.Style()

        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(
            ".",
            background=THEME_BG,
            foreground=THEME_TEXT,
            fieldbackground=THEME_CARD,
            bordercolor=THEME_BORDER,
            darkcolor=THEME_CARD,
            lightcolor=THEME_CARD,
            troughcolor=THEME_PANEL,
        )
        style.configure("TFrame", background=THEME_BG)
        style.configure(
            "TLabel",
            background=THEME_BG,
            foreground=THEME_TEXT,
        )
        style.configure(
            "TLabelframe",
            background=THEME_BG,
            bordercolor=THEME_BORDER,
        )
        style.configure(
            "TLabelframe.Label",
            background=THEME_BG,
            foreground=THEME_TEXT_MUTED,
            font=("Sans", 10, "bold"),
        )
        style.configure(
            "TScrollbar",
            background=THEME_CARD,
            troughcolor=THEME_PANEL,
            bordercolor=THEME_BORDER,
            arrowcolor=THEME_TEXT_MUTED,
        )
        style.configure(
            "TButton",
            background=THEME_CARD,
            foreground=THEME_TEXT,
            bordercolor=THEME_BORDER,
            focuscolor=THEME_ACCENT,
            padding=(6, 5),
        )
        style.map(
            "TButton",
            background=[
                ("disabled", THEME_PANEL),
                ("pressed", THEME_ACCENT_ACTIVE),
                ("active", THEME_CARD_ALT),
            ],
            foreground=[
                ("disabled", THEME_TEXT_MUTED),
            ],
        )
        style.configure(
            "Horizontal.TScale",
            background=THEME_BG,
            troughcolor=THEME_PANEL,
        )
        style.configure(
            "Danger.Horizontal.TScale",
            troughcolor=THEME_DANGER_DARK,
            background=THEME_CARD,
        )

    def build_layout(self) -> None:
        self.root.columnconfigure(0, weight=3)
        self.root.columnconfigure(1, weight=2)
        self.root.rowconfigure(3, weight=1)

        self.banner_label = tk.Label(
            self.root,
            foreground="#ffffff",
            font=("Sans", 12, "bold"),
            pady=8,
        )
        self.banner_label.grid(row=0, column=0, columnspan=2, sticky="ew")
        # 위젯 생성 직후 같은 콜스택에서 바로 .configure()하면 Tcl 8.6.12의
        # 변수-트레이스 재진입 버그로 "Tcl_Release couldn't find reference"
        # 크래시가 날 수 있다(실측 확인됨). after_idle로 한 틱 늦춰서 위젯
        # 생성과 분리한다.
        self.root.after_idle(self.update_banner)

        button_frame = ttk.Frame(self.root, padding=10)
        button_frame.grid(row=1, column=0, columnspan=2, sticky="ew")
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)
        button_frame.columnconfigure(2, weight=1)

        self.start_button = ttk.Button(
            button_frame,
            text="제어 시작",
            command=self.request_start,
        )
        self.start_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.stop_button = ttk.Button(
            button_frame,
            text="정지",
            command=self.request_stop,
        )
        self.stop_button.grid(row=0, column=1, sticky="ew", padx=(4, 4))

        self.dry_run_button_var = tk.StringVar()
        self.dry_run_button = ttk.Button(
            button_frame,
            textvariable=self.dry_run_button_var,
            command=self.request_toggle_dry_run,
        )
        self.dry_run_button.grid(row=0, column=2, sticky="ew", padx=(4, 0))
        self.root.after_idle(self.update_dry_run_button)

        # 왼쪽 = 조정 가능한 것들 (게인/한계/속도), 오른쪽 = 현재 상태 (읽기 전용 + 기울기 시각화)
        left_column = ttk.Frame(self.root)
        left_column.grid(row=2, column=0, sticky="nsew", padx=(10, 5))
        left_column.columnconfigure(0, weight=1)

        right_column = ttk.Frame(self.root)
        right_column.grid(row=2, column=1, sticky="nsew", padx=(5, 10))
        right_column.columnconfigure(0, weight=1)
        right_column.rowconfigure(1, weight=1)

        gain_frame = ttk.LabelFrame(
            left_column, text="실시간 게인 / 한계 조정", padding=10
        )
        gain_frame.grid(row=0, column=0, sticky="ew")
        gain_frame.columnconfigure(1, weight=1)

        self.param_vars: Dict[str, tk.DoubleVar] = {}
        self.param_value_vars: Dict[str, tk.StringVar] = {}

        for index, (
            name,
            label,
            lo,
            hi,
            default,
            fmt,
            description,
        ) in enumerate(TUNABLE_PARAM_SPECS):
            value_row = index * 2
            desc_row = value_row + 1

            ttk.Label(gain_frame, text=label).grid(
                row=value_row, column=0, sticky="w", padx=(0, 8), pady=(3, 0)
            )

            value_var = tk.StringVar(value=fmt.format(default))
            self.param_value_vars[name] = value_var

            scale_var = tk.DoubleVar(value=default)
            self.param_vars[name] = scale_var

            scale = ttk.Scale(
                gain_frame,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                variable=scale_var,
                command=self.make_param_callback(name, fmt, value_var),
            )
            scale.grid(row=value_row, column=1, sticky="ew", pady=(3, 0))

            ttk.Label(
                gain_frame, textvariable=value_var, width=10, anchor="e"
            ).grid(row=value_row, column=2, sticky="e", padx=(8, 0), pady=(3, 0))

            ttk.Label(
                gain_frame,
                text=description,
                foreground=THEME_TEXT_MUTED,
                font=("Sans", 8),
                wraplength=360,
                justify="left",
            ).grid(
                row=desc_row,
                column=0,
                columnspan=3,
                sticky="w",
                padx=(0, 8),
                pady=(0, 3),
            )

        ttk.Button(
            gain_frame,
            text="기본값 복원",
            command=self.reset_params,
        ).grid(
            row=len(TUNABLE_PARAM_SPECS) * 2,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(6, 0),
        )

        self.build_speed_control(parent=left_column, row=1)

        status_frame = ttk.LabelFrame(right_column, text="상태", padding=10)
        status_frame.grid(row=0, column=0, sticky="ew")
        status_frame.columnconfigure(1, weight=1)

        self.safety_var = tk.StringVar(value="⚪ 대기 중")
        self.safety_label = tk.Label(
            status_frame,
            textvariable=self.safety_var,
            font=("Sans", 11, "bold"),
            anchor="w",
            background=THEME_BG,
        )
        self.safety_label.grid(
            row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8)
        )
        self.root.after_idle(self.update_safety_indicator)

        self.status_vars: Dict[str, tk.StringVar] = {}
        rows = [
            ("제어 상태", "enabled"),
            ("STATUS 수신", "connected"),
            ("물체 감지", "object_present"),
            ("자세 유지", "pose_stable"),
            ("위치모델 준비", "model_ready"),
            ("추정 오차 Y / Z", "error"),
            ("현재 틸트 J5 / J6", "tilt"),
            ("비고", "note"),
        ]
        for row, (title, key) in enumerate(rows, start=1):
            ttk.Label(status_frame, text=title).grid(
                row=row, column=0, sticky="w", pady=2, padx=(0, 8)
            )
            var = tk.StringVar(value="--")
            self.status_vars[key] = var
            ttk.Label(status_frame, textvariable=var).grid(
                row=row, column=1, sticky="w", pady=2
            )

        self.build_tilt_gauge(parent=right_column, row=1)

        log_frame = ttk.LabelFrame(self.root, text="로그", padding=6)
        log_frame.grid(
            row=3, column=0, columnspan=2, sticky="nsew", padx=10, pady=(8, 10)
        )
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.log_text = tk.Text(
            log_frame,
            height=6,
            wrap="word",
            state="disabled",
            font=("Monospace", 9),
            background=THEME_PANEL,
            foreground=THEME_TEXT,
            insertbackground=THEME_TEXT,
            highlightthickness=1,
            highlightbackground=THEME_BORDER,
            relief="flat",
            bd=0,
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(
            log_frame, orient=tk.VERTICAL, command=self.log_text.yview
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def build_tilt_gauge(self, parent, row: int) -> None:
        gauge_frame = ttk.LabelFrame(
            parent, text="현재 판 기울기 (J5 / J6) — 3D", padding=8
        )
        gauge_frame.grid(row=row, column=0, sticky="nsew", pady=(8, 0))
        parent.rowconfigure(row, weight=1)
        gauge_frame.columnconfigure(0, weight=1)
        gauge_frame.rowconfigure(0, weight=1)

        self.tilt_canvas = tk.Canvas(
            gauge_frame,
            height=260,
            background=THEME_PANEL,
            highlightthickness=0,
        )
        self.tilt_canvas.grid(row=0, column=0, sticky="nsew")
        self.tilt_canvas.bind(
            "<Configure>", lambda _event: self.draw_tilt_gauge()
        )

    def draw_tilt_gauge(self) -> None:
        """J5/J6 값으로 판을 3D처럼 기울여 그린다 (원판 rim을 회전시켜
        투영하는 방식). 실제 각도(최대 수 도)는 눈으로 보기엔 너무 작아서,
        방향을 알아보기 쉽도록 TILT_VISUAL_EXAGGERATION 배로 과장해서
        그린다 — 숫자 표시는 항상 실제값 그대로."""
        canvas = self.tilt_canvas
        canvas.delete("all")

        width = max(canvas.winfo_width(), 200)
        height = max(canvas.winfo_height(), 200)
        center_x = width / 2.0
        center_y = height / 2.0
        radius_px = min(width, height) * 0.34

        exaggeration = 4.0
        angle_x = math.radians(self.latest_tilt_j6 * exaggeration)
        angle_y = math.radians(self.latest_tilt_j5 * exaggeration)
        camera_elev = math.radians(50.0)

        def project(x: float, y: float, z: float):
            y1 = y * math.cos(angle_x) - z * math.sin(angle_x)
            z1 = y * math.sin(angle_x) + z * math.cos(angle_x)
            x1 = x

            x2 = x1 * math.cos(angle_y) + z1 * math.sin(angle_y)
            z2 = -x1 * math.sin(angle_y) + z1 * math.cos(angle_y)
            y2 = y1

            screen_x = x2
            screen_y = y2 * math.cos(camera_elev) - z2 * math.sin(camera_elev)
            return screen_x, screen_y

        rim_points = []
        segments = 40
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            local_x = radius_px * math.cos(theta)
            local_y = radius_px * math.sin(theta)
            sx, sy = project(local_x, local_y, 0.0)
            rim_points.append(center_x + sx)
            rim_points.append(center_y - sy)

        canvas.create_polygon(
            rim_points,
            fill=THEME_CARD,
            outline="#4b5563",
            width=2,
            smooth=True,
        )

        for local_x, local_y in ((radius_px, 0.0), (-radius_px, 0.0)):
            sx, sy = project(local_x, local_y, 0.0)
            canvas.create_line(
                center_x, center_y, center_x + sx, center_y - sy,
                fill="#374151",
            )
        for local_x, local_y in ((0.0, radius_px), (0.0, -radius_px)):
            sx, sy = project(local_x, local_y, 0.0)
            canvas.create_line(
                center_x, center_y, center_x + sx, center_y - sy,
                fill="#374151",
            )

        canvas.create_text(
            center_x, center_y - radius_px - 16,
            text="J6 +", fill="#9ca3af", anchor="s",
        )
        canvas.create_text(
            center_x + radius_px + 16, center_y,
            text="J5 +", fill="#9ca3af", anchor="w",
        )

        normal_length = radius_px * 0.7
        nx, ny = project(0.0, 0.0, normal_length)
        tip_x = center_x + nx
        tip_y = center_y - ny

        dot_color = "#10b981" if self.enabled else "#6b7280"

        canvas.create_line(
            center_x, center_y, tip_x, tip_y,
            fill="#fbbf24", width=3, arrow=tk.LAST,
        )
        canvas.create_oval(
            center_x - 6, center_y - 6, center_x + 6, center_y + 6,
            fill=dot_color, outline="#ffffff", width=2,
        )

        canvas.create_text(
            center_x,
            height - 14,
            text=(
                f"J5 {self.latest_tilt_j5:+.3f}°  /  "
                f"J6 {self.latest_tilt_j6:+.3f}°  "
                f"(화면 표시는 {exaggeration:.0f}배 과장)"
            ),
            fill="#d1d5db",
            font=("Monospace", 10),
        )

    def build_speed_control(self, parent, row: int) -> None:
        """다른 5개 게인 슬라이더와 다르게, 이건 로봇의 실제 이동 속도 자체라
        더 위험하다. 그래서 색을 다르게 하고 경고 문구를 붙여 시각적으로
        구분한다."""
        name, label, lo, hi, default, fmt, description = SPEED_PARAM_SPEC

        outer = tk.Frame(
            parent,
            background=THEME_DANGER_DARK,
            highlightbackground=THEME_WARNING,
            highlightthickness=2,
        )
        outer.grid(row=row, column=0, sticky="ew", pady=(10, 0))
        outer.columnconfigure(0, weight=1)

        header = tk.Label(
            outer,
            text="⚠ 로봇 관절 속도 상한 — 직접 물리적 속도입니다",
            background=THEME_DANGER_DARK,
            foreground="#ffffff",
            font=("Sans", 11, "bold"),
            anchor="w",
            padx=10,
            pady=6,
        )
        header.grid(row=0, column=0, sticky="ew")

        caution = tk.Label(
            outer,
            text=(
                "낮을수록 안전합니다. 확신 없으면 절대 올리지 마세요.\n"
                "값을 올리기 전엔 반드시 DRY_RUN으로 먼저 확인하고,\n"
                "실기에서는 항상 비상정지에 손을 올린 채로 조정하세요."
            ),
            background=THEME_DANGER_DARK,
            foreground="#fecaca",
            font=("Sans", 9),
            justify="left",
            anchor="w",
            padx=10,
        )
        caution.grid(row=1, column=0, sticky="ew", pady=(0, 6))

        body = tk.Frame(outer, background=THEME_CARD)
        body.grid(row=2, column=0, sticky="ew", padx=6, pady=(0, 6))
        body.columnconfigure(1, weight=1)

        tk.Label(
            body,
            text=label,
            background=THEME_CARD,
            foreground="#ffffff",
        ).grid(row=0, column=0, sticky="w", padx=(4, 8), pady=(6, 0))

        value_var = tk.StringVar(value=fmt.format(default))
        self.param_value_vars[name] = value_var

        scale_var = tk.DoubleVar(value=default)
        self.param_vars[name] = scale_var

        scale = ttk.Scale(
            body,
            from_=lo,
            to=hi,
            orient=tk.HORIZONTAL,
            variable=scale_var,
            style="Danger.Horizontal.TScale",
            command=self.make_param_callback(name, fmt, value_var),
        )
        scale.grid(row=0, column=1, sticky="ew", pady=(6, 0))

        tk.Label(
            body,
            textvariable=value_var,
            background=THEME_CARD,
            foreground="#fbbf24",
            font=("Monospace", 10, "bold"),
            width=10,
            anchor="e",
        ).grid(row=0, column=2, sticky="e", padx=(8, 4), pady=(6, 0))

        tk.Label(
            body,
            text=description,
            background=THEME_CARD,
            foreground="#fca5a5",
            font=("Sans", 8),
            wraplength=360,
            justify="left",
            anchor="w",
        ).grid(row=1, column=0, columnspan=3, sticky="w", padx=(4, 4), pady=(0, 6))

    # -------------------------------------------------------------------------
    # 명령
    # -------------------------------------------------------------------------

    def request_start(self) -> None:
        self.command_queue.put(("start", None))

    def request_stop(self) -> None:
        self.command_queue.put(("stop", None))

    def request_toggle_dry_run(self) -> None:
        if self.enabled:
            messagebox.showwarning(
                "전환 불가",
                "제어가 켜져 있는 동안엔 DRY_RUN을 전환할 수 없습니다.\n"
                "먼저 '정지'를 누르십시오.",
            )
            return

        if self.dry_run:
            # DRY_RUN -> 실제 모션: 되돌리기 쉬운 방향이 아니므로 확인받는다.
            if not messagebox.askyesno(
                "실제 모션 전송 모드로 전환",
                "DRY_RUN을 끄면 로봇에 실제 모션 명령이 나갑니다.\n\n"
                "- 로봇이 저속/티치 모드입니까?\n"
                "- 비상정지 버튼에 손을 올릴 준비가 됐습니까?\n\n"
                "위 조건이 확인됐으면 '예'를 누르십시오.",
                icon="warning",
            ):
                return

        self.command_queue.put(("set_dry_run", not self.dry_run))

    def update_banner(self) -> None:
        if self.dry_run:
            text = "⚠ DRY_RUN 모드 — 실제 모션 명령을 보내지 않습니다"
            bg = "#065f46"
        else:
            text = "🔴 실제 모션 전송 모드 — 로봇이 실제로 움직입니다"
            bg = THEME_DANGER_DARK

        self.banner_label.configure(text=text, background=bg)

    def update_dry_run_button(self) -> None:
        self.dry_run_button_var.set(
            "⚠ 실제 모션 켜기" if self.dry_run else "🛡 DRY_RUN으로 전환"
        )
        self.dry_run_button.configure(
            state=tk.DISABLED if self.enabled else tk.NORMAL
        )

    def make_param_callback(self, name: str, fmt: str, value_var: tk.StringVar):
        def callback(value_str: str) -> None:
            value = float(value_str)
            value_var.set(fmt.format(value))
            self.command_queue.put(
                ("set_param", {"name": name, "value": value})
            )

        return callback

    def reset_params(self) -> None:
        for name, _label, _lo, _hi, default, fmt, _description in (
            TUNABLE_PARAM_SPECS + [SPEED_PARAM_SPEC]
        ):
            self.param_vars[name].set(default)
            self.param_value_vars[name].set(fmt.format(default))
            self.command_queue.put(
                ("set_param", {"name": name, "value": default})
            )
        self.append_log("게인/한계/속도 상한을 기본값으로 복원했습니다.")

    # -------------------------------------------------------------------------
    # 안전 상태 요약
    # -------------------------------------------------------------------------

    def update_safety_indicator(self) -> None:
        """DRY_RUN, 제어 on/off, 게이팅, 틸트 포화 여부를 종합해서 한 줄로
        보여준다. 로그를 스크롤해서 안전한지 판단할 필요 없이 이 줄만 보면
        된다."""
        if self.dry_run:
            text = "🟢 안전 — DRY_RUN (실제 모션 명령 없음)"
            color = "#10b981"
        elif not self.enabled:
            text = "🟢 안전 — 제어 꺼짐 (실제 모션 없음)"
            color = "#10b981"
        elif self.last_gating_ok is False:
            text = "🟡 주의 — 게이팅 미충족, 중립 복귀 중 (실제 모션 있음)"
            color = "#f59e0b"
        elif self.last_saturated:
            text = "🟡 주의 — 틸트 한계 포화 (실제 모션, 한계에 붙어있음)"
            color = "#f59e0b"
        elif self.last_gating_ok is True:
            text = "🔴 실제 모션 활성 — 정상 추종 중"
            color = "#ef4444"
        else:
            text = "⚪ 대기 중 — 아직 STATUS 신호 없음"
            color = "#9ca3af"

        self.safety_var.set(text)
        # safety_var.set() 직후 같은 위젯에 바로 foreground(색상) configure를
        # 호출하면 Tcl 8.6.12에서 "Tcl_Release couldn't find reference" 크래시가
        # 100% 재현됨(실측 확인). set()과 색상 configure를 서로 다른 이벤트
        # 루프 틱으로 분리해서 이 조합을 피한다.
        self.root.after_idle(
            lambda c=color: self.safety_label.configure(foreground=c)
        )

    # -------------------------------------------------------------------------
    # UI 큐
    # -------------------------------------------------------------------------

    def poll_ui_queue(self) -> None:
        if self.closing:
            return

        # 터미널에서 Ctrl+C를 누르면 rclpy가 자체 SIGINT 핸들러로 종료되지만
        # Tkinter mainloop는 그걸 모르고 계속 돌아 창이 좀비처럼 남는다.
        # rclpy가 외부에서 종료됐으면 여기서 감지해 창도 같이 닫는다.
        if not rclpy.ok():
            self.closing = True
            self.cancel_pending_after()
            try:
                self.root.destroy()
            except tk.TclError:
                pass
            return

        try:
            while True:
                event = self.ui_queue.get_nowait()
                self.handle_ui_event(event)
        except queue.Empty:
            pass

        self.poll_after_id = self.root.after(GUI_POLL_MS, self.poll_ui_queue)

    def cancel_pending_after(self) -> None:
        """destroy() 전에 아직 Tcl 이벤트 큐에 남아있는 주기 타이머를 취소한다.

        closing 플래그만으로는 콜백의 파이썬 로직만 막을 뿐, 이미 예약된
        after() 타이머 자체는 그대로 남아 destroy() 이후 발화하면서
        "Tcl_Release couldn't find reference" 크래시를 유발할 수 있다.
        """
        for attr in ("poll_after_id", "drive_after_id"):
            job_id = getattr(self, attr)
            if job_id is None:
                continue
            try:
                self.root.after_cancel(job_id)
            except (tk.TclError, ValueError):
                pass
            setattr(self, attr, None)

    def handle_ui_event(self, event: Dict[str, Any]) -> None:
        name = event["event"]

        if name == "log":
            self.append_log(event["message"])

        elif name == "enabled_state":
            self.enabled = bool(event["enabled"])
            self.status_vars["enabled"].set(
                "ON" if self.enabled else "OFF"
            )
            if not self.enabled:
                self.last_gating_ok = None
                self.last_saturated = False
            self.update_safety_indicator()
            self.update_dry_run_button()
            self.draw_tilt_gauge()

        elif name == "dry_run_state":
            self.dry_run = bool(event["dry_run"])
            self.update_banner()
            self.update_dry_run_button()
            self.update_safety_indicator()

        elif name == "tick_status":
            self.status_vars["connected"].set("O")
            self.status_vars["object_present"].set(
                "O" if event["object_present"] else "X"
            )
            self.status_vars["pose_stable"].set(
                "O" if event["pose_stable"] else "X"
            )
            self.status_vars["model_ready"].set(
                "O" if event["model_ready"] else "X"
            )

            y = event.get("estimated_y_mm")
            z = event.get("estimated_z_mm")
            if isinstance(y, (int, float)) and isinstance(z, (int, float)):
                self.status_vars["error"].set(f"{y:+.1f} / {z:+.1f} mm")
            else:
                self.status_vars["error"].set("--")

            self.status_vars["tilt"].set(
                f"{event['tilt_j5']:+.3f} / {event['tilt_j6']:+.3f} deg"
            )
            self.status_vars["note"].set(event.get("note") or "-")

            self.latest_tilt_j5 = float(event["tilt_j5"])
            self.latest_tilt_j6 = float(event["tilt_j6"])
            self.draw_tilt_gauge()

            self.last_gating_ok = bool(event.get("gating_ok"))
            self.last_saturated = bool(event.get("saturated"))
            self.update_safety_indicator()

        elif name == "error":
            self.append_log("[오류] " + event["message"])

        elif name == "backend_stopped":
            self.status_vars["enabled"].set("OFF")
            self.status_vars["connected"].set("X")
            self.enabled = False
            self.last_gating_ok = None
            self.last_saturated = False
            self.update_safety_indicator()
            self.update_dry_run_button()

    def append_log(self, message: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state="disabled")

    # -------------------------------------------------------------------------
    # 종료
    # -------------------------------------------------------------------------

    def on_close(self) -> None:
        self.closing = True
        self.command_queue.put(("stop", None))
        self.command_queue.put(("shutdown", None))
        self.cancel_pending_after()
        self.root.after(200, self.root.destroy)


# =============================================================================
# main
# =============================================================================

def main(args=None) -> None:
    # rclpy.init() 이후 DSR_ROBOT2를 import하면 그 안에서 클라이언트/퍼블리셔/
    # 구독을 수십 개 생성하면서 DDS 디스커버리 스레드들이 한꺼번에 바빠진다.
    # 이 타이밍에 맞물려 tk.Tk()를 생성하면(Tcl 인터프리터 초기화가 그 동시
    # 스레드 활동과 겹침) "Tcl_Release couldn't find reference" 크래시로 이어질
    # 수 있다. 그래서 프로세스가 아직 조용한 시점, 즉 rclpy/DSR_ROBOT2보다
    # 먼저 tk.Tk()를 만들어 이 레이스 윈도우를 원천적으로 피한다.
    root = tk.Tk()

    rclpy.init(args=args)

    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    import DSR_ROBOT2 as dsr

    command_queue: queue.Queue = queue.Queue()
    ui_queue: queue.Queue = queue.Queue()

    backend = PlateBalanceBackend(
        node=node,
        dsr_module=dsr,
        command_queue=command_queue,
        ui_queue=ui_queue,
    )

    # 단일 스레드: 백엔드를 별도 스레드로 돌리지 않고, Tk 이벤트 루프의
    # after() 콜백으로 메인 스레드에서 주기 실행한다. 이렇게 하면 rclpy 스핀과
    # Tkinter(Tcl)가 같은(메인) 스레드에서만 돌아 "Tcl_Release couldn't find
    # reference" 크래시(Tcl 싱글스레드 위반으로 인한 레이스)가 원천 제거된다.
    # (root 자체는 위에서 rclpy.init()보다 먼저 이미 생성해 뒀다.)
    gui = PlateBalanceGUI(
        root=root,
        command_queue=command_queue,
        ui_queue=ui_queue,
    )

    backend.start()

    control_period_ms = max(1, int(round(CONTROL_PERIOD_SEC * 1000.0)))

    def drive_backend() -> None:
        # 창이 닫히는 중이거나 rclpy가 종료됐거나 셧다운 명령을 받았으면
        # 더 스케줄하지 않고 백엔드를 정리한다.
        gui.drive_after_id = None
        if gui.closing or not rclpy.ok() or not backend.running:
            try:
                backend.stop()
            except Exception:
                pass
            return

        try:
            backend.tick()
        except Exception:
            traceback.print_exc()

        try:
            gui.drive_after_id = root.after(control_period_ms, drive_backend)
        except tk.TclError:
            # root가 이미 파괴된 뒤면 재예약이 실패할 수 있다 -> 조용히 종료.
            pass

    # mainloop가 시작된 뒤 첫 tick이 돌도록 예약한다.
    gui.drive_after_id = root.after(0, drive_backend)

    try:
        root.mainloop()

    finally:
        backend.running = False
        try:
            backend.disable()
        except Exception:
            pass

        # Ctrl+C로 이미 rclpy가 종료된 뒤엔 destroy_node/shutdown이 예외를
        # 낼 수 있으므로 조용히 넘긴다.
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
