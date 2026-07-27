#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plate_position_gui.py

M0609 + OnRobot RG2 + 반지름 100 mm 원형 판
external joint torque 기반 판 위 물체 중심 이탈 GUI

핵심
====
- 로봇 모션 명령을 전혀 보내지 않는다.
- get_external_torque()를 중심 위치 신호의 주 입력으로 사용한다.
- 빈 판 기준과 물체 중심 기준을 GUI에서 순서대로 측정한다.
- 선택적으로 Tool ±Y / ±Z 4방향 위치 보정을 수행한다.
- 실시간으로 ERR, Z, DEV, TREND, J1~J6 변화, 추정 위치를 표시한다.
- 4방향 보정 전에도 중심 이탈 크기와 증가/감소 추세는 표시한다.
- CSV와 모델 JSON을 자동 저장한다.

GUI 실행 순서
==============
1. [1. 빈 판 측정]
2. 동일 물체를 중심에 놓고 [2. 중심 기준 측정]
3. 바로 [모니터 시작] 또는 4방향 보정 수행
4. 물체를 옮긴 뒤 손을 떼고 GUI 값 확인

주의
====
- external torque는 Tool wrench가 아니다.
- 4방향 위치 보정 전에는 실제 Y/Z 방향과 mm 위치를 출력할 수 없다.
- 손으로 물체를 움직이는 동안에는 손의 힘이 센서에 포함된다.
- 같은 물체, 같은 로봇 자세, 같은 RG2 파지 상태를 유지한다.
"""

# =============================================================================
# 변경 이력 (plate_monitor_v3, 실험용 수정 브랜치)
# =============================================================================
# - 2026-07-22: plate_monitor_v2.py에서 분기해서 생성(v2는 원래 상태로 복원).
#   NODE_NAME을 "plate_position_gui_v3"로 변경해 v1/v2와 동시 실행 시 노드
#   충돌 방지. 아래 "get_tool_force" 관련 항목이 이 브랜치의 핵심 변경점이다.
# - 2026-07-21: plate_monitor.py에서 분기해서 생성. NODE_NAME을
#   "plate_position_gui_v2"로 변경해 원본과 동시 실행 시 노드/토픽 충돌 방지.
# - 2026-07-21: 캘리브레이션(빈 판 측정) 시점의 joint 각도(reference_posj)를
#   함께 저장/불러오도록 추가. 이를 이용해 "저장된 joint로 이동" 버튼을
#   신설했다 — monitor_v2의 "무모션 관찰 도구" 원칙에 대한 유일하고 의도적인
#   예외이며, 반드시 확인창 + 저속(REFERENCE_MOVE_VEL_DEG_S)으로만 동작한다.
#   v1 프리셋(이 날짜 이전 저장분)엔 reference_posj가 없어 이동 버튼이
#   비활성 상태로 남는다 — 하위호환.
# - 2026-07-21: GUI 렉 진단/완화. poll_ui_queue에서 큐 드레인 이벤트 개수와
#   소요 시간을 계측해 임계치 초과 시 로그로 남기고, 폭주하는 "state" 이벤트는
#   poll당 마지막 1개만 반영(코얼레싱)하도록 했다. 또한 handle_ui_event
#   끝에서 이벤트 종류와 무관하게 매번 캔버스를 다시 그리던 부분(이중
#   렌더링의 원인)을 제거하고, 실제로 필요한 지점(state, reference_state)
#   에서만 그리도록 정리했다.
# - 2026-07-21: Z축 위치 정확도 개선. (1) 대각선 캘리 지점 4개(±Y±Z, 반경
#   60mm/45°)를 선택 항목으로 추가하고, fit_position_model이 축 4방향만이
#   아니라 캡처된 모든 방향점을 회귀에 넣도록 바꿔 Y/Z 커플링을 분리한다.
#   그리퍼가 Z=-90mm에서 판을 잡는 외팔보 구조라 -Z쪽 신호가 약해 Z가 특히
#   부정확했던 문제 대응. (2) 위치 추정 출력단에 시간상수 기반 EMA를 추가하고
#   Z를 Y보다 강하게 스무딩(POSITION_OUTPUT_TIME_CONSTANT_Z_SEC) 해 약신호축
#   노이즈를 줄인다.
# - 2026-07-22: 기존 위치 모델은 get_external_torque()(관절 공간, τ=J(q)^T·F)를
#   그대로 회귀에 넣기 때문에, 캘리브레이션 당시 joint 자세(q)에서만 계수가
#   유효하고 자세가 바뀌면 무효화됐다. Doosan 컨트롤러의 get_tool_force(ref)가
#   내부적으로 J(q)^T를 역산해 tool 좌표계 Cartesian wrench(힘+모멘트)를 주는
#   점을 이용해, 같은 방식(6채널 정규화 + 리지 회귀)으로 별도의
#   wrench_position_model을 병행 생성하도록 추가했다 — capture_empty/
#   capture_center/capture_direction이 이제 get_tool_force()도 함께 샘플링하고,
#   fit_position_model()이 관절 torque 모델과 wrench 모델을 둘 다 만든다.
#   GUI "상태" 패널에 "위치 (tool wrench, 자세 무관)" 행을 추가해 두 추정치를
#   나란히 비교할 수 있다. 구버전(v1/v2) 프리셋은 tool_force 샘플이 없어 이
#   모델을 건너뛴다 — 방향 보정을 다시 캡처하면 생성된다.
#   주의: GetToolForce.srv 문서에 "force는 base 좌표, moment는 tool 좌표
#   기준"이라 적혀 있어 축 정의가 애매하다. "자세 무관"이 실제로 성립하는지는
#   같은 TCP pose를 서로 다른 joint 해(팔꿈치 업/다운 등)로 도달시켜
#   get_tool_force() 값이 일치하는지 실측으로 검증해야 한다.
#
# TODO: 이후 실험적으로 수정하는 내용은 위 목록에 날짜와 함께 계속 추가할 것.
# =============================================================================

from __future__ import annotations

import csv
import json
import math
import os
import queue
import subprocess
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
import DR_init
from std_msgs.msg import Float64MultiArray, String

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, simpledialog, ttk
except ImportError as error:
    raise RuntimeError(
        "Tkinter가 설치되지 않았습니다. "
        "Ubuntu에서 'sudo apt install python3-tk'를 실행하십시오."
    ) from error


# =============================================================================
# 사용자 설정
# =============================================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "plate_position_gui_v3"

GRIPPER_MODEL = "OnRobot RG2"
PLATE_RADIUS_MM = 100.0
CALIBRATION_RADIUS_MM = 60.0

SAMPLE_HZ = 50.0  # 100 -> 50 Hz. 실제 도달 가능한 속도는 read_robot_state()의
                  # 블로킹 API 호출 6개의 왕복 시간에 의해 상한이 걸린다.
                  # GUI의 "실측 Hz" / "이론상 최대 Hz" 로그로 실제 루프 속도를
                  # 확인하고, 목표치에 못 미치면 이 값을 낮출 것.
SAMPLE_PERIOD_SEC = 1.0 / SAMPLE_HZ

BASELINE_DURATION_SEC = 5.0
CENTER_REFERENCE_DURATION_SEC = 4.0
DIRECTION_REFERENCE_DURATION_SEC = 3.0

# --- 레거시 실시간 필터: 고정 alpha EMA. ------------------------------------
# alpha가 샘플 주기(dt)와 무관한 상수이기 때문에, SAMPLE_HZ를 올리면 같은
# 실제 시간(wall-clock) 동안 더 많이 감쇠해 체감 스무딩이 "약해진다".
# ("원본" 판 시각화 비교용으로 그대로 유지)
EXTERNAL_TORQUE_EMA_ALPHA = 0.20
JOINT_TORQUE_EMA_ALPHA = 0.20
DERIVATIVE_EMA_ALPHA = 0.25

# --- 신규 실시간 필터: 이동중앙값(스파이크 제거) + 시간상수 기반 EMA. -------
# alpha를 매 스텝 dt로부터 계산하므로 SAMPLE_HZ를 바꿔도 체감 스무딩 정도가
# 동일하게 유지된다. ("New" 판 시각화에 사용)
EXTERNAL_TORQUE_TIME_CONSTANT_SEC = 0.15
MEDIAN_PREFILTER_WINDOW = 5

# --- 위치 추정(estimated_y_mm/estimated_z_mm) 출력단 EMA. -------------------
# 토크를 필터링해도 선형모델을 거쳐 나온 위치 추정값에는 여전히 노이즈가
# 남는다. 특히 Z축은 그리퍼가 판을 Z=-90mm에서 잡고 있어(외팔보 구조)
# -Z쪽으로 갈수록 모멘트 팔이 짧아 신호가 약하고 노이즈에 더 취약하다.
# 그래서 위치 추정값 자체에 시간상수 기반 EMA를 한 번 더 걸되, Z를 Y보다
# 더 강하게(시간상수 크게) 스무딩한다. dt 기반이라 SAMPLE_HZ와 무관하다.
# 값이 클수록 부드럽지만 그만큼 반응이 느려진다(지연 증가) — 컨트롤러가
# 이 값을 P항으로 쓰므로 과하게 키우면 추종이 늘어진다.
POSITION_OUTPUT_TIME_CONSTANT_Y_SEC = 0.12
POSITION_OUTPUT_TIME_CONSTANT_Z_SEC = 0.30

# GUI 슬라이더로 노출할 "New" 파이프라인 전처리 파라미터.
# (속성명, 표시 라벨, 최소, 최대, 기본값, 표시 포맷, 설명)
# Original(레거시) 판은 비교용으로 EXTERNAL_TORQUE_EMA_ALPHA를 고정값 그대로
# 쓰므로 여기엔 포함하지 않는다.
PREPROCESS_PARAM_SPECS = [
    (
        "external_torque_time_constant_sec",
        "외력 토크 스무딩 시간상수",
        0.02,
        1.0,
        EXTERNAL_TORQUE_TIME_CONSTANT_SEC,
        "{:.3f} s",
        "external torque(New 파이프라인) EMA 시간상수. 클수록 부드럽지만 "
        "반응은 느려짐.",
    ),
    (
        "joint_torque_ema_alpha",
        "관절 토크 EMA α",
        0.01,
        1.0,
        JOINT_TORQUE_EMA_ALPHA,
        "{:.3f}",
        "관절(joint) 토크 EMA 계수. 클수록 최신 값을 더 반영해 반응은 "
        "빠르지만 노이즈도 더 남음.",
    ),
    (
        "derivative_ema_alpha",
        "미분(변화율) EMA α",
        0.01,
        1.0,
        DERIVATIVE_EMA_ALPHA,
        "{:.3f}",
        "토크 변화율(미분) 스무딩 계수. 작을수록 미분 노이즈를 더 억제함.",
    ),
    (
        "position_output_time_constant_y_sec",
        "위치추정 Y 스무딩 시간상수",
        0.02,
        1.0,
        POSITION_OUTPUT_TIME_CONSTANT_Y_SEC,
        "{:.3f} s",
        "estimated_y_mm 출력단 EMA 시간상수. 클수록 화면 움직임이 부드럽지만 "
        "지연은 커짐.",
    ),
    (
        "position_output_time_constant_z_sec",
        "위치추정 Z 스무딩 시간상수",
        0.02,
        1.0,
        POSITION_OUTPUT_TIME_CONSTANT_Z_SEC,
        "{:.3f} s",
        "estimated_z_mm 출력단 EMA 시간상수. Z는 약신호축이라 기본값이 Y보다 "
        "큼.",
    ),
    (
        "median_prefilter_window",
        "이동중앙값 윈도우",
        1,
        15,
        MEDIAN_PREFILTER_WINDOW,
        "{:.0f} 샘플",
        "스파이크 제거용 이동중앙값 필터 창 크기(샘플 수). 클수록 스파이크에 "
        "강하지만 반응은 느려짐.",
    ),
]

PREPROCESS_PARAM_NAMES = {
    spec[0] for spec in PREPROCESS_PARAM_SPECS
}

EXTERNAL_TORQUE_NOISE_FLOOR_NM = 0.02

# --- Tool wrench(Jacobian 보정 완료) 기반 위치 추정 파이프라인 -------------
# get_external_torque()는 관절 공간 신호라 J(q)^T가 곱해진 상태이고, 그
# 계수는 캘리브레이션 당시 joint 각도(q)에서만 유효하다(그래서 자세가
# 바뀌면 무효화됨). Doosan 컨트롤러의 get_tool_force(ref)는 내부적으로
# 이 J(q)^T를 이미 역산해 tool 좌표계의 Cartesian wrench(힘 3 + 모멘트 3)를
# 돌려주므로, 이론상 어떤 joint 자세로 같은 TCP pose에 도달하든 값이
# 일정해야 한다. 이 상수들은 그 wrench 신호 전용 필터/노이즈 설정이며,
# 기존 external_torque 파이프라인과 독립적으로 동작한다.
# 주의: GetToolForce.srv 주석엔 "force는 base 좌표, moment는 tool 좌표
# 기준"이라고 돼 있어 축 정의가 애매하다. 아래 회귀(fit)는 6개 성분을
# 그대로 정규화해 넣고 계수를 학습하므로 부호/축 해석에 의존하지 않지만,
# 실제로 "자세 무관"이 맞는지는 같은 TCP pose를 서로 다른 joint 해(예:
# 팔꿈치 업/다운)로 도달시켜 tool_force가 일치하는지 실측 검증이 필요하다.
TOOL_FORCE_TIME_CONSTANT_SEC = 0.15
TOOL_FORCE_NOISE_FLOOR = 0.05

MIN_CENTER_THRESHOLD_Z = 4.0
CENTER_THRESHOLD_MARGIN = 1.25
CENTER_ENTER_RATIO = 0.90
CENTER_EXIT_RATIO = 1.10
CENTER_CONFIRM_CYCLES = 3

TREND_WINDOW_SEC = 0.8
TREND_MIN_ABS_Z_PER_SEC = 1.0

MIN_OBJECT_LOAD_DELTA_NORM_NM = 0.15
OBJECT_PRESENT_RATIO = 0.25

MAX_TCP_POSITION_DRIFT_MM = 2.0
MAX_TCP_ORIENTATION_DRIFT_DEG = 1.0
MAX_TCP_LINEAR_SPEED_MM_S = 1.0
MAX_TCP_ANGULAR_SPEED_DEG_S = 0.5
MAX_JOINT_SPEED_DEG_S = 0.5

RIDGE_LAMBDA = 1.0
MAX_POSITION_MODEL_RMSE_MM = 25.0

FEATURE_TOPIC = f"/{ROBOT_ID}/plate_position/features"
STATUS_TOPIC = f"/{ROBOT_ID}/plate_position/status"
COMMAND_TOPIC = f"/{ROBOT_ID}/plate_position/command"

LOG_DIRECTORY = Path.home() / "plate_balance_logs"

# 캘리브레이션 프리셋 저장 폴더. 이 소스 파일과 같은 폴더(rokey 모듈)의
# cali_preset 하위에 저장한다. --symlink-install로 빌드하면 __file__은 install의
# 심볼릭 링크이고, resolve()가 실제 src 파일을 따라가므로 프리셋이 src 트리(즉
# git에 남는 곳)에 저장된다. 심볼릭 링크 없이 빌드했다면 install 아래에 생겨
# 재빌드 때 사라질 수 있으니 --symlink-install 사용을 권장.
CALI_PRESET_DIR = Path(__file__).resolve().parent / "cali_preset"

# --- "저장된 joint로 이동" 기능 -----------------------------------------
# 이 파일은 원래 "로봇 모션 명령을 전혀 보내지 않는" 순수 관찰 도구다.
# 캘리브레이션 프리셋을 불러온 뒤 그 기준 joint 자세로 돌아가려는 실용적
# 필요 때문에 이 기능 하나만 의도적 예외로 추가했다(2026-07-21). 반드시
# 저속으로 움직이고, 사용 전 GUI에서 확인창을 거치도록 한다.
# 2026-07-22: 실기에서 20 deg/s가 너무 빠르게 느껴져 5 deg/s로 하향.
REFERENCE_MOVE_VEL_DEG_S = 5.0
REFERENCE_MOVE_ACC_DEG_S2 = 5.0

GUI_POLL_MS = 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =============================================================================
# 자료형
# =============================================================================

@dataclass
class Measurement:
    mean: np.ndarray
    std: np.ndarray
    samples: np.ndarray


@dataclass
class PositionModel:
    coefficients: np.ndarray
    calibration_rmse_mm: float
    calibration_points: Dict[str, Dict[str, Any]]
    edge_reference_z: float
    condition_number: float


def measurement_to_dict(measurement: "Measurement") -> Dict[str, Any]:
    return {
        "mean": measurement.mean.tolist(),
        "std": measurement.std.tolist(),
        "samples": measurement.samples.tolist(),
    }


def measurement_from_dict(data: Dict[str, Any]) -> "Measurement":
    return Measurement(
        mean=np.asarray(data["mean"], dtype=float),
        std=np.asarray(data["std"], dtype=float),
        samples=np.asarray(data["samples"], dtype=float),
    )


# =============================================================================
# 보조 함수
# =============================================================================

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


def vector_norm(values: Sequence[float]) -> float:
    return float(np.linalg.norm(np.asarray(values, dtype=float)))


def robust_mean(samples: Sequence[Sequence[float]]) -> np.ndarray:
    array = np.asarray(samples, dtype=float)

    if array.ndim != 2 or array.shape[0] == 0:
        raise RuntimeError("측정 샘플이 없습니다.")

    median = np.median(array, axis=0)
    absolute_deviation = np.abs(array - median)
    mad = np.median(absolute_deviation, axis=0)
    scale = 1.4826 * np.maximum(mad, 1.0e-12)

    valid = absolute_deviation <= 4.5 * scale
    filtered = np.where(valid, array, np.nan)
    mean = np.nanmean(filtered, axis=0)

    fallback = np.mean(array, axis=0)
    invalid = ~np.isfinite(mean)
    mean[invalid] = fallback[invalid]

    return mean


def sample_std(samples: Sequence[Sequence[float]]) -> np.ndarray:
    array = np.asarray(samples, dtype=float)

    if array.ndim != 2:
        raise RuntimeError("표준편차 입력 형식이 잘못됐습니다.")

    if array.shape[0] < 2:
        return np.zeros(array.shape[1], dtype=float)

    return np.std(array, axis=0, ddof=1)


def ema_array(
    current: np.ndarray,
    previous: Optional[np.ndarray],
    alpha: float,
) -> np.ndarray:
    if previous is None:
        return current.copy()

    return alpha * current + (1.0 - alpha) * previous


def wrapped_angle_difference_deg(current: float, reference: float) -> float:
    return (current - reference + 180.0) % 360.0 - 180.0


def pose_drift(
    current_pose: np.ndarray,
    reference_pose: np.ndarray,
) -> Tuple[float, float]:
    position_drift = vector_norm(current_pose[:3] - reference_pose[:3])

    orientation_difference = np.asarray(
        [
            wrapped_angle_difference_deg(
                current_pose[index],
                reference_pose[index],
            )
            for index in range(3, 6)
        ],
        dtype=float,
    )

    orientation_drift = vector_norm(orientation_difference)

    return position_drift, orientation_drift


def trend_slope(history: deque) -> float:
    if len(history) < 3:
        return 0.0

    times = np.asarray([item[0] for item in history], dtype=float)
    values = np.asarray([item[1] for item in history], dtype=float)

    times = times - times[0]
    centered_times = times - np.mean(times)
    denominator = float(np.sum(np.square(centered_times)))

    if denominator < 1.0e-12:
        return 0.0

    numerator = float(
        np.sum(centered_times * (values - np.mean(values)))
    )

    return numerator / denominator


def time_constant_alpha(dt: float, time_constant_sec: float) -> float:
    if not math.isfinite(dt) or dt <= 0.0:
        return 1.0
    return 1.0 - math.exp(-dt / time_constant_sec)


def rolling_median(history: deque) -> np.ndarray:
    return np.median(
        np.asarray(history, dtype=float),
        axis=0,
    )


def latch_transition(
    center_error_z_norm: float,
    threshold_z: float,
    is_latched: bool,
    enter_count: int,
    exit_count: int,
) -> Tuple[bool, int, int]:
    enter_threshold = threshold_z * CENTER_ENTER_RATIO
    exit_threshold = threshold_z * CENTER_EXIT_RATIO

    if is_latched:
        if center_error_z_norm > exit_threshold:
            exit_count += 1
        else:
            exit_count = 0

        if exit_count >= CENTER_CONFIRM_CYCLES:
            return False, 0, 0

        return True, enter_count, exit_count

    if center_error_z_norm < enter_threshold:
        enter_count += 1
    else:
        enter_count = 0

    if enter_count >= CENTER_CONFIRM_CYCLES:
        return True, 0, 0

    return False, enter_count, exit_count


def finite_text(value: float, fmt: str = ".2f", fallback: str = "--") -> str:
    if not math.isfinite(value):
        return fallback
    return format(value, fmt)


# =============================================================================
# ROS/센서 백엔드
# =============================================================================

class PlatePositionBackend:
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

        self.feature_publisher = node.create_publisher(
            Float64MultiArray,
            FEATURE_TOPIC,
            10,
        )
        self.status_publisher = node.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )
        self.command_subscriber = node.create_subscription(
            String,
            COMMAND_TOPIC,
            self.ros_command_callback,
            10,
        )

        self.running = True
        self.monitor_enabled = False
        self.current_label = "unlabeled"

        self.empty_external: Optional[Measurement] = None
        self.empty_joint: Optional[Measurement] = None
        self.center_external: Optional[Measurement] = None
        self.center_joint: Optional[Measurement] = None
        self.reference_pose: Optional[np.ndarray] = None
        # 캘리브레이션(빈 판 측정) 당시 joint 각도. "저장된 joint로 이동"
        # 기능의 목표값으로 쓰인다.
        self.reference_posj: Optional[np.ndarray] = None

        self.noise_sigma: Optional[np.ndarray] = None
        self.center_threshold_z = MIN_CENTER_THRESHOLD_Z
        self.object_load_delta_norm_nm = math.nan

        # Tool wrench(get_tool_force) 기반, 조인트 자세 무관 추정 파이프라인.
        # noise_sigma/position_model과 각각 대응하는 별도 세트를 둔다.
        self.empty_tool_force: Optional[Measurement] = None
        self.center_tool_force: Optional[Measurement] = None
        self.wrench_noise_sigma: Optional[np.ndarray] = None
        self.wrench_position_model: Optional[PositionModel] = None
        self.filtered_tool_force: Optional[np.ndarray] = None

        # 실시간 조정 가능한 전처리(필터) 파라미터. 기본값은 위 모듈 상수에서
        # 가져오되, GUI 슬라이더에서 "set_param" 명령으로 실행 중 값을 바꿀 수
        # 있다(set_param() 참고). New 파이프라인(주 상태값/New 판 시각화)에만
        # 영향을 준다 — Original(레거시) 판은 비교용으로 고정 alpha를 그대로 쓴다.
        self.external_torque_time_constant_sec = (
            EXTERNAL_TORQUE_TIME_CONSTANT_SEC
        )
        self.joint_torque_ema_alpha = JOINT_TORQUE_EMA_ALPHA
        self.derivative_ema_alpha = DERIVATIVE_EMA_ALPHA
        self.position_output_time_constant_y_sec = (
            POSITION_OUTPUT_TIME_CONSTANT_Y_SEC
        )
        self.position_output_time_constant_z_sec = (
            POSITION_OUTPUT_TIME_CONSTANT_Z_SEC
        )
        self.median_prefilter_window = MEDIAN_PREFILTER_WINDOW

        self.direction_points: Dict[str, Dict[str, Any]] = {}
        self.position_model: Optional[PositionModel] = None

        self.filtered_external: Optional[np.ndarray] = None
        self.filtered_joint: Optional[np.ndarray] = None
        self.filtered_external_derivative: Optional[np.ndarray] = None
        self.filtered_joint_derivative: Optional[np.ndarray] = None
        self.previous_filtered_external: Optional[np.ndarray] = None
        self.previous_filtered_joint: Optional[np.ndarray] = None
        self.previous_time: Optional[float] = None

        # 위치 추정 출력단 EMA 상태 (new 파이프라인 전용)
        self.filtered_estimated_y: Optional[float] = None
        self.filtered_estimated_z: Optional[float] = None

        self.score_history: deque = deque()
        self.center_latched = True
        self.center_enter_count = 0
        self.center_exit_count = 0

        # New 파이프라인의 이동중앙값(스파이크 제거) 윈도우. 창 크기는 위
        # self.median_prefilter_window로 조정하며, set_param()으로 바뀌면
        # deque를 통째로 새로 만들어 maxlen을 갱신한다.
        self.raw_external_history: deque = deque(
            maxlen=self.median_prefilter_window,
        )
        # 레거시(원본) 실시간 필터 상태 — "Original" 판 시각화 비교용
        self.filtered_external_legacy: Optional[np.ndarray] = None
        self.legacy_center_latched = True
        self.legacy_center_enter_count = 0
        self.legacy_center_exit_count = 0

        self.filtered_loop_hz: Optional[float] = None
        # 로봇 API 6개 호출 자체에 걸리는 시간(ms) — Hz 상한을 좌우하는 병목
        self.filtered_loop_api_ms: Optional[float] = None
        # spin_once+API+build_state+publish+CSV 기록까지 전체 루프 작업 시간(ms)
        self.filtered_loop_work_ms: Optional[float] = None
        self.last_hz_log_monotonic: Optional[float] = None

        self.csv_file = None
        self.csv_writer = None
        self.csv_path: Optional[Path] = None
        self.model_path: Optional[Path] = None

    # -------------------------------------------------------------------------
    # UI 전달
    # -------------------------------------------------------------------------

    def send_ui(self, event: str, **payload: Any) -> None:
        self.ui_queue.put(
            {
                "event": event,
                **payload,
            }
        )

    def send_log(self, message: str) -> None:
        self.send_ui(
            "log",
            message=message,
        )

    def send_reference_state(self) -> None:
        self.send_ui(
            "reference_state",
            empty_ready=self.empty_external is not None,
            center_ready=self.center_external is not None,
            direction_points=list(self.direction_points.keys()),
            model_ready=self.position_model is not None,
            wrench_model_ready=self.wrench_position_model is not None,
            monitor_enabled=self.monitor_enabled,
            center_threshold_z=self.center_threshold_z,
            reference_posj_ready=self.reference_posj is not None,
        )

    # -------------------------------------------------------------------------
    # ROS 명령
    # -------------------------------------------------------------------------

    def ros_command_callback(self, message: String) -> None:
        command = message.data.strip()
        lower = command.lower()

        if lower in ("stop", "quit", "exit"):
            self.command_queue.put(("shutdown", None))

        elif lower == "pause":
            self.command_queue.put(("pause_monitor", None))

        elif lower == "resume":
            self.command_queue.put(("start_monitor", None))

        elif lower.startswith("label="):
            self.command_queue.put(
                ("set_label", command.split("=", 1)[1].strip())
            )

        elif lower.startswith("label:"):
            self.command_queue.put(
                ("set_label", command.split(":", 1)[1].strip())
            )

    # -------------------------------------------------------------------------
    # 로봇 API
    # -------------------------------------------------------------------------

    def get_external_torque(self) -> np.ndarray:
        return to_fixed_array(
            self.dsr.get_external_torque(),
            6,
        )

    def get_joint_torque(self) -> np.ndarray:
        return to_fixed_array(
            self.dsr.get_joint_torque(),
            6,
        )

    def get_tool_force(self) -> np.ndarray:
        # ref=DR_TOOL: 그리퍼(플레이트)에 붙어 회전하는 tool 좌표계 기준
        # wrench. 컨트롤러가 내부적으로 J(q)^T를 역산해서 주기 때문에,
        # 이론상 joint 자세(q)가 달라져도 같은 TCP pose/그리퍼 파지
        # 상태라면 값이 일정해야 한다 — external_torque(관절 공간)와의
        # 핵심 차이.
        return to_fixed_array(
            self.dsr.get_tool_force(ref=self.dsr.DR_TOOL),
            6,
        )

    def get_current_posj(self) -> np.ndarray:
        return to_fixed_array(
            self.dsr.get_current_posj(),
            6,
        )

    def get_current_velj(self) -> np.ndarray:
        return to_fixed_array(
            self.dsr.get_current_velj(),
            6,
        )

    def get_current_posx_base(self) -> np.ndarray:
        try:
            value = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        except TypeError:
            value = self.dsr.get_current_posx(self.dsr.DR_BASE)

        return to_fixed_array(value, 6)

    def get_current_velx_base(self) -> np.ndarray:
        try:
            value = self.dsr.get_current_velx(ref=self.dsr.DR_BASE)
        except TypeError:
            value = self.dsr.get_current_velx(self.dsr.DR_BASE)

        return to_fixed_array(value, 6)

    def read_robot_state(self) -> Dict[str, np.ndarray]:
        return {
            "external_torque": self.get_external_torque(),
            "joint_torque": self.get_joint_torque(),
            "tool_force": self.get_tool_force(),
            "joint_position": self.get_current_posj(),
            "joint_velocity": self.get_current_velj(),
            "tcp_pose": self.get_current_posx_base(),
            "tcp_velocity": self.get_current_velx_base(),
        }

    # -------------------------------------------------------------------------
    # 측정
    # -------------------------------------------------------------------------

    def collect_measurement(
        self,
        duration_sec: float,
        title: str,
    ) -> Tuple[Measurement, Measurement, Measurement]:
        sample_count = max(
            5,
            int(round(duration_sec * SAMPLE_HZ)),
        )

        external_samples = []
        joint_samples = []
        tool_force_samples = []

        self.send_ui(
            "measurement_started",
            title=title,
            total=sample_count,
        )
        self.send_log(title)

        for index in range(sample_count):
            if not self.running:
                raise RuntimeError("프로그램 종료 요청")

            loop_start = time.monotonic()

            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

            external_samples.append(self.get_external_torque())
            joint_samples.append(self.get_joint_torque())
            tool_force_samples.append(self.get_tool_force())

            self.send_ui(
                "measurement_progress",
                current=index + 1,
                total=sample_count,
            )

            remaining = (
                SAMPLE_PERIOD_SEC
                - (time.monotonic() - loop_start)
            )

            if remaining > 0.0:
                time.sleep(remaining)

        external_array = np.asarray(external_samples, dtype=float)
        joint_array = np.asarray(joint_samples, dtype=float)
        tool_force_array = np.asarray(tool_force_samples, dtype=float)

        result = (
            Measurement(
                mean=robust_mean(external_array),
                std=sample_std(external_array),
                samples=external_array,
            ),
            Measurement(
                mean=robust_mean(joint_array),
                std=sample_std(joint_array),
                samples=joint_array,
            ),
            Measurement(
                mean=robust_mean(tool_force_array),
                std=sample_std(tool_force_array),
                samples=tool_force_array,
            ),
        )

        self.send_ui("measurement_finished")

        return result

    def capture_empty(self) -> None:
        self.monitor_enabled = False

        (
            self.empty_external,
            self.empty_joint,
            self.empty_tool_force,
        ) = self.collect_measurement(
            BASELINE_DURATION_SEC,
            "빈 판 external/joint torque + tool wrench 측정",
        )

        self.reference_pose = self.get_current_posx_base()
        self.reference_posj = self.get_current_posj()

        self.center_external = None
        self.center_joint = None
        self.center_tool_force = None
        self.noise_sigma = None
        self.wrench_noise_sigma = None
        self.direction_points.clear()
        self.position_model = None
        self.wrench_position_model = None
        self.reset_filters()

        self.send_log(
            "빈 판 external torque 평균: "
            + np.array2string(
                self.empty_external.mean,
                precision=7,
            )
        )
        self.send_log(
            "빈 판 external torque 표준편차: "
            + np.array2string(
                self.empty_external.std,
                precision=7,
            )
        )
        self.send_log(
            "기준 TCP pose: "
            + np.array2string(
                self.reference_pose,
                precision=5,
            )
        )
        self.send_log(
            "기준 joint 각도(posj): "
            + np.array2string(
                self.reference_posj,
                precision=3,
            )
        )
        self.send_log(
            "빈 판 tool wrench(tool 좌표계) 평균: "
            + np.array2string(
                self.empty_tool_force.mean,
                precision=5,
            )
        )

        self.send_ui(
            "empty_captured",
            mean=self.empty_external.mean.tolist(),
            std=self.empty_external.std.tolist(),
            pose=self.reference_pose.tolist(),
            posj=self.reference_posj.tolist(),
        )
        self.send_reference_state()

    def capture_center(self) -> None:
        if self.empty_external is None:
            raise RuntimeError("먼저 빈 판 기준을 측정하십시오.")

        self.monitor_enabled = False

        (
            self.center_external,
            self.center_joint,
            self.center_tool_force,
        ) = self.collect_measurement(
            CENTER_REFERENCE_DURATION_SEC,
            "물체 중심 위치 external/joint torque + tool wrench 측정",
        )

        empty_delta = (
            self.center_external.mean
            - self.empty_external.mean
        )
        self.object_load_delta_norm_nm = vector_norm(empty_delta)

        if self.object_load_delta_norm_nm < MIN_OBJECT_LOAD_DELTA_NORM_NM:
            raise RuntimeError(
                "빈 판 대비 물체 하중 변화가 너무 작습니다. "
                f"|Δtau|={self.object_load_delta_norm_nm:.4f} Nm"
            )

        self.noise_sigma = np.maximum.reduce(
            [
                self.empty_external.std,
                self.center_external.std,
                np.full(
                    6,
                    EXTERNAL_TORQUE_NOISE_FLOOR_NM,
                    dtype=float,
                ),
            ]
        )

        center_residual_z_norms = np.linalg.norm(
            (
                self.center_external.samples
                - self.center_external.mean
            )
            / self.noise_sigma,
            axis=1,
        )

        center_noise_percentile = float(
            np.percentile(
                center_residual_z_norms,
                95.0,
            )
        )

        self.center_threshold_z = max(
            MIN_CENTER_THRESHOLD_Z,
            center_noise_percentile * CENTER_THRESHOLD_MARGIN,
        )

        self.wrench_noise_sigma = np.maximum.reduce(
            [
                self.empty_tool_force.std,
                self.center_tool_force.std,
                np.full(
                    6,
                    TOOL_FORCE_NOISE_FLOOR,
                    dtype=float,
                ),
            ]
        )

        self.direction_points.clear()
        self.position_model = None
        self.wrench_position_model = None
        self.reset_filters()
        self.center_latched = True
        self.center_enter_count = 0
        self.center_exit_count = 0

        self.send_log(
            "중심 external torque 평균: "
            + np.array2string(
                self.center_external.mean,
                precision=7,
            )
        )
        self.send_log(
            f"빈 판 대비 물체 하중 norm: "
            f"{self.object_load_delta_norm_nm:.5f} Nm"
        )
        self.send_log(
            "축별 noise sigma: "
            + np.array2string(
                self.noise_sigma,
                precision=6,
            )
        )
        self.send_log(
            f"중심 허용 z-norm: {self.center_threshold_z:.3f}"
        )
        self.send_log(
            "축별 tool wrench noise sigma: "
            + np.array2string(
                self.wrench_noise_sigma,
                precision=6,
            )
        )

        self.send_ui(
            "center_captured",
            mean=self.center_external.mean.tolist(),
            std=self.center_external.std.tolist(),
            noise_sigma=self.noise_sigma.tolist(),
            load_norm=self.object_load_delta_norm_nm,
            center_threshold_z=self.center_threshold_z,
        )
        self.send_reference_state()

    def capture_direction(
        self,
        name: str,
        y_mm: float,
        z_mm: float,
        description: str,
    ) -> None:
        if self.center_external is None or self.noise_sigma is None:
            raise RuntimeError("먼저 중심 기준을 측정하십시오.")

        self.monitor_enabled = False

        external, joint, tool_force = self.collect_measurement(
            DIRECTION_REFERENCE_DURATION_SEC,
            f"{name} 위치 external/joint torque + tool wrench 측정",
        )

        center_error = (
            external.mean
            - self.center_external.mean
        )
        center_error_z = (
            center_error
            / self.noise_sigma
        )
        center_error_z_norm = vector_norm(center_error_z)

        self.direction_points[name] = {
            "name": name,
            "y_mm": y_mm,
            "z_mm": z_mm,
            "description": description,
            "external": external,
            "joint": joint,
            "tool_force": tool_force,
            "center_error": center_error,
            "center_error_z": center_error_z,
            "center_error_z_norm": center_error_z_norm,
        }

        self.position_model = None
        self.wrench_position_model = None

        self.send_log(
            f"{name}: center error norm="
            f"{vector_norm(center_error):.5f} Nm, "
            f"z-norm={center_error_z_norm:.3f}"
        )
        self.send_log(
            f"{name}: dTau="
            + np.array2string(
                center_error,
                precision=7,
            )
        )

        self.send_ui(
            "direction_captured",
            name=name,
            center_error=center_error.tolist(),
            center_error_z=center_error_z.tolist(),
            center_error_z_norm=center_error_z_norm,
        )
        self.send_reference_state()

    # -------------------------------------------------------------------------
    # 위치 모델
    # -------------------------------------------------------------------------

    def _fit_linear_position_model(
        self,
        center_mean: np.ndarray,
        noise_sigma: np.ndarray,
        center_samples: np.ndarray,
        sample_key: str,
        mean_std_key_prefix: str,
    ) -> PositionModel:
        """정규화된 6채널 신호(z-score) -> (y_mm, z_mm) 리지 회귀.

        sample_key로 direction_points 안의 어떤 Measurement(예: "external"
        또는 "tool_force")를 쓸지 선택한다. 신호가 무엇이든(관절 토크든
        Jacobian 보정이 끝난 tool wrench든) 6채널을 그대로 정규화해 넣고
        계수를 학습하므로 축/부호 해석에 의존하지 않는다.
        """
        feature_rows = []
        target_rows = []

        calibration_points: Dict[
            str,
            Dict[str, Any],
        ] = {}

        for sample in center_samples:
            normalized = (
                sample
                - center_mean
            ) / noise_sigma

            feature_rows.append(
                np.concatenate(
                    [
                        normalized,
                        np.ones(1, dtype=float),
                    ]
                )
            )
            target_rows.append([0.0, 0.0])

        calibration_points["CENTER"] = {
            "y_mm": 0.0,
            "z_mm": 0.0,
            f"{mean_std_key_prefix}_mean": center_mean.tolist(),
            "center_error_z_norm": 0.0,
        }

        edge_z_norms = []

        # 축방향 4개(required)뿐 아니라 캡처된 모든 방향점(대각선 등 추가분
        # 포함)을 회귀에 넣는다. 대각선 점이 있으면 Y/Z 커플링을 분리할
        # 코너 데이터가 생겨 축을 벗어난 위치의 편향이 줄어든다. 정렬해서
        # 순회 순서를 결정적으로 만든다.
        for name in sorted(self.direction_points.keys()):
            point = self.direction_points[name]
            measurement: Measurement = point[sample_key]

            for sample in measurement.samples:
                normalized = (
                    sample
                    - center_mean
                ) / noise_sigma

                feature_rows.append(
                    np.concatenate(
                        [
                            normalized,
                            np.ones(1, dtype=float),
                        ]
                    )
                )
                target_rows.append(
                    [
                        point["y_mm"],
                        point["z_mm"],
                    ]
                )

            point_center_error_z = (
                (measurement.mean - center_mean)
                / noise_sigma
            )
            point_center_error_z_norm = vector_norm(
                point_center_error_z
            )

            calibration_points[name] = {
                "y_mm": point["y_mm"],
                "z_mm": point["z_mm"],
                "description": point["description"],
                f"{mean_std_key_prefix}_mean": measurement.mean.tolist(),
                f"{mean_std_key_prefix}_std": measurement.std.tolist(),
                "center_error_z_norm": point_center_error_z_norm,
            }

            edge_z_norms.append(
                point_center_error_z_norm
            )

        matrix_x = np.asarray(
            feature_rows,
            dtype=float,
        )
        matrix_y = np.asarray(
            target_rows,
            dtype=float,
        )

        regularization = np.eye(
            matrix_x.shape[1],
            dtype=float,
        )
        regularization[-1, -1] = 0.01

        normal_matrix = (
            matrix_x.T @ matrix_x
            + RIDGE_LAMBDA * regularization
        )

        coefficients = np.linalg.solve(
            normal_matrix,
            matrix_x.T @ matrix_y,
        )

        predictions = matrix_x @ coefficients

        rmse = float(
            np.sqrt(
                np.mean(
                    np.sum(
                        np.square(
                            predictions - matrix_y
                        ),
                        axis=1,
                    )
                )
            )
        )

        singular_values = np.linalg.svd(
            matrix_x,
            compute_uv=False,
        )

        if singular_values[-1] <= 1.0e-12:
            condition_number = math.inf
        else:
            condition_number = float(
                singular_values[0]
                / singular_values[-1]
            )

        edge_reference_z = float(
            np.median(
                np.asarray(
                    edge_z_norms,
                    dtype=float,
                )
            )
        )

        return PositionModel(
            coefficients=coefficients,
            calibration_rmse_mm=rmse,
            calibration_points=calibration_points,
            edge_reference_z=edge_reference_z,
            condition_number=condition_number,
        )

    def fit_position_model(self) -> None:
        required = {
            "Y_PLUS",
            "Y_MINUS",
            "Z_PLUS",
            "Z_MINUS",
        }

        missing = required - set(self.direction_points.keys())

        if missing:
            raise RuntimeError(
                "다음 방향 보정이 누락됐습니다: "
                + ", ".join(sorted(missing))
            )

        # 1) 관절 external torque 기반 위치 모델(기존 동작 그대로). 이 모델의
        #    계수는 캘리브레이션 당시 joint 자세에서만 유효하다.
        self.position_model = self._fit_linear_position_model(
            center_mean=self.center_external.mean,
            noise_sigma=self.noise_sigma,
            center_samples=self.center_external.samples,
            sample_key="external",
            mean_std_key_prefix="external",
        )

        rmse = self.position_model.calibration_rmse_mm
        condition_number = self.position_model.condition_number
        edge_reference_z = self.position_model.edge_reference_z

        self.send_log(
            f"위치 모델(관절 torque) 생성 완료: RMSE={rmse:.2f} mm, "
            f"condition={condition_number:.2f}, "
            f"edge z={edge_reference_z:.2f}"
        )

        if rmse > MAX_POSITION_MODEL_RMSE_MM:
            self.send_log(
                "[경고] 위치 회귀 RMSE가 큽니다. "
                "방향 보정 위치와 손 접촉 상태를 확인하십시오."
            )

        # 2) tool wrench(get_tool_force) 기반 위치 모델. 컨트롤러가 J(q)^T를
        #    이미 역산해 준 신호라, 이론상 joint 자세가 바뀌어도 이 모델은
        #    그대로 재사용할 수 있어야 한다(반드시 실측으로 검증할 것).
        #    구버전 프리셋을 불러온 뒤 다시 학습하면 tool_force 샘플이 없어
        #    건너뛸 수 있다 — 그 경우 방향 보정을 다시 캡처하면 된다.
        wrench_ready = (
            self.center_tool_force is not None
            and self.wrench_noise_sigma is not None
            and all(
                "tool_force" in point
                for point in self.direction_points.values()
            )
        )

        wrench_rmse_mm: Optional[float] = None

        if wrench_ready:
            self.wrench_position_model = self._fit_linear_position_model(
                center_mean=self.center_tool_force.mean,
                noise_sigma=self.wrench_noise_sigma,
                center_samples=self.center_tool_force.samples,
                sample_key="tool_force",
                mean_std_key_prefix="tool_force",
            )
            wrench_rmse_mm = (
                self.wrench_position_model.calibration_rmse_mm
            )

            self.send_log(
                "위치 모델(tool wrench, 조인트 자세 무관) 생성 완료: "
                f"RMSE={wrench_rmse_mm:.2f} mm, "
                "condition="
                f"{self.wrench_position_model.condition_number:.2f}"
            )

            if wrench_rmse_mm > MAX_POSITION_MODEL_RMSE_MM:
                self.send_log(
                    "[경고] tool wrench 위치 회귀 RMSE가 큽니다."
                )
        else:
            self.wrench_position_model = None
            self.send_log(
                "tool wrench 위치 모델은 건너뜀 "
                "(구버전 캘리브레이션이라 tool_force 샘플이 없음 — "
                "방향 보정을 다시 캡처하면 생성됩니다)."
            )

        self.send_ui(
            "model_built",
            rmse_mm=rmse,
            condition_number=condition_number,
            edge_reference_z=edge_reference_z,
            coefficients=self.position_model.coefficients.tolist(),
            wrench_rmse_mm=wrench_rmse_mm,
        )

        self.save_model()
        self.send_reference_state()

    # -------------------------------------------------------------------------
    # 필터/상태
    # -------------------------------------------------------------------------

    def reset_filters(self) -> None:
        self.filtered_external = None
        self.filtered_joint = None
        self.filtered_external_derivative = None
        self.filtered_joint_derivative = None
        self.previous_filtered_external = None
        self.previous_filtered_joint = None
        self.previous_time = None
        self.filtered_estimated_y = None
        self.filtered_estimated_z = None
        self.filtered_tool_force = None
        self.score_history.clear()

        self.raw_external_history.clear()
        self.filtered_external_legacy = None
        self.legacy_center_latched = True
        self.legacy_center_enter_count = 0
        self.legacy_center_exit_count = 0

        self.filtered_loop_hz = None
        self.filtered_loop_api_ms = None
        self.filtered_loop_work_ms = None
        self.last_hz_log_monotonic = None

    def update_center_latch(self, center_error_z_norm: float) -> bool:
        (
            self.center_latched,
            self.center_enter_count,
            self.center_exit_count,
        ) = latch_transition(
            center_error_z_norm,
            self.center_threshold_z,
            self.center_latched,
            self.center_enter_count,
            self.center_exit_count,
        )
        return self.center_latched

    def update_legacy_center_latch(self, center_error_z_norm: float) -> bool:
        (
            self.legacy_center_latched,
            self.legacy_center_enter_count,
            self.legacy_center_exit_count,
        ) = latch_transition(
            center_error_z_norm,
            self.center_threshold_z,
            self.legacy_center_latched,
            self.legacy_center_enter_count,
            self.legacy_center_exit_count,
        )
        return self.legacy_center_latched

    def estimate_position(
        self,
        center_error_z: np.ndarray,
        model: Optional[PositionModel],
    ) -> Tuple[float, float, float, float, str]:
        if model is None:
            return (
                math.nan,
                math.nan,
                math.nan,
                math.nan,
                "DIRECTION_NOT_CALIBRATED",
            )

        feature = np.concatenate(
            [
                center_error_z,
                np.ones(1, dtype=float),
            ]
        )

        estimated = (
            feature
            @ model.coefficients
        )

        estimated_y = float(estimated[0])
        estimated_z = float(estimated[1])
        estimated_radius = math.hypot(
            estimated_y,
            estimated_z,
        )
        estimated_angle = math.degrees(
            math.atan2(
                estimated_z,
                estimated_y,
            )
        )

        y_hint = ""
        z_hint = ""

        if abs(estimated_y) >= 5.0:
            y_hint = (
                "-Y"
                if estimated_y > 0.0
                else "+Y"
            )

        if abs(estimated_z) >= 5.0:
            z_hint = (
                "-Z"
                if estimated_z > 0.0
                else "+Z"
            )

        parts = [
            part
            for part in (y_hint, z_hint)
            if part
        ]

        move_hint = (
            "HOLD_CENTER"
            if not parts
            else "MOVE_" + "_".join(parts)
        )

        return (
            estimated_y,
            estimated_z,
            estimated_radius,
            estimated_angle,
            move_hint,
        )

    def build_state(
        self,
        raw: Dict[str, np.ndarray],
        now_monotonic: float,
    ) -> Dict[str, Any]:
        external_raw = raw["external_torque"]
        joint_raw = raw["joint_torque"]

        if self.previous_time is not None:
            dt_for_filter = now_monotonic - self.previous_time
        else:
            dt_for_filter = math.nan

        if math.isfinite(dt_for_filter) and dt_for_filter > 0.0:
            instantaneous_hz = 1.0 / dt_for_filter
            self.filtered_loop_hz = (
                instantaneous_hz
                if self.filtered_loop_hz is None
                else (
                    0.1 * instantaneous_hz
                    + 0.9 * self.filtered_loop_hz
                )
            )

        # 레거시 파이프라인: 고정 alpha EMA (원본 판 시각화용)
        self.filtered_external_legacy = ema_array(
            external_raw,
            self.filtered_external_legacy,
            EXTERNAL_TORQUE_EMA_ALPHA,
        )

        # 신규 파이프라인: 이동중앙값으로 스파이크 제거 후,
        # dt로부터 계산한 시간상수 기반 alpha로 EMA (New 판 시각화 + 주 상태값)
        self.raw_external_history.append(external_raw)
        median_external = (
            rolling_median(self.raw_external_history)
            if len(self.raw_external_history) >= 3
            else external_raw
        )
        new_external_alpha = time_constant_alpha(
            dt_for_filter,
            self.external_torque_time_constant_sec,
        )
        self.filtered_external = ema_array(
            median_external,
            self.filtered_external,
            new_external_alpha,
        )

        self.filtered_joint = ema_array(
            joint_raw,
            self.filtered_joint,
            self.joint_torque_ema_alpha,
        )

        if (
            self.previous_time is not None
            and self.previous_filtered_external is not None
            and self.previous_filtered_joint is not None
        ):
            dt = (
                now_monotonic
                - self.previous_time
            )
        else:
            dt = math.nan

        if math.isfinite(dt) and 1.0e-4 < dt < 1.0:
            external_derivative_raw = (
                self.filtered_external
                - self.previous_filtered_external
            ) / dt
            joint_derivative_raw = (
                self.filtered_joint
                - self.previous_filtered_joint
            ) / dt
        else:
            external_derivative_raw = np.zeros(
                6,
                dtype=float,
            )
            joint_derivative_raw = np.zeros(
                6,
                dtype=float,
            )

        self.filtered_external_derivative = ema_array(
            external_derivative_raw,
            self.filtered_external_derivative,
            self.derivative_ema_alpha,
        )
        self.filtered_joint_derivative = ema_array(
            joint_derivative_raw,
            self.filtered_joint_derivative,
            self.derivative_ema_alpha,
        )

        self.previous_filtered_external = self.filtered_external.copy()
        self.previous_filtered_joint = self.filtered_joint.copy()
        self.previous_time = now_monotonic

        empty_delta = (
            self.filtered_external
            - self.empty_external.mean
        )
        center_error = (
            self.filtered_external
            - self.center_external.mean
        )
        center_error_z = (
            center_error
            / self.noise_sigma
        )

        joint_empty_delta = (
            self.filtered_joint
            - self.empty_joint.mean
        )
        joint_center_error = (
            self.filtered_joint
            - self.center_joint.mean
        )

        empty_delta_norm = vector_norm(empty_delta)
        center_error_nm = vector_norm(center_error)
        center_error_z_norm = vector_norm(center_error_z)

        deviation_index = (
            center_error_z_norm
            / max(
                self.center_threshold_z,
                1.0e-9,
            )
        )

        if self.position_model is not None:
            edge_index = (
                center_error_z_norm
                / max(
                    self.position_model.edge_reference_z,
                    1.0e-9,
                )
            )
        else:
            edge_index = math.nan

        self.score_history.append(
            (
                now_monotonic,
                center_error_z_norm,
            )
        )

        while (
            self.score_history
            and now_monotonic
            - self.score_history[0][0]
            > TREND_WINDOW_SEC
        ):
            self.score_history.popleft()

        trend_z_per_sec = trend_slope(
            self.score_history
        )

        (
            estimated_y,
            estimated_z,
            estimated_radius,
            estimated_angle,
            move_hint,
        ) = self.estimate_position(
            center_error_z,
            self.position_model,
        )

        # 위치 출력단 EMA: 토크 필터를 거쳐도 남는 위치 추정 노이즈를 줄인다.
        # Z는 그리퍼측(외팔보) 약신호축이라 Y보다 더 강하게 스무딩한다.
        # dt_for_filter는 build_state 상단에서 이미 계산해 둔 값을 재사용.
        if (
            self.position_model is not None
            and math.isfinite(estimated_y)
            and math.isfinite(estimated_z)
        ):
            alpha_y = time_constant_alpha(
                dt_for_filter,
                self.position_output_time_constant_y_sec,
            )
            alpha_z = time_constant_alpha(
                dt_for_filter,
                self.position_output_time_constant_z_sec,
            )

            if self.filtered_estimated_y is None:
                self.filtered_estimated_y = estimated_y
            else:
                self.filtered_estimated_y = (
                    alpha_y * estimated_y
                    + (1.0 - alpha_y) * self.filtered_estimated_y
                )

            if self.filtered_estimated_z is None:
                self.filtered_estimated_z = estimated_z
            else:
                self.filtered_estimated_z = (
                    alpha_z * estimated_z
                    + (1.0 - alpha_z) * self.filtered_estimated_z
                )

            estimated_y = self.filtered_estimated_y
            estimated_z = self.filtered_estimated_z
            estimated_radius = math.hypot(estimated_y, estimated_z)
            estimated_angle = math.degrees(
                math.atan2(estimated_z, estimated_y)
            )

        # --- Tool wrench(get_tool_force, Jacobian 보정 완료) 기반 위치 추정 ---
        # 관절 external torque와 별개로, 이론상 joint 자세 변화에 영향받지
        # 않아야 하는 추정치. capture_center 이전(모델 없음)이거나 구버전
        # 프리셋(tool_force 미포함)이면 wrench_position_model이 None이라
        # NaN을 반환한다.
        tool_force_raw = raw["tool_force"]

        wrench_alpha = time_constant_alpha(
            dt_for_filter,
            TOOL_FORCE_TIME_CONSTANT_SEC,
        )
        self.filtered_tool_force = ema_array(
            tool_force_raw,
            self.filtered_tool_force,
            wrench_alpha,
        )

        if (
            self.center_tool_force is not None
            and self.wrench_noise_sigma is not None
        ):
            wrench_center_error = (
                self.filtered_tool_force
                - self.center_tool_force.mean
            )
            wrench_center_error_z = (
                wrench_center_error
                / self.wrench_noise_sigma
            )
        else:
            wrench_center_error = np.full(6, math.nan, dtype=float)
            wrench_center_error_z = np.full(6, math.nan, dtype=float)

        wrench_center_error_z_norm = vector_norm(
            wrench_center_error_z
        )

        (
            estimated_y_wrench,
            estimated_z_wrench,
            estimated_radius_wrench,
            estimated_angle_wrench,
            move_hint_wrench,
        ) = self.estimate_position(
            wrench_center_error_z,
            self.wrench_position_model,
        )

        is_center = self.update_center_latch(
            center_error_z_norm
        )

        if is_center:
            center_state = "CENTER"
            movement_state = "HOLD"
            move_hint = "HOLD_CENTER"

        elif trend_z_per_sec < -TREND_MIN_ABS_Z_PER_SEC:
            center_state = "OFF_CENTER"
            movement_state = "TOWARD_CENTER"

        elif trend_z_per_sec > TREND_MIN_ABS_Z_PER_SEC:
            center_state = "OFF_CENTER"
            movement_state = "AWAY_FROM_CENTER"

        else:
            center_state = "OFF_CENTER"
            movement_state = "STABLE_OFF_CENTER"

        object_threshold = max(
            MIN_OBJECT_LOAD_DELTA_NORM_NM,
            self.object_load_delta_norm_nm
            * OBJECT_PRESENT_RATIO,
        )

        object_present = (
            empty_delta_norm
            >= object_threshold
        )

        (
            tcp_position_drift,
            tcp_orientation_drift,
        ) = pose_drift(
            raw["tcp_pose"],
            self.reference_pose,
        )

        tcp_linear_speed = vector_norm(
            raw["tcp_velocity"][:3]
        )
        tcp_angular_speed = vector_norm(
            raw["tcp_velocity"][3:]
        )
        max_joint_speed = float(
            np.max(
                np.abs(
                    raw["joint_velocity"]
                )
            )
        )

        robot_stationary = (
            tcp_linear_speed
            <= MAX_TCP_LINEAR_SPEED_MM_S
            and tcp_angular_speed
            <= MAX_TCP_ANGULAR_SPEED_DEG_S
            and max_joint_speed
            <= MAX_JOINT_SPEED_DEG_S
        )

        pose_stable = (
            tcp_position_drift
            <= MAX_TCP_POSITION_DRIFT_MM
            and tcp_orientation_drift
            <= MAX_TCP_ORIENTATION_DRIFT_DEG
        )

        dominant_axis = int(
            np.argmax(
                np.abs(
                    center_error_z
                )
            )
        ) + 1
        dominant_axis_z = float(
            center_error_z[
                dominant_axis - 1
            ]
        )

        # --- 레거시 파이프라인 결과 (Original 판 시각화 비교용) ---------------
        legacy_center_error = (
            self.filtered_external_legacy
            - self.center_external.mean
        )
        legacy_center_error_z = (
            legacy_center_error
            / self.noise_sigma
        )
        legacy_center_error_nm = vector_norm(legacy_center_error)
        legacy_center_error_z_norm = vector_norm(legacy_center_error_z)

        legacy_deviation_index = (
            legacy_center_error_z_norm
            / max(
                self.center_threshold_z,
                1.0e-9,
            )
        )

        (
            legacy_estimated_y,
            legacy_estimated_z,
            legacy_estimated_radius,
            legacy_estimated_angle,
            _legacy_move_hint,
        ) = self.estimate_position(
            legacy_center_error_z,
            self.position_model,
        )

        legacy_is_center = self.update_legacy_center_latch(
            legacy_center_error_z_norm
        )

        legacy_center_state = (
            "CENTER" if legacy_is_center else "OFF_CENTER"
        )
        legacy_movement_state = (
            "HOLD" if legacy_is_center else "STABLE_OFF_CENTER"
        )

        return {
            "external_raw": external_raw,
            "external_filtered": self.filtered_external.copy(),
            "external_empty_delta": empty_delta,
            "external_center_error": center_error,
            "external_center_error_z": center_error_z,
            "external_derivative": self.filtered_external_derivative.copy(),

            "joint_raw": joint_raw,
            "joint_filtered": self.filtered_joint.copy(),
            "joint_empty_delta": joint_empty_delta,
            "joint_center_error": joint_center_error,
            "joint_derivative": self.filtered_joint_derivative.copy(),

            "joint_position": raw["joint_position"],
            "joint_velocity": raw["joint_velocity"],
            "tcp_pose": raw["tcp_pose"],
            "tcp_velocity": raw["tcp_velocity"],

            "empty_delta_norm_nm": empty_delta_norm,
            "center_error_nm": center_error_nm,
            "center_error_z": center_error_z_norm,
            "deviation_index": deviation_index,
            "edge_index": edge_index,
            "trend_z_per_sec": trend_z_per_sec,

            "estimated_y_mm": estimated_y,
            "estimated_z_mm": estimated_z,
            "estimated_radius_mm": estimated_radius,
            "estimated_angle_deg": estimated_angle,

            "center_state": center_state,
            "movement_state": movement_state,
            "move_hint": move_hint,

            "object_present": object_present,
            "robot_stationary": robot_stationary,
            "pose_stable": pose_stable,

            "dominant_axis": dominant_axis,
            "dominant_axis_z": dominant_axis_z,

            "tcp_position_drift_mm": tcp_position_drift,
            "tcp_orientation_drift_deg": tcp_orientation_drift,
            "tcp_linear_speed_mm_s": tcp_linear_speed,
            "tcp_angular_speed_deg_s": tcp_angular_speed,
            "max_joint_speed_deg_s": max_joint_speed,

            "achieved_hz": (
                self.filtered_loop_hz
                if self.filtered_loop_hz is not None
                else math.nan
            ),

            "tool_force_raw": tool_force_raw,
            "tool_force_filtered": self.filtered_tool_force.copy(),
            "wrench_center_error": wrench_center_error,
            "wrench_center_error_z": wrench_center_error_z,
            "wrench_center_error_z_norm": wrench_center_error_z_norm,

            "estimated_y_mm_wrench": estimated_y_wrench,
            "estimated_z_mm_wrench": estimated_z_wrench,
            "estimated_radius_mm_wrench": estimated_radius_wrench,
            "estimated_angle_deg_wrench": estimated_angle_wrench,
            "move_hint_wrench": move_hint_wrench,
            "wrench_model_ready": self.wrench_position_model is not None,

            "legacy": {
                "center_state": legacy_center_state,
                "movement_state": legacy_movement_state,
                "estimated_y_mm": legacy_estimated_y,
                "estimated_z_mm": legacy_estimated_z,
                "estimated_radius_mm": legacy_estimated_radius,
                "estimated_angle_deg": legacy_estimated_angle,
                "center_error_nm": legacy_center_error_nm,
                "center_error_z": legacy_center_error_z_norm,
                "deviation_index": legacy_deviation_index,
            },
        }

    # -------------------------------------------------------------------------
    # ROS 발행
    # -------------------------------------------------------------------------

    def publish_features(self, state: Dict[str, Any]) -> None:
        values = np.concatenate(
            [
                state["external_raw"],
                state["external_empty_delta"],
                state["external_center_error"],
                state["external_center_error_z"],
                state["external_derivative"],

                state["joint_raw"],
                state["joint_empty_delta"],
                state["joint_center_error"],
                state["joint_derivative"],

                state["joint_position"],
                state["joint_velocity"],
                state["tcp_pose"],
                state["tcp_velocity"],

                np.asarray(
                    [
                        state["empty_delta_norm_nm"],
                        state["center_error_nm"],
                        state["center_error_z"],
                        state["deviation_index"],
                        state["edge_index"],
                        state["trend_z_per_sec"],

                        state["estimated_y_mm"],
                        state["estimated_z_mm"],
                        state["estimated_radius_mm"],
                        state["estimated_angle_deg"],

                        float(state["object_present"]),
                        float(state["robot_stationary"]),
                        float(state["pose_stable"]),
                        float(self.position_model is not None),
                    ],
                    dtype=float,
                ),

                state["tool_force_raw"],
                state["tool_force_filtered"],
                state["wrench_center_error"],
                state["wrench_center_error_z"],

                np.asarray(
                    [
                        state["wrench_center_error_z_norm"],
                        state["estimated_y_mm_wrench"],
                        state["estimated_z_mm_wrench"],
                        state["estimated_radius_mm_wrench"],
                        state["estimated_angle_deg_wrench"],
                        float(state["wrench_model_ready"]),
                    ],
                    dtype=float,
                ),
            ]
        )

        message = Float64MultiArray()
        message.data = values.tolist()
        self.feature_publisher.publish(message)

    def publish_status(self, state: Dict[str, Any]) -> None:
        payload = {
            "label": self.current_label,
            "achieved_hz": state["achieved_hz"],
            "center_state": state["center_state"],
            "movement_state": state["movement_state"],
            "move_hint": state["move_hint"],

            "center_error_nm": state["center_error_nm"],
            "center_error_z": state["center_error_z"],
            "deviation_index": state["deviation_index"],
            "edge_index": state["edge_index"],
            "trend_z_per_sec": state["trend_z_per_sec"],

            "estimated_y_mm": state["estimated_y_mm"],
            "estimated_z_mm": state["estimated_z_mm"],
            "estimated_radius_mm": state["estimated_radius_mm"],
            "estimated_angle_deg": state["estimated_angle_deg"],

            "external_center_error": (
                state["external_center_error"].tolist()
            ),
            "external_center_error_z": (
                state["external_center_error_z"].tolist()
            ),

            "dominant_axis": state["dominant_axis"],
            "dominant_axis_z": state["dominant_axis_z"],

            "object_present": state["object_present"],
            "robot_stationary": state["robot_stationary"],
            "pose_stable": state["pose_stable"],
            "position_model_ready": self.position_model is not None,

            "estimated_y_mm_wrench": state["estimated_y_mm_wrench"],
            "estimated_z_mm_wrench": state["estimated_z_mm_wrench"],
            "estimated_radius_mm_wrench": (
                state["estimated_radius_mm_wrench"]
            ),
            "wrench_center_error_z": state["wrench_center_error_z_norm"],
            "wrench_model_ready": state["wrench_model_ready"],
        }

        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
        )
        self.status_publisher.publish(message)

    # -------------------------------------------------------------------------
    # 로그
    # -------------------------------------------------------------------------

    def open_logs(self) -> None:
        if self.csv_file is not None:
            return

        LOG_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True,
        )

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        self.csv_path = (
            LOG_DIRECTORY
            / f"plate_position_gui_{timestamp}.csv"
        )
        self.model_path = (
            LOG_DIRECTORY
            / f"plate_position_gui_{timestamp}_model.json"
        )

        self.csv_file = self.csv_path.open(
            "w",
            newline="",
            encoding="utf-8",
            buffering=1,
        )
        self.csv_writer = csv.writer(
            self.csv_file
        )

        header = [
            "wall_time",
            "label",

            "center_state",
            "movement_state",
            "move_hint",

            "object_present",
            "robot_stationary",
            "pose_stable",
            "position_model_ready",

            "achieved_hz",
            "loop_api_ms",

            "empty_delta_norm_nm",
            "center_error_nm",
            "center_error_z",
            "deviation_index",
            "edge_index",
            "trend_z_per_sec",

            "estimated_y_mm",
            "estimated_z_mm",
            "estimated_radius_mm",
            "estimated_angle_deg",

            "dominant_axis",
            "dominant_axis_z",

            "tcp_position_drift_mm",
            "tcp_orientation_drift_deg",
            "tcp_linear_speed_mm_s",
            "tcp_angular_speed_deg_s",
            "max_joint_speed_deg_s",

            "wrench_model_ready",
            "wrench_center_error_z_norm",
            "estimated_y_mm_wrench",
            "estimated_z_mm_wrench",
            "estimated_radius_mm_wrench",
            "estimated_angle_deg_wrench",
        ]

        for prefix in (
            "external_raw",
            "external_filtered",
            "external_empty_delta",
            "external_center_error",
            "external_center_error_z",
            "external_derivative",
            "joint_raw",
            "joint_filtered",
            "joint_empty_delta",
            "joint_center_error",
            "joint_derivative",
            "joint_position",
            "joint_velocity",
            "tcp_pose",
            "tcp_velocity",
            "tool_force_raw",
            "tool_force_filtered",
            "wrench_center_error",
            "wrench_center_error_z",
        ):
            for index in range(1, 7):
                header.append(
                    f"{prefix}_{index}"
                )

        self.csv_writer.writerow(header)

        self.send_ui(
            "log_paths",
            csv_path=str(self.csv_path),
            model_path=str(self.model_path),
        )

    def write_csv(self, state: Dict[str, Any]) -> None:
        if self.csv_writer is None:
            return

        row: List[Any] = [
            datetime.now().isoformat(
                timespec="milliseconds"
            ),
            self.current_label,

            state["center_state"],
            state["movement_state"],
            state["move_hint"],

            int(state["object_present"]),
            int(state["robot_stationary"]),
            int(state["pose_stable"]),
            int(self.position_model is not None),

            state["achieved_hz"],
            state["loop_api_ms"],

            state["empty_delta_norm_nm"],
            state["center_error_nm"],
            state["center_error_z"],
            state["deviation_index"],
            state["edge_index"],
            state["trend_z_per_sec"],

            state["estimated_y_mm"],
            state["estimated_z_mm"],
            state["estimated_radius_mm"],
            state["estimated_angle_deg"],

            state["dominant_axis"],
            state["dominant_axis_z"],

            state["tcp_position_drift_mm"],
            state["tcp_orientation_drift_deg"],
            state["tcp_linear_speed_mm_s"],
            state["tcp_angular_speed_deg_s"],
            state["max_joint_speed_deg_s"],

            int(state["wrench_model_ready"]),
            state["wrench_center_error_z_norm"],
            state["estimated_y_mm_wrench"],
            state["estimated_z_mm_wrench"],
            state["estimated_radius_mm_wrench"],
            state["estimated_angle_deg_wrench"],
        ]

        vectors = [
            state["external_raw"],
            state["external_filtered"],
            state["external_empty_delta"],
            state["external_center_error"],
            state["external_center_error_z"],
            state["external_derivative"],

            state["joint_raw"],
            state["joint_filtered"],
            state["joint_empty_delta"],
            state["joint_center_error"],
            state["joint_derivative"],

            state["joint_position"],
            state["joint_velocity"],
            state["tcp_pose"],
            state["tcp_velocity"],

            state["tool_force_raw"],
            state["tool_force_filtered"],
            state["wrench_center_error"],
            state["wrench_center_error_z"],
        ]

        for vector in vectors:
            row.extend(
                np.asarray(
                    vector,
                    dtype=float,
                ).tolist()
            )

        self.csv_writer.writerow(row)

    def save_model(self) -> None:
        if self.empty_external is None:
            return

        LOG_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True,
        )

        if self.model_path is None:
            timestamp = datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )
            self.model_path = (
                LOG_DIRECTORY
                / f"plate_position_gui_{timestamp}_model.json"
            )

        data: Dict[str, Any] = {
            "created_at": datetime.now().isoformat(),
            "robot_id": ROBOT_ID,
            "robot_model": ROBOT_MODEL,
            "gripper_model": GRIPPER_MODEL,
            "plate_radius_mm": PLATE_RADIUS_MM,
            "calibration_radius_mm": CALIBRATION_RADIUS_MM,

            "empty_external_mean": self.empty_external.mean.tolist(),
            "empty_external_std": self.empty_external.std.tolist(),
            "reference_pose": self.reference_pose.tolist(),
            "reference_posj": (
                self.reference_posj.tolist()
                if self.reference_posj is not None
                else None
            ),

            "center_external_mean": (
                self.center_external.mean.tolist()
                if self.center_external is not None
                else None
            ),
            "center_external_std": (
                self.center_external.std.tolist()
                if self.center_external is not None
                else None
            ),
            "noise_sigma": (
                self.noise_sigma.tolist()
                if self.noise_sigma is not None
                else None
            ),
            "object_load_delta_norm_nm": self.object_load_delta_norm_nm,
            "center_threshold_z": self.center_threshold_z,

            # Tool wrench(get_tool_force) 기반, 이론상 조인트 자세 무관 모델.
            "empty_tool_force_mean": (
                self.empty_tool_force.mean.tolist()
                if self.empty_tool_force is not None
                else None
            ),
            "empty_tool_force_std": (
                self.empty_tool_force.std.tolist()
                if self.empty_tool_force is not None
                else None
            ),
            "center_tool_force_mean": (
                self.center_tool_force.mean.tolist()
                if self.center_tool_force is not None
                else None
            ),
            "center_tool_force_std": (
                self.center_tool_force.std.tolist()
                if self.center_tool_force is not None
                else None
            ),
            "wrench_noise_sigma": (
                self.wrench_noise_sigma.tolist()
                if self.wrench_noise_sigma is not None
                else None
            ),

            "feature_topic": FEATURE_TOPIC,
            "status_topic": STATUS_TOPIC,
            "command_topic": COMMAND_TOPIC,

            "direction_points": {},
            "position_model": None,
            "wrench_position_model": None,
        }

        for name, point in self.direction_points.items():
            entry = {
                "y_mm": point["y_mm"],
                "z_mm": point["z_mm"],
                "description": point["description"],
                "external_mean": point["external"].mean.tolist(),
                "external_std": point["external"].std.tolist(),
                "center_error": point["center_error"].tolist(),
                "center_error_z": point["center_error_z"].tolist(),
                "center_error_z_norm": point["center_error_z_norm"],
            }

            if "tool_force" in point:
                entry["tool_force_mean"] = point["tool_force"].mean.tolist()
                entry["tool_force_std"] = point["tool_force"].std.tolist()

            data["direction_points"][name] = entry

        if self.position_model is not None:
            data["position_model"] = {
                "coefficients": self.position_model.coefficients.tolist(),
                "calibration_rmse_mm": (
                    self.position_model.calibration_rmse_mm
                ),
                "condition_number": self.position_model.condition_number,
                "edge_reference_z": self.position_model.edge_reference_z,
                "calibration_points": (
                    self.position_model.calibration_points
                ),
            }

        if self.wrench_position_model is not None:
            data["wrench_position_model"] = {
                "coefficients": (
                    self.wrench_position_model.coefficients.tolist()
                ),
                "calibration_rmse_mm": (
                    self.wrench_position_model.calibration_rmse_mm
                ),
                "condition_number": (
                    self.wrench_position_model.condition_number
                ),
                "edge_reference_z": (
                    self.wrench_position_model.edge_reference_z
                ),
                "calibration_points": (
                    self.wrench_position_model.calibration_points
                ),
            }

        self.model_path.write_text(
            json.dumps(
                data,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

    # -------------------------------------------------------------------------
    # 캘리브레이션 프리셋 저장/불러오기
    # -------------------------------------------------------------------------

    def save_preset(self, name: str) -> None:
        """빈 판/중심(+선택적 방향/모델) 캘리브레이션 상태를 통째로 저장한다.
        save_model()과 달리 joint 측정값과 원시 샘플까지 포함해서, 나중에
        불러오면 바로 모니터를 재개할 수 있다."""
        if self.empty_external is None or self.empty_joint is None:
            raise RuntimeError("먼저 빈 판 기준을 측정하십시오.")
        if (
            self.center_external is None
            or self.center_joint is None
            or self.noise_sigma is None
        ):
            raise RuntimeError("먼저 중심 기준을 측정하십시오.")

        safe_name = "".join(
            ch if (ch.isalnum() or ch in ("-", "_")) else "_"
            for ch in name.strip()
        ).strip("_")
        if not safe_name:
            safe_name = datetime.now().strftime("preset_%Y%m%d_%H%M%S")

        CALI_PRESET_DIR.mkdir(parents=True, exist_ok=True)
        path = CALI_PRESET_DIR / f"{safe_name}.json"

        data: Dict[str, Any] = {
            "format_version": 3,  # v3: tool_force(wrench) 캘리브레이션 추가
            "created_at": datetime.now().isoformat(),
            "robot_id": ROBOT_ID,
            "robot_model": ROBOT_MODEL,
            "gripper_model": GRIPPER_MODEL,
            "plate_radius_mm": PLATE_RADIUS_MM,
            "calibration_radius_mm": CALIBRATION_RADIUS_MM,

            "empty_external": measurement_to_dict(self.empty_external),
            "empty_joint": measurement_to_dict(self.empty_joint),
            "center_external": measurement_to_dict(self.center_external),
            "center_joint": measurement_to_dict(self.center_joint),

            "reference_pose": self.reference_pose.tolist(),
            "reference_posj": (
                self.reference_posj.tolist()
                if self.reference_posj is not None
                else None
            ),
            "noise_sigma": self.noise_sigma.tolist(),
            "center_threshold_z": self.center_threshold_z,
            "object_load_delta_norm_nm": self.object_load_delta_norm_nm,

            # v3: tool wrench(get_tool_force) 기반, 조인트 자세 무관 모델용 데이터.
            "empty_tool_force": (
                measurement_to_dict(self.empty_tool_force)
                if self.empty_tool_force is not None
                else None
            ),
            "center_tool_force": (
                measurement_to_dict(self.center_tool_force)
                if self.center_tool_force is not None
                else None
            ),
            "wrench_noise_sigma": (
                self.wrench_noise_sigma.tolist()
                if self.wrench_noise_sigma is not None
                else None
            ),

            "direction_points": {},
            "position_model": None,
            "wrench_position_model": None,
        }

        for point_name, point in self.direction_points.items():
            entry = {
                "name": point["name"],
                "y_mm": point["y_mm"],
                "z_mm": point["z_mm"],
                "description": point["description"],
                "external": measurement_to_dict(point["external"]),
                "joint": measurement_to_dict(point["joint"]),
                "center_error": point["center_error"].tolist(),
                "center_error_z": point["center_error_z"].tolist(),
                "center_error_z_norm": point["center_error_z_norm"],
            }

            if "tool_force" in point:
                entry["tool_force"] = measurement_to_dict(point["tool_force"])

            data["direction_points"][point_name] = entry

        if self.position_model is not None:
            data["position_model"] = {
                "coefficients": self.position_model.coefficients.tolist(),
                "calibration_rmse_mm": self.position_model.calibration_rmse_mm,
                "condition_number": self.position_model.condition_number,
                "edge_reference_z": self.position_model.edge_reference_z,
                "calibration_points": self.position_model.calibration_points,
            }

        if self.wrench_position_model is not None:
            data["wrench_position_model"] = {
                "coefficients": (
                    self.wrench_position_model.coefficients.tolist()
                ),
                "calibration_rmse_mm": (
                    self.wrench_position_model.calibration_rmse_mm
                ),
                "condition_number": (
                    self.wrench_position_model.condition_number
                ),
                "edge_reference_z": (
                    self.wrench_position_model.edge_reference_z
                ),
                "calibration_points": (
                    self.wrench_position_model.calibration_points
                ),
            }

        path.write_text(
            json.dumps(data, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        self.send_log(f"캘리 프리셋 저장 완료: {path}")
        self.send_ui("preset_saved", path=str(path), name=safe_name)
        self.send_reference_state()

    def load_preset(self, path_str: str) -> None:
        path = Path(path_str)
        if not path.is_file():
            raise RuntimeError(f"프리셋 파일이 없습니다: {path}")

        data = json.loads(path.read_text(encoding="utf-8"))

        self.monitor_enabled = False

        self.empty_external = measurement_from_dict(data["empty_external"])
        self.empty_joint = measurement_from_dict(data["empty_joint"])
        self.center_external = measurement_from_dict(data["center_external"])
        self.center_joint = measurement_from_dict(data["center_joint"])

        self.reference_pose = np.asarray(data["reference_pose"], dtype=float)

        # v1 프리셋(2026-07-21 이전 저장분)엔 reference_posj가 없다. 없으면
        # None으로 두고, "저장된 joint로 이동" 버튼은 비활성 상태로 남는다.
        raw_reference_posj = data.get("reference_posj")
        self.reference_posj = (
            np.asarray(raw_reference_posj, dtype=float)
            if raw_reference_posj is not None
            else None
        )

        self.noise_sigma = np.asarray(data["noise_sigma"], dtype=float)
        self.center_threshold_z = float(data["center_threshold_z"])
        self.object_load_delta_norm_nm = float(
            data["object_load_delta_norm_nm"]
        )

        # v3 이전 프리셋(2026-07-22 이전 저장분)엔 tool_force 관련 키가 없다.
        # 없으면 None으로 두고, 위치 모델 생성 시 wrench 모델은 건너뛴다
        # (방향 보정을 다시 캡처하면 그때부터 생성됨).
        raw_empty_tool_force = data.get("empty_tool_force")
        self.empty_tool_force = (
            measurement_from_dict(raw_empty_tool_force)
            if raw_empty_tool_force is not None
            else None
        )
        raw_center_tool_force = data.get("center_tool_force")
        self.center_tool_force = (
            measurement_from_dict(raw_center_tool_force)
            if raw_center_tool_force is not None
            else None
        )
        raw_wrench_noise_sigma = data.get("wrench_noise_sigma")
        self.wrench_noise_sigma = (
            np.asarray(raw_wrench_noise_sigma, dtype=float)
            if raw_wrench_noise_sigma is not None
            else None
        )

        self.direction_points.clear()
        for point_name, point in data.get("direction_points", {}).items():
            entry = {
                "name": point["name"],
                "y_mm": point["y_mm"],
                "z_mm": point["z_mm"],
                "description": point["description"],
                "external": measurement_from_dict(point["external"]),
                "joint": measurement_from_dict(point["joint"]),
                "center_error": np.asarray(
                    point["center_error"], dtype=float
                ),
                "center_error_z": np.asarray(
                    point["center_error_z"], dtype=float
                ),
                "center_error_z_norm": float(point["center_error_z_norm"]),
            }

            raw_tool_force = point.get("tool_force")
            if raw_tool_force is not None:
                entry["tool_force"] = measurement_from_dict(raw_tool_force)

            self.direction_points[point_name] = entry

        position_model = data.get("position_model")
        if position_model is not None:
            self.position_model = PositionModel(
                coefficients=np.asarray(
                    position_model["coefficients"], dtype=float
                ),
                calibration_rmse_mm=float(
                    position_model["calibration_rmse_mm"]
                ),
                calibration_points=position_model["calibration_points"],
                edge_reference_z=float(position_model["edge_reference_z"]),
                condition_number=float(position_model["condition_number"]),
            )
        else:
            self.position_model = None

        wrench_position_model = data.get("wrench_position_model")
        if wrench_position_model is not None:
            self.wrench_position_model = PositionModel(
                coefficients=np.asarray(
                    wrench_position_model["coefficients"], dtype=float
                ),
                calibration_rmse_mm=float(
                    wrench_position_model["calibration_rmse_mm"]
                ),
                calibration_points=wrench_position_model[
                    "calibration_points"
                ],
                edge_reference_z=float(
                    wrench_position_model["edge_reference_z"]
                ),
                condition_number=float(
                    wrench_position_model["condition_number"]
                ),
            )
        else:
            self.wrench_position_model = None

        self.reset_filters()
        self.center_latched = True
        self.center_enter_count = 0
        self.center_exit_count = 0

        self.send_log(
            f"캘리 프리셋 불러오기 완료: {path.name} · "
            f"방향보정 {len(self.direction_points)}개 · "
            f"위치모델 {'있음' if self.position_model is not None else '없음'} · "
            f"wrench모델 {'있음' if self.wrench_position_model is not None else '없음'} · "
            f"기준 joint {'있음' if self.reference_posj is not None else '없음(구버전 프리셋)'}"
        )
        self.send_ui("preset_loaded", path=str(path), name=path.stem)
        self.send_reference_state()

    # -------------------------------------------------------------------------
    # 저장된 joint로 이동 (이 파일에서 유일하게 실제 모션 명령을 보내는 지점)
    # -------------------------------------------------------------------------

    def move_to_reference_posj(self) -> None:
        if self.reference_posj is None:
            raise RuntimeError(
                "이 캘리브레이션엔 기준 joint 정보가 없습니다. "
                "구버전 프리셋이거나 아직 빈 판 측정을 하지 않았습니다."
            )

        was_monitoring = self.monitor_enabled
        self.monitor_enabled = False

        target = self.reference_posj.tolist()

        self.send_log(
            "기준 joint로 이동 시작 -> "
            + ", ".join(f"{v:.2f}" for v in target)
            + f" (vel={REFERENCE_MOVE_VEL_DEG_S:.1f} deg/s)"
        )
        self.send_ui("move_started")

        try:
            self.dsr.movej(
                target,
                vel=REFERENCE_MOVE_VEL_DEG_S,
                acc=REFERENCE_MOVE_ACC_DEG_S2,
                mod=self.dsr.DR_MV_MOD_ABS,
            )
        except Exception as error:
            self.send_ui(
                "error",
                message=f"기준 joint 이동 실패: {error}",
                traceback=traceback.format_exc(),
            )
            self.send_ui("move_finished", ok=False)
            self.send_reference_state()
            return

        self.send_log("기준 joint 이동 완료")
        self.send_ui("move_finished", ok=True)

        if was_monitoring:
            self.reset_filters()
            self.monitor_enabled = True
            self.send_log("실시간 모니터 재개")

        self.send_reference_state()

    # -------------------------------------------------------------------------
    # 명령 처리
    # -------------------------------------------------------------------------

    def handle_command(self, command: Tuple[str, Any]) -> None:
        name, payload = command

        if name == "capture_empty":
            self.capture_empty()

        elif name == "capture_center":
            self.capture_center()

        elif name == "capture_direction":
            self.capture_direction(**payload)

        elif name == "build_model":
            self.fit_position_model()

        elif name == "save_preset":
            self.save_preset(payload)

        elif name == "load_preset":
            self.load_preset(payload)

        elif name == "move_to_reference_posj":
            self.move_to_reference_posj()

        elif name == "start_monitor":
            if self.center_external is None:
                raise RuntimeError("먼저 중심 기준을 측정하십시오.")

            self.open_logs()
            self.reset_filters()
            self.monitor_enabled = True
            self.send_log("실시간 모니터 시작")
            self.send_reference_state()

        elif name == "pause_monitor":
            self.monitor_enabled = False
            self.send_log("실시간 모니터 일시정지")
            self.send_reference_state()

        elif name == "set_label":
            self.current_label = (
                str(payload).strip()
                or "unlabeled"
            )
            self.send_log(
                f"실험 라벨 변경: {self.current_label}"
            )

        elif name == "set_param":
            self.set_param(
                payload["name"],
                payload["value"],
            )

        elif name == "shutdown":
            self.running = False

        else:
            raise RuntimeError(
                f"알 수 없는 명령: {name}"
            )

    def set_param(
        self,
        name: str,
        value: float,
    ) -> None:
        if name not in PREPROCESS_PARAM_NAMES:
            raise RuntimeError(
                f"알 수 없는 전처리 파라미터: {name}"
            )

        if name == "median_prefilter_window":
            window = max(
                1,
                int(round(value)),
            )
            self.median_prefilter_window = window
            # maxlen은 생성 후 변경할 수 없어 기존 샘플을 유지한 채
            # deque를 새로 만든다(창을 줄이면 오래된 샘플부터 잘려나간다).
            self.raw_external_history = deque(
                self.raw_external_history,
                maxlen=window,
            )
        else:
            setattr(
                self,
                name,
                float(value),
            )

    # -------------------------------------------------------------------------
    # 메인 백엔드 루프
    # -------------------------------------------------------------------------

    def run(self) -> None:
        self.send_log(
            "ROS 백엔드 시작. 로봇 모션 명령은 전송하지 않습니다."
        )
        self.send_reference_state()

        try:
            while self.running and rclpy.ok():
                loop_start = time.monotonic()

                rclpy.spin_once(
                    self.node,
                    timeout_sec=0.0,
                )

                try:
                    command = self.command_queue.get_nowait()
                except queue.Empty:
                    command = None

                if command is not None:
                    try:
                        self.handle_command(command)
                    except Exception as error:
                        self.send_ui(
                            "error",
                            message=str(error),
                            traceback=traceback.format_exc(),
                        )
                        self.send_reference_state()

                if (
                    self.monitor_enabled
                    and self.center_external is not None
                    and self.noise_sigma is not None
                ):
                    try:
                        api_start = time.monotonic()
                        raw = self.read_robot_state()
                        api_elapsed_ms = (
                            (time.monotonic() - api_start)
                            * 1000.0
                        )
                        self.filtered_loop_api_ms = (
                            api_elapsed_ms
                            if self.filtered_loop_api_ms is None
                            else (
                                0.1 * api_elapsed_ms
                                + 0.9 * self.filtered_loop_api_ms
                            )
                        )

                        state = self.build_state(
                            raw,
                            loop_start,
                        )
                        state["loop_api_ms"] = api_elapsed_ms

                        self.publish_features(state)
                        self.publish_status(state)
                        self.write_csv(state)

                        self.send_ui(
                            "state",
                            state=self.serialize_state(state),
                        )

                        work_elapsed_ms = (
                            (time.monotonic() - loop_start)
                            * 1000.0
                        )
                        self.filtered_loop_work_ms = (
                            work_elapsed_ms
                            if self.filtered_loop_work_ms is None
                            else (
                                0.1 * work_elapsed_ms
                                + 0.9 * self.filtered_loop_work_ms
                            )
                        )

                        if (
                            self.filtered_loop_hz is not None
                            and self.filtered_loop_work_ms is not None
                            and (
                                self.last_hz_log_monotonic is None
                                or loop_start
                                - self.last_hz_log_monotonic
                                >= 5.0
                            )
                        ):
                            self.last_hz_log_monotonic = loop_start
                            max_possible_hz = (
                                1000.0 / self.filtered_loop_work_ms
                                if self.filtered_loop_work_ms > 0.0
                                else math.nan
                            )
                            self.send_log(
                                f"실측 Hz: {self.filtered_loop_hz:.1f} "
                                f"(목표 {SAMPLE_HZ:.0f}) · "
                                f"로봇 API {self.filtered_loop_api_ms:.1f} ms · "
                                f"루프 전체 {self.filtered_loop_work_ms:.1f} ms · "
                                f"이론상 최대 ≈ {max_possible_hz:.0f} Hz"
                            )

                    except Exception as error:
                        self.monitor_enabled = False
                        self.send_ui(
                            "error",
                            message=(
                                "실시간 모니터 오류: "
                                + str(error)
                            ),
                            traceback=traceback.format_exc(),
                        )
                        self.send_reference_state()

                remaining = (
                    SAMPLE_PERIOD_SEC
                    - (time.monotonic() - loop_start)
                )

                if remaining > 0.0:
                    time.sleep(remaining)

        finally:
            self.monitor_enabled = False

            if self.csv_file is not None:
                self.csv_file.flush()
                self.csv_file.close()

            try:
                self.save_model()
            except Exception:
                pass

            self.send_ui("backend_stopped")

    @staticmethod
    def serialize_state(
        state: Dict[str, Any],
    ) -> Dict[str, Any]:
        output: Dict[str, Any] = {}

        for key, value in state.items():
            if isinstance(value, np.ndarray):
                output[key] = value.tolist()
            elif isinstance(value, np.floating):
                output[key] = float(value)
            elif isinstance(value, np.integer):
                output[key] = int(value)
            else:
                output[key] = value

        return output


# =============================================================================
# GUI 색상 팔레트 (다크, 저채도 — 너무 밝지 않은 깔끔한 톤)
# =============================================================================

THEME_BG = "#0f172a"          # 창 배경
THEME_PANEL = "#111827"       # 캔버스/패널 배경
THEME_CARD = "#1e293b"        # 프레임/입력창/버튼 배경
THEME_CARD_ALT = "#243043"    # 카드 위 강조 요소
THEME_BORDER = "#334155"      # 테두리/구분선
THEME_TEXT = "#e2e8f0"        # 기본 텍스트
THEME_TEXT_MUTED = "#94a3b8"  # 보조 텍스트
THEME_ACCENT = "#38bdf8"      # 주요 강조색(하늘색)
THEME_ACCENT_ACTIVE = "#0ea5e9"
THEME_SUCCESS = "#10b981"
THEME_WARNING = "#f59e0b"
THEME_DANGER = "#ef4444"
THEME_DANGER_DARK = "#7f1d1d"
THEME_DANGER_DARK_ACTIVE = "#991b1b"


# =============================================================================
# GUI
# =============================================================================

class PlatePositionGUI:
    def __init__(
        self,
        root: tk.Tk,
        command_queue: queue.Queue,
        ui_queue: queue.Queue,
    ) -> None:
        self.root = root
        self.command_queue = command_queue
        self.ui_queue = ui_queue

        self.root.title(
            "M0609 Plate Center Monitor"
        )
        self.root.geometry("1480x900")
        self.root.minsize(1250, 760)

        self.busy = False
        self.empty_ready = False
        self.center_ready = False
        self.model_ready = False
        self.monitor_enabled = False
        self.reference_posj_ready = False
        self.direction_points: set[str] = set()

        self.latest_state: Optional[
            Dict[str, Any]
        ] = None
        self.latest_legacy_state: Optional[
            Dict[str, Any]
        ] = None

        self.csv_path = ""
        self.model_path = ""

        # on_close() 이후 root.destroy() 예약 시간(200ms) 안에 poll_ui_queue가
        # 한 번 더 스스로를 재예약하면, destroy 이후 시점에 실행돼 이미 파괴된
        # Tk 위젯을 건드리다 "Tcl_Release couldn't find reference" 같은 크래시가
        # 날 수 있다. 이 플래그로 종료 시작과 동시에 재예약을 끊는다.
        self.closing = False

        self.metric_variables: Dict[
            str,
            tk.StringVar,
        ] = {}

        self.axis_value_labels: List[
            ttk.Label
        ] = []

        self.direction_buttons: Dict[
            str,
            ttk.Button,
        ] = {}

        self.build_styles()
        self.build_layout()
        self.update_button_states()

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.on_close,
        )

        self.root.after(
            GUI_POLL_MS,
            self.poll_ui_queue,
        )

    # -------------------------------------------------------------------------
    # 스타일
    # -------------------------------------------------------------------------

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
        style.configure(
            "TFrame",
            background=THEME_BG,
        )
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
            "TPanedwindow",
            background=THEME_BG,
        )
        style.configure(
            "TSeparator",
            background=THEME_BORDER,
        )
        style.configure(
            "TEntry",
            fieldbackground=THEME_CARD,
            foreground=THEME_TEXT,
            insertcolor=THEME_TEXT,
            bordercolor=THEME_BORDER,
        )
        style.configure(
            "TScrollbar",
            background=THEME_CARD,
            troughcolor=THEME_PANEL,
            bordercolor=THEME_BORDER,
            arrowcolor=THEME_TEXT_MUTED,
        )
        style.configure(
            "Horizontal.TScale",
            background=THEME_BG,
            troughcolor=THEME_PANEL,
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
            "Title.TLabel",
            font=("Sans", 18, "bold"),
            foreground=THEME_ACCENT,
        )
        style.configure(
            "Section.TLabel",
            font=("Sans", 12, "bold"),
            foreground=THEME_TEXT,
        )
        style.configure(
            "MetricName.TLabel",
            font=("Sans", 10),
            foreground=THEME_TEXT_MUTED,
        )
        style.configure(
            "MetricValue.TLabel",
            font=("Monospace", 13, "bold"),
            foreground=THEME_ACCENT,
        )
        style.configure(
            "Status.TLabel",
            font=("Sans", 17, "bold"),
            anchor="center",
            foreground=THEME_ACCENT,
        )
        style.configure(
            "Big.TButton",
            font=("Sans", 11, "bold"),
            padding=(8, 8),
        )
        style.map(
            "Big.TButton",
            background=[
                ("disabled", THEME_PANEL),
                ("pressed", THEME_ACCENT_ACTIVE),
                ("active", THEME_ACCENT),
            ],
            foreground=[
                ("disabled", THEME_TEXT_MUTED),
                ("active", "#0b1120"),
            ],
        )

    # -------------------------------------------------------------------------
    # 레이아웃
    # -------------------------------------------------------------------------

    def build_layout(self) -> None:
        self.root.columnconfigure(
            0,
            weight=1,
        )
        self.root.rowconfigure(
            1,
            weight=1,
        )

        header = ttk.Frame(
            self.root,
            padding=10,
        )
        header.grid(
            row=0,
            column=0,
            sticky="ew",
        )
        header.columnconfigure(
            1,
            weight=1,
        )

        ttk.Label(
            header,
            text="M0609 판 위 물체 중심 이탈 GUI",
            style="Title.TLabel",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.connection_var = tk.StringVar(
            value="ROS 백엔드 초기화 중"
        )
        ttk.Label(
            header,
            textvariable=self.connection_var,
        ).grid(
            row=0,
            column=1,
            sticky="e",
        )

        main = ttk.Panedwindow(
            self.root,
            orient=tk.HORIZONTAL,
        )
        main.grid(
            row=1,
            column=0,
            sticky="nsew",
            padx=10,
            pady=(0, 10),
        )

        left = ttk.Frame(
            main,
            padding=8,
        )
        right = ttk.Frame(
            main,
            padding=8,
        )

        main.add(
            left,
            weight=3,
        )
        main.add(
            right,
            weight=2,
        )

        left.columnconfigure(
            0,
            weight=1,
        )
        left.rowconfigure(
            1,
            weight=3,
        )
        left.rowconfigure(
            3,
            weight=2,
        )

        right.columnconfigure(
            0,
            weight=1,
        )
        right.rowconfigure(
            4,
            weight=1,
        )

        # 왼쪽: 상태
        self.status_var = tk.StringVar(
            value="기준 측정 필요"
        )
        self.status_label = ttk.Label(
            left,
            textvariable=self.status_var,
            style="Status.TLabel",
        )
        self.status_label.grid(
            row=0,
            column=0,
            sticky="ew",
            pady=(0, 6),
        )

        # 판 표시 (Original / New 나란히 비교)
        plate_frame = ttk.LabelFrame(
            left,
            text="판 위 위치 — Original vs New(전처리 적용)",
            padding=6,
        )
        plate_frame.grid(
            row=1,
            column=0,
            sticky="nsew",
        )
        plate_frame.rowconfigure(
            1,
            weight=1,
        )
        plate_frame.columnconfigure(
            0,
            weight=1,
        )
        plate_frame.columnconfigure(
            1,
            weight=1,
        )

        ttk.Label(
            plate_frame,
            text="Original (고정 alpha EMA)",
            style="MetricName.TLabel",
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )
        ttk.Label(
            plate_frame,
            text="New (이동중앙값 + 시간상수 EMA)",
            style="MetricName.TLabel",
        ).grid(
            row=0,
            column=1,
            sticky="w",
        )

        self.plate_canvas_original = tk.Canvas(
            plate_frame,
            background=THEME_PANEL,
            highlightthickness=0,
        )
        self.plate_canvas_original.grid(
            row=1,
            column=0,
            sticky="nsew",
            padx=(0, 4),
        )
        self.plate_canvas_original.bind(
            "<Configure>",
            lambda _event: self.draw_plate(),
        )

        self.plate_canvas_new = tk.Canvas(
            plate_frame,
            background=THEME_PANEL,
            highlightthickness=0,
        )
        self.plate_canvas_new.grid(
            row=1,
            column=1,
            sticky="nsew",
            padx=(4, 0),
        )
        self.plate_canvas_new.bind(
            "<Configure>",
            lambda _event: self.draw_plate(),
        )

        # 주요 지표
        metric_frame = ttk.LabelFrame(
            left,
            text="중심 이탈 지표",
            padding=8,
        )
        metric_frame.grid(
            row=2,
            column=0,
            sticky="ew",
            pady=(8, 0),
        )

        metric_names = [
            ("ERR", "center_error_nm", "Nm"),
            ("Z", "center_error_z", ""),
            ("DEV", "deviation_index", ""),
            ("TREND", "trend_z_per_sec", "/s"),
            ("EDGE", "edge_index", ""),
            ("DOM", "dominant", ""),
        ]

        for column, (
            title,
            key,
            unit,
        ) in enumerate(metric_names):
            metric_frame.columnconfigure(
                column,
                weight=1,
            )

            ttk.Label(
                metric_frame,
                text=title,
                style="MetricName.TLabel",
            ).grid(
                row=0,
                column=column,
            )

            variable = tk.StringVar(
                value="--"
            )
            self.metric_variables[key] = variable

            ttk.Label(
                metric_frame,
                textvariable=variable,
                style="MetricValue.TLabel",
            ).grid(
                row=1,
                column=column,
                padx=6,
            )

            if unit:
                ttk.Label(
                    metric_frame,
                    text=unit,
                ).grid(
                    row=2,
                    column=column,
                )

        # 전처리 파라미터 (실시간 조정)
        self.build_preprocess_panel(left, row=3)

        # 오른쪽: 기준 측정
        control_frame = ttk.LabelFrame(
            right,
            text="측정 및 제어",
            padding=8,
        )
        control_frame.grid(
            row=0,
            column=0,
            sticky="ew",
        )
        control_frame.columnconfigure(
            0,
            weight=1,
        )
        control_frame.columnconfigure(
            1,
            weight=1,
        )

        self.empty_button = ttk.Button(
            control_frame,
            text="1. 빈 판 측정",
            style="Big.TButton",
            command=self.request_empty_capture,
        )
        self.empty_button.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
            pady=4,
        )

        self.center_button = ttk.Button(
            control_frame,
            text="2. 중심 기준 측정",
            style="Big.TButton",
            command=self.request_center_capture,
        )
        self.center_button.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(4, 0),
            pady=4,
        )

        ttk.Separator(
            control_frame,
            orient=tk.HORIZONTAL,
        ).grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=6,
        )

        ttk.Label(
            control_frame,
            text=(
                f"위치 보정 (중심에서 {CALIBRATION_RADIUS_MM:.0f} mm)\n"
                f"축 4방향 필수 · 대각 4방향은 Z 정확도 향상 권장\n"
                f"아래 원판에서 실제 위치에 해당하는 버튼을 누르십시오"
            ),
            style="Section.TLabel",
        ).grid(
            row=2,
            column=0,
            columnspan=2,
            pady=(0, 5),
        )

        direction_definitions = [
            (
                "Y_PLUS",
                "+Y",
                +CALIBRATION_RADIUS_MM,
                0.0,
                "중심에서 Tool +Y 방향, 60 mm",
            ),
            (
                "Y_MINUS",
                "-Y",
                -CALIBRATION_RADIUS_MM,
                0.0,
                "중심에서 Tool -Y 방향, 60 mm",
            ),
            (
                "Z_PLUS",
                "+Z",
                0.0,
                +CALIBRATION_RADIUS_MM,
                "중심에서 Tool +Z 방향, 60 mm",
            ),
            (
                "Z_MINUS",
                "-Z",
                0.0,
                -CALIBRATION_RADIUS_MM,
                "중심에서 Tool -Z 방향, 60 mm",
            ),
            # 대각선 4점(반경 60mm, 45°). 각 성분 = 60/√2 ≈ 42.4mm.
            # Y/Z 커플링을 분리해 특히 Z 정확도를 높이기 위한 추가(선택) 점.
            # 모두 반경 60mm라 그리퍼(Z=-90mm)와 안전하게 떨어져 있다.
            (
                "Y_PLUS_Z_PLUS",
                "+Y\n+Z",
                +CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                +CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                "중심에서 Tool +Y,+Z 대각, 60 mm",
            ),
            (
                "Y_PLUS_Z_MINUS",
                "+Y\n-Z",
                +CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                -CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                "중심에서 Tool +Y,-Z 대각, 60 mm",
            ),
            (
                "Y_MINUS_Z_PLUS",
                "-Y\n+Z",
                -CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                +CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                "중심에서 Tool -Y,+Z 대각, 60 mm",
            ),
            (
                "Y_MINUS_Z_MINUS",
                "-Y\n-Z",
                -CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                -CALIBRATION_RADIUS_MM / math.sqrt(2.0),
                "중심에서 Tool -Y,-Z 대각, 60 mm",
            ),
        ]

        self.build_direction_pad(
            control_frame,
            direction_definitions,
        )

        self.model_button = ttk.Button(
            control_frame,
            text="위치 모델 생성",
            style="Big.TButton",
            command=self.request_build_model,
        )
        self.model_button.grid(
            row=7,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(6, 3),
        )

        ttk.Separator(
            control_frame,
            orient=tk.HORIZONTAL,
        ).grid(
            row=8,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=6,
        )

        ttk.Label(
            control_frame,
            text="캘리브레이션 프리셋 (rokey/cali_preset)",
            style="Section.TLabel",
        ).grid(
            row=9,
            column=0,
            columnspan=2,
            pady=(0, 3),
        )

        self.save_preset_button = ttk.Button(
            control_frame,
            text="캘리 저장",
            style="Big.TButton",
            command=self.request_save_preset,
        )
        self.save_preset_button.grid(
            row=10,
            column=0,
            sticky="ew",
            padx=(0, 4),
            pady=4,
        )

        self.load_preset_button = ttk.Button(
            control_frame,
            text="캘리 불러오기",
            style="Big.TButton",
            command=self.request_load_preset,
        )
        self.load_preset_button.grid(
            row=10,
            column=1,
            sticky="ew",
            padx=(4, 0),
            pady=4,
        )

        # "저장된 joint로 이동" — 이 GUI에서 유일하게 실제 로봇을 움직이는
        # 버튼이다(monitor_v2는 원래 무모션 관찰 도구). 실수 클릭 방지를 위해
        # 별도 색상 + 확인창을 둔다.
        self.move_to_posj_button = tk.Button(
            control_frame,
            text="⚠ 저장된 joint로 이동",
            font=("Sans", 11, "bold"),
            background=THEME_DANGER_DARK,
            foreground="#ffffff",
            activebackground=THEME_DANGER_DARK_ACTIVE,
            activeforeground="#ffffff",
            relief="flat",
            bd=0,
            command=self.request_move_to_reference_posj,
        )
        self.move_to_posj_button.grid(
            row=11,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(4, 4),
            ipady=4,
        )

        ttk.Separator(
            control_frame,
            orient=tk.HORIZONTAL,
        ).grid(
            row=12,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=6,
        )

        self.start_button = ttk.Button(
            control_frame,
            text="모니터 시작",
            style="Big.TButton",
            command=self.request_start_monitor,
        )
        self.start_button.grid(
            row=13,
            column=0,
            sticky="ew",
            padx=(0, 4),
            pady=4,
        )

        self.pause_button = ttk.Button(
            control_frame,
            text="일시정지",
            style="Big.TButton",
            command=self.request_pause_monitor,
        )
        self.pause_button.grid(
            row=13,
            column=1,
            sticky="ew",
            padx=(4, 0),
            pady=4,
        )

        # 진행률
        progress_frame = ttk.LabelFrame(
            right,
            text="측정 진행",
            padding=8,
        )
        progress_frame.grid(
            row=1,
            column=0,
            sticky="ew",
            pady=(8, 0),
        )
        progress_frame.columnconfigure(
            0,
            weight=1,
        )

        self.progress_label_var = tk.StringVar(
            value="대기"
        )
        ttk.Label(
            progress_frame,
            textvariable=self.progress_label_var,
        ).grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.progress = ttk.Progressbar(
            progress_frame,
            orient=tk.HORIZONTAL,
            mode="determinate",
            maximum=100.0,
        )
        self.progress.grid(
            row=1,
            column=0,
            sticky="ew",
            pady=(4, 0),
        )

        # J1~J6
        axis_frame = ttk.LabelFrame(
            right,
            text="중심 대비 external torque — 축별 z-score (New 전처리 기준)",
            padding=8,
        )
        axis_frame.grid(
            row=2,
            column=0,
            sticky="ew",
            pady=(8, 0),
        )
        axis_frame.columnconfigure(
            1,
            weight=1,
        )

        self.axis_canvases: List[
            tk.Canvas
        ] = []

        for index in range(6):
            ttk.Label(
                axis_frame,
                text=f"J{index + 1}",
                width=3,
            ).grid(
                row=index,
                column=0,
                sticky="w",
            )

            canvas = tk.Canvas(
                axis_frame,
                height=22,
                background=THEME_PANEL,
                highlightthickness=1,
                highlightbackground=THEME_BORDER,
            )
            canvas.grid(
                row=index,
                column=1,
                sticky="ew",
                padx=4,
                pady=2,
            )
            canvas.bind(
                "<Configure>",
                lambda _event: self.draw_axis_bars(),
            )
            self.axis_canvases.append(
                canvas
            )

            label = ttk.Label(
                axis_frame,
                text="+0.00 z / +0.000 Nm",
                width=22,
                anchor="e",
            )
            label.grid(
                row=index,
                column=2,
                sticky="e",
            )
            self.axis_value_labels.append(
                label
            )

        # 세부 상태
        detail_frame = ttk.LabelFrame(
            right,
            text="상태",
            padding=8,
        )
        detail_frame.grid(
            row=3,
            column=0,
            sticky="ew",
            pady=(8, 0),
        )
        detail_frame.columnconfigure(
            1,
            weight=1,
        )

        detail_items = [
            ("위치", "position"),
            ("위치 (tool wrench, 자세 무관)", "wrench_position"),
            ("복귀 힌트", "hint"),
            ("물체", "object"),
            ("로봇 정지", "stationary"),
            ("자세 유지", "pose"),
            ("실측 Hz", "achieved_hz"),
            ("실험 라벨", "label"),
            ("CSV", "csv"),
        ]

        for row, (
            title,
            key,
        ) in enumerate(detail_items):
            ttk.Label(
                detail_frame,
                text=title,
            ).grid(
                row=row,
                column=0,
                sticky="nw",
                padx=(0, 8),
                pady=2,
            )

            variable = tk.StringVar(
                value="--"
            )
            self.metric_variables[key] = variable

            ttk.Label(
                detail_frame,
                textvariable=variable,
                wraplength=500,
            ).grid(
                row=row,
                column=1,
                sticky="w",
                pady=2,
            )

        label_entry_frame = ttk.Frame(
            detail_frame,
        )
        label_entry_frame.grid(
            row=len(detail_items),
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(6, 0),
        )
        label_entry_frame.columnconfigure(
            0,
            weight=1,
        )

        self.label_entry_var = tk.StringVar(
            value="unlabeled"
        )
        ttk.Entry(
            label_entry_frame,
            textvariable=self.label_entry_var,
        ).grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
        )
        ttk.Button(
            label_entry_frame,
            text="라벨 적용",
            command=self.apply_label,
        ).grid(
            row=0,
            column=1,
        )

        # 로그
        log_frame = ttk.LabelFrame(
            right,
            text="로그",
            padding=6,
        )
        log_frame.grid(
            row=4,
            column=0,
            sticky="nsew",
            pady=(8, 0),
        )
        log_frame.rowconfigure(
            0,
            weight=1,
        )
        log_frame.columnconfigure(
            0,
            weight=1,
        )

        self.log_text = tk.Text(
            log_frame,
            height=10,
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
        self.log_text.grid(
            row=0,
            column=0,
            sticky="nsew",
        )

        scrollbar = ttk.Scrollbar(
            log_frame,
            orient=tk.VERTICAL,
            command=self.log_text.yview,
        )
        scrollbar.grid(
            row=0,
            column=1,
            sticky="ns",
        )
        self.log_text.configure(
            yscrollcommand=scrollbar.set,
        )

        log_buttons = ttk.Frame(
            log_frame,
        )
        log_buttons.grid(
            row=1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(5, 0),
        )
        log_buttons.columnconfigure(
            0,
            weight=1,
        )
        log_buttons.columnconfigure(
            1,
            weight=1,
        )

        ttk.Button(
            log_buttons,
            text="로그 폴더 열기",
            command=self.open_log_directory,
        ).grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 3),
        )
        ttk.Button(
            log_buttons,
            text="로그 지우기",
            command=self.clear_log,
        ).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(3, 0),
        )

    def build_direction_pad(
        self,
        parent: ttk.Frame,
        direction_definitions: List[
            Tuple[str, str, float, float, str]
        ],
    ) -> None:
        """8방향 보정 버튼을, 목록이 아니라 실제 판 위 위치에 맞춰 원판
        모양으로 배치한다. draw_plate_variant()와 같은 좌표 규약(+Y는
        오른쪽, +Z는 위쪽)을 써서 다른 화면의 위치 표시와 방향이 항상
        일치하게 만든다."""
        pad_size = 250
        pad_center = pad_size / 2.0
        pad_radius_px = pad_size * 0.34
        pad_scale = pad_radius_px / CALIBRATION_RADIUS_MM

        pad_canvas = tk.Canvas(
            parent,
            width=pad_size,
            height=pad_size,
            background=THEME_PANEL,
            highlightthickness=0,
        )
        pad_canvas.grid(
            row=3,
            column=0,
            columnspan=2,
            pady=(0, 6),
        )

        for ratio, dash in (
            (1.0, ()),
            (0.5, (4, 3)),
        ):
            radius = pad_radius_px * ratio
            pad_canvas.create_oval(
                pad_center - radius,
                pad_center - radius,
                pad_center + radius,
                pad_center + radius,
                outline=THEME_BORDER,
                width=2 if ratio == 1.0 else 1,
                dash=dash,
            )

        pad_canvas.create_line(
            pad_center - pad_radius_px,
            pad_center,
            pad_center + pad_radius_px,
            pad_center,
            fill=THEME_BORDER,
        )
        pad_canvas.create_line(
            pad_center,
            pad_center - pad_radius_px,
            pad_center,
            pad_center + pad_radius_px,
            fill=THEME_BORDER,
        )

        for x, y, text, anchor in (
            (pad_center + pad_radius_px + 14, pad_center, "+Y", "w"),
            (pad_center - pad_radius_px - 14, pad_center, "-Y", "e"),
            (pad_center, pad_center - pad_radius_px - 10, "+Z", "s"),
            (pad_center, pad_center + pad_radius_px + 10, "-Z", "n"),
        ):
            pad_canvas.create_text(
                x,
                y,
                text=text,
                fill=THEME_TEXT_MUTED,
                anchor=anchor,
                font=("Sans", 9, "bold"),
            )

        pad_canvas.create_oval(
            pad_center - 4,
            pad_center - 4,
            pad_center + 4,
            pad_center + 4,
            fill=THEME_ACCENT,
            outline="",
        )
        pad_canvas.create_text(
            pad_center,
            pad_center + 14,
            text="중심(2)",
            fill=THEME_TEXT_MUTED,
            justify="center",
            font=("Sans", 7),
        )

        for (
            name,
            text,
            y_mm,
            z_mm,
            description,
        ) in direction_definitions:
            button = tk.Button(
                pad_canvas,
                text=text,
                font=("Sans", 8, "bold"),
                background=THEME_CARD,
                foreground=THEME_TEXT,
                activebackground=THEME_ACCENT,
                activeforeground="#0b1120",
                relief="flat",
                bd=0,
                width=3,
                padx=2,
                pady=1,
                command=lambda n=name, y=y_mm, z=z_mm, d=description: (
                    self.request_direction_capture(
                        n,
                        y,
                        z,
                        d,
                    )
                ),
            )
            pad_canvas.create_window(
                pad_center + y_mm * pad_scale,
                pad_center - z_mm * pad_scale,
                window=button,
            )
            self.direction_buttons[name] = button

    def build_preprocess_panel(
        self,
        parent: ttk.Frame,
        row: int,
    ) -> None:
        """New 파이프라인 전처리(필터) 파라미터를 실시간으로 조정하는 슬라이더
        패널. 슬라이더를 움직이면 즉시 set_param 명령이 백엔드로 나가 다음
        샘플부터 반영되므로, 값을 바꾸면서 New 판 시각화가 어떻게 달라지는지
        바로 눈으로 볼 수 있다."""
        preprocess_frame = ttk.LabelFrame(
            parent,
            text="전처리 파라미터 (실시간 조정)",
            padding=8,
        )
        preprocess_frame.grid(
            row=row,
            column=0,
            sticky="nsew",
            pady=(8, 0),
        )
        preprocess_frame.columnconfigure(
            1,
            weight=1,
        )

        self.preprocess_param_vars: Dict[str, tk.DoubleVar] = {}
        self.preprocess_value_vars: Dict[str, tk.StringVar] = {}

        for index, (
            name,
            label,
            lo,
            hi,
            default,
            fmt,
            description,
        ) in enumerate(PREPROCESS_PARAM_SPECS):
            value_row = index * 2
            desc_row = value_row + 1

            ttk.Label(
                preprocess_frame,
                text=label,
            ).grid(
                row=value_row,
                column=0,
                sticky="w",
                padx=(0, 8),
                pady=(3, 0),
            )

            value_var = tk.StringVar(
                value=fmt.format(default)
            )
            self.preprocess_value_vars[name] = value_var

            scale_var = tk.DoubleVar(value=default)
            self.preprocess_param_vars[name] = scale_var

            scale = ttk.Scale(
                preprocess_frame,
                from_=lo,
                to=hi,
                orient=tk.HORIZONTAL,
                variable=scale_var,
                command=self.make_preprocess_param_callback(
                    name,
                    fmt,
                    value_var,
                ),
            )
            scale.grid(
                row=value_row,
                column=1,
                sticky="ew",
                pady=(3, 0),
            )

            ttk.Label(
                preprocess_frame,
                textvariable=value_var,
                width=10,
                anchor="e",
            ).grid(
                row=value_row,
                column=2,
                sticky="e",
                padx=(8, 0),
                pady=(3, 0),
            )

            ttk.Label(
                preprocess_frame,
                text=description,
                foreground=THEME_TEXT_MUTED,
                font=("Sans", 8),
                wraplength=320,
                justify="left",
            ).grid(
                row=desc_row,
                column=0,
                columnspan=3,
                sticky="w",
                padx=(0, 8),
                pady=(0, 4),
            )

        ttk.Button(
            preprocess_frame,
            text="기본값 복원",
            command=self.reset_preprocess_params,
        ).grid(
            row=len(PREPROCESS_PARAM_SPECS) * 2,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(6, 0),
        )

    def make_preprocess_param_callback(
        self,
        name: str,
        fmt: str,
        value_var: tk.StringVar,
    ):
        def callback(value_str: str) -> None:
            value = float(value_str)
            if name == "median_prefilter_window":
                value = round(value)
            value_var.set(fmt.format(value))
            self.enqueue(
                "set_param",
                {
                    "name": name,
                    "value": value,
                },
            )

        return callback

    def reset_preprocess_params(self) -> None:
        for (
            name,
            _label,
            _lo,
            _hi,
            default,
            fmt,
            _description,
        ) in PREPROCESS_PARAM_SPECS:
            self.preprocess_param_vars[name].set(default)
            self.preprocess_value_vars[name].set(
                fmt.format(default)
            )
            self.enqueue(
                "set_param",
                {
                    "name": name,
                    "value": default,
                },
            )
        self.append_log(
            "전처리 파라미터를 기본값으로 복원했습니다."
        )

    # -------------------------------------------------------------------------
    # 요청
    # -------------------------------------------------------------------------

    def request_empty_capture(self) -> None:
        if not messagebox.askokcancel(
            "빈 판 기준",
            "판 위 물체를 모두 제거하십시오.\n"
            "로봇과 RG2, 판을 건드리지 마십시오.\n\n"
            "측정을 시작합니까?",
        ):
            return

        self.enqueue(
            "capture_empty",
            None,
        )

    def request_center_capture(self) -> None:
        if not messagebox.askokcancel(
            "중심 기준",
            "동일한 물체를 목표 중심점에 놓고 손을 완전히 떼십시오.\n\n"
            "측정을 시작합니까?",
        ):
            return

        self.enqueue(
            "capture_center",
            None,
        )

    def request_direction_capture(
        self,
        name: str,
        y_mm: float,
        z_mm: float,
        description: str,
    ) -> None:
        if not messagebox.askokcancel(
            f"{name} 보정",
            f"동일한 물체를 {description}, "
            f"중심에서 {CALIBRATION_RADIUS_MM:.0f} mm 위치에 놓으십시오.\n"
            "손을 완전히 뗀 뒤 측정하십시오.\n\n"
            "측정을 시작합니까?",
        ):
            return

        self.enqueue(
            "capture_direction",
            {
                "name": name,
                "y_mm": y_mm,
                "z_mm": z_mm,
                "description": description,
            },
        )

    def request_build_model(self) -> None:
        self.enqueue(
            "build_model",
            None,
        )

    def request_save_preset(self) -> None:
        if not (self.empty_ready and self.center_ready):
            messagebox.showwarning(
                "저장 불가",
                "빈 판 기준과 중심 기준을 먼저 측정해야 저장할 수 있습니다.",
            )
            return

        name = simpledialog.askstring(
            "캘리 프리셋 저장",
            "프리셋 이름을 입력하십시오\n(영문/숫자/-/_ 만 사용, 비우면 시간으로 자동 생성):",
            parent=self.root,
        )
        if name is None:
            return

        self.enqueue("save_preset", name)

    def request_load_preset(self) -> None:
        CALI_PRESET_DIR.mkdir(parents=True, exist_ok=True)

        path = filedialog.askopenfilename(
            title="캘리 프리셋 불러오기",
            initialdir=str(CALI_PRESET_DIR),
            filetypes=[("JSON 프리셋", "*.json"), ("모든 파일", "*.*")],
        )
        if not path:
            return

        if self.monitor_enabled:
            if not messagebox.askokcancel(
                "모니터 실행 중",
                "모니터가 실행 중입니다. 프리셋을 불러오면 모니터가 멈추고\n"
                "기준값이 교체됩니다. 계속합니까?",
            ):
                return

        self.enqueue("load_preset", path)

    def request_move_to_reference_posj(self) -> None:
        if not self.reference_posj_ready:
            messagebox.showwarning(
                "이동 불가",
                "현재 캘리브레이션엔 기준 joint 정보가 없습니다.\n"
                "구버전 프리셋이거나 아직 빈 판 측정을 하지 않았습니다.",
            )
            return

        if not messagebox.askokcancel(
            "⚠ 로봇이 실제로 움직입니다",
            "저장된 기준 joint 각도로 로봇을 이동시킵니다.\n\n"
            "- 로봇 주변에 사람/장애물이 없습니까?\n"
            "- 비상정지 버튼에 손을 올릴 준비가 됐습니까?\n\n"
            "위 조건이 확인됐으면 '예'를 누르십시오.",
            icon="warning",
        ):
            return

        self.enqueue("move_to_reference_posj", None)

    def request_start_monitor(self) -> None:
        self.enqueue(
            "start_monitor",
            None,
        )

    def request_pause_monitor(self) -> None:
        self.enqueue(
            "pause_monitor",
            None,
        )

    def apply_label(self) -> None:
        self.enqueue(
            "set_label",
            self.label_entry_var.get(),
        )

    def enqueue(
        self,
        name: str,
        payload: Any,
    ) -> None:
        if self.busy:
            return

        self.command_queue.put(
            (
                name,
                payload,
            )
        )

        if name.startswith("capture_") or name in (
            "build_model",
            "save_preset",
            "load_preset",
            "move_to_reference_posj",
        ):
            self.busy = True
            self.update_button_states()

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
            try:
                self.root.destroy()
            except tk.TclError:
                pass
            return

        drain_start = time.monotonic()

        events: List[Dict[str, Any]] = []
        try:
            while True:
                events.append(self.ui_queue.get_nowait())
        except queue.Empty:
            pass

        # "state" 이벤트는 최대 50Hz로 쏟아지는데, GUI가 poll 주기(기본
        # 50ms)마다 큐를 전부 비우다 보니 밀리면 그만큼 재렌더링(캔버스
        # 다시 그리기 등)이 쌓여 렉으로 이어진다. 여러 개가 몰려 있으면
        # 마지막 1개만 반영하고 나머지는 버려서(=화면은 최신값만 표시)
        # GUI 처리량을 poll 주기로 상한을 둔다. 다른 이벤트(로그, 측정
        # 진행 등)는 순서를 그대로 보존해서 전부 처리한다.
        filtered_events: List[Dict[str, Any]] = []
        pending_state_event: Optional[Dict[str, Any]] = None

        for event in events:
            if event["event"] == "state":
                pending_state_event = event
                continue

            if pending_state_event is not None:
                filtered_events.append(pending_state_event)
                pending_state_event = None

            filtered_events.append(event)

        if pending_state_event is not None:
            filtered_events.append(pending_state_event)

        for event in filtered_events:
            self.handle_ui_event(event)

        drain_elapsed_ms = (time.monotonic() - drain_start) * 1000.0
        coalesced_count = len(events) - len(filtered_events)

        # 렉 진단: 이번 poll에서 처리한 이벤트가 비정상적으로 많거나(큐가
        # 밀렸다는 뜻) 드레인 자체가 오래 걸리면(렌더링이 무겁다는 뜻)
        # 로그로 남긴다. GUI_POLL_MS(기본 50ms)에 근접/초과하면 다음 tick도
        # 밀리기 시작하므로 그 전에 알 수 있게 임계값을 여유 있게 잡았다.
        if len(events) >= 20 or drain_elapsed_ms >= 30.0:
            self.append_log(
                f"[진단] GUI 큐 드레인: 이벤트 {len(events)}개"
                + (
                    f" (state {coalesced_count}개 코얼레싱)"
                    if coalesced_count
                    else ""
                )
                + f" · 처리 {drain_elapsed_ms:.1f} ms"
                + f" (poll 주기 {GUI_POLL_MS} ms)"
            )

        self.root.after(
            GUI_POLL_MS,
            self.poll_ui_queue,
        )

    def handle_ui_event(
        self,
        event: Dict[str, Any],
    ) -> None:
        name = event["event"]

        if name == "log":
            self.append_log(
                event["message"]
            )

        elif name == "reference_state":
            self.empty_ready = bool(
                event["empty_ready"]
            )
            self.center_ready = bool(
                event["center_ready"]
            )
            self.direction_points = set(
                event["direction_points"]
            )
            self.model_ready = bool(
                event["model_ready"]
            )
            self.monitor_enabled = bool(
                event["monitor_enabled"]
            )
            self.reference_posj_ready = bool(
                event.get("reference_posj_ready", False)
            )
            self.busy = False
            self.update_button_states()

            self.connection_var.set(
                "ROS 연결됨 · "
                + (
                    "모니터 실행 중"
                    if self.monitor_enabled
                    else "대기"
                )
            )

            # model_ready 등 플래그 변화(측정/모델생성/프리셋 로드 직후)를
            # 곧바로 반영. "state" 이벤트만큼 자주 오지 않는 이벤트라 매번
            # 다시 그려도 부담이 없다 — 렉의 원인이었던 건 아래에서 제거한,
            # 모든 이벤트마다 무조건 다시 그리던 부분이다.
            self.draw_plate()
            self.draw_axis_bars()

        elif name == "measurement_started":
            self.busy = True
            self.progress_label_var.set(
                event["title"]
            )
            self.progress["value"] = 0.0
            self.update_button_states()

        elif name == "measurement_progress":
            total = max(
                int(event["total"]),
                1,
            )
            current = int(
                event["current"]
            )
            self.progress["value"] = (
                current / total * 100.0
            )

        elif name == "measurement_finished":
            self.progress["value"] = 100.0
            self.progress_label_var.set(
                "측정 완료"
            )

        elif name == "empty_captured":
            self.append_log(
                "빈 판 기준 저장 완료"
            )

        elif name == "center_captured":
            self.append_log(
                "중심 기준 저장 완료"
            )
            self.metric_variables[
                "position"
            ].set(
                "방향 보정 전: 중심 이탈 크기만 표시"
            )
            self.metric_variables[
                "wrench_position"
            ].set(
                "방향 보정 전: 중심 이탈 크기만 표시"
            )

        elif name == "direction_captured":
            self.append_log(
                f"{event['name']} 보정 저장 완료 · "
                f"z-norm={event['center_error_z_norm']:.2f}"
            )

        elif name == "model_built":
            wrench_rmse = event.get("wrench_rmse_mm")
            self.append_log(
                "위치 모델 생성 완료 · "
                f"RMSE={event['rmse_mm']:.2f} mm · "
                f"condition={event['condition_number']:.2f}"
                + (
                    f" · wrench RMSE={wrench_rmse:.2f} mm"
                    if wrench_rmse is not None
                    else " · wrench 모델 없음"
                )
            )
            self.metric_variables[
                "position"
            ].set(
                "4방향 위치 모델 준비 완료"
            )
            self.metric_variables[
                "wrench_position"
            ].set(
                "4방향 wrench 위치 모델 준비 완료"
                if wrench_rmse is not None
                else "wrench 모델 미생성(구버전 캘리)"
            )

        elif name == "state":
            self.update_state(
                event["state"]
            )

        elif name == "log_paths":
            self.csv_path = event[
                "csv_path"
            ]
            self.model_path = event[
                "model_path"
            ]
            self.metric_variables[
                "csv"
            ].set(
                self.csv_path
            )

        elif name == "preset_saved":
            messagebox.showinfo(
                "캘리 저장 완료",
                f"프리셋 '{event['name']}' 저장됨:\n{event['path']}",
            )

        elif name == "preset_loaded":
            self.append_log(
                f"프리셋 '{event['name']}' 불러오기 완료 · "
                "이제 [모니터 시작]을 누르면 됩니다."
            )

        elif name == "move_started":
            self.progress_label_var.set("기준 joint로 이동 중...")
            self.progress["value"] = 0.0

        elif name == "move_finished":
            ok = bool(event.get("ok", False))
            self.progress["value"] = 100.0 if ok else 0.0
            self.progress_label_var.set(
                "이동 완료" if ok else "이동 실패"
            )

        elif name == "error":
            self.busy = False
            self.update_button_states()
            self.append_log(
                "[오류] " + event["message"]
            )
            messagebox.showerror(
                "오류",
                event["message"],
            )

        elif name == "backend_stopped":
            self.connection_var.set(
                "ROS 백엔드 종료"
            )

    # -------------------------------------------------------------------------
    # 상태 업데이트
    # -------------------------------------------------------------------------

    def update_state(
        self,
        state: Dict[str, Any],
    ) -> None:
        self.latest_state = state
        self.latest_legacy_state = state.get("legacy")

        center_state = state[
            "center_state"
        ]
        movement_state = state[
            "movement_state"
        ]

        if center_state == "CENTER":
            status_text = "CENTER · HOLD"
        elif movement_state == "TOWARD_CENTER":
            status_text = "OFF CENTER · 중심으로 이동 중"
        elif movement_state == "AWAY_FROM_CENTER":
            status_text = "OFF CENTER · 가장자리로 이동 중"
        else:
            status_text = "OFF CENTER · 이탈 위치 유지"

        if not state["object_present"]:
            status_text = "물체 감지 안 됨"

        if not state["pose_stable"]:
            status_text += " · 로봇 자세 변경"

        self.status_var.set(
            status_text
        )

        self.metric_variables[
            "center_error_nm"
        ].set(
            finite_text(
                state["center_error_nm"],
                ".3f",
            )
        )
        self.metric_variables[
            "center_error_z"
        ].set(
            finite_text(
                state["center_error_z"],
                ".2f",
            )
        )
        self.metric_variables[
            "deviation_index"
        ].set(
            finite_text(
                state["deviation_index"],
                ".2f",
            )
        )
        self.metric_variables[
            "trend_z_per_sec"
        ].set(
            finite_text(
                state["trend_z_per_sec"],
                "+.2f",
            )
        )
        self.metric_variables[
            "edge_index"
        ].set(
            finite_text(
                state["edge_index"],
                ".2f",
            )
        )
        self.metric_variables[
            "dominant"
        ].set(
            f"J{state['dominant_axis']} "
            f"{state['dominant_axis_z']:+.1f}z"
        )

        if self.model_ready:
            self.metric_variables[
                "position"
            ].set(
                f"Y={finite_text(state['estimated_y_mm'], '+.1f')} mm · "
                f"Z={finite_text(state['estimated_z_mm'], '+.1f')} mm · "
                f"R={finite_text(state['estimated_radius_mm'], '.1f')} mm"
            )
        else:
            self.metric_variables[
                "position"
            ].set(
                "Y/Z 방향 미보정"
            )

        if state.get("wrench_model_ready"):
            self.metric_variables[
                "wrench_position"
            ].set(
                f"Y={finite_text(state['estimated_y_mm_wrench'], '+.1f')} mm · "
                f"Z={finite_text(state['estimated_z_mm_wrench'], '+.1f')} mm · "
                f"R={finite_text(state['estimated_radius_mm_wrench'], '.1f')} mm"
            )
        else:
            self.metric_variables[
                "wrench_position"
            ].set(
                "wrench 모델 미보정(구버전 캘리 또는 방향 보정 필요)"
            )

        self.metric_variables[
            "hint"
        ].set(
            state["move_hint"]
        )
        self.metric_variables[
            "object"
        ].set(
            "감지됨"
            if state["object_present"]
            else "감지 안 됨"
        )
        self.metric_variables[
            "stationary"
        ].set(
            "정지"
            if state["robot_stationary"]
            else "움직임 감지"
        )
        self.metric_variables[
            "pose"
        ].set(
            "기준 자세 유지"
            if state["pose_stable"]
            else (
                f"기준 이탈: "
                f"{state['tcp_position_drift_mm']:.2f} mm"
            )
        )
        self.metric_variables[
            "label"
        ].set(
            self.label_entry_var.get()
        )
        self.metric_variables[
            "achieved_hz"
        ].set(
            finite_text(
                state["achieved_hz"],
                ".1f",
            )
            + f" Hz (목표 {SAMPLE_HZ:.0f} Hz)"
        )

        self.draw_plate()
        self.draw_axis_bars()

    # -------------------------------------------------------------------------
    # 판
    # -------------------------------------------------------------------------

    def draw_plate(self) -> None:
        self.draw_plate_variant(
            self.plate_canvas_original,
            self.latest_legacy_state,
        )
        self.draw_plate_variant(
            self.plate_canvas_new,
            self.latest_state,
        )

    def draw_plate_variant(
        self,
        canvas: tk.Canvas,
        state: Optional[Dict[str, Any]],
    ) -> None:
        canvas.delete("all")

        width = max(
            canvas.winfo_width(),
            200,
        )
        height = max(
            canvas.winfo_height(),
            200,
        )

        center_x = width / 2.0
        center_y = height / 2.0
        radius_px = min(
            width,
            height,
        ) * 0.40

        # 외곽 및 영역
        for ratio, color, dash in (
            (1.0, "#e5e7eb", ()),
            (0.7, "#374151", (4, 3)),
            (0.3, "#374151", (4, 3)),
        ):
            radius = radius_px * ratio
            canvas.create_oval(
                center_x - radius,
                center_y - radius,
                center_x + radius,
                center_y + radius,
                outline=color,
                width=2 if ratio == 1.0 else 1,
                dash=dash,
            )

        canvas.create_line(
            center_x - radius_px,
            center_y,
            center_x + radius_px,
            center_y,
            fill="#4b5563",
        )
        canvas.create_line(
            center_x,
            center_y - radius_px,
            center_x,
            center_y + radius_px,
            fill="#4b5563",
        )

        canvas.create_text(
            center_x + radius_px + 12,
            center_y,
            text="+Y",
            fill="#d1d5db",
            anchor="w",
        )
        canvas.create_text(
            center_x - radius_px - 12,
            center_y,
            text="-Y",
            fill="#d1d5db",
            anchor="e",
        )
        canvas.create_text(
            center_x,
            center_y - radius_px - 12,
            text="+Z",
            fill="#d1d5db",
            anchor="s",
        )
        canvas.create_text(
            center_x,
            center_y + radius_px + 12,
            text="-Z",
            fill="#d1d5db",
            anchor="n",
        )

        canvas.create_oval(
            center_x - 5,
            center_y - 5,
            center_x + 5,
            center_y + 5,
            fill="#10b981",
            outline="",
        )

        if state is None:
            canvas.create_text(
                center_x,
                center_y + radius_px * 0.65,
                text="빈 판 및 중심 기준을 측정하십시오.",
                fill="#9ca3af",
                font=("Sans", 12),
            )
            return

        if self.model_ready and math.isfinite(
            float(
                state["estimated_y_mm"]
            )
        ):
            estimated_y = float(
                state["estimated_y_mm"]
            )
            estimated_z = float(
                state["estimated_z_mm"]
            )

            scale = radius_px / PLATE_RADIUS_MM
            point_x = center_x + estimated_y * scale
            point_y = center_y - estimated_z * scale

            distance_from_center = math.hypot(
                point_x - center_x,
                point_y - center_y,
            )

            max_draw_radius = radius_px * 1.10

            if distance_from_center > max_draw_radius:
                factor = (
                    max_draw_radius
                    / distance_from_center
                )
                point_x = (
                    center_x
                    + (point_x - center_x)
                    * factor
                )
                point_y = (
                    center_y
                    + (point_y - center_y)
                    * factor
                )

            if state["center_state"] == "CENTER":
                point_color = "#10b981"
            elif state["movement_state"] == "TOWARD_CENTER":
                point_color = "#f59e0b"
            else:
                point_color = "#ef4444"

            canvas.create_line(
                point_x,
                point_y,
                center_x,
                center_y,
                fill="#fbbf24",
                width=2,
                arrow=tk.LAST,
            )
            canvas.create_oval(
                point_x - 10,
                point_y - 10,
                point_x + 10,
                point_y + 10,
                fill=point_color,
                outline="#ffffff",
                width=2,
            )
            canvas.create_text(
                point_x,
                point_y - 18,
                text=(
                    f"Y {estimated_y:+.1f} / "
                    f"Z {estimated_z:+.1f} mm"
                ),
                fill="#ffffff",
                font=("Monospace", 10, "bold"),
            )

        else:
            # 위치 방향을 모르므로 DEV에 따라 원형 게이지로 표현
            deviation_index = float(
                state["deviation_index"]
            )
            gauge_ratio = min(
                deviation_index / 2.0,
                1.0,
            )
            gauge_radius = (
                12.0
                + gauge_ratio
                * radius_px
                * 0.45
            )

            if state["center_state"] == "CENTER":
                gauge_color = "#10b981"
            elif state["movement_state"] == "TOWARD_CENTER":
                gauge_color = "#f59e0b"
            else:
                gauge_color = "#ef4444"

            canvas.create_oval(
                center_x - gauge_radius,
                center_y - gauge_radius,
                center_x + gauge_radius,
                center_y + gauge_radius,
                outline=gauge_color,
                width=5,
            )
            canvas.create_text(
                center_x,
                center_y,
                text=(
                    f"DEV {deviation_index:.2f}\n"
                    "방향 보정 필요"
                ),
                fill="#ffffff",
                justify="center",
                font=("Sans", 13, "bold"),
            )

        canvas.create_text(
            center_x,
            height - 16,
            text=(
                f"ERR {state['center_error_nm']:.3f} Nm · "
                f"Z {state['center_error_z']:.2f} · "
                f"DEV {state['deviation_index']:.2f}"
            ),
            fill="#d1d5db",
            font=("Monospace", 11),
        )

    # -------------------------------------------------------------------------
    # 축별 막대
    # -------------------------------------------------------------------------

    def draw_axis_bars(self) -> None:
        if self.latest_state is None:
            z_values = [0.0] * 6
            nm_values = [0.0] * 6
        else:
            z_values = self.latest_state[
                "external_center_error_z"
            ]
            nm_values = self.latest_state[
                "external_center_error"
            ]

        for index, canvas in enumerate(
            self.axis_canvases
        ):
            canvas.delete("all")

            width = max(
                canvas.winfo_width(),
                40,
            )
            height = max(
                canvas.winfo_height(),
                20,
            )
            middle = width / 2.0

            canvas.create_line(
                middle,
                0,
                middle,
                height,
                fill="#6b7280",
            )

            z_value = float(
                z_values[index]
            )
            maximum = 8.0
            length = (
                min(
                    abs(z_value),
                    maximum,
                )
                / maximum
                * middle
            )

            if abs(z_value) < 2.0:
                color = "#10b981"
            elif abs(z_value) < 4.0:
                color = "#f59e0b"
            else:
                color = "#ef4444"

            if z_value >= 0.0:
                x0 = middle
                x1 = middle + length
            else:
                x0 = middle - length
                x1 = middle

            canvas.create_rectangle(
                x0,
                3,
                x1,
                height - 3,
                fill=color,
                outline="",
            )

            self.axis_value_labels[
                index
            ].configure(
                text=(
                    f"{z_value:+.2f} z / "
                    f"{float(nm_values[index]):+.3f} Nm"
                )
            )

    # -------------------------------------------------------------------------
    # 버튼 상태
    # -------------------------------------------------------------------------

    def update_button_states(self) -> None:
        disabled = (
            tk.DISABLED
            if self.busy
            else tk.NORMAL
        )

        self.empty_button.configure(
            state=disabled
        )

        self.center_button.configure(
            state=(
                tk.NORMAL
                if self.empty_ready and not self.busy
                else tk.DISABLED
            )
        )

        for name, button in self.direction_buttons.items():
            button.configure(
                state=(
                    tk.NORMAL
                    if self.center_ready and not self.busy
                    else tk.DISABLED
                )
            )

            if name in self.direction_points:
                button.configure(
                    text=button.cget("text").split(" ✓")[0] + " ✓"
                )

        required = {
            "Y_PLUS",
            "Y_MINUS",
            "Z_PLUS",
            "Z_MINUS",
        }

        self.model_button.configure(
            state=(
                tk.NORMAL
                if (
                    required.issubset(
                        self.direction_points
                    )
                    and not self.busy
                )
                else tk.DISABLED
            )
        )

        self.save_preset_button.configure(
            state=(
                tk.NORMAL
                if self.empty_ready and self.center_ready and not self.busy
                else tk.DISABLED
            )
        )
        self.load_preset_button.configure(
            state=(
                tk.DISABLED if self.busy else tk.NORMAL
            )
        )
        self.move_to_posj_button.configure(
            state=(
                tk.NORMAL
                if self.reference_posj_ready and not self.busy
                else tk.DISABLED
            )
        )

        self.start_button.configure(
            state=(
                tk.NORMAL
                if self.center_ready and not self.busy
                else tk.DISABLED
            )
        )
        self.pause_button.configure(
            state=(
                tk.NORMAL
                if self.monitor_enabled and not self.busy
                else tk.DISABLED
            )
        )

    # -------------------------------------------------------------------------
    # 로그
    # -------------------------------------------------------------------------

    def append_log(
        self,
        message: str,
    ) -> None:
        self.log_text.configure(
            state="normal"
        )
        self.log_text.insert(
            tk.END,
            f"[{datetime.now():%H:%M:%S}] {message}\n",
        )
        self.log_text.see(
            tk.END
        )
        self.log_text.configure(
            state="disabled"
        )

    def clear_log(self) -> None:
        self.log_text.configure(
            state="normal"
        )
        self.log_text.delete(
            "1.0",
            tk.END,
        )
        self.log_text.configure(
            state="disabled"
        )

    def open_log_directory(self) -> None:
        LOG_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True,
        )

        try:
            subprocess.Popen(
                [
                    "xdg-open",
                    str(LOG_DIRECTORY),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as error:
            messagebox.showerror(
                "폴더 열기 실패",
                str(error),
            )

    # -------------------------------------------------------------------------
    # 종료
    # -------------------------------------------------------------------------

    def on_close(self) -> None:
        if not messagebox.askokcancel(
            "종료",
            "GUI와 ROS 모니터를 종료합니까?",
        ):
            return

        self.closing = True

        self.command_queue.put(
            ("shutdown", None)
        )
        self.root.after(
            200,
            self.root.destroy,
        )


# =============================================================================
# main
# =============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = rclpy.create_node(
        NODE_NAME,
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = node

    import DSR_ROBOT2 as dsr

    command_queue: queue.Queue = queue.Queue()
    ui_queue: queue.Queue = queue.Queue()

    backend = PlatePositionBackend(
        node=node,
        dsr_module=dsr,
        command_queue=command_queue,
        ui_queue=ui_queue,
    )

    # Tkinter는 반드시 메인 스레드에서 "먼저" 완전히 초기화한다. rclpy를 스핀하는
    # 백엔드 스레드가 tk.Tk() 초기화 도중에 동시에 돌면 Tcl 인터프리터가 아직
    # 준비되지 않은 상태를 건드려 "Tcl_Release couldn't find reference" 크래시가
    # 난다. 그래서 GUI를 다 만든 뒤에 백엔드 스레드를 시작한다.
    root = tk.Tk()
    gui = PlatePositionGUI(
        root=root,
        command_queue=command_queue,
        ui_queue=ui_queue,
    )

    backend_thread = threading.Thread(
        target=backend.run,
        daemon=True,
    )
    backend_thread.start()

    try:
        root.mainloop()

    finally:
        command_queue.put(
            ("shutdown", None)
        )
        backend_thread.join(
            timeout=2.0,
        )

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
