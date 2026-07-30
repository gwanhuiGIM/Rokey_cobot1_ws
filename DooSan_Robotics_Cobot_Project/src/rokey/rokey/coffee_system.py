"""
coffee_system_single_tilt_v2.py

M0609 + OnRobot RG2 협동로봇 자동 커피 추출 시스템
원본 Task Writer(DRL) 프로그램(m0609_coffe_system.drl, System.drvar)을
ROS2 단일 파일 노드로 변환한 것입니다.

동작 순서
=========
1. 원두 선택       : DI 13~16 물리 버튼으로 원두 종류 선택
2. bean_drop       : 스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입
3. 분쇄 선택       : DI 13~16 물리 버튼으로 3/5/7/10회전 선택
4. grinder         : 선택된 회전 수만큼 원호 모션으로 분쇄
5. dripper_in      : 분쇄 원두가 든 병을 필터에 투입
6. spiral_pour     : 주전자를 파지하고 보상 내향 스파이럴 드립 후 원위치 반환
7. final_drip      : mug TCP 원점을 고정해 한 번 기울인 뒤 저장한 시작 자세로 직접 복귀

실행
====
    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609
    ros2 run rokey coffee_system_single_tilt_v2
    (또는 패키지에 등록하지 않고 바로:  python3 coffee_system_single_tilt_v2.py)

이 변형의 final_drip 차이
=========================
- ``System_final_j2`` MoveJ와 계산된 J6 +45 deg를 연속 실행하던 중복 기울임을 제거한다.
- ``System_final_l`` 도달 자세에서 mug TCP 기준 J6 +45 deg 기울임을 정확히 한 번 실행한다.
- 복귀 시 현재 관절에 -45 deg를 다시 추정 적용하지 않고, 기울이기 직전에 저장한
  TCP 회전행렬을 직접 목표로 사용한다.

주의
====
- 이 노드는 시작과 동시에 실제 로봇 모션을 수행합니다. 원본 좌표는 teach pendant에서
  교시된 값(System.drvar)을 그대로 사용하므로, 실행 전 작업 공간(그라인더, 드리퍼,
  병, 컵, 스푼 위치)이 교시 당시와 동일한지 반드시 확인하십시오.
- OnRobot RG2 그리퍼는 컨트롤러 Digital Output 1, 2번 접점 조합으로 제어됩니다.
  (그리퍼 컨트롤 박스가 해당 접점에 매핑되어 있어야 합니다.)
- 그라인더 서브루틴은 task_compliance_ctrl() 진입 후 movec()으로 원호 모션을 수행합니다.
  컴플라이언스 구간 진입 전 주변 장애물 여부를 확인하십시오.
- 원본 DRL의 set_singular_handling(DR_AVOID)는 프로그래밍 매뉴얼에 공식 문서화된
  set_singularity_handling(mode)로 치환하였습니다 (동일 기능, 특이점 자동회피).
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from collections import deque
from typing import Any, Callable, Optional

import numpy as np
import rclpy
from onrobot_rg_msgs.msg import OnRobotRGInput
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

import DR_init
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveJoint, MoveStop

# =============================================================================
# 사용자 설정
# =============================================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_coffee_system_single_tilt_v2"

# 티치펜던트에 등록된 이름 (gear.py/move.py와 동일한 이름 사용)
TOOL_NAME = "Tool Weight_gripper"
TCP_NAME = "GripperDA_v1"

# 원본 DRL 하단 전역 설정값
VELJ_DEFAULT = 60.0
ACCJ_DEFAULT = 100.0
VELX_LIN_DEFAULT = 250.0
VELX_ROT_DEFAULT = 80.0
ACCX_LIN_DEFAULT = 1000.0
ACCX_ROT_DEFAULT = 322.5


# =============================================================================
# 스파이럴 드립 Sub 설정
# =============================================================================

SPOUT_TCP_NAME = "pot"

SPIRAL_RADIUS_MM = 44.0
SPIRAL_REVOLUTIONS = 5.0
SPIRAL_DURATION_SEC = 15.0
SPIRAL_J6_DELTA_DEG = -60.0
SPIRAL_PATH_POINTS = 100

# region 드립 주전자 복귀 시간
CENTER_PIVOT_RETURN_DURATION_SEC = 2.0
CENTER_PIVOT_RETURN_POINTS = 30

POT_RELEASE_BASE_Z_OFFSET_MM = 10.0
POT_RELEASE_SETTLE_SEC = 0.8

MOVESX_MAX_LINEAR_STEP_MM = 15.0
MOVESX_MAX_ANGULAR_STEP_DEG = 5.0
MOVESX_LINEAR_VEL_MM_S = 120.0
MOVESX_ANGULAR_VEL_DEG_S = 25.0
MOVESX_LINEAR_ACC_MM_S2 = 300.0
MOVESX_ANGULAR_ACC_DEG_S2 = 80.0

SPIRAL_CENTER_OFFSET_SIGN_X = -1.0
SPIRAL_ROTATION_SIGN = +1.0

# 주전자 교시 좌표
SYSTEM_POT_GRIP_VALUES = [831.36, -177.93, 108.26, 3.26, 90.23, 86.66]
SYSTEM_POT_GRIP_JOINT_VALUES = [-28.73, 38.17, 112.55, 146.06, 65.17, -73.99]
POUR_START_JOINT_VALUES = [-13.00, 23.41, 100.08, 144.04, 35.00, -122.57]
PICKUP_APPROACH_REL_VALUES = [0.0, -80.0, 100.0, 0.0, 0.0, 0.0]
PICKUP_LIFT_REL_VALUES = [-150.0, 0.0, 200.0, 0.0, 0.0, 0.0]


# =============================================================================
# final_drip 물 붓기 설정
# =============================================================================

# plate_outline(2).py의 회전 및 복귀 구조를 final_drip에 통합한다.
# System_final_l 도달 후 J6 상대 +30 deg를 적용하고 현재 자세를 물 붓기 시작점으로 사용한다.
# System_final_j2 교시 MoveJ는 실행하지 않고 고정 mug TCP 회전 경로로 이어 간다.
FINAL_POUR_TCP_NAME = "mug"
FINAL_PRE_POUR_J6_REL_DEG = 30.0
FINAL_POUR_J6_DELTA_DEG = 55.0
FINAL_POUR_PATH_POINTS = 100

# region 물 붓기 시간
FINAL_POUR_ANGULAR_VEL_DEG_S = 120.0
FINAL_POUR_ANGULAR_ACC_DEG_S2 = 350.0
FINAL_RETURN_ANGULAR_VEL_DEG_S = 120.0
FINAL_RETURN_ANGULAR_ACC_DEG_S2 = 350.0
FINAL_MUG_RETURN_BASE_X_OFFSET_MM = -200.0
FINAL_MUG_RETURN_Z_OFFSET_MM = 5.0
FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM = -150.0
FINAL_MUG_RELEASE_BASE_Z_LIFT_MM = 200.0

FINAL_POUR_MAX_LINEAR_STEP_MM = 5.0
FINAL_POUR_MAX_ANGULAR_STEP_DEG = 3.0

# final_drip의 물 붓기와 시작 자세 복귀 회전에 전용 속도/가속도를 사용한다.
# 물컵을 원래 위치에 내려놓는 Cartesian 모션은 전역 기본값을 유지한다.

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =============================================================================
# 물리 버튼 / Web UI 상태 연동
# =============================================================================

STATUS_TOPIC = "/coffee_system/status"
CONTROL_TOPIC = "/coffee_system/control"

OPERATION_SPEED_MIN_PERCENT = 10
OPERATION_SPEED_MAX_PERCENT = 100
OPERATION_SPEED_DEFAULT_PERCENT = 100

# change_operation_speed()가 상대 비율을 적용하는 명명된 속도 설정값.
# 아래 Python 상수 자체를 덮어쓰는 것은 아니며, 로그에서는 "명령 기준값 → 유효 적용값"을 표시한다.
OPERATION_SPEED_VARIABLES: tuple[tuple[str, float, str], ...] = (
    ("VELJ_DEFAULT", VELJ_DEFAULT, "deg/s"),
    ("VELX_LIN_DEFAULT", VELX_LIN_DEFAULT, "mm/s"),
    ("VELX_ROT_DEFAULT", VELX_ROT_DEFAULT, "deg/s"),
    ("MOVESX_LINEAR_VEL_MM_S", MOVESX_LINEAR_VEL_MM_S, "mm/s"),
    ("MOVESX_ANGULAR_VEL_DEG_S", MOVESX_ANGULAR_VEL_DEG_S, "deg/s"),
    (
        "FINAL_POUR_ANGULAR_VEL_DEG_S",
        FINAL_POUR_ANGULAR_VEL_DEG_S,
        "deg/s",
    ),
    (
        "FINAL_RETURN_ANGULAR_VEL_DEG_S",
        FINAL_RETURN_ANGULAR_VEL_DEG_S,
        "deg/s",
    ),
)

# change_operation_speed()는 현재 설정된 모션 속도에 상대 비율을 적용한다.
# 가속도 상수는 재할당하지 않으므로 별도 "변경 없음" 로그로 표시한다.
OPERATION_ACCELERATION_VARIABLES: tuple[tuple[str, float, str], ...] = (
    ("ACCJ_DEFAULT", ACCJ_DEFAULT, "deg/s^2"),
    ("ACCX_LIN_DEFAULT", ACCX_LIN_DEFAULT, "mm/s^2"),
    ("ACCX_ROT_DEFAULT", ACCX_ROT_DEFAULT, "deg/s^2"),
    ("MOVESX_LINEAR_ACC_MM_S2", MOVESX_LINEAR_ACC_MM_S2, "mm/s^2"),
    ("MOVESX_ANGULAR_ACC_DEG_S2", MOVESX_ANGULAR_ACC_DEG_S2, "deg/s^2"),
    (
        "FINAL_POUR_ANGULAR_ACC_DEG_S2",
        FINAL_POUR_ANGULAR_ACC_DEG_S2,
        "deg/s^2",
    ),
    (
        "FINAL_RETURN_ANGULAR_ACC_DEG_S2",
        FINAL_RETURN_ANGULAR_ACC_DEG_S2,
        "deg/s^2",
    ),
)


def _effective_speed_value(base_value: float, percent: int) -> float:
    """작동 속도 비율을 반영한 표시용 유효 속도를 계산한다."""
    return float(base_value) * float(percent) / 100.0


def _speed_change_lines(previous_percent: int, requested_percent: int) -> list[str]:
    """터미널/UI에 공통으로 사용할 속도 변경 상세 로그를 생성한다."""
    previous = max(
        OPERATION_SPEED_MIN_PERCENT,
        min(OPERATION_SPEED_MAX_PERCENT, int(previous_percent)),
    )
    requested = max(
        OPERATION_SPEED_MIN_PERCENT,
        min(OPERATION_SPEED_MAX_PERCENT, int(requested_percent)),
    )
    lines = [
        f"작동 속도 비율: {previous}% -> {requested}%",
        "Python 상수는 유지되고 로봇 모션의 유효 속도 비율만 변경됩니다.",
    ]
    for name, base_value, unit in OPERATION_SPEED_VARIABLES:
        old_value = _effective_speed_value(base_value, previous)
        new_value = _effective_speed_value(base_value, requested)
        lines.append(
            f"{name}: {old_value:.3f} -> {new_value:.3f} {unit} "
            f"(기준값 {base_value:.3f})"
        )
    unchanged = ", ".join(
        f"{name}={value:.3f} {unit}"
        for name, value, unit in OPERATION_ACCELERATION_VARIABLES
    )
    lines.append(f"가속도 변수 변경 없음: {unchanged}")
    return lines


def _effective_speed_payload(percent: int) -> dict[str, dict[str, float | str]]:
    """웹 UI 상태에 전달할 현재 유효 속도 변수 값을 만든다."""
    return {
        name: {
            "base": round(float(base_value), 4),
            "effective": round(_effective_speed_value(base_value, percent), 4),
            "unit": unit,
        }
        for name, base_value, unit in OPERATION_SPEED_VARIABLES
    }

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

TEST_GRIP_OPEN_MODES = {
    "spoon_cup": "스푼·컵 열기",
    "jar": "병 열기",
    "handle": "손잡이 열기",
}

PHYSICAL_BUTTONS = (13, 14, 15, 16)
BEAN_BY_BUTTON = {
    13: {"id": "ethiopia", "name": "에티오피아 예가체프"},
    14: {"id": "colombia", "name": "콜롬비아 수프리모"},
    15: {"id": "brazil", "name": "브라질 산토스"},
    16: {"id": "guatemala", "name": "과테말라 안티구아"},
}

# 분쇄 굵기 선택: 1회전은 movec angle 기준 360도입니다.
GRIND_BY_BUTTON = {
    13: {"id": "coarse", "name": "굵게 분쇄", "turns": 3},
    14: {"id": "medium_coarse", "name": "중간 굵게 분쇄", "turns": 5},
    15: {"id": "medium_fine", "name": "중간 곱게 분쇄", "turns": 7},
    16: {"id": "fine", "name": "곱게 분쇄", "turns": 10},
}
DEGREES_PER_GRINDER_TURN = 360.0

# 그라인더 실제 회전 진행률 측정 설정
GRINDER_USER_COORD = 101
GRINDER_PROGRESS_UPDATE_SEC = 0.10
GRINDER_MOTION_START_TIMEOUT_SEC = 2.0
MOTION_IDLE = 0

# move_periodic 종료 후 사람이 병을 가볍게 쳤는지 확인하는 외력 조건입니다.
# 대기 시작 시 측정한 기준 힘에서 10 N 이상 변화하면 완료로 판정합니다.
EXTERNAL_FORCE_TRIGGER_N = 4.0
EXTERNAL_FORCE_SETTLE_SEC = 0.60
EXTERNAL_FORCE_BASELINE_SAMPLES = 20
EXTERNAL_FORCE_SAMPLE_SEC = 0.02
EXTERNAL_FORCE_CONFIRM_SAMPLES = 2
EXTERNAL_FORCE_UI_UPDATE_SEC = 0.20

# region 선택 시간
SELECTION_TIMEOUT_SEC = 15.0
EXTERNAL_FORCE_TIMEOUT_SEC = 10.0
DEFAULT_SELECTION_BUTTON = 13

# GRIP_DETECTION_README의 RG2 판정 계약.
# 전체 gSTA 상태를 장비 건강 신호로 사용하고, grip 비트와 JointState는 파지 판정에 쓴다.
GRIP_STATUS_TOPIC = "/OnRobotRGInput"
GRIP_DETECTED_TOPIC = "/onrobot/grip_detected"
GRIP_JOINT_STATE_TOPIC = "/onrobot_joint_states"
GRIP_VERIFY_TIMEOUT_SEC = 2.0
GRIP_SIGNAL_TIMEOUT_SEC = 1.0
GRIP_SIGNAL_STARTUP_WAIT_SEC = 2.0
GRIP_SIGNAL_RECOVERY_STABLE_SEC = 0.5
GRIP_SIGNAL_RESTART_COUNTDOWN_SEC = 3.0
GRIP_SIGNAL_RESTART_BUTTON = 13
GRIP_EMPTY_CLOSED_POSITION_RAD = 0.7587
GRIP_POSITION_MARGIN_RAD = 0.02
GRIP_EFFORT_MIN = 1.0e-6
GRIP_SIGNAL_STOP_MODE = 1

# RG2 User Manual의 register 268 (gSTA) bit 정의. 자동 공정에서는
# 안전 스위치가 눌린 상태도 위험 상태로 포함해 fail-closed 처리한다.
RG2_GSTA_SAFETY_FLAGS: tuple[tuple[int, str], ...] = (
    (1 << 2, "S1 안전 스위치 눌림"),
    (1 << 3, "S1 안전회로 작동"),
    (1 << 4, "S2 안전 스위치 눌림"),
    (1 << 5, "S2 안전회로 작동"),
    (1 << 6, "안전 오류"),
)
RG2_GSTA_SAFETY_MASK = sum(mask for mask, _ in RG2_GSTA_SAFETY_FLAGS)


def _rg2_gsta_safety_names(gsta: int) -> list[str]:
    value = int(gsta)
    return [
        name
        for mask, name in RG2_GSTA_SAFETY_FLAGS
        if value & mask
    ]

BUTTON_POLL_SEC = 0.03
BUTTON_DEBOUNCE_SEC = 0.05
BUTTON_BASELINE_STABLE_SEC = 0.20
BUTTON_STATE_STALE_SEC = 1.00
BUTTON_SOURCE_WAIT_SEC = 3.00

ROBOT_STATE_TYPE = "dsr_msgs2/msg/RobotState"
ROBOT_STATE_TOPIC_CANDIDATES = (
    "/dsr01/msg/robot_state",
    "/dsr01/robot_state",
    "/dsr01/dsr_controller2/robot_state",
)


def _pose6_from_dsr(value: Any) -> tuple[float, float, float, float, float, float]:
    """get_current_posx() 반환값에서 6축 pose를 추출한다.

    DSR_ROBOT2 버전에 따라 posx 자체 또는 (posx, solution_space) 형태로
    반환될 수 있어 두 형식을 모두 처리한다.
    """
    candidate = value

    if isinstance(value, (list, tuple)) and len(value) == 2:
        first = value[0]
        if hasattr(first, "__iter__") and not isinstance(first, (str, bytes)):
            first_values = list(first)
            if len(first_values) >= 6:
                candidate = first_values

    if hasattr(candidate, "__iter__") and not isinstance(
        candidate, (str, bytes)
    ):
        values = [float(item) for item in list(candidate)]
    else:
        raise RuntimeError(f"현재 TCP pose 형식이 올바르지 않습니다: {value!r}")

    if len(values) < 6 or not all(math.isfinite(item) for item in values[:6]):
        raise RuntimeError(f"현재 TCP pose 값이 올바르지 않습니다: {value!r}")

    return tuple(values[:6])


def _wrap_radians(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 정규화한다."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _json_turn_value(value: float) -> int | float:
    """정수 회전은 JSON 정수로, 부분 회전은 소수로 반환한다."""
    rounded = round(float(value), 3)
    nearest = round(rounded)
    if abs(rounded - nearest) < 1.0e-6:
        return int(nearest)
    return rounded



# =============================================================================
# 스파이럴 자세/경로 수학 유틸리티
# =============================================================================

def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _smoothstep5(progress: float) -> float:
    u = _clamp(progress, 0.0, 1.0)
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def _rot_y_deg(angle_deg: float) -> np.ndarray:
    angle = math.radians(float(angle_deg))
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
        dtype=float,
    )


def _rot_z_deg(angle_deg: float) -> np.ndarray:
    angle = math.radians(float(angle_deg))
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def _zyz_to_rotm(abc_deg) -> np.ndarray:
    a_deg, b_deg, c_deg = [float(value) for value in abc_deg[:3]]
    return _rot_z_deg(a_deg) @ _rot_y_deg(b_deg) @ _rot_z_deg(c_deg)


def _project_rotation(rotation: np.ndarray) -> np.ndarray:
    u_matrix, _, vt_matrix = np.linalg.svd(
        np.asarray(rotation, dtype=float).reshape(3, 3)
    )
    output = u_matrix @ vt_matrix
    if np.linalg.det(output) < 0.0:
        u_matrix[:, -1] *= -1.0
        output = u_matrix @ vt_matrix
    return output


def _rotvec_to_rotm(rotvec_rad) -> np.ndarray:
    vector = np.asarray(rotvec_rad, dtype=float).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-12:
        return np.eye(3, dtype=float)

    axis = vector / angle
    x_axis, y_axis, z_axis = axis
    skew = np.array(
        [
            [0.0, -z_axis, y_axis],
            [z_axis, 0.0, -x_axis],
            [-y_axis, x_axis, 0.0],
        ],
        dtype=float,
    )
    return (
        np.eye(3, dtype=float)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def _rotm_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    matrix = _project_rotation(rotation)
    cosine_angle = _clamp(
        (float(np.trace(matrix)) - 1.0) * 0.5,
        -1.0,
        1.0,
    )
    angle = math.acos(cosine_angle)

    if angle < 1.0e-10:
        return np.zeros(3, dtype=float)

    if abs(math.pi - angle) < 1.0e-6:
        diagonal = np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        if matrix[2, 1] - matrix[1, 2] < 0.0:
            axis[0] *= -1.0
        if matrix[0, 2] - matrix[2, 0] < 0.0:
            axis[1] *= -1.0
        if matrix[1, 0] - matrix[0, 1] < 0.0:
            axis[2] *= -1.0
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1.0e-9:
            axis = np.array([1.0, 0.0, 0.0], dtype=float)
        else:
            axis = axis / axis_norm
        return axis * angle

    factor = angle / (2.0 * math.sin(angle))
    return factor * np.array(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=float,
    )


def _wrapped_delta_deg(value: float, reference: float) -> float:
    return (float(value) - float(reference) + 180.0) % 360.0 - 180.0


def _rotm_to_zyz_candidates(rotation: np.ndarray) -> list[np.ndarray]:
    matrix = _project_rotation(rotation)
    b_angle = math.acos(_clamp(float(matrix[2, 2]), -1.0, 1.0))
    sine_b = math.sin(b_angle)

    if abs(sine_b) > 1.0e-8:
        a_angle = math.atan2(float(matrix[1, 2]), float(matrix[0, 2]))
        c_angle = math.atan2(float(matrix[2, 1]), -float(matrix[2, 0]))
        first = np.degrees(
            np.array([a_angle, b_angle, c_angle], dtype=float)
        )
        second = np.array(
            [first[0] + 180.0, -first[1], first[2] + 180.0],
            dtype=float,
        )
        return [first, second]

    combined = math.degrees(
        math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    )
    b_deg = math.degrees(b_angle)
    return [
        np.array([combined, b_deg, 0.0], dtype=float),
        np.array([0.0, b_deg, combined], dtype=float),
    ]


def _rotm_to_zyz_near(
    rotation: np.ndarray,
    reference_abc_deg,
) -> np.ndarray:
    reference = np.asarray(reference_abc_deg, dtype=float).reshape(3)
    best = None
    best_cost = math.inf

    for candidate in _rotm_to_zyz_candidates(rotation):
        adjusted = candidate.copy()
        for index in (0, 2):
            adjusted[index] = reference[index] + _wrapped_delta_deg(
                adjusted[index],
                reference[index],
            )

        cost = float(
            np.linalg.norm(
                np.array(
                    [
                        _wrapped_delta_deg(adjusted[0], reference[0]),
                        adjusted[1] - reference[1],
                        _wrapped_delta_deg(adjusted[2], reference[2]),
                    ],
                    dtype=float,
                )
            )
        )
        if cost < best_cost:
            best = adjusted
            best_cost = cost

    if best is None:
        raise RuntimeError(
            "회전행렬을 Doosan Z-Y-Z Euler로 변환하지 못했습니다."
        )
    return best


def _numeric6(value: Any, *, label: str) -> np.ndarray:
    candidate = value

    if isinstance(candidate, tuple) and len(candidate) >= 1:
        first = candidate[0]
        if hasattr(first, "__iter__") and not isinstance(
            first,
            (str, bytes),
        ):
            candidate = first
    elif isinstance(candidate, list) and len(candidate) == 2:
        first = candidate[0]
        if hasattr(first, "__iter__") and not isinstance(
            first,
            (str, bytes),
        ):
            first_values = list(first)
            if len(first_values) >= 6:
                candidate = first_values

    try:
        values = np.asarray(
            [float(item) for item in candidate],
            dtype=float,
        )
    except Exception as exc:
        raise RuntimeError(f"{label} 변환 실패: {value!r}") from exc

    if values.size < 6:
        raise RuntimeError(f"{label} 길이 오류: {values.tolist()!r}")

    result = values[:6].copy()
    if not np.all(np.isfinite(result)):
        raise RuntimeError(
            f"{label}에 유효하지 않은 값이 있습니다: {result!r}"
        )
    return result


class GripFailureError(RuntimeError):
    """물체 파지가 확인되지 않아 현재 단계를 다시 시작해야 하는 오류."""

    def __init__(
        self,
        stage_id: str,
        stage_name: str,
        grip_task: str,
        diagnostics: dict[str, Any],
    ) -> None:
        self.stage_id = str(stage_id)
        self.stage_name = str(stage_name)
        self.grip_task = str(grip_task)
        self.diagnostics = dict(diagnostics)
        super().__init__(
            f"{self.stage_name}의 {self.grip_task} 그립을 확인하지 못했습니다."
        )


class GripperSignalLostError(RuntimeError):
    """그리퍼 통신 또는 안전 상태 이상으로 현재 단계를 다시 시작하는 오류."""

    def __init__(self, stage_id: str, stage_name: str, reason: str) -> None:
        self.stage_id = str(stage_id)
        self.stage_name = str(stage_name)
        self.reason = str(reason)
        super().__init__(
            f"{self.stage_name} 실행 중 그리퍼 장비 오류: {self.reason}"
        )


class ButtonWaitCancelled(RuntimeError):
    """안전 조건이 다시 나빠져 물리 버튼 대기를 취소하는 내부 예외."""


class GripMonitor:
    """RG2 그립 판정과 실행 중 통신/안전 상태 감시를 담당한다.

    ``/OnRobotRGInput``의 gSTA와 수신 주기를 장비 건강 상태의 권위 있는
    신호로 사용한다. 파지 여부는 ``/onrobot/grip_detected``를 우선하고
    ``/onrobot_joint_states``의 effort/position을 보조 판정에 사용한다.
    """

    def __init__(self, node) -> None:
        self._node = node
        self._lock = threading.RLock()
        self._bit_value: Optional[bool] = None
        self._bit_stamp = 0.0
        self._joint_position: Optional[float] = None
        self._joint_effort: Optional[float] = None
        self._joint_stamp = 0.0
        self._status_gsta: Optional[int] = None
        self._status_stamp = 0.0
        self._stage_active = False
        self._stage_id = ""
        self._stage_name = ""
        self._signal_lost = False
        self._loss_reason = ""
        self._stop_requested = False
        self._stop_completed = False
        self._stop_success: Optional[bool] = None
        self._last_stop_request_stamp = 0.0
        self._signal_loss_callback: Optional[
            Callable[[GripperSignalLostError], None]
        ] = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._bit_subscription = node.create_subscription(
            Bool,
            GRIP_DETECTED_TOPIC,
            self._bit_callback,
            qos,
        )
        self._joint_subscription = node.create_subscription(
            JointState,
            GRIP_JOINT_STATE_TOPIC,
            self._joint_callback,
            qos,
        )
        self._status_subscription = node.create_subscription(
            OnRobotRGInput,
            GRIP_STATUS_TOPIC,
            self._status_callback,
            qos,
        )
        self._stop_client = node.create_client(MoveStop, "motion/move_stop")
        self._watchdog_timer = node.create_timer(0.05, self._watchdog)

    def _bit_callback(self, msg: Bool) -> None:
        with self._lock:
            self._bit_value = bool(msg.data)
            self._bit_stamp = time.monotonic()

    def _joint_callback(self, msg: JointState) -> None:
        position = None
        effort = None
        if msg.position:
            candidate = float(msg.position[0])
            if math.isfinite(candidate):
                position = candidate
        if msg.effort:
            candidate = float(msg.effort[0])
            if math.isfinite(candidate):
                effort = candidate
        with self._lock:
            self._joint_position = position
            self._joint_effort = effort
            self._joint_stamp = time.monotonic()

    def _status_callback(self, msg: OnRobotRGInput) -> None:
        with self._lock:
            self._status_gsta = int(msg.gsta)
            self._status_stamp = time.monotonic()

    def set_signal_loss_callback(
        self,
        callback: Callable[[GripperSignalLostError], None],
    ) -> None:
        """최초 오류 latch 순간 호출할 비차단 UI 알림 콜백을 등록한다."""
        with self._lock:
            self._signal_loss_callback = callback

    def _signal_health_locked(self, now: float) -> tuple[bool, str]:
        if self._status_stamp <= 0.0:
            return False, f"{GRIP_STATUS_TOPIC} 상태를 한 번도 받지 못했습니다."

        status_age = now - self._status_stamp
        if status_age > GRIP_SIGNAL_TIMEOUT_SEC:
            return (
                False,
                f"{GRIP_STATUS_TOPIC}을 {status_age:.1f}초 동안 받지 못했습니다.",
            )

        gsta = int(self._status_gsta or 0)
        safety_names = _rg2_gsta_safety_names(gsta)
        if safety_names:
            return (
                False,
                "RG2 안전 상태 이상 "
                f"(gSTA=0x{gsta:02X}: {', '.join(safety_names)})",
            )

        return True, ""

    def _signal_online_locked(self, now: float) -> bool:
        return self._signal_health_locked(now)[0]

    def signal_online(self) -> bool:
        with self._lock:
            return self._signal_online_locked(time.monotonic())

    def signal_health_reason(self) -> str:
        with self._lock:
            return self._signal_health_locked(time.monotonic())[1]

    def safety_fault_active(self) -> bool:
        now = time.monotonic()
        with self._lock:
            return bool(
                self._status_stamp > 0.0
                and now - self._status_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
                and int(self._status_gsta or 0) & RG2_GSTA_SAFETY_MASK
            )

    def wait_until_online(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            if self.signal_online():
                return True
            if self.safety_fault_active():
                return False
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        raise KeyboardInterrupt

    def begin_stage(self, stage_id: str, stage_name: str) -> None:
        with self._lock:
            self._stage_active = True
            self._stage_id = str(stage_id)
            self._stage_name = str(stage_name)

    def end_stage(self) -> None:
        with self._lock:
            self._stage_active = False

    def stage_context(self) -> tuple[str, str]:
        with self._lock:
            return self._stage_id, self._stage_name

    def diagnostics(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            status_age = (
                round(now - self._status_stamp, 3)
                if self._status_stamp > 0.0
                else None
            )
            bit_age = (
                round(now - self._bit_stamp, 3)
                if self._bit_stamp > 0.0
                else None
            )
            joint_age = (
                round(now - self._joint_stamp, 3)
                if self._joint_stamp > 0.0
                else None
            )
            signal_online, signal_reason = self._signal_health_locked(now)
            gsta = self._status_gsta
            return {
                "status_topic": GRIP_STATUS_TOPIC,
                "status_age_sec": status_age,
                "gsta": gsta,
                "gsta_hex": (
                    f"0x{int(gsta):02X}"
                    if gsta is not None
                    else None
                ),
                "gsta_safety_flags": (
                    _rg2_gsta_safety_names(gsta)
                    if gsta is not None
                    else []
                ),
                "grip_bit": self._bit_value,
                "grip_bit_age_sec": bit_age,
                "joint_position_rad": (
                    round(self._joint_position, 5)
                    if self._joint_position is not None
                    else None
                ),
                "joint_effort": (
                    round(self._joint_effort, 5)
                    if self._joint_effort is not None
                    else None
                ),
                "joint_state_age_sec": joint_age,
                "signal_online": signal_online,
                "signal_reason": signal_reason,
                "empty_closed_position_rad": GRIP_EMPTY_CLOSED_POSITION_RAD,
                "position_margin_rad": GRIP_POSITION_MARGIN_RAD,
            }

    def _notify_signal_loss(
        self,
        error: GripperSignalLostError,
        callback: Optional[Callable[[GripperSignalLostError], None]],
    ) -> None:
        if callback is None:
            return
        try:
            callback(error)
        except Exception as callback_error:
            self._node.get_logger().error(
                "그리퍼 장비 오류 UI 즉시 알림 실패: "
                f"{type(callback_error).__name__}: {callback_error}"
            )

    def _watchdog(self) -> None:
        now = time.monotonic()
        with self._lock:
            signal_online, signal_reason = self._signal_health_locked(now)
            should_stop = (
                self._stage_active
                and not self._signal_lost
                and not signal_online
            )
            if not should_stop:
                return
            self._signal_lost = True
            self._loss_reason = signal_reason
            stage_id = self._stage_id
            stage_name = self._stage_name
            callback = self._signal_loss_callback

        error = GripperSignalLostError(
            stage_id,
            stage_name,
            signal_reason,
        )

        self._node.get_logger().error(
            f"[그리퍼 장비 오류] 단계={stage_name}: {signal_reason}"
        )
        self.request_motion_stop("그리퍼 장비 오류")
        self._notify_signal_loss(error, callback)

    def force_signal_loss(self, reason: str) -> None:
        with self._lock:
            if self._signal_lost:
                return
            self._signal_lost = True
            self._loss_reason = str(reason)
            stage_id = self._stage_id
            stage_name = self._stage_name
            callback = self._signal_loss_callback
        error = GripperSignalLostError(stage_id, stage_name, str(reason))
        self._node.get_logger().error(f"[그리퍼 장비 오류] {reason}")
        self.request_motion_stop("그리퍼 장비 오류")
        self._notify_signal_loss(error, callback)

    def request_motion_stop(self, reason: str) -> None:
        with self._lock:
            if self._stop_requested:
                return
            self._stop_requested = True
            self._stop_completed = False
            self._stop_success = None
            self._last_stop_request_stamp = time.monotonic()

        if not self._stop_client.service_is_ready():
            with self._lock:
                self._stop_completed = True
                self._stop_success = False
            self._node.get_logger().error(
                f"[{reason}] MoveStop 서비스를 찾지 못해 정지 확인에 실패했습니다."
            )
            return

        request = MoveStop.Request()
        request.stop_mode = GRIP_SIGNAL_STOP_MODE
        future = self._stop_client.call_async(request)
        future.add_done_callback(
            lambda done, stop_reason=reason: self._motion_stop_done(
                done,
                stop_reason,
            )
        )

    def _motion_stop_done(self, future: Any, reason: str) -> None:
        success = False
        error_text = ""
        try:
            response = future.result()
            success = bool(getattr(response, "success", False))
            if not success:
                error_text = "MoveStop 서비스가 실패를 반환했습니다."
        except Exception as error:
            error_text = f"MoveStop 서비스 오류: {error}"
        with self._lock:
            self._stop_completed = True
            self._stop_success = success
        if success:
            self._node.get_logger().warning(f"[{reason}] 로봇 모션 정지 완료")
        else:
            self._node.get_logger().error(f"[{reason}] {error_text}")

    def wait_for_stop_result(self, timeout_sec: float = 3.0) -> Optional[bool]:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            with self._lock:
                if not self._stop_requested:
                    return True
                if self._stop_completed:
                    return self._stop_success
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)
        raise KeyboardInterrupt

    def retry_motion_stop(self, reason: str) -> None:
        """이전 정지 요청이 실패한 경우 서비스가 복구됐을 때 다시 요청한다."""
        with self._lock:
            if self._stop_success is True or not self._stop_completed:
                return
            if time.monotonic() - self._last_stop_request_stamp < 1.0:
                return
            self._stop_requested = False
            self._stop_completed = False
            self._stop_success = None
            self._last_stop_request_stamp = 0.0
        self.request_motion_stop(reason)

    def raise_if_signal_lost(self) -> None:
        with self._lock:
            if not self._signal_lost:
                return
            stage_id = self._stage_id
            stage_name = self._stage_name
            reason = self._loss_reason
        raise GripperSignalLostError(stage_id, stage_name, reason)

    def clear_signal_loss_if_healthy(self) -> bool:
        """lock 안에서 최종 건강 상태를 확인하고 오류 latch를 해제한다."""
        with self._lock:
            signal_online, _ = self._signal_health_locked(time.monotonic())
            if not signal_online:
                return False
            self._signal_lost = False
            self._loss_reason = ""
            self._stop_requested = False
            self._stop_completed = False
            self._stop_success = None
            self._last_stop_request_stamp = 0.0
            return True

    def verify_grip(
        self,
        command_stamp: float,
        timeout_sec: float = GRIP_VERIFY_TIMEOUT_SEC,
    ) -> tuple[bool, dict[str, Any]]:
        """그리퍼 닫기 이후 새 상태 표본으로 파지 여부를 판정한다."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))

        while rclpy.ok():
            self.raise_if_signal_lost()
            now = time.monotonic()
            with self._lock:
                bit_is_new = self._bit_stamp >= command_stamp
                bit_is_fresh = (
                    bit_is_new
                    and now - self._bit_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
                )
                bit_value = self._bit_value

            # 하드웨어 비트 true는 즉시 성공으로 확정한다. false는 닫힘 동작이
            # 끝날 때까지 바뀔 수 있으므로 검증 제한시간까지 기다린다.
            if bit_is_fresh and bit_value is True:
                diagnostics = self.diagnostics()
                diagnostics["decision_source"] = "hardware_grip_bit"
                return True, diagnostics

            if now >= deadline:
                break
            time.sleep(0.02)

        now = time.monotonic()
        diagnostics = self.diagnostics()
        with self._lock:
            bit_is_fresh = (
                self._bit_stamp >= command_stamp
                and now - self._bit_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
            )
            joint_is_fresh = (
                self._joint_stamp >= command_stamp
                and now - self._joint_stamp <= GRIP_SIGNAL_TIMEOUT_SEC
            )
            bit_value = self._bit_value
            joint_position = self._joint_position
            joint_effort = self._joint_effort

        if bit_is_fresh:
            diagnostics["decision_source"] = "hardware_grip_bit"
            return bool(bit_value), diagnostics

        if joint_is_fresh:
            effort_grip = (
                joint_effort is not None
                and abs(joint_effort) > GRIP_EFFORT_MIN
            )
            position_grip = (
                joint_position is not None
                and joint_position
                < GRIP_EMPTY_CLOSED_POSITION_RAD - GRIP_POSITION_MARGIN_RAD
            )
            diagnostics["decision_source"] = (
                "joint_effort" if effort_grip else "joint_position"
            )
            return effort_grip or position_grip, diagnostics

        self.force_signal_loss("그립 확인용 새 상태 표본을 받지 못했습니다.")
        self.raise_if_signal_lost()
        raise AssertionError("unreachable")


class StatusReporter:
    """FastAPI Web UI가 구독할 JSON 상태를 발행한다."""

    def __init__(
        self,
        node,
        speed_getter: Optional[Callable[[], int]] = None,
    ) -> None:
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = node.create_publisher(String, STATUS_TOPIC, qos)
        self._speed_getter = speed_getter
        self._publish_lock = threading.RLock()
        self._equipment_error_active = False
        self._selected_bean = ""
        self._selected_button: Optional[int] = None
        self._selected_grind = ""
        self._selected_grind_button: Optional[int] = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0
        self._cycle_id = 0
        self._test_mode = False
        self._test_stage_id = ""
        self._test_stage_name = ""
        self._test_result = ""

    def begin_cycle(
        self,
        *,
        test_mode: bool,
        test_stage_id: str = "",
        test_stage_name: str = "",
    ) -> None:
        self._cycle_id += 1
        self._test_mode = bool(test_mode)
        self._test_stage_id = str(test_stage_id)
        self._test_stage_name = str(test_stage_name)
        self._test_result = "RUNNING"
        self._selected_bean = ""
        self._selected_button = None
        self._selected_grind = ""
        self._selected_grind_button = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0

    def finish_cycle(self, result: str) -> None:
        self._test_result = str(result)

    def clear_test_context(self) -> None:
        self._test_mode = False
        self._test_stage_id = ""
        self._test_stage_name = ""
        self._test_result = ""

    def reset_order_state(self) -> None:
        """새 주문 초기 화면에 이전 주문의 선택/진행 상태를 남기지 않는다."""
        self._selected_bean = ""
        self._selected_button = None
        self._selected_grind = ""
        self._selected_grind_button = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0

    def set_selection(
        self,
        button: Optional[int],
        bean_name: str,
    ) -> None:
        self._selected_button = button
        self._selected_bean = bean_name

    def set_grind_selection(
        self,
        button: Optional[int],
        grind_name: str,
        turns: int,
    ) -> None:
        self._selected_grind_button = button
        self._selected_grind = grind_name
        self._grind_turns = turns
        self._grind_current_turns = 0.0

    def set_equipment_error_active(self, active: bool) -> None:
        """장비 오류 중 일반 공정 화면이 오류 화면을 덮지 못하게 한다."""
        with self._publish_lock:
            self._equipment_error_active = bool(active)

    def publish(self, **kwargs: Any) -> None:
        with self._publish_lock:
            if (
                self._equipment_error_active
                and not bool(kwargs.get("equipment_error", False))
            ):
                return
            self._publish_impl(**kwargs)

    def _publish_impl(
        self,
        *,
        phase: str,
        screen: int,
        progress: int,
        title: str,
        message: str,
        busy: bool,
        waiting_physical_button: bool = False,
        waiting_external_force: bool = False,
        force_delta_n: Optional[float] = None,
        force_peak_n: Optional[float] = None,
        grind_current_turns: Optional[float] = None,
        spiral_stage: Optional[str] = None,
        spiral_progress: Optional[float] = None,
        final_drip_stage: Optional[str] = None,
        final_drip_progress: Optional[float] = None,
        selection_timeout_sec: Optional[float] = None,
        wait_remaining_sec: Optional[float] = None,
        grip_failure: bool = False,
        equipment_error: bool = False,
        failed_stage_id: str = "",
        failed_stage_name: str = "",
        failed_grip_task: str = "",
        recovery_kind: str = "",
        recovery_state: str = "",
        grip_diagnostics: Optional[dict[str, Any]] = None,
        gripper_signal_online: Optional[bool] = None,
        recovery_countdown_sec: float = 0.0,
        button: Optional[int] = None,
        error: str = "",
    ) -> None:
        if force_delta_n is not None:
            self._force_delta_n = float(force_delta_n)
        if force_peak_n is not None:
            self._force_peak_n = float(force_peak_n)
        if grind_current_turns is not None:
            current = max(0.0, float(grind_current_turns))
            if self._grind_turns > 0:
                current = min(current, float(self._grind_turns))
            self._grind_current_turns = current
        if spiral_stage is not None:
            self._spiral_stage = str(spiral_stage)
        if spiral_progress is not None:
            self._spiral_progress = max(
                0.0,
                min(100.0, float(spiral_progress)),
            )
        if final_drip_stage is not None:
            self._final_drip_stage = str(final_drip_stage)
        if final_drip_progress is not None:
            self._final_drip_progress = max(
                0.0,
                min(100.0, float(final_drip_progress)),
            )

        grind_progress = 0.0
        if self._grind_turns > 0:
            grind_progress = (
                self._grind_current_turns / float(self._grind_turns) * 100.0
            )

        speed_percent = OPERATION_SPEED_DEFAULT_PERCENT
        if self._speed_getter is not None:
            try:
                speed_percent = int(self._speed_getter())
            except Exception:
                speed_percent = OPERATION_SPEED_DEFAULT_PERCENT

        payload = {
            "phase": phase,
            "screen": screen,
            "progress": progress,
            "title": title,
            "message": message,
            "busy": busy,
            "waiting_physical_button": waiting_physical_button,
            "waiting_external_force": waiting_external_force,
            "selection_timeout_sec": (
                round(max(0.0, float(selection_timeout_sec)), 1)
                if selection_timeout_sec is not None
                else None
            ),
            "external_force_timeout_sec": EXTERNAL_FORCE_TIMEOUT_SEC,
            "wait_remaining_sec": (
                round(max(0.0, float(wait_remaining_sec)), 1)
                if wait_remaining_sec is not None
                else None
            ),
            "force_threshold_n": EXTERNAL_FORCE_TRIGGER_N,
            "force_delta_n": round(self._force_delta_n, 3),
            "force_peak_n": round(self._force_peak_n, 3),
            "selected_bean": self._selected_bean,
            "selected_button": self._selected_button,
            "selected_grind": self._selected_grind,
            "selected_grind_button": self._selected_grind_button,
            "grind_turns": self._grind_turns,
            "grind_current_turns": _json_turn_value(
                self._grind_current_turns
            ),
            "grind_progress": round(grind_progress, 1),
            "spiral_stage": self._spiral_stage,
            "spiral_progress": round(self._spiral_progress, 1),
            "spiral_radius_mm": SPIRAL_RADIUS_MM,
            "spiral_revolutions": SPIRAL_REVOLUTIONS,
            "spiral_duration_sec": SPIRAL_DURATION_SEC,
            "spiral_return_duration_sec": CENTER_PIVOT_RETURN_DURATION_SEC,
            "spiral_j6_delta_deg": SPIRAL_J6_DELTA_DEG,
            "final_drip_stage": self._final_drip_stage,
            "final_drip_progress": round(self._final_drip_progress, 1),
            "final_pour_linear_vel_mm_s": VELX_LIN_DEFAULT,
            "final_pour_angular_vel_deg_s": FINAL_POUR_ANGULAR_VEL_DEG_S,
            "final_pour_linear_acc_mm_s2": ACCX_LIN_DEFAULT,
            "final_pour_angular_acc_deg_s2": FINAL_POUR_ANGULAR_ACC_DEG_S2,
            "final_return_angular_vel_deg_s": FINAL_RETURN_ANGULAR_VEL_DEG_S,
            "final_return_angular_acc_deg_s2": FINAL_RETURN_ANGULAR_ACC_DEG_S2,
            "final_pour_j6_delta_deg": FINAL_POUR_J6_DELTA_DEG,
            "final_pour_tcp_name": FINAL_POUR_TCP_NAME,
            "final_pour_restore_tcp_name": TCP_NAME,
            "operation_speed_percent": speed_percent,
            "test_mode": self._test_mode,
            "test_stage_id": self._test_stage_id,
            "test_stage_name": self._test_stage_name,
            "test_result": self._test_result,
            "grip_failure": bool(grip_failure),
            "equipment_error": bool(equipment_error),
            "failed_stage_id": str(failed_stage_id),
            "failed_stage_name": str(failed_stage_name),
            "failed_grip_task": str(failed_grip_task),
            "recovery_kind": str(recovery_kind),
            "recovery_state": str(recovery_state),
            "grip_diagnostics": dict(grip_diagnostics or {}),
            "gripper_signal_online": gripper_signal_online,
            "recovery_countdown_sec": round(
                max(0.0, float(recovery_countdown_sec)),
                1,
            ),
            "button": button,
            "cycle_id": self._cycle_id,
            "error": error,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)


class CoffeeControlBridge:
    """웹 테스트 명령 큐와 실시간 작동 속도 서비스를 관리한다.

    메인 로봇 노드와 별도 ROS 노드/Executor를 사용하므로 동기 movej/movel 실행
    중에도 ``change_operation_speed`` 서비스 요청을 처리할 수 있다.
    """

    def __init__(self, speed_service_type: Any) -> None:
        self.node = rclpy.create_node(
            f"{NODE_NAME}_web_control",
            namespace=ROBOT_ID,
        )
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._lock = threading.RLock()
        self._test_commands: deque[dict[str, Any]] = deque(maxlen=4)
        self._busy = False
        self._desired_speed = OPERATION_SPEED_DEFAULT_PERCENT
        self._applied_speed = OPERATION_SPEED_DEFAULT_PERCENT
        self._speed_dirty = True
        self._speed_future = None
        self._last_speed_error = ""
        self._last_speed_change_log = _speed_change_lines(
            OPERATION_SPEED_DEFAULT_PERCENT,
            OPERATION_SPEED_DEFAULT_PERCENT,
        )
        self._speed_service_type = speed_service_type
        self._speed_client = None

        self._status_publisher = self.node.create_publisher(
            String,
            STATUS_TOPIC,
            status_qos,
        )
        self.node.create_subscription(
            String,
            CONTROL_TOPIC,
            self._command_callback,
            qos,
        )
        if speed_service_type is not None:
            self._speed_client = self.node.create_client(
                speed_service_type,
                "motion/change_operation_speed",
            )
        self.node.create_timer(0.10, self._flush_speed_request)
        self.node.get_logger().info(
            "실시간 속도 로그 활성화: change_operation_speed 적용 시 "
            "명명된 속도 변수의 이전/신규 유효값을 출력합니다."
        )

    def _publish_partial(self, **payload: Any) -> None:
        payload["timestamp"] = time.time()
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(msg)

    def _command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        command = str(payload.get("cmd", "")).strip()
        if command == "set_speed":
            try:
                requested = int(round(float(payload.get("speed_percent"))))
            except (TypeError, ValueError):
                self._publish_partial(
                    speed_update_error="속도 값이 숫자가 아닙니다.",
                )
                return
            requested = max(
                OPERATION_SPEED_MIN_PERCENT,
                min(OPERATION_SPEED_MAX_PERCENT, requested),
            )
            with self._lock:
                previous_applied = int(self._applied_speed)
                self._desired_speed = requested
                self._speed_dirty = True
                self._last_speed_error = ""
            self.node.get_logger().info(
                f"[속도 변경 요청] 현재 적용={previous_applied}%, 요청={requested}%"
            )
            self._publish_partial(
                operation_speed_percent=requested,
                speed_previous_percent=previous_applied,
                speed_update_pending=True,
                speed_update_error="",
            )
            return

        if command != "start_test":
            return

        stage = str(payload.get("stage", "")).strip()
        if stage not in TEST_STAGE_NAMES:
            self._publish_partial(
                test_command_error=f"허용되지 않은 테스트 단계: {stage}",
            )
            return

        try:
            grind_turns = int(payload.get("grind_turns", 3))
        except (TypeError, ValueError):
            grind_turns = 3
        if grind_turns not in {3, 5, 7, 10}:
            grind_turns = 3

        open_mode = str(
            payload.get("gripper_open_mode", "spoon_cup")
        ).strip()
        if open_mode not in TEST_GRIP_OPEN_MODES:
            open_mode = "spoon_cup"

        normalized = {
            "stage": stage,
            "grind_turns": grind_turns,
            "gripper_open_mode": open_mode,
        }
        with self._lock:
            if self._busy or self._test_commands:
                self._publish_partial(
                    test_command_error="현재 테스트 또는 공정이 실행 중입니다.",
                )
                return
            self._test_commands.append(normalized)
        self._publish_partial(test_command_error="")

    def _flush_speed_request(self) -> None:
        with self._lock:
            if not self._speed_dirty or self._speed_future is not None:
                return
            desired = int(self._desired_speed)

        if self._speed_client is None or self._speed_service_type is None:
            with self._lock:
                self._speed_dirty = False
                self._last_speed_error = (
                    "dsr_msgs2/srv/ChangeOperationSpeed를 불러오지 못했습니다."
                )
            self.node.get_logger().error(
                f"[속도 변경 실패] {self._last_speed_error}"
            )
            self._publish_partial(
                speed_service_ready=False,
                speed_update_pending=False,
                speed_update_error=self._last_speed_error,
                speed_change_log=[self._last_speed_error],
            )
            return

        if not self._speed_client.service_is_ready():
            self._publish_partial(speed_service_ready=False)
            return

        request = self._speed_service_type.Request()
        request.speed = desired
        future = self._speed_client.call_async(request)
        with self._lock:
            self._speed_future = future
        future.add_done_callback(
            lambda done, requested=desired: self._speed_done(done, requested)
        )

    def _speed_done(self, future: Any, requested: int) -> None:
        success = False
        error_text = ""
        try:
            response = future.result()
            success = bool(getattr(response, "success", False))
            if not success:
                error_text = "change_operation_speed 서비스가 실패를 반환했습니다."
        except Exception as error:
            error_text = f"change_operation_speed 서비스 오류: {error}"

        with self._lock:
            previous_applied = int(self._applied_speed)
            self._speed_future = None
            if success:
                self._applied_speed = requested
                self._last_speed_error = ""
                self._speed_dirty = self._desired_speed != requested
                change_lines = _speed_change_lines(previous_applied, requested)
                self._last_speed_change_log = list(change_lines)
            else:
                self._last_speed_error = error_text
                self._speed_dirty = False
                change_lines = [
                    f"작동 속도 변경 실패: {previous_applied}% -> {requested}%",
                    error_text,
                ]
                self._last_speed_change_log = list(change_lines)

        if success:
            self.node.get_logger().info(
                f"[속도 변경 적용 완료] {previous_applied}% -> {requested}%"
            )
            for line in change_lines[1:]:
                self.node.get_logger().info(f"[속도 변수] {line}")
        else:
            self.node.get_logger().error(
                f"[속도 변경 실패] {previous_applied}% -> {requested}%: {error_text}"
            )

        applied_percent = requested if success else previous_applied
        self._publish_partial(
            operation_speed_percent=applied_percent,
            speed_previous_percent=previous_applied,
            speed_applied_percent=applied_percent,
            speed_effective_variables=_effective_speed_payload(applied_percent),
            speed_change_log=change_lines,
            speed_service_ready=True,
            speed_update_pending=False,
            speed_update_error=error_text,
        )

    def pop_test_command(self) -> Optional[dict[str, Any]]:
        with self._lock:
            if not self._test_commands:
                return None
            return self._test_commands.popleft()

    def set_busy(self, busy: bool) -> None:
        with self._lock:
            self._busy = bool(busy)
            if not busy:
                self._test_commands.clear()

    def speed_percent(self) -> int:
        with self._lock:
            return int(self._desired_speed)

    def destroy(self) -> None:
        self.node.destroy_node()


class PhysicalButtonInput:
    """DI 13~16을 RobotState 우선, get_digital_input() 보조로 감지한다.

    버튼 해제 상태를 0으로 가정하지 않고 안정된 현재값을 기준값으로 사용하므로
    Active-High와 Active-Low 배선을 모두 처리한다.
    """

    def __init__(
        self,
        node,
        get_digital_input: Callable[[int], int],
    ) -> None:
        self._node = node
        self._get_digital_input = get_digital_input
        self._subscriptions = {}
        self._robot_state_buttons = {}
        self._robot_state_stamp = 0.0
        self._source = "uninitialized"
        self._last_wait_defaulted = False

        for topic in ROBOT_STATE_TOPIC_CANDIDATES:
            self._register_topic(topic)

    def _register_topic(self, topic: str) -> None:
        if not topic or topic in self._subscriptions:
            return

        subscription = self._node.create_subscription(
            RobotState,
            topic,
            lambda msg, source=topic: self._robot_state_callback(msg, source),
            10,
        )
        self._subscriptions[topic] = subscription
        self._node.get_logger().info(f"RobotState 구독 등록: {topic}")

    def _discover_topics(self) -> None:
        try:
            topics = self._node.get_topic_names_and_types()
        except Exception as error:
            self._node.get_logger().warning(f"RobotState 토픽 탐색 실패: {error}")
            return

        for topic, type_names in topics:
            if ROBOT_STATE_TYPE in type_names:
                self._register_topic(topic)

    def _robot_state_callback(self, msg: RobotState, source: str) -> None:
        values = {}

        if hasattr(msg, "controller_digital_input"):
            mask = int(getattr(msg, "controller_digital_input"))
            values = {
                index: (mask >> (index - 1)) & 0x1
                for index in PHYSICAL_BUTTONS
            }
        elif hasattr(msg, "ctrlbox_digital_input"):
            raw = list(getattr(msg, "ctrlbox_digital_input"))
            if len(raw) >= max(PHYSICAL_BUTTONS):
                values = {
                    index: int(raw[index - 1])
                    for index in PHYSICAL_BUTTONS
                }

        if not values:
            return

        self._robot_state_buttons = values
        self._robot_state_stamp = time.monotonic()
        new_source = f"RobotState:{source}"

        if self._source != new_source:
            self._source = new_source
            self._node.get_logger().info(
                f"물리 버튼 입력 소스 전환: {self._source}, raw={values}"
            )

    def _spin_once(self, timeout_sec: float = 0.0) -> None:
        rclpy.spin_once(self._node, timeout_sec=max(0.0, timeout_sec))

    def _robot_state_is_fresh(self) -> bool:
        return (
            bool(self._robot_state_buttons)
            and time.monotonic() - self._robot_state_stamp
            <= BUTTON_STATE_STALE_SEC
        )

    @staticmethod
    def _raise_if_cancelled(
        cancel_if: Optional[Callable[[], bool]],
    ) -> None:
        if cancel_if is not None and cancel_if():
            raise ButtonWaitCancelled("안전 조건이 해제되어 버튼 대기를 취소합니다.")

    def _wait_for_source(
        self,
        cancel_if: Optional[Callable[[], bool]] = None,
    ) -> None:
        deadline = time.monotonic() + BUTTON_SOURCE_WAIT_SEC
        self._raise_if_cancelled(cancel_if)
        self._discover_topics()

        while rclpy.ok():
            self._raise_if_cancelled(cancel_if)
            self._spin_once(0.05)
            self._raise_if_cancelled(cancel_if)
            self._discover_topics()

            if self._robot_state_is_fresh():
                return

            if time.monotonic() >= deadline:
                self._node.get_logger().warning(
                    "RobotState DI를 받지 못해 get_digital_input()으로 대체합니다."
                )
                return

    def read(self) -> dict[int, int]:
        self._spin_once(0.0)
        self._discover_topics()

        if self._robot_state_is_fresh():
            return dict(self._robot_state_buttons)

        values = {
            index: int(self._get_digital_input(index))
            for index in PHYSICAL_BUTTONS
        }

        if self._source != "DSR:get_digital_input":
            self._source = "DSR:get_digital_input"
            self._node.get_logger().warning(
                f"물리 버튼 입력 소스: {self._source}, raw={values}"
            )

        return values

    def _stable_baseline(
        self,
        cancel_if: Optional[Callable[[], bool]] = None,
    ) -> dict[int, int]:
        self._wait_for_source(cancel_if)
        baseline = None
        stable_since = time.monotonic()

        while rclpy.ok():
            self._raise_if_cancelled(cancel_if)
            current = self.read()
            self._raise_if_cancelled(cancel_if)
            now = time.monotonic()

            if baseline != current:
                baseline = dict(current)
                stable_since = now
                self._node.get_logger().info(
                    f"버튼 기준값 확인 ({self._source}): {baseline}"
                )

            if now - stable_since >= BUTTON_BASELINE_STABLE_SEC:
                return dict(baseline)

            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt

    def wait_for_button(
        self,
        *,
        timeout_sec: Optional[float] = None,
        default_button: Optional[int] = None,
        allowed_buttons: Optional[set[int]] = None,
        cancel_if: Optional[Callable[[], bool]] = None,
    ) -> int:
        self._last_wait_defaulted = False
        wait_started = time.monotonic()
        baseline = self._stable_baseline(cancel_if)
        allowed = set(PHYSICAL_BUTTONS)
        if allowed_buttons is not None:
            allowed &= {int(index) for index in allowed_buttons}
        if not allowed:
            raise ValueError("대기할 물리 버튼이 하나도 없습니다.")
        if default_button is not None and default_button not in allowed:
            raise ValueError("기본 버튼은 허용된 버튼 중 하나여야 합니다.")
        deadline = (
            wait_started + max(0.0, float(timeout_sec))
            if timeout_sec is not None
            else None
        )
        self._node.get_logger().info(
            f"DI {sorted(allowed)} 버튼 입력 대기: "
            f"source={self._source}, baseline={baseline}, "
            f"timeout={timeout_sec}"
        )

        while rclpy.ok():
            self._raise_if_cancelled(cancel_if)
            if deadline is not None and time.monotonic() >= deadline:
                if default_button is None:
                    raise TimeoutError("물리 버튼 입력 제한시간을 초과했습니다.")
                self._node.get_logger().warning(
                    f"물리 버튼 입력 {float(timeout_sec):.1f}초 초과: "
                    f"DI {default_button}을 기본값으로 선택합니다."
                )
                self._last_wait_defaulted = True
                return int(default_button)

            current = self.read()
            self._raise_if_cancelled(cancel_if)
            changed = [
                index
                for index in sorted(allowed)
                if current[index] != baseline[index]
            ]

            if changed:
                index = changed[0]
                time.sleep(BUTTON_DEBOUNCE_SEC)
                self._raise_if_cancelled(cancel_if)
                confirmed = self.read()
                self._raise_if_cancelled(cancel_if)

                if confirmed[index] != baseline[index]:
                    self._node.get_logger().info(
                        f"물리 버튼 DI {index} 감지: "
                        f"{baseline[index]} -> {confirmed[index]}"
                    )

                    while rclpy.ok():
                        self._raise_if_cancelled(cancel_if)
                        released = self.read()
                        self._raise_if_cancelled(cancel_if)
                        if released[index] == baseline[index]:
                            self._node.get_logger().info(
                                f"물리 버튼 DI {index} 해제 확인"
                            )
                            return index
                        time.sleep(BUTTON_POLL_SEC)

            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt


    def wait_for_button_or_command(
        self,
        command_getter: Callable[[], Optional[dict[str, Any]]],
        *,
        timeout_sec: Optional[float] = None,
        default_button: Optional[int] = None,
    ) -> tuple[str, int | dict[str, Any]]:
        """대기 중 DI 버튼과 웹 테스트 명령 중 먼저 들어온 항목을 반환한다."""
        self._last_wait_defaulted = False
        wait_started = time.monotonic()
        baseline = self._stable_baseline()
        deadline = (
            wait_started + max(0.0, float(timeout_sec))
            if timeout_sec is not None
            else None
        )
        if default_button is not None and default_button not in PHYSICAL_BUTTONS:
            raise ValueError("기본 버튼은 DI 13~16 중 하나여야 합니다.")
        self._node.get_logger().info(
            "대기 상태: DI 13~16 또는 웹 단계 테스트 명령 입력 대기, "
            f"source={self._source}, baseline={baseline}, "
            f"timeout={timeout_sec}"
        )

        while rclpy.ok():
            command = command_getter()
            if command is not None:
                return "command", command

            if deadline is not None and time.monotonic() >= deadline:
                if default_button is None:
                    raise TimeoutError("버튼/명령 입력 제한시간을 초과했습니다.")
                self._node.get_logger().warning(
                    f"원두 선택 {float(timeout_sec):.1f}초 초과: "
                    f"DI {default_button}을 기본값으로 선택합니다."
                )
                self._last_wait_defaulted = True
                return "button", int(default_button)

            current = self.read()
            changed = [
                index
                for index in PHYSICAL_BUTTONS
                if current[index] != baseline[index]
            ]
            if changed:
                index = changed[0]
                time.sleep(BUTTON_DEBOUNCE_SEC)
                confirmed = self.read()
                if confirmed[index] != baseline[index]:
                    self._node.get_logger().info(
                        f"물리 버튼 DI {index} 감지: "
                        f"{baseline[index]} -> {confirmed[index]}"
                    )
                    while rclpy.ok():
                        command = command_getter()
                        if command is not None:
                            return "command", command
                        released = self.read()
                        if released[index] == baseline[index]:
                            self._node.get_logger().info(
                                f"물리 버튼 DI {index} 해제 확인"
                            )
                            return "button", index
                        time.sleep(BUTTON_POLL_SEC)
            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt

    def last_wait_defaulted(self) -> bool:
        return bool(self._last_wait_defaulted)


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

    try:
        import DSR_ROBOT2 as dsr2
    except ImportError as error:
        node.get_logger().error(f"DSR_ROBOT2 모듈을 불러오지 못했습니다: {error}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # DSR_ROBOT2 import 시점에 생성되는 ~100개 서비스 클라이언트가 컨트롤러와 DDS
    # discovery를 마칠 때까지 기다린다. set_singular_handling() 등 일부 API는
    # movej()와 달리 wait_for_service() 없이 곧바로 call_async()를 던지므로,
    # discovery가 끝나기 전에 첫 호출이 나가면 응답이 영영 오지 않아
    # spin_until_future_complete()가 무한 대기에 빠진다. (고정 sleep은 시스템
    # 부하에 따라 부족할 수 있어 실패 사례가 있었다 — 같은 노드로 대표 서비스
    # 하나가 실제로 매칭될 때까지 기다려서 나머지 클라이언트도 함께 뜨게 한다.)
    startup_probe = node.create_client(MoveJoint, "motion/move_joint")
    while not startup_probe.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("컨트롤러 서비스 연결 대기 중...")
    node.destroy_client(startup_probe)

    # -------------------------------------------------------------------
    # API 바인딩 (DRL 원본과 동일한 이름으로 사용)
    # -------------------------------------------------------------------
    movej = dsr2.movej
    movel = dsr2.movel
    movec = dsr2.movec
    movesx = dsr2.movesx
    amovec = getattr(dsr2, "amovec", None)
    amovel = dsr2.amovel
    move_periodic = dsr2.move_periodic
    task_compliance_ctrl = dsr2.task_compliance_ctrl
    release_compliance_ctrl = dsr2.release_compliance_ctrl
    set_stiffnessx = dsr2.set_stiffnessx
    set_digital_output = dsr2.set_digital_output
    get_digital_input = dsr2.get_digital_input
    get_tool_force = dsr2.get_tool_force
    get_current_posx = dsr2.get_current_posx
    get_current_posj = dsr2.get_current_posj
    fkin = dsr2.fkin
    check_motion = getattr(dsr2, "check_motion", None)
    mwait = getattr(dsr2, "mwait", None)
    set_tool = dsr2.set_tool
    set_tcp = dsr2.set_tcp
    get_tool = dsr2.get_tool
    get_tcp = dsr2.get_tcp
    drl_script_run = dsr2.drl_script_run
    get_drl_state = dsr2.get_drl_state
    get_robot_system = dsr2.get_robot_system
    set_singularity_handling = dsr2.set_singularity_handling
    set_velj = dsr2.set_velj
    set_accj = dsr2.set_accj
    set_velx = dsr2.set_velx
    set_accx = dsr2.set_accx
    wait = dsr2.wait
    posj = dsr2.posj
    posx = dsr2.posx

    DR_BASE = dsr2.DR_BASE
    DR_TOOL = dsr2.DR_TOOL
    DR_MV_MOD_ABS = dsr2.DR_MV_MOD_ABS
    DR_MV_MOD_REL = dsr2.DR_MV_MOD_REL
    DR_MV_RA_DUPLICATE = dsr2.DR_MV_RA_DUPLICATE
    DR_MV_RA_OVERRIDE = dsr2.DR_MV_RA_OVERRIDE
    DR_MVS_VEL_NONE = getattr(dsr2, "DR_MVS_VEL_NONE", 0)
    DR_AVOID = dsr2.DR_AVOID
    # DR_OFF = dsr2.DR_OFF  # 원본에 있었으나 DSR_ROBOT2에 DR_OFF 상수 자체가 없어
    # AttributeError로 즉시 죽었음. set_velx() 3번째 인자로만 쓰였는데 그 인자
    # 자체가 실제 시그니처에 없어서 통째로 제거함 (아래 set_velx() 호출부 참고).
    ON = getattr(dsr2, "ON", 1)
    OFF = getattr(dsr2, "OFF", 0)

    try:
        from dsr_msgs2.srv import ChangeOperationSpeed
    except ImportError as error:
        ChangeOperationSpeed = None
        node.get_logger().warning(
            "ChangeOperationSpeed 서비스 타입을 불러오지 못했습니다: "
            f"{error}"
        )

    from rclpy.executors import SingleThreadedExecutor

    control = CoffeeControlBridge(ChangeOperationSpeed)
    grip_monitor = GripMonitor(control.node)
    control_executor = SingleThreadedExecutor()
    control_executor.add_node(control.node)
    control_spin_thread = threading.Thread(
        target=control_executor.spin,
        name="coffee-control-spin",
        daemon=True,
    )
    control_spin_thread.start()

    def guarded_motion(function: Callable[..., Any]) -> Callable[..., Any]:
        """신호 단절 전후에 새 모션 명령이 이어지지 않도록 감싼다."""

        def call(*motion_args: Any, **motion_kwargs: Any) -> Any:
            grip_monitor.raise_if_signal_lost()
            try:
                result = function(*motion_args, **motion_kwargs)
            except BaseException:
                # MoveStop 때문에 원래 모션 서비스가 먼저 예외를 반환하더라도
                # 장비 오류 복구 경로가 일반 로봇 오류 화면으로 빠지지 않게 한다.
                grip_monitor.raise_if_signal_lost()
                raise
            grip_monitor.raise_if_signal_lost()
            return result

        return call

    movej = guarded_motion(movej)
    movel = guarded_motion(movel)
    movec = guarded_motion(movec)
    movesx = guarded_motion(movesx)
    amovel = guarded_motion(amovel)
    move_periodic = guarded_motion(move_periodic)
    if amovec is not None:
        amovec = guarded_motion(amovec)
    if callable(mwait):
        mwait = guarded_motion(mwait)

    status = StatusReporter(node, speed_getter=control.speed_percent)
    buttons = PhysicalButtonInput(node, get_digital_input)

    # =====================================================================
    # Teach 포즈 (System.drvar 원본 값)
    # =====================================================================
    System_spoon_j = posj(-69.6, 53.27, 96.56, 25.63, 114.79, -171.03)
    System_spoon_l = posx(251.01, -118.46, 40.17, 87.01, 92.73, -3.35)
    System_grinder_j = posj(-40.58, -9.25, 131.96, 78.44, 107.42, -125.46)
    System_grinder_l = posx(371.82, 168.64, 360.0, 61.05, 86.52, 56.67)
    System_handle_j = posj(15.05, 35.75, 18.15, -1.7, 124.03, 13.38)
    System_handle_l = posx(532.780, 127.380, 279.83, 148.81, 178.39, 148.08)
    System_bottle_j_1 = posj(-19.18, 34.8, 22.24, -2.53, 124.42, -13.34)
    System_bottle_j_2 = posj(-43.18, 55.32, 88.81, 53.44, 114.38, -60.57)
    System_bottle_l = posx(401.53, 160.78, 71.68, 92.22, 89.54, 89.25)
    System_bottle_l2 = posx(401.53, 160.78, 76.68, 92.22, 89.54, 89.25)
    System_drip_j = posj(-26.64, 44.1, 73.65, 89.85, 91.27, -26.21)
    System_drip_l = posx(751.71, 41.51, 238.65, 60.13, 91.27, -92.13)
    System_home = posj(-71.33, 47.91, 97.52, 18.17, 114.49, -168.37)
    System_spoon_l_2 = posx(251.01, -118.46, 50.17, 87.01, 92.73, -3.35)

    # 스파이럴 드립 Sub 교시 포즈
    System_pot_grip = posx(*SYSTEM_POT_GRIP_VALUES)
    System_pot_grip_joint = posj(*SYSTEM_POT_GRIP_JOINT_VALUES)
    Pour_start_joint = posj(*POUR_START_JOINT_VALUES)
    Pickup_approach_rel = posx(*PICKUP_APPROACH_REL_VALUES)
    Pickup_lift_rel = posx(*PICKUP_LIFT_REL_VALUES)

    # final_drip Sub 교시 포즈 (System(5).drvar)
    System_fitter_j = posj(-30.99, 52.68, 70.94, 74.39, 112.41, -306.08)
    System_filtter_l = posx(653.6, 17.98, 181.99, 85.67, 90.12, -179.93)
    System_filtter_l2 = posx(349.44, 4.5, 96.59, 90.83, 90.78, 174.75)
    System_mug_j = posj(-32.73, 54.93, 83.19, 82.8, 93.84, -10.72)
    System_mug_l = posx(655.06, 48.93, 82.17, 70.19, 92.83, 123.59)
    System_mug_release_l = posx(648.06, 48.93, 87.17, 70.19, 92.83, 123.59)
    System_final_l = posx(699.92, -340.33, 332.35, 26.41, 87.7, 167.11)
    # System_final_l = posx(699.92, -400.33, 332.35, 26.41, 87.7, 167.11)
    # 원본 교시값은 전체 코드/좌표 이력 보존을 위해 남기되 이 변형에서는 실행하지 않는다.
    System_final_j2 = posj(-53.67, 53.18, 48.53, 96.63, 74.0, 146.0)
    System_final_approach_j = posj(
        -50.40, 31.33, 84.53, 98.98, 78.15, 40.42
    )

    # =====================================================================
    # OnRobot RG2 그리퍼 제어 (DO 1, 2 접점 조합)
    # =====================================================================
    def grip_close() -> float:
        """RG2 닫기 명령 시각을 반환하고 기구 동작 안정화를 기다린다."""
        grip_monitor.raise_if_signal_lost()
        wait(0.50)
        grip_monitor.raise_if_signal_lost()
        command_stamp = time.monotonic()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.00)
        grip_monitor.raise_if_signal_lost()
        return command_stamp

    def jar_grip_open() -> None:
        grip_monitor.raise_if_signal_lost()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.50)
        grip_monitor.raise_if_signal_lost()

    def handle_grip_open() -> None:
        grip_monitor.raise_if_signal_lost()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.50)
        grip_monitor.raise_if_signal_lost()

    def spoon_cup_grip_open() -> None:
        grip_monitor.raise_if_signal_lost()
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        wait(0.50)
        grip_monitor.raise_if_signal_lost()

    def close_and_verify_grip(stage_id: str, grip_task: str) -> None:
        """닫기 후 README의 우선순위로 실제 물체 파지를 확인한다."""
        command_stamp = grip_close()
        success, diagnostics = grip_monitor.verify_grip(command_stamp)
        stage_name = TEST_STAGE_NAMES.get(stage_id, stage_id)
        diagnostics["stage_id"] = stage_id
        diagnostics["grip_task"] = grip_task
        if not success:
            node.get_logger().error(
                f"[그립 실패] 단계={stage_name}, 작업={grip_task}, "
                f"진단={json.dumps(diagnostics, ensure_ascii=False)}"
            )
            raise GripFailureError(
                stage_id,
                stage_name,
                grip_task,
                diagnostics,
            )
        node.get_logger().info(
            f"[그립 확인] 단계={stage_name}, 작업={grip_task}, "
            f"판정={diagnostics.get('decision_source', 'unknown')}"
        )

    def apply_tcp(name: str, timeout_sec: float = 6.0, poll_interval: float = 0.1) -> bool:
        """set_tcp()를 DRL 프로그램으로 실행해서 TCP를 바꾼다.

        set_tcp()를 ROS2 서비스로 직접 부르면 컨트롤러가 매번
        "this command can only be used in manual mode"로 거부한다 (실측: 6초
        재시도 내내 100% 거부, 반영된 적 0회). 티치펜던트 Task Writer 프로그램은
        같은 명령이 통과되는 걸로 봐서, 로봇 모드(AUTONOMOUS/MANUAL) 자체보다는
        "실행 중인 DRL 프로그램 컨텍스트에서 나온 호출이냐"가 실제 조건으로
        보인다. drl_script_run()으로 set_tcp(...) 한 줄짜리 코드를 컨트롤러에
        네이티브 프로그램처럼 로드해서 실행시켜 이 조건을 맞춘다.
        """
        code = f'set_tcp("{name}")'
        start_ret = drl_script_run(get_robot_system(), code)
        if start_ret != 0:
            node.get_logger().error(f"drl_script_run 시작 실패: {code!r}, ret={start_ret}")
            return False

        deadline = time.monotonic() + timeout_sec
        # get_drl_state(): 0=PLAY, 1=STOP, 2=HOLD, 3=LAST. PLAY를 벗어날 때까지 대기.
        while get_drl_state() == 0:
            if time.monotonic() >= deadline:
                node.get_logger().error(
                    f"drl_script_run({code!r}) {timeout_sec:.0f}초 내에 끝나지 않음"
                )
                return False
            time.sleep(poll_interval)

        active = get_tcp()
        if active != name:
            node.get_logger().error(
                f"drl_script_run으로도 set_tcp('{name}') 반영 안 됨: 실제 활성={active}"
            )
            return False
        return True


    def read_external_force_xyz() -> tuple[float, float, float]:
        """현재 TCP 외력의 병진 성분 Fx, Fy, Fz를 Tool 좌표계로 읽는다."""
        value = get_tool_force(DR_TOOL)

        if not isinstance(value, (list, tuple)) or len(value) < 3:
            raise RuntimeError(
                f"get_tool_force() 반환값이 올바르지 않습니다: {value!r}"
            )

        force_xyz = tuple(float(value[index]) for index in range(3))
        if not all(math.isfinite(component) for component in force_xyz):
            raise RuntimeError(
                f"get_tool_force()에 유효하지 않은 값이 포함되어 있습니다: "
                f"{force_xyz!r}"
            )

        return force_xyz

    def wait_for_external_force(
        threshold_n: float = EXTERNAL_FORCE_TRIGGER_N,
        timeout_sec: float = EXTERNAL_FORCE_TIMEOUT_SEC,
    ) -> Optional[float]:
        """외력을 기다리되 제한시간이 지나면 ``None``으로 다음 동작을 허용한다.

        move_periodic 직후의 잔류 진동과 정적 하중을 오인하지 않도록 잠시 안정화한
        뒤 기준 힘을 평균 측정한다. 이후 3축 병진 힘 변화량의 벡터 크기가 연속
        EXTERNAL_FORCE_CONFIRM_SAMPLES회 임계값 이상이면 병을 친 것으로 판정한다.
        """
        node.get_logger().info(
            f"외력 감지 준비: {EXTERNAL_FORCE_SETTLE_SEC:.2f}초 안정화"
        )
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        time.sleep(EXTERNAL_FORCE_SETTLE_SEC)
        grip_monitor.raise_if_signal_lost()

        baseline_samples: list[tuple[float, float, float]] = []
        for _ in range(EXTERNAL_FORCE_BASELINE_SAMPLES):
            grip_monitor.raise_if_signal_lost()
            baseline_samples.append(read_external_force_xyz())
            time.sleep(EXTERNAL_FORCE_SAMPLE_SEC)

        baseline = tuple(
            sum(sample[axis] for sample in baseline_samples)
            / len(baseline_samples)
            for axis in range(3)
        )

        node.get_logger().info(
            "외력 기준값 설정 완료: "
            f"Fx={baseline[0]:.3f}, Fy={baseline[1]:.3f}, "
            f"Fz={baseline[2]:.3f} N, threshold={threshold_n:.1f} N"
        )

        consecutive = 0
        peak_force = 0.0
        next_ui_update = 0.0

        while rclpy.ok():
            grip_monitor.raise_if_signal_lost()
            current = read_external_force_xyz()
            delta = tuple(
                current[axis] - baseline[axis]
                for axis in range(3)
            )
            delta_norm = math.sqrt(sum(component * component for component in delta))
            peak_force = max(peak_force, delta_norm)

            now = time.monotonic()
            remaining_sec = max(0.0, deadline - now)
            if now >= next_ui_update:
                status.publish(
                    phase="WAIT_EXTERNAL_FORCE",
                    screen=5,
                    progress=70,
                    title="병 바닥을 가볍게 쳐주세요",
                    message=(
                        f"현재 외력 변화량은 {delta_norm:.1f} N입니다. "
                        f"{threshold_n:.1f} N 이상 감지되면 진행하며, "
                        f"미감지 시 {remaining_sec:.1f}초 후 자동 진행합니다."
                    ),
                    busy=True,
                    waiting_external_force=True,
                    force_delta_n=delta_norm,
                    force_peak_n=peak_force,
                    wait_remaining_sec=remaining_sec,
                )
                next_ui_update = now + EXTERNAL_FORCE_UI_UPDATE_SEC

            if delta_norm >= threshold_n:
                consecutive += 1
                if consecutive >= EXTERNAL_FORCE_CONFIRM_SAMPLES:
                    node.get_logger().info(
                        f"외력 감지 완료: {delta_norm:.3f} N "
                        f"(peak={peak_force:.3f} N)"
                    )
                    return delta_norm
            else:
                consecutive = 0

            if now >= deadline:
                node.get_logger().warning(
                    f"외력 입력 {timeout_sec:.1f}초 초과: "
                    f"peak={peak_force:.3f} N, 다음 동작을 계속합니다."
                )
                return None

            time.sleep(EXTERNAL_FORCE_SAMPLE_SEC)

        raise KeyboardInterrupt

    # status = StatusReporter(node)  # UI 상태 발행용 — 비활성화

    # 아래 movel()/amovel() 호출들: 원본에는 전부 app_type=DR_MV_APP_NONE 키워드가
    # 붙어 있었으나, 실제 movel()/amovel() 시그니처에 app_type 파라미터가 없어
    # TypeError가 나서 전부 제거함 (DRL 원본의 app 개념이 Python API 변환 과정에서
    # 잘못 옮겨진 것으로 보임, movec()의 ori 제거와 같은 종류의 수정).
    # =====================================================================
    # 서브 루틴 (Task Writer 서브 프로그램 대응)
    # =====================================================================
    def bean_drop() -> None:
        """스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입."""
        # status.step("bean_drop", 0)  # UI 상태 발행용 — 비활성화
        spoon_cup_grip_open()

        # status.step("bean_drop", 1)  # UI 상태 발행용 — 비활성화
        movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # status.step("bean_drop", 2)  # UI 상태 발행용 — 비활성화
        movej(System_spoon_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_spoon_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        close_and_verify_grip("bean_drop", "스푼 잡기")

        # status.step("bean_drop", 3)  # UI 상태 발행용 — 비활성화
        movel(
            posx(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_grinder_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("bean_drop", 4)  # UI 상태 발행용 — 비활성화
        amovel(
            posx(32.0, -15.0, 0.0, 0.0, 0.0, 0.0), time=1.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(
            posj(0.0, 0.0, 0.0, 0.0, 0.0, 45.0), time=1.0,
            radius=0.0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("bean_drop", 5)  # UI 상태 발행용 — 비활성화
        movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # status.step("bean_drop", 6)  # UI 상태 발행용 — 비활성화
        movel(
            System_spoon_l_2, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(0.0, 0.0, -10.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        spoon_cup_grip_open()
        wait(1.0)

        # status.step("bean_drop", 7)  # UI 상태 발행용 — 비활성화
        movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

    def publish_grinding_progress(
        current_turns: float,
        total_turns: int,
    ) -> None:
        """웹 UI에 실제 그라인더 회전량과 전체 회전량을 발행한다."""
        total = max(1, int(total_turns))
        current = max(0.0, min(float(current_turns), float(total)))
        ratio = current / float(total)
        percent = ratio * 100.0
        overall_progress = int(round(42.0 + 25.0 * ratio))

        status.publish(
            phase="GRINDING",
            screen=4,
            progress=overall_progress,
            title="원두를 갈고 있습니다",
            message=(
                f"현재 {current:.2f} / {total}회전 "
                f"({percent:.0f}%) 진행했습니다."
            ),
            busy=True,
            grind_current_turns=current,
        )

    def grind_with_continuous_progress(
        grind_via,
        grind_end,
        grind_turns: int,
    ) -> None:
        """단일 amovec를 유지하며 현재 TCP 각도로 실제 회전량을 계산한다."""
        total_angle = DEGREES_PER_GRINDER_TURN * float(grind_turns)
        start_pose = _pose6_from_dsr(
            get_current_posx(ref=GRINDER_USER_COORD)
        )
        start_angle = math.atan2(start_pose[1], start_pose[0])

        # 현재점 -> 경유점 방향으로 원호 진행 방향을 결정한다.
        via_angle = math.atan2(-124.5, 0.0)
        direction_delta = _wrap_radians(via_angle - start_angle)
        direction = -1.0 if direction_delta < 0.0 else 1.0

        publish_grinding_progress(0.0, grind_turns)
        amovec(
            grind_via,
            grind_end,
            ref=GRINDER_USER_COORD,
            angle=[total_angle, 0.0],
            ra=DR_MV_RA_OVERRIDE,
        )

        last_angle = start_angle
        accumulated_angle = 0.0
        motion_seen = False
        motion_start_deadline = (
            time.monotonic() + GRINDER_MOTION_START_TIMEOUT_SEC
        )
        next_update = 0.0
        monitor_warning_logged = False

        while rclpy.ok():
            grip_monitor.raise_if_signal_lost()
            now = time.monotonic()

            try:
                motion_state = int(check_motion())
            except Exception as error:
                if not monitor_warning_logged:
                    node.get_logger().warning(
                        "check_motion() 실패로 회전 중 실시간 측정을 중단하고 "
                        f"모션 종료만 기다립니다: {error}"
                    )
                    monitor_warning_logged = True
                if callable(mwait):
                    mwait(0)
                break

            if motion_state != MOTION_IDLE:
                motion_seen = True

            try:
                pose = _pose6_from_dsr(
                    get_current_posx(ref=GRINDER_USER_COORD)
                )
                current_angle = math.atan2(pose[1], pose[0])
                angular_step = _wrap_radians(current_angle - last_angle)
                directed_step = direction * angular_step

                # 반대 방향의 미세 진동은 진행량에서 제외한다.
                if directed_step > 0.0:
                    accumulated_angle += directed_step

                last_angle = current_angle
                current_turns = min(
                    accumulated_angle / (2.0 * math.pi),
                    float(grind_turns),
                )

                if now >= next_update:
                    publish_grinding_progress(current_turns, grind_turns)
                    next_update = now + GRINDER_PROGRESS_UPDATE_SEC
            except Exception as error:
                if not monitor_warning_logged:
                    node.get_logger().warning(
                        "그라인더 현재 pose를 읽지 못해 마지막 진행률을 유지합니다: "
                        f"{error}"
                    )
                    monitor_warning_logged = True

            if motion_seen and motion_state == MOTION_IDLE:
                break

            if not motion_seen and now >= motion_start_deadline:
                node.get_logger().warning(
                    "amovec 시작 상태를 확인하지 못했습니다. 모션 종료를 기다립니다."
                )
                if callable(mwait):
                    mwait(0)
                break

            time.sleep(0.03)

        if callable(mwait):
            mwait(0)
        grip_monitor.raise_if_signal_lost()

        publish_grinding_progress(float(grind_turns), grind_turns)

    def grind_with_turn_steps(
        grind_via,
        grind_end,
        grind_turns: int,
    ) -> None:
        """비동기 API 미지원 시 1회전 movec 단위로 진행률을 발행한다."""
        publish_grinding_progress(0.0, grind_turns)

        for completed_turns in range(1, grind_turns + 1):
            movec(
                grind_via,
                grind_end,
                radius=0.0,
                ref=GRINDER_USER_COORD,
                angle=[DEGREES_PER_GRINDER_TURN, 0.0],
                ra=DR_MV_RA_OVERRIDE,
            )
            publish_grinding_progress(float(completed_turns), grind_turns)

    def grinder(grind_turns: int) -> None:
        """선택된 회전 수만큼 그라인더 손잡이를 돌려 원두를 분쇄."""
        if grind_turns <= 0:
            raise ValueError(f"그라인더 회전 수는 1 이상이어야 합니다: {grind_turns}")

        # status.step("grinder", 0)  # UI 상태 발행용 — 비활성화
        movej(System_handle_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        jar_grip_open()
        movel(
            System_handle_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        close_and_verify_grip("grinder", "그라인더 손잡이 잡기")

        # status.step("grinder", 1)  # UI 상태 발행용 — 비활성화
        movej(
            posj(12.35, 34.84, 27.41, 1.47, 118.92, 12.01),
            radius=0.0, ra=DR_MV_RA_DUPLICATE,
        )

        grind_via = posx(
            0.0, -124.5, 0.0, 90.0, -179.99, 88.27
        )
        grind_end = posx(
            -124.5, 0.0, 0.0, 90.0, -179.99, 88.27
        )

        # status.step("grinder", 2)  # UI 상태 발행용 — 비활성화
        task_compliance_ctrl()
        set_stiffnessx(
            [1500.0, 1500.0, 2500.0, 150.0, 150.0, 200.0],
            time=0.5,
        )

        try:
            # amovec + check_motion이 있으면 기존처럼 한 번의 연속 원호 모션을
            # 유지하면서 좌표계 101의 현재 X/Y 각도로 실제 회전량을 측정한다.
            if callable(amovec) and callable(check_motion):
                grind_with_continuous_progress(
                    grind_via, grind_end, grind_turns
                )
            else:
                node.get_logger().warning(
                    "amovec/check_motion 미지원: 1회전 movec 단위 진행률로 대체합니다."
                )
                grind_with_turn_steps(grind_via, grind_end, grind_turns)
        finally:
            # status.step("grinder", 4)  # UI 상태 발행용 — 비활성화
            release_compliance_ctrl()
            jar_grip_open()

    def dripper_in() -> None:
        """드립 병을 잡고 주기 운동으로 커피를 추출."""
        # status.step("dripper_in", 0)  # UI 상태 발행용 — 비활성화
        jar_grip_open()
        movej(System_bottle_j_1, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_bottle_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        close_and_verify_grip("dripper_in", "분쇄 원두 병 잡기")
        movel(
            posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("dripper_in", 1)  # UI 상태 발행용 — 비활성화
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            posx(0.0, 0.0, 300.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_drip_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_drip_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        # 반영될 때까지 재시도 (apply_tcp 참고). 끝내 실패해도 move_periodic은
        # 진행한다 — 병을 잡고 있는 상태라 여기서 멈추는 게 더 위험할 수 있다.
        apply_tcp("joint4")

        # status.step("dripper_in", 2)  # UI 상태 발행용 — 비활성화
        move_periodic(
            amp=[0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            period=[1.0, 1.0, 1.0, 0.3, 1.0, 1.0],
            atime=0.0,
            repeat=5,
            ref=DR_TOOL,
        )

        status.publish(
            phase="WAIT_EXTERNAL_FORCE",
            screen=5,
            progress=70,
            title="병 바닥을 가볍게 쳐주세요",
            message=(
                f"기준 힘 대비 {EXTERNAL_FORCE_TRIGGER_N:.1f} N 이상의 외력이 "
                f"감지되면 진행합니다. {EXTERNAL_FORCE_TIMEOUT_SEC:.0f}초 동안 "
                "감지되지 않아도 자동으로 다음 단계로 진행합니다."
            ),
            busy=True,
            waiting_external_force=True,
            force_delta_n=0.0,
            force_peak_n=0.0,
        )
        detected_force_n = wait_for_external_force(
            EXTERNAL_FORCE_TRIGGER_N,
            EXTERNAL_FORCE_TIMEOUT_SEC,
        )
        if detected_force_n is None:
            finishing_message = (
                f"{EXTERNAL_FORCE_TIMEOUT_SEC:.0f}초 동안 외력이 감지되지 않아 "
                "자동으로 다음 동작을 진행합니다. 병을 제자리로 옮깁니다."
            )
            reported_force_n = 0.0
        else:
            finishing_message = (
                f"{detected_force_n:.1f} N의 외력을 감지했습니다. "
                "병을 제자리로 옮깁니다."
            )
            reported_force_n = detected_force_n
        status.publish(
            phase="FILTER_FINISHING",
            screen=5,
            progress=74,
            title="커피 필터 투입을 마무리하는 중",
            message=finishing_message,
            busy=True,
            force_delta_n=reported_force_n,
            force_peak_n=reported_force_n,
        )

        # status.step("dripper_in", 3)  # UI 상태 발행용 — 비활성화
        apply_tcp(TCP_NAME)
        movel(
            posx(0.0, 0.0, -150.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_TOOL,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_bottle_l2, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("dripper_in", 4)  # UI 상태 발행용 — 비활성화
        jar_grip_open()

    def spiral_pour() -> None:
        """주전자 파지부터 보상 스파이럴, 중심 회전, 반환까지 수행하는 Sub."""

        def publish_spiral(
            *,
            phase: str,
            progress: int,
            spiral_progress: float,
            stage: str,
            title: str,
            message: str,
        ) -> None:
            overall_progress = max(
                75,
                min(88, 75 + round(float(spiral_progress) * 0.13)),
            )
            status.publish(
                phase=phase,
                screen=6,
                progress=overall_progress,
                title=title,
                message=message,
                busy=True,
                spiral_stage=stage,
                spiral_progress=spiral_progress,
            )

        def current_pose_base() -> np.ndarray:
            try:
                value = get_current_posx(ref=DR_BASE)
            except TypeError:
                value = get_current_posx(DR_BASE)
            return _numeric6(value, label="현재 TCP Base pose")

        def current_joint() -> np.ndarray:
            return _numeric6(
                get_current_posj(),
                label="현재 joint pose",
            )

        def forward_kinematics_base(joint_deg) -> np.ndarray:
            joint_pose = posj(
                *[float(value) for value in joint_deg[:6]]
            )
            try:
                value = fkin(joint_pose, ref=DR_BASE)
            except TypeError:
                value = fkin(joint_pose, DR_BASE)
            return _numeric6(value, label="fkin 결과")

        def rotation_distance_deg(first_abc, second_abc) -> float:
            first_rotation = _zyz_to_rotm(first_abc)
            second_rotation = _zyz_to_rotm(second_abc)
            relative = _rotm_to_rotvec(
                first_rotation.T @ second_rotation
            )
            return math.degrees(float(np.linalg.norm(relative)))

        def validate_path(
            start_pose: np.ndarray,
            numeric_path: list[np.ndarray],
            *,
            label: str,
        ) -> None:
            if not numeric_path:
                raise RuntimeError(f"{label} 경로가 비어 있습니다.")

            previous = start_pose
            max_linear_step = 0.0
            max_angular_step = 0.0

            for index, point in enumerate(numeric_path, start=1):
                if point.shape != (6,) or not np.all(np.isfinite(point)):
                    raise RuntimeError(
                        f"{label} 경유점 {index}가 올바르지 않습니다: "
                        f"{point!r}"
                    )

                linear_step = float(
                    np.linalg.norm(point[:3] - previous[:3])
                )
                angular_step = rotation_distance_deg(
                    previous[3:],
                    point[3:],
                )
                max_linear_step = max(max_linear_step, linear_step)
                max_angular_step = max(
                    max_angular_step,
                    angular_step,
                )
                previous = point

            node.get_logger().info(
                f"{label} 경로 검증: points={len(numeric_path)}, "
                f"max linear step={max_linear_step:.3f} mm, "
                f"max angular step={max_angular_step:.3f} deg"
            )

            if max_linear_step > MOVESX_MAX_LINEAR_STEP_MM:
                raise RuntimeError(
                    f"{label} 경유점 병진 간격이 너무 큽니다: "
                    f"{max_linear_step:.3f} mm"
                )
            if max_angular_step > MOVESX_MAX_ANGULAR_STEP_DEG:
                raise RuntimeError(
                    f"{label} 경유점 회전 간격이 너무 큽니다: "
                    f"{max_angular_step:.3f} deg"
                )

        def call_movesx(
            path,
            *,
            duration_sec: float,
        ):
            kwargs = {
                "ref": DR_BASE,
                "mod": DR_MV_MOD_ABS,
                "vel_opt": DR_MVS_VEL_NONE,
            }
            attempts = (
                lambda: movesx(
                    path,
                    time=duration_sec,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    t=duration_sec,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    vel=[
                        MOVESX_LINEAR_VEL_MM_S,
                        MOVESX_ANGULAR_VEL_DEG_S,
                    ],
                    acc=[
                        MOVESX_LINEAR_ACC_MM_S2,
                        MOVESX_ANGULAR_ACC_DEG_S2,
                    ],
                    time=duration_sec,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    [
                        MOVESX_LINEAR_VEL_MM_S,
                        MOVESX_ANGULAR_VEL_DEG_S,
                    ],
                    [
                        MOVESX_LINEAR_ACC_MM_S2,
                        MOVESX_ANGULAR_ACC_DEG_S2,
                    ],
                    duration_sec,
                    DR_BASE,
                    DR_MV_MOD_ABS,
                    DR_MVS_VEL_NONE,
                ),
            )

            last_type_error = None
            for attempt in attempts:
                try:
                    return attempt()
                except TypeError as error:
                    last_type_error = error

            raise RuntimeError(
                "현재 DSR_ROBOT2 movesx() 시그니처와 맞지 않습니다: "
                f"{last_type_error}"
            )

        # -------------------------------------------------------------
        # 1. 주전자 접근 및 파지
        # -------------------------------------------------------------
        publish_spiral(
            phase="SPIRAL_PICKUP",
            progress=76,
            spiral_progress=5,
            stage="주전자 파지",
            title="드립 주전자를 집는 중",
            message="주전자를 파지한 뒤 스파이럴 시작 자세로 이동합니다.",
        )
        apply_tcp(TCP_NAME)

        movej(
            System_bottle_j_2,
            radius=0.0,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            Pickup_approach_rel,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movej(
            System_pot_grip_joint,
            radius=0.0,
            ra=DR_MV_RA_DUPLICATE,
        )

        handle_grip_open()
        time.sleep(0.30)

        movel(
            System_pot_grip,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        close_and_verify_grip("spiral_pour", "드립 주전자 잡기")

        movel(
            Pickup_lift_rel,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movej(
            Pour_start_joint,
            radius=0.0,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(-5.0, -5.0, 0.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )

        # -------------------------------------------------------------
        # 2. pot TCP 기준 내향 스파이럴 경로 생성
        # -------------------------------------------------------------
        publish_spiral(
            phase="SPIRAL_PREPARING",
            progress=81,
            spiral_progress=20,
            stage="경로 준비",
            title="스파이럴 드립 경로를 준비하는 중",
            message=(
                f"반경 {SPIRAL_RADIUS_MM:.0f} mm, "
                f"{SPIRAL_REVOLUTIONS:.0f}회전 경로를 계산합니다."
            ),
        )
        if not apply_tcp(SPOUT_TCP_NAME):
            raise RuntimeError(
                f"주둥이 TCP '{SPOUT_TCP_NAME}' 적용 실패"
            )

        start_pose = current_pose_base()
        start_joint = current_joint()

        final_joint = start_joint.copy()
        final_joint[5] += SPIRAL_J6_DELTA_DEG
        final_virtual_pose = forward_kinematics_base(final_joint)

        virtual_drift = final_virtual_pose[:3] - start_pose[:3]
        required_compensation = -virtual_drift

        center_x = (
            start_pose[0]
            + SPIRAL_CENTER_OFFSET_SIGN_X * SPIRAL_RADIUS_MM
        )
        center_y = start_pose[1]
        hold_z = start_pose[2]

        start_rotation = _zyz_to_rotm(start_pose[3:])
        final_rotation = _zyz_to_rotm(final_virtual_pose[3:])
        relative_rotvec = _rotm_to_rotvec(
            start_rotation.T @ final_rotation
        )

        node.get_logger().info(
            "Joint6 단독 가상 회전 시 주둥이 Base 변위 [mm]: "
            f"dX={virtual_drift[0]:+.3f}, "
            f"dY={virtual_drift[1]:+.3f}, "
            f"dZ={virtual_drift[2]:+.3f}"
        )
        node.get_logger().info(
            "로봇 본체의 반대 보상 [mm]: "
            f"X={required_compensation[0]:+.3f}, "
            f"Y={required_compensation[1]:+.3f}, "
            f"Z={required_compensation[2]:+.3f}"
        )

        dense_count = max(4000, SPIRAL_PATH_POINTS * 80)
        dense_s = np.linspace(
            0.0,
            1.0,
            dense_count + 1,
            dtype=float,
        )
        dense_radius = SPIRAL_RADIUS_MM * (1.0 - dense_s)
        dense_theta = (
            SPIRAL_ROTATION_SIGN
            * 2.0
            * math.pi
            * SPIRAL_REVOLUTIONS
            * dense_s
        )
        dense_x = center_x + dense_radius * np.cos(dense_theta)
        dense_y = center_y + dense_radius * np.sin(dense_theta)
        dense_xyz = np.column_stack(
            (
                dense_x,
                dense_y,
                np.full_like(dense_x, hold_z),
            )
        )

        segment_length = np.linalg.norm(
            np.diff(dense_xyz, axis=0),
            axis=1,
        )
        cumulative_length = np.concatenate(
            (
                np.array([0.0], dtype=float),
                np.cumsum(segment_length),
            )
        )
        total_path_length = float(cumulative_length[-1])
        if (
            not math.isfinite(total_path_length)
            or total_path_length <= 0.0
        ):
            raise RuntimeError(
                f"스파이럴 호길이 계산 실패: {total_path_length}"
            )

        target_lengths = np.linspace(
            total_path_length / float(SPIRAL_PATH_POINTS),
            total_path_length,
            SPIRAL_PATH_POINTS,
            dtype=float,
        )
        spiral_s_samples = np.interp(
            target_lengths,
            cumulative_length,
            dense_s,
        )

        numeric_path: list[np.ndarray] = []
        path = []
        previous_abc = start_pose[3:].copy()

        for index, spiral_s in enumerate(
            spiral_s_samples,
            start=1,
        ):
            radius = SPIRAL_RADIUS_MM * (
                1.0 - float(spiral_s)
            )
            theta = (
                SPIRAL_ROTATION_SIGN
                * 2.0
                * math.pi
                * SPIRAL_REVOLUTIONS
                * float(spiral_s)
            )

            target_x = center_x + radius * math.cos(theta)
            target_y = center_y + radius * math.sin(theta)
            target_z = hold_z

            orientation_progress = _smoothstep5(
                index / float(SPIRAL_PATH_POINTS)
            )
            target_rotation = start_rotation @ _rotvec_to_rotm(
                relative_rotvec * orientation_progress
            )
            target_abc = _rotm_to_zyz_near(
                target_rotation,
                previous_abc,
            )
            previous_abc = target_abc

            target = np.array(
                [
                    target_x,
                    target_y,
                    target_z,
                    target_abc[0],
                    target_abc[1],
                    target_abc[2],
                ],
                dtype=float,
            )
            numeric_path.append(target)
            path.append(posx(*[float(value) for value in target]))

        validate_path(
            start_pose,
            numeric_path,
            label="스파이럴",
        )

        # -------------------------------------------------------------
        # 3. 내향 스파이럴 + Joint6 등가 기울임
        # -------------------------------------------------------------
        publish_spiral(
            phase="SPIRAL_POURING",
            progress=84,
            spiral_progress=35,
            stage="스파이럴 드립",
            title="스파이럴 방식으로 물을 붓는 중",
            message=(
                f"{SPIRAL_DURATION_SEC:.0f}초 동안 "
                f"{SPIRAL_REVOLUTIONS:.0f}회 내향 스파이럴을 수행합니다."
            ),
        )
        node.get_logger().info(
            "통합 워크플로우 스파이럴 Sub 시작: "
            f"radius={SPIRAL_RADIUS_MM:.1f} mm, "
            f"rev={SPIRAL_REVOLUTIONS:.1f}, "
            f"duration={SPIRAL_DURATION_SEC:.1f} s, "
            f"J6 equivalent={SPIRAL_J6_DELTA_DEG:.1f} deg"
        )

        result = call_movesx(
            path,
            duration_sec=SPIRAL_DURATION_SEC,
        )
        if isinstance(result, (int, float)) and result < 0:
            raise RuntimeError(
                f"스파이럴 movesx 실패 반환값={result}"
            )

        end_pose = current_pose_base()
        target_end = numeric_path[-1]
        end_error = target_end[:3] - end_pose[:3]
        node.get_logger().info(
            "스파이럴 완료 주둥이 Base 오차 [mm]: "
            f"X={end_error[0]:+.3f}, "
            f"Y={end_error[1]:+.3f}, "
            f"Z={end_error[2]:+.3f}"
        )

        # -------------------------------------------------------------
        # 4. 스파이럴 중심에서 주둥이 XYZ 고정, 주전자 자세만 원복
        # -------------------------------------------------------------
        publish_spiral(
            phase="SPIRAL_CENTER_RETURN",
            progress=92,
            spiral_progress=75,
            stage="중심 고정 자세 복원",
            title="주둥이를 고정한 채 주전자를 세우는 중",
            message=(
                "스파이럴 중심에서 주둥이 위치를 유지하며 "
                f"{CENTER_PIVOT_RETURN_DURATION_SEC:.1f}초 동안 "
                "주전자 자세를 빠르게 복원합니다."
            ),
        )

        fixed_center_xyz = end_pose[:3].copy()
        current_rotation = _zyz_to_rotm(end_pose[3:])
        target_rotation = _zyz_to_rotm(start_pose[3:])
        return_rotvec = _rotm_to_rotvec(
            current_rotation.T @ target_rotation
        )
        total_return_deg = math.degrees(
            float(np.linalg.norm(return_rotvec))
        )

        if total_return_deg >= 0.05:
            return_numeric_path: list[np.ndarray] = []
            return_path = []
            previous_abc = end_pose[3:].copy()

            for index in range(
                1,
                CENTER_PIVOT_RETURN_POINTS + 1,
            ):
                progress_value = _smoothstep5(
                    index / float(CENTER_PIVOT_RETURN_POINTS)
                )
                interpolated_rotation = (
                    current_rotation
                    @ _rotvec_to_rotm(
                        return_rotvec * progress_value
                    )
                )
                target_abc = _rotm_to_zyz_near(
                    interpolated_rotation,
                    previous_abc,
                )
                previous_abc = target_abc

                target = np.array(
                    [
                        fixed_center_xyz[0],
                        fixed_center_xyz[1],
                        fixed_center_xyz[2],
                        target_abc[0],
                        target_abc[1],
                        target_abc[2],
                    ],
                    dtype=float,
                )
                return_numeric_path.append(target)
                return_path.append(
                    posx(*[float(value) for value in target])
                )

            validate_path(
                end_pose,
                return_numeric_path,
                label="중심 고정 자세복원",
            )
            return_start_time = time.monotonic()
            result = call_movesx(
                return_path,
                duration_sec=CENTER_PIVOT_RETURN_DURATION_SEC,
            )
            return_elapsed = time.monotonic() - return_start_time
            if (
                isinstance(result, (int, float))
                and result < 0
            ):
                raise RuntimeError(
                    "중심 고정 자세복원 movesx 실패 "
                    f"반환값={result}"
                )

            pivot_end_pose = current_pose_base()
            pivot_drift = (
                pivot_end_pose[:3] - fixed_center_xyz
            )
            node.get_logger().info(
                "중심 고정 자세복원 결과 [mm]: "
                f"dX={pivot_drift[0]:+.3f}, "
                f"dY={pivot_drift[1]:+.3f}, "
                f"dZ={pivot_drift[2]:+.3f}, "
                f"설정={CENTER_PIVOT_RETURN_DURATION_SEC:.1f}s, "
                f"실측={return_elapsed:.3f}s"
            )

        # -------------------------------------------------------------
        # 5. 원래 파지 위치 + Base Z 10 mm로 이동 후 놓기
        # -------------------------------------------------------------
        publish_spiral(
            phase="SPIRAL_RETURNING",
            progress=96,
            spiral_progress=90,
            stage="주전자 반환",
            title="주전자를 원래 위치로 옮기는 중",
            message="원래 파지 위치의 Base +Z 10 mm 지점으로 이동합니다.",
        )

        if not apply_tcp(TCP_NAME):
            raise RuntimeError(
                f"기본 TCP '{TCP_NAME}' 복원 실패"
            )

        movej(
            System_pot_grip_joint,
            radius=0.0,
            ra=DR_MV_RA_DUPLICATE,
        )

        release_values = [
            float(value) for value in SYSTEM_POT_GRIP_VALUES
        ]
        release_values[2] += POT_RELEASE_BASE_Z_OFFSET_MM
        release_target = posx(*release_values)

        movel(
            release_target,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )

        publish_spiral(
            phase="SPIRAL_RELEASING",
            progress=99,
            spiral_progress=98,
            stage="주전자 놓기",
            title="주전자를 내려놓는 중",
            message="그리퍼를 열어 주전자를 배치합니다.",
        )
        handle_grip_open()
        time.sleep(POT_RELEASE_SETTLE_SEC)

        publish_spiral(
            phase="SPIRAL_DONE",
            progress=99,
            spiral_progress=100,
            stage="완료",
            title="스파이럴 드립 완료",
            message="주전자를 원래 위치에 놓았습니다.",
        )
        node.get_logger().info(
            "통합 워크플로우 스파이럴 Sub 완료"
        )

    # region 마지막 단계
    def final_drip() -> None:
        """DRL final_drip 동작 후 plate_outline 기반 물 붓기를 수행한다.

        순서
        ----
        1. 필터 홀더를 집어 지정 위치로 이동
        2. 물컵을 집어 최종 드립 시작 자세로 이동
        3. 정확히 교시된 ``mug`` TCP를 활성화
        4. Joint6 상대 +30 deg 후 mug TCP 원점을 고정한 +55 deg 회전 경로 실행
        5. 저장한 물 붓기 시작 회전행렬을 직접 목표로 새 복귀 경로를 계산
        6. final_drip 진입 전 활성 TCP로 복원
        7. Base -X 200 mm 상대 이동 후 ``System_mug_l``의 Base +Z 5 mm에서 물컵 해제
        8. Tool -Z 150 mm, Base +Z 150 mm 순서로 이동한 뒤 ``System_home`` 복귀

        mug TCP 자체가 실제 주둥이 끝이므로 별도의 Tool XYZ 오프셋은 사용하지 않는다.
        """

        # 다른 모든 Sub와 동일한 전역 속도/가속도를 적용한다.
        set_velj(VELJ_DEFAULT)
        set_accj(ACCJ_DEFAULT)
        set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
        set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)

        def publish_final(
            *,
            phase: str,
            overall_progress: int,
            final_progress: float,
            stage: str,
            title: str,
            message: str,
        ) -> None:
            status.publish(
                phase=phase,
                screen=7,
                progress=overall_progress,
                title=title,
                message=message,
                busy=True,
                final_drip_stage=stage,
                final_drip_progress=final_progress,
            )

        def current_pose_base() -> np.ndarray:
            try:
                value = get_current_posx(ref=DR_BASE)
            except TypeError:
                value = get_current_posx(DR_BASE)
            return _numeric6(value, label="현재 mug TCP Base pose")

        def current_joint() -> np.ndarray:
            return _numeric6(get_current_posj(), label="현재 joint pose")

        def forward_kinematics_base(joint_deg: np.ndarray) -> np.ndarray:
            joint = posj(*[float(value) for value in joint_deg[:6]])
            try:
                value = fkin(joint, ref=DR_BASE)
            except TypeError:
                value = fkin(joint, DR_BASE)
            return _numeric6(value, label="fkin 결과")

        def rotation_distance_deg(first_abc, second_abc) -> float:
            first_rotation = _zyz_to_rotm(first_abc)
            second_rotation = _zyz_to_rotm(second_abc)
            relative = _rotm_to_rotvec(first_rotation.T @ second_rotation)
            return math.degrees(float(np.linalg.norm(relative)))

        def mug_tcp_point_base(tcp_pose: np.ndarray) -> np.ndarray:
            """mug TCP 원점은 실제 주둥이 끝과 동일하므로 Base XYZ만 반환한다."""
            return np.asarray(tcp_pose[:3], dtype=float).copy()

        def validate_path(
            start_pose: np.ndarray,
            numeric_path: list[np.ndarray],
        ) -> None:
            if not numeric_path:
                raise RuntimeError("final_drip 물 붓기 MoveSX 경로가 비어 있습니다.")

            previous = start_pose
            max_linear_step = 0.0
            max_angular_step = 0.0

            for index, point in enumerate(numeric_path, start=1):
                if point.shape != (6,) or not np.all(np.isfinite(point)):
                    raise RuntimeError(
                        f"물 붓기 경유점 {index}가 올바르지 않습니다: {point!r}"
                    )
                linear_step = float(np.linalg.norm(point[:3] - previous[:3]))
                angular_step = rotation_distance_deg(
                    previous[3:], point[3:]
                )
                max_linear_step = max(max_linear_step, linear_step)
                max_angular_step = max(max_angular_step, angular_step)
                previous = point

            node.get_logger().info(
                "final_drip 물 붓기 경로 검증: "
                f"points={len(numeric_path)}, "
                f"max linear step={max_linear_step:.3f} mm, "
                f"max angular step={max_angular_step:.3f} deg"
            )

            if max_linear_step > FINAL_POUR_MAX_LINEAR_STEP_MM:
                raise RuntimeError(
                    "물 붓기 경유점 병진 간격이 "
                    f"{max_linear_step:.3f} mm로 제한값 "
                    f"{FINAL_POUR_MAX_LINEAR_STEP_MM:.3f} mm를 초과합니다."
                )
            if max_angular_step > FINAL_POUR_MAX_ANGULAR_STEP_DEG:
                raise RuntimeError(
                    "물 붓기 경유점 회전 간격이 "
                    f"{max_angular_step:.3f} deg로 제한값 "
                    f"{FINAL_POUR_MAX_ANGULAR_STEP_DEG:.3f} deg를 초과합니다."
                )

        def call_final_movesx(
            path,
            *,
            angular_vel_deg_s: float = VELX_ROT_DEFAULT,
            angular_acc_deg_s2: float = ACCX_ROT_DEFAULT,
        ) -> Any:
            """지정된 회전 속도/가속도로 final_drip MoveSX를 실행한다."""
            kwargs = {
                "ref": DR_BASE,
                "mod": DR_MV_MOD_ABS,
                "vel_opt": DR_MVS_VEL_NONE,
            }
            common_vel = [VELX_LIN_DEFAULT, float(angular_vel_deg_s)]
            common_acc = [ACCX_LIN_DEFAULT, float(angular_acc_deg_s2)]
            attempts = (
                lambda: movesx(
                    path,
                    vel=common_vel,
                    acc=common_acc,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    common_vel,
                    common_acc,
                    0.0,
                    DR_BASE,
                    DR_MV_MOD_ABS,
                    DR_MVS_VEL_NONE,
                ),
            )

            last_type_error: Optional[TypeError] = None
            for attempt in attempts:
                try:
                    return attempt()
                except TypeError as error:
                    last_type_error = error
            raise RuntimeError(
                "현재 DSR_ROBOT2 movesx() 속도 인자 시그니처와 맞지 않습니다: "
                f"{last_type_error}"
            )

        def build_fixed_spout_orientation_path(
            start_pose: np.ndarray,
            target_rotation: np.ndarray,
            fixed_spout_base: np.ndarray,
            *,
            label: str,
        ) -> tuple[list[Any], list[np.ndarray]]:
            """mug TCP XYZ를 고정하고 지정 회전행렬까지 한 구간을 생성한다."""
            start_rotation = _zyz_to_rotm(start_pose[3:])
            requested_target_rotation = _project_rotation(target_rotation)
            fixed_spout_base = np.asarray(
                fixed_spout_base,
                dtype=float,
            ).reshape(3)
            relative_rotvec = _rotm_to_rotvec(
                start_rotation.T @ requested_target_rotation
            )
            rotation_change_deg = math.degrees(
                float(np.linalg.norm(relative_rotvec))
            )
            node.get_logger().info(
                f"{label} 경로 생성: 회전 변화={rotation_change_deg:.3f} deg, "
                "mug TCP Base XYZ 고정"
            )

            numeric_path: list[np.ndarray] = []
            path = []
            previous_abc = start_pose[3:].copy()

            for index in range(1, FINAL_POUR_PATH_POINTS + 1):
                progress_value = _smoothstep5(
                    index / float(FINAL_POUR_PATH_POINTS)
                )
                interpolated_rotation = start_rotation @ _rotvec_to_rotm(
                    relative_rotvec * progress_value
                )
                target_abc = _rotm_to_zyz_near(
                    interpolated_rotation,
                    previous_abc,
                )
                previous_abc = target_abc

                # mug TCP가 실제 주둥이 끝이므로 회전 중 XYZ는 시작값으로 고정한다.
                target_tcp_xyz = fixed_spout_base.copy()
                target = np.array(
                    [
                        target_tcp_xyz[0],
                        target_tcp_xyz[1],
                        target_tcp_xyz[2],
                        target_abc[0],
                        target_abc[1],
                        target_abc[2],
                    ],
                    dtype=float,
                )

                predicted_spout = target_tcp_xyz
                residual = predicted_spout - fixed_spout_base
                if float(np.linalg.norm(residual)) > 1.0e-6:
                    raise RuntimeError(
                        f"{label} 경유점 {index} 보상 계산 오류: "
                        f"residual={residual.tolist()}"
                    )

                numeric_path.append(target)
                path.append(
                    posx(*[float(value) for value in target])
                )

            validate_path(start_pose, numeric_path)
            planned_target_error_deg = rotation_distance_deg(
                numeric_path[-1][3:],
                _rotm_to_zyz_near(
                    requested_target_rotation,
                    numeric_path[-1][3:],
                ),
            )
            if planned_target_error_deg > 0.01:
                raise RuntimeError(
                    f"{label} 최종 회전 목표 오차가 큽니다: "
                    f"{planned_target_error_deg:.6f} deg"
                )
            return path, numeric_path

        def build_fixed_spout_path(
            start_pose: np.ndarray,
            start_joint: np.ndarray,
            j6_delta_deg: float,
        ) -> tuple[list[Any], list[np.ndarray], np.ndarray]:
            # mug TCP 원점 자체가 주둥이 끝이므로 별도 Tool 오프셋 없이
            # 시작 TCP Base XYZ를 회전 중심으로 그대로 사용한다.
            fixed_spout_base = start_pose[:3].copy()

            virtual_final_joint = start_joint.copy()
            virtual_final_joint[5] += float(j6_delta_deg)
            virtual_final_pose = forward_kinematics_base(virtual_final_joint)
            final_rotation = _zyz_to_rotm(virtual_final_pose[3:])

            virtual_uncompensated_spout = virtual_final_pose[:3].copy()
            uncompensated_drift = (
                virtual_uncompensated_spout - fixed_spout_base
            )
            node.get_logger().info(
                "가상 J6 단독 회전 시 mug TCP 원점 변위 [mm]: "
                f"dX={uncompensated_drift[0]:+.3f}, "
                f"dY={uncompensated_drift[1]:+.3f}, "
                f"dZ={uncompensated_drift[2]:+.3f}"
            )

            path, numeric_path = build_fixed_spout_orientation_path(
                start_pose,
                final_rotation,
                fixed_spout_base,
                label=f"J6 등가 {j6_delta_deg:+.1f} deg 기울임",
            )
            return path, numeric_path, fixed_spout_base

        # -------------------------------------------------------------
        # 1. DRL final_drip: 필터 홀더 이동
        # -------------------------------------------------------------
        publish_final(
            phase="FINAL_DRIP_FILTER_HOLDER",
            overall_progress=90,
            final_progress=5,
            stage="필터 홀더 이동",
            title="필터 홀더를 옮기는 중",
            message="필터 홀더를 집어 최종 드립 위치에 배치합니다.",
        )
        handle_grip_open()
        movel(
            posx(0.0, 0.0, -150.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_fitter_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_filtter_l,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        close_and_verify_grip("final_drip", "필터 홀더 잡기")
        movel(
            posx(0.0, 0.0, 100.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            System_filtter_l2,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        spoon_cup_grip_open()
        movel(
            posx(0.0, 0.0, -90.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(200.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )

        # -------------------------------------------------------------
        # 2. DRL final_drip: 물컵 파지 및 붓기 시작 자세 이동
        # -------------------------------------------------------------
        publish_final(
            phase="FINAL_DRIP_MUG_PICKUP",
            overall_progress=92,
            final_progress=30,
            stage="물컵 파지",
            title="물컵을 집는 중",
            message="물컵을 파지한 뒤 드리퍼 위 최종 물 붓기 자세로 이동합니다.",
        )
        movej(System_mug_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_mug_l,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        close_and_verify_grip("final_drip", "물컵 잡기")
        movel(
            posx(0.0, 0.0, 200.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(-100.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )

        publish_final(
            phase="FINAL_DRIP_POSITIONING",
            overall_progress=94,
            final_progress=55,
            stage="드립 위치 이동",
            title="물 붓기 시작 자세로 이동하는 중",
            message=(
                "교시된 최종 드립 위치 도달 후 Joint6을 상대 "
                f"{FINAL_PRE_POUR_J6_REL_DEG:+.0f}° 회전하고 "
                "mug TCP 고정 물 붓기를 실행합니다."
            ),
        )
        movej(System_final_approach_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_final_l,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        movej(
            posj(0.0, 0.0, 0.0, 0.0, 0.0, FINAL_PRE_POUR_J6_REL_DEG),
            radius=0.0,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        node.get_logger().info(
            f"final_drip 사전 Joint6 상대 회전: {FINAL_PRE_POUR_J6_REL_DEG:+.1f} deg"
        )

        # -------------------------------------------------------------
        # 3. DRL의 tp_log("물 붓기") 위치: plate_outline 통합
        # -------------------------------------------------------------
        original_tcp_name = get_tcp()
        if not isinstance(original_tcp_name, str) or not original_tcp_name.strip():
            original_tcp_name = TCP_NAME
        original_tcp_name = original_tcp_name.strip()

        publish_final(
            phase="FINAL_DRIP_POUR_READY",
            overall_progress=96,
            final_progress=70,
            stage="물 붓기 준비",
            title="mug TCP 회전 경로를 준비하는 중",
            message=(
                f"TCP '{FINAL_POUR_TCP_NAME}'를 적용하고 mug TCP 원점을 고정한 "
                f"Joint6 등가 {FINAL_POUR_J6_DELTA_DEG:+.0f}° 경로를 계산합니다."
            ),
        )
        if not apply_tcp(FINAL_POUR_TCP_NAME):
            raise RuntimeError(
                f"물 붓기 TCP '{FINAL_POUR_TCP_NAME}' 적용 실패"
            )

        start_pose = current_pose_base()
        start_joint = current_joint()
        path, numeric_path, fixed_spout_base = build_fixed_spout_path(
            start_pose,
            start_joint,
            FINAL_POUR_J6_DELTA_DEG,
        )

        node.get_logger().warning(
            "final_drip 물 붓기 시작: 고정 mug TCP Base XYZ="
            f"[{fixed_spout_base[0]:.3f}, "
            f"{fixed_spout_base[1]:.3f}, "
            f"{fixed_spout_base[2]:.3f}] mm"
        )
        publish_final(
            phase="FINAL_DRIP_POURING",
            overall_progress=97,
            final_progress=78,
            stage="물 붓기",
            title="mug TCP 원점을 유지하며 물을 붓는 중",
            message=(
                f"공통 Cartesian 속도 [{VELX_LIN_DEFAULT:.0f} mm/s, "
                f"{FINAL_POUR_ANGULAR_VEL_DEG_S:.0f} deg/s]로 "
                "mug TCP Base XYZ를 고정하고 "
                f"Joint6 등가 {FINAL_POUR_J6_DELTA_DEG:+.0f}°로 기울입니다."
            ),
        )

        start_time = time.monotonic()
        result = call_final_movesx(
            path,
            angular_vel_deg_s=FINAL_POUR_ANGULAR_VEL_DEG_S,
            angular_acc_deg_s2=FINAL_POUR_ANGULAR_ACC_DEG_S2,
        )
        elapsed = time.monotonic() - start_time
        if isinstance(result, (int, float)) and result < 0:
            raise RuntimeError(
                f"final_drip 물 붓기 movesx 실패 반환값={result}"
            )

        # 동기 MoveSX 완료 후 0.5초 정지하여 실제 종료 상태를 안정화한다.
        # 이후 읽은 TCP/관절값을 복귀 경로의 새 시작 상태로 그대로 사용한다.
        wait(0.50)
        end_pose = current_pose_base()
        end_joint = current_joint()
        measured_spout_base = mug_tcp_point_base(end_pose)
        spout_error = measured_spout_base - fixed_spout_base
        final_pose_error = numeric_path[-1][:3] - end_pose[:3]
        joint_delta = end_joint - start_joint

        node.get_logger().info(
            "final_drip 물 붓기 완료: "
            f"ret={result}, elapsed={elapsed:.3f}s"
        )
        node.get_logger().info(
            "mug TCP 원점 고정 오차 [mm]: "
            f"X={spout_error[0]:+.3f}, "
            f"Y={spout_error[1]:+.3f}, "
            f"Z={spout_error[2]:+.3f}, "
            f"norm={float(np.linalg.norm(spout_error)):.3f}"
        )
        node.get_logger().info(
            "실제 관절 변화 [deg]: ["
            + ", ".join(f"{value:+.3f}" for value in joint_delta)
            + "]"
        )
        node.get_logger().info(
            "최종 mug TCP 목표 오차 [mm]: "
            f"X={final_pose_error[0]:+.3f}, "
            f"Y={final_pose_error[1]:+.3f}, "
            f"Z={final_pose_error[2]:+.3f}"
        )

        if float(np.linalg.norm(spout_error)) > 3.0:
            node.get_logger().warning(
                "mug TCP 원점 고정 오차가 3 mm를 초과했습니다. "
                "티치펜던트의 mug TCP 교시값을 확인하십시오."
            )

        # -------------------------------------------------------------
        # 4. 저장해 둔 물 붓기 시작 회전행렬로 직접 복귀
        # -------------------------------------------------------------
        publish_final(
            phase="FINAL_DRIP_RETURNING",
            overall_progress=98,
            final_progress=90,
            stage="시작 자세 복귀",
            title="물 붓기 전 자세로 돌아가는 중",
            message=(
                "현재 관절에 반대 J6 값을 다시 적용하지 않고, 기울이기 직전에 "
                "저장한 mug TCP 자세로 직접 복귀합니다. "
                f"복귀 회전 속도는 {FINAL_RETURN_ANGULAR_VEL_DEG_S:.0f} deg/s입니다."
            ),
        )

        # 현재 실제 자세에서 저장한 최초 회전행렬을 직접 목표로 삼는다.
        # 따라서 첫 구간의 실제 IK가 J6 회전을 다른 관절에 분배했더라도 복귀 목표가
        # 다시 +방향으로 계산되지 않으며, 계획 자체가 시작 자세로 끝난다.
        return_fixed_spout_base = fixed_spout_base.copy()
        return_target_rotation = _zyz_to_rotm(start_pose[3:])
        return_path, return_numeric_path = build_fixed_spout_orientation_path(
            end_pose,
            return_target_rotation,
            return_fixed_spout_base,
            label="저장한 물 붓기 시작 자세 복귀",
        )

        return_start_time = time.monotonic()
        return_result = call_final_movesx(
            return_path,
            angular_vel_deg_s=FINAL_RETURN_ANGULAR_VEL_DEG_S,
            angular_acc_deg_s2=FINAL_RETURN_ANGULAR_ACC_DEG_S2,
        )
        return_elapsed = time.monotonic() - return_start_time
        node.get_logger().info(
            "final_drip 복귀 구간 고정 mug TCP Base XYZ="
            f"[{return_fixed_spout_base[0]:.3f}, "
            f"{return_fixed_spout_base[1]:.3f}, "
            f"{return_fixed_spout_base[2]:.3f}] mm"
        )
        if isinstance(return_result, (int, float)) and return_result < 0:
            raise RuntimeError(
                f"final_drip 시작 자세 복귀 movesx 실패 반환값={return_result}"
            )

        returned_pose = current_pose_base()
        returned_joint = current_joint()
        return_xyz_error = returned_pose[:3] - start_pose[:3]
        return_orientation_error = rotation_distance_deg(
            start_pose[3:],
            returned_pose[3:],
        )
        return_joint_error = returned_joint - start_joint

        node.get_logger().info(
            "final_drip 시작 자세 복귀 완료: "
            f"ret={return_result}, elapsed={return_elapsed:.3f}s, "
            f"XYZ error={float(np.linalg.norm(return_xyz_error)):.3f} mm, "
            f"orientation error={return_orientation_error:.3f} deg"
        )
        node.get_logger().info(
            "복귀 후 관절 오차 [deg]: ["
            + ", ".join(f"{value:+.3f}" for value in return_joint_error)
            + "]"
        )

        # -------------------------------------------------------------
        # 5. final_drip 진입 전 TCP로 복원
        # -------------------------------------------------------------
        publish_final(
            phase="FINAL_DRIP_TCP_RESTORE",
            overall_progress=99,
            final_progress=97,
            stage="TCP 복원",
            title="기본 TCP로 복원하는 중",
            message=f"활성 TCP를 '{original_tcp_name}'로 되돌립니다.",
        )
        if not apply_tcp(original_tcp_name):
            raise RuntimeError(
                f"원래 TCP '{original_tcp_name}' 복원 실패"
            )

        # -------------------------------------------------------------
        # 6. Base -X 200 mm 이격 후 원위치 위 5 mm에서 그리퍼 열기
        # -------------------------------------------------------------
        publish_final(
            phase="FINAL_DRIP_MUG_RETURN",
            overall_progress=99,
            final_progress=99,
            stage="물컵 원위치 복귀",
            title="물컵을 원위치로 옮기는 중",
            message=(
                f"Base X {FINAL_MUG_RETURN_BASE_X_OFFSET_MM:+.1f} mm 상대 이동 후 "
                "System_mug_l의 Base +Z "
                f"{FINAL_MUG_RETURN_Z_OFFSET_MM:.1f} mm 위치에서 그리퍼를 엽니다."
            ),
        )
        movel(
            posx(
                FINAL_MUG_RETURN_BASE_X_OFFSET_MM,
                80.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            System_mug_release_l,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        spoon_cup_grip_open()
        movel(
            posx(
                0.0,
                0.0,
                FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM,
                0.0,
                0.0,
                0.0,
            ),
            radius=0.0,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(
                0.0,
                0.0,
                FINAL_MUG_RELEASE_BASE_Z_LIFT_MM,
                0.0,
                0.0,
                0.0,
            ),
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        node.get_logger().info(
            "final_drip 물컵 반환 완료: "
            f"Base X {FINAL_MUG_RETURN_BASE_X_OFFSET_MM:+.1f} mm 상대 이동, "
            f"System_mug_l Base +Z {FINAL_MUG_RETURN_Z_OFFSET_MM:.1f} mm, "
            f"그리퍼 열림, Tool Z {FINAL_MUG_RELEASE_TOOL_Z_RETREAT_MM:+.1f} mm, "
            f"Base Z {FINAL_MUG_RELEASE_BASE_Z_LIFT_MM:+.1f} mm, 홈 복귀"
        )

        publish_final(
            phase="FINAL_DRIP_DONE",
            overall_progress=99,
            final_progress=100,
            stage="완료",
            title="최종 물 붓기 완료",
            message=(
                "물 붓기 전 자세로 복귀하고 활성 TCP를 "
                f"'{original_tcp_name}'로 복원한 뒤 물컵을 반환하고 홈 자세로 복귀했습니다."
            ),
        )
        node.get_logger().info(
            f"final_drip Sub 완료: TCP={original_tcp_name}"
        )

    # =====================================================================
    # 반복 실행 디스패처
    # =====================================================================
    set_singularity_handling(DR_AVOID)
    set_velj(VELJ_DEFAULT)
    set_accj(ACCJ_DEFAULT)
    set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
    set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)

    def ensure_tool_tcp() -> None:
        """각 공정/테스트 시작 전에 Tool과 기본 TCP를 확인한다."""
        tool_ret = set_tool(TOOL_NAME)
        tcp_ret = set_tcp(TCP_NAME)
        active_tool = get_tool()
        active_tcp = get_tcp()
        node.get_logger().info(
            f"set_tool ret={tool_ret}, 요청={TOOL_NAME}, 실제 활성={active_tool}"
        )
        node.get_logger().info(
            f"set_tcp ret={tcp_ret}, 요청={TCP_NAME}, 실제 활성={active_tcp}"
        )
        if active_tool != TOOL_NAME or active_tcp != TCP_NAME:
            raise RuntimeError(
                "Tool/TCP가 요청한 이름으로 활성화되지 않았습니다. "
                "티치펜던트에 해당 이름이 등록되어 있는지 확인하세요."
            )

    stage_recovery_ui = {
        "bean_drop": (2, 10),
        "grinder": (4, 42),
        "dripper_in": (5, 68),
        "spiral_pour": (6, 75),
        "final_drip": (7, 89),
        "gripper_open": (1, 0),
        "gripper_close": (1, 0),
    }

    def restore_default_context_after_recovery() -> None:
        """중단 지점의 전용 TCP가 남았으면 기본 TCP로 되돌린다."""
        active_tcp = get_tcp()
        if active_tcp != TCP_NAME and not apply_tcp(TCP_NAME):
            raise RuntimeError(
                f"복구 후 기본 TCP({TCP_NAME})로 되돌리지 못했습니다."
            )
        ensure_tool_tcp()

    def recover_grip_failure(error: GripFailureError) -> None:
        """DI13 관리자 호출, DI14 환경 정리 완료 순서로 복구한다."""
        _, progress = stage_recovery_ui.get(error.stage_id, (10, 0))
        status.publish(
            phase="GRIP_FAILURE",
            screen=10,
            progress=progress,
            title="그립 실패로 작업을 정지했습니다",
            message=(
                f"기억한 단계: {error.stage_name} / 작업: {error.grip_task}. "
                "관리자를 호출하려면 DI 13번 버튼을 누르세요."
            ),
            busy=True,
            waiting_physical_button=True,
            grip_failure=True,
            failed_stage_id=error.stage_id,
            failed_stage_name=error.stage_name,
            failed_grip_task=error.grip_task,
            recovery_kind="GRIP_FAILURE",
            recovery_state="WAIT_ADMIN_CALL",
            grip_diagnostics=error.diagnostics,
            gripper_signal_online=grip_monitor.signal_online(),
            error=str(error),
        )
        node.get_logger().warning(
            f"그립 실패 복구 대기: 단계={error.stage_name}, "
            f"작업={error.grip_task}, DI13 관리자 호출 대기"
        )

        buttons.wait_for_button(allowed_buttons={13})
        status.publish(
            phase="ADMIN_CALLED",
            screen=10,
            progress=progress,
            title="관리자가 호출되었습니다",
            message=(
                "관리자가 작업 환경을 정리한 뒤 DI 14번 버튼을 누르면 "
                f"기억한 {error.stage_name} 단계를 처음부터 다시 시작합니다."
            ),
            busy=True,
            waiting_physical_button=True,
            grip_failure=True,
            failed_stage_id=error.stage_id,
            failed_stage_name=error.stage_name,
            failed_grip_task=error.grip_task,
            recovery_kind="GRIP_FAILURE",
            recovery_state="ADMIN_CALLED",
            grip_diagnostics=error.diagnostics,
            gripper_signal_online=grip_monitor.signal_online(),
            button=13,
            error=str(error),
        )

        buttons.wait_for_button(allowed_buttons={14})
        status.publish(
            phase="STAGE_RESTARTING",
            screen=10,
            progress=progress,
            title=f"{error.stage_name} 단계를 다시 시작합니다",
            message="DI 14번 환경 정리 완료 입력을 확인했습니다.",
            busy=True,
            grip_failure=True,
            failed_stage_id=error.stage_id,
            failed_stage_name=error.stage_name,
            failed_grip_task=error.grip_task,
            recovery_kind="GRIP_FAILURE",
            recovery_state="RESTARTING",
            grip_diagnostics=error.diagnostics,
            gripper_signal_online=grip_monitor.signal_online(),
            button=14,
        )
        node.get_logger().warning(
            f"환경 정리 완료 입력 확인: {error.stage_name} 단계 재시작"
        )
        time.sleep(0.3)

    def publish_signal_recovery(
        error: GripperSignalLostError,
        *,
        recovery_state: str,
        message: str,
        countdown_sec: float = 0.0,
        waiting_physical_button: bool = False,
        button: Optional[int] = None,
    ) -> None:
        _, progress = stage_recovery_ui.get(error.stage_id, (11, 0))
        status.publish(
            phase="GRIPPER_SIGNAL_ERROR",
            screen=11,
            progress=progress,
            title="장비 오류: 그리퍼 상태를 확인해 주세요",
            message=message,
            busy=True,
            waiting_physical_button=waiting_physical_button,
            equipment_error=True,
            failed_stage_id=error.stage_id,
            failed_stage_name=error.stage_name,
            recovery_kind="GRIPPER_SIGNAL_LOSS",
            recovery_state=recovery_state,
            grip_diagnostics=grip_monitor.diagnostics(),
            gripper_signal_online=grip_monitor.signal_online(),
            recovery_countdown_sec=countdown_sec,
            button=button,
            error=str(error),
        )

    def notify_signal_loss_immediately(error: GripperSignalLostError) -> None:
        """watchdog 스레드에서 화면 11을 즉시 latch하고 표시한다."""
        status.set_equipment_error_active(True)
        publish_signal_recovery(
            error,
            recovery_state="WAIT_MOTION_STOP",
            message=(
                f"{error.reason} 로봇 모션 정지를 요청했습니다. "
                "정지 응답이 확인될 때까지 재시작할 수 없습니다."
            ),
        )

    grip_monitor.set_signal_loss_callback(notify_signal_loss_immediately)

    def recover_gripper_signal(error: GripperSignalLostError) -> None:
        """정지와 신호 정상 확인 후 DI13+3초 승인으로 기억 단계를 재시작한다."""
        status.set_equipment_error_active(True)
        node.get_logger().error(
            f"장비 오류 복구 대기: 단계={error.stage_name}, 원인={error.reason}"
        )
        online_since: Optional[float] = None
        next_publish = 0.0

        while rclpy.ok():
            stop_result = grip_monitor.wait_for_stop_result(0.1)
            if stop_result is not True:
                if stop_result is False:
                    grip_monitor.retry_motion_stop("그리퍼 장비 오류 재정지")
                now = time.monotonic()
                if now >= next_publish:
                    publish_signal_recovery(
                        error,
                        recovery_state="WAIT_MOTION_STOP",
                        message=(
                            "로봇 모션 정지 응답을 확인하는 중입니다. "
                            "정지가 확인될 때까지 재시작 승인을 받지 않습니다."
                        ),
                    )
                    next_publish = now + 0.5
                time.sleep(0.05)
                continue

            now = time.monotonic()
            if not grip_monitor.signal_online():
                online_since = None
                if now >= next_publish:
                    health_reason = grip_monitor.signal_health_reason()
                    publish_signal_recovery(
                        error,
                        recovery_state="WAIT_GRIPPER_SIGNAL",
                        message=(
                            "그리퍼 연결과 빨간 오류 표시를 확인해 주세요. "
                            f"현재 상태: {health_reason or error.reason}"
                        ),
                    )
                    next_publish = now + 0.5
                time.sleep(0.05)
                continue

            if online_since is None:
                online_since = now
            online_duration = now - online_since
            if online_duration < GRIP_SIGNAL_RECOVERY_STABLE_SEC:
                if now >= next_publish:
                    publish_signal_recovery(
                        error,
                        recovery_state="SIGNAL_STABILIZING",
                        message="그리퍼 정상 신호가 안정적으로 유지되는지 확인 중입니다.",
                    )
                    next_publish = now + 0.2
                time.sleep(0.05)
                continue

            publish_signal_recovery(
                error,
                recovery_state="WAIT_RESTART_CONFIRMATION",
                message=(
                    "그리퍼 신호 정상 확인됨. 기억한 "
                    f"{error.stage_name} 단계를 다시 시작하려면 "
                    f"DI {GRIP_SIGNAL_RESTART_BUTTON}번 버튼을 눌러주세요."
                ),
                waiting_physical_button=True,
            )

            try:
                buttons.wait_for_button(
                    allowed_buttons={GRIP_SIGNAL_RESTART_BUTTON},
                    cancel_if=lambda: not grip_monitor.signal_online(),
                )
            except ButtonWaitCancelled:
                node.get_logger().warning(
                    "재시작 승인 대기 중 그리퍼 상태가 다시 나빠졌습니다."
                )
                online_since = None
                next_publish = 0.0
                continue

            countdown_deadline = (
                time.monotonic() + GRIP_SIGNAL_RESTART_COUNTDOWN_SEC
            )
            countdown_cancelled = False
            next_publish = 0.0
            while rclpy.ok():
                if not grip_monitor.signal_online():
                    countdown_cancelled = True
                    break
                remaining = max(0.0, countdown_deadline - time.monotonic())
                if time.monotonic() >= next_publish:
                    publish_signal_recovery(
                        error,
                        recovery_state="RESTART_COUNTDOWN",
                        message=(
                            f"DI {GRIP_SIGNAL_RESTART_BUTTON} 입력을 확인했습니다. "
                            f"{remaining:.1f}초 후 {error.stage_name} 단계를 "
                            "처음부터 다시 시작합니다."
                        ),
                        countdown_sec=remaining,
                        button=GRIP_SIGNAL_RESTART_BUTTON,
                    )
                    next_publish = time.monotonic() + 0.1
                if remaining <= 0.0:
                    break
                time.sleep(0.05)

            if not rclpy.ok():
                raise KeyboardInterrupt
            if countdown_cancelled:
                node.get_logger().warning(
                    "3초 카운트다운 중 그리퍼 상태가 다시 나빠져 재시작을 취소합니다."
                )
                online_since = None
                next_publish = 0.0
                continue

            if not grip_monitor.clear_signal_loss_if_healthy():
                online_since = None
                next_publish = 0.0
                continue
            break

        if not rclpy.ok():
            raise KeyboardInterrupt

        _, progress = stage_recovery_ui.get(error.stage_id, (11, 0))
        status.publish(
            phase="STAGE_RESTARTING",
            screen=11,
            progress=progress,
            title=f"{error.stage_name} 단계를 다시 시작합니다",
            message=(
                f"DI {GRIP_SIGNAL_RESTART_BUTTON} 입력과 3초 카운트다운을 "
                "완료했습니다."
            ),
            busy=True,
            equipment_error=True,
            failed_stage_id=error.stage_id,
            failed_stage_name=error.stage_name,
            recovery_kind="GRIPPER_SIGNAL_LOSS",
            recovery_state="RESTARTING",
            grip_diagnostics=grip_monitor.diagnostics(),
            gripper_signal_online=True,
            button=GRIP_SIGNAL_RESTART_BUTTON,
        )
        status.set_equipment_error_active(False)
        node.get_logger().warning(
            f"그리퍼 정상 및 DI {GRIP_SIGNAL_RESTART_BUTTON} 승인 확인: "
            f"{error.stage_name} 단계 재시작"
        )

    def run_protected_stage(
        stage_id: str,
        action: Callable[[], Any],
    ) -> Any:
        """현재 단계를 기억하고 그립/신호 오류 뒤 같은 단계를 재시작한다."""
        stage_name = TEST_STAGE_NAMES.get(stage_id, stage_id)
        screen, progress = stage_recovery_ui.get(stage_id, (1, 0))
        retry_count = 0

        while rclpy.ok():
            if retry_count > 0:
                status.publish(
                    phase="STAGE_RESTARTED",
                    screen=screen,
                    progress=progress,
                    title=f"{stage_name} 단계 재시작",
                    message=(
                        f"기억한 {stage_name} 단계를 처음부터 다시 실행합니다. "
                        f"재시도 {retry_count}회"
                    ),
                    busy=True,
                )

            if not grip_monitor.wait_until_online(
                GRIP_SIGNAL_STARTUP_WAIT_SEC
            ):
                grip_monitor.begin_stage(stage_id, stage_name)
                grip_monitor.force_signal_loss(
                    grip_monitor.signal_health_reason()
                    or "단계 시작 전에 그리퍼 정상 상태를 확인하지 못했습니다."
                )
                try:
                    grip_monitor.raise_if_signal_lost()
                except GripperSignalLostError as error:
                    grip_monitor.end_stage()
                    recover_gripper_signal(error)
                    restore_default_context_after_recovery()
                    retry_count += 1
                    continue

            grip_monitor.begin_stage(stage_id, stage_name)
            try:
                grip_monitor.raise_if_signal_lost()
                result = action()
                grip_monitor.raise_if_signal_lost()
            except GripFailureError as error:
                grip_monitor.end_stage()
                recover_grip_failure(error)
                restore_default_context_after_recovery()
                retry_count += 1
                continue
            except GripperSignalLostError as error:
                grip_monitor.end_stage()
                recover_gripper_signal(error)
                restore_default_context_after_recovery()
                retry_count += 1
                continue
            except BaseException:
                grip_monitor.end_stage()
                raise
            else:
                grip_monitor.end_stage()
                return result

        raise KeyboardInterrupt

    def select_grind_by_button() -> int:
        status.publish(
            phase="GRIND_SELECT",
            screen=3,
            progress=34,
            title="원하는 분쇄 굵기를 선택해 주세요",
            message=(
                "DI 13 굵게(3회전) · DI 14 중간 굵게(5회전) · "
                "DI 15 중간 곱게(7회전) · DI 16 곱게(10회전). "
                f"{SELECTION_TIMEOUT_SEC:.0f}초 동안 입력이 없으면 "
                "DI 13(굵게)을 자동 선택합니다."
            ),
            busy=True,
            waiting_physical_button=True,
            selection_timeout_sec=SELECTION_TIMEOUT_SEC,
        )
        grind_button = buttons.wait_for_button(
            timeout_sec=SELECTION_TIMEOUT_SEC,
            default_button=DEFAULT_SELECTION_BUTTON,
        )
        grind_defaulted = buttons.last_wait_defaulted()
        grind_option = GRIND_BY_BUTTON[grind_button]
        grind_name = str(grind_option["name"])
        grind_turns = int(grind_option["turns"])
        status.set_grind_selection(grind_button, grind_name, grind_turns)
        status.publish(
            phase=("GRIND_DEFAULTED" if grind_defaulted else "GRIND_SELECTED"),
            screen=3,
            progress=37,
            title=f"{grind_name}를 선택했습니다",
            message=(
                (
                    f"{SELECTION_TIMEOUT_SEC:.0f}초 입력이 없어 DI "
                    f"{grind_button}을 자동 선택했습니다. "
                    if grind_defaulted
                    else f"물리 버튼 {grind_button} 입력을 확인했습니다. "
                )
                + f"그라인더를 {grind_turns}회전합니다."
            ),
            busy=True,
            button=grind_button,
        )
        return grind_turns

    def run_full_sequence(
        preset_grind_turns: Optional[int] = None,
    ) -> None:
        node.get_logger().info(
            "커피 시스템 시작: bean_drop -> grinder -> dripper_in -> "
            "spiral_pour -> final_drip"
        )

        status.publish(
            phase="BEAN_LOADING",
            screen=2,
            progress=10,
            title="원두를 그라인더에 넣는 중",
            message="스푼을 집어 원두를 그라인더 투입구로 옮기고 있습니다.",
            busy=True,
        )
        run_protected_stage("bean_drop", bean_drop)

        if preset_grind_turns is None:
            grind_turns = select_grind_by_button()
        else:
            grind_turns = int(preset_grind_turns)
            status.set_grind_selection(
                None,
                f"테스트 설정 · {grind_turns}회전",
                grind_turns,
            )

        status.publish(
            phase="GRINDER_MOVE",
            screen=4,
            progress=42,
            title="그라인더로 이동하는 중",
            message=f"그라인더를 {grind_turns}회전합니다.",
            busy=True,
            grind_current_turns=0.0,
        )
        run_protected_stage("grinder", lambda: grinder(grind_turns))

        status.publish(
            phase="FILTER_LOADING",
            screen=5,
            progress=68,
            title="갈린 원두를 커피 필터에 넣는 중",
            message="분쇄 원두가 담긴 병을 집어 필터 위로 이동하고 있습니다.",
            busy=True,
        )
        run_protected_stage("dripper_in", dripper_in)

        status.publish(
            phase="SPIRAL_START",
            screen=6,
            progress=75,
            title="스파이럴 드립을 시작합니다",
            message="주전자를 집어 드리퍼 위에서 내향 스파이럴을 수행합니다.",
            busy=True,
            spiral_stage="시작",
            spiral_progress=0.0,
        )
        run_protected_stage("spiral_pour", spiral_pour)

        status.publish(
            phase="FINAL_DRIP_START",
            screen=7,
            progress=89,
            title="최종 드립 공정을 시작합니다",
            message=(
                "필터 홀더와 물컵을 배치한 뒤 mug TCP 원점을 고정해 "
                "물을 붓고 시작 자세로 복귀합니다."
            ),
            busy=True,
            final_drip_stage="시작",
            final_drip_progress=0.0,
        )
        run_protected_stage("final_drip", final_drip)

    def execute_test(command: dict[str, Any]) -> None:
        stage = str(command["stage"])
        stage_name = TEST_STAGE_NAMES[stage]
        grind_turns = int(command.get("grind_turns", 3))
        open_mode = str(command.get("gripper_open_mode", "spoon_cup"))

        status.begin_cycle(
            test_mode=True,
            test_stage_id=stage,
            test_stage_name=stage_name,
        )
        ensure_tool_tcp()
        node.get_logger().warning(
            f"실제 로봇 단계 테스트 실행: {stage} ({stage_name})"
        )

        if stage == "full_sequence":
            status.set_selection(None, "단계 테스트")
            run_full_sequence(grind_turns)
        elif stage == "bean_drop":
            status.publish(
                phase="TEST_BEAN_DROP",
                screen=2,
                progress=10,
                title="원두 투입 테스트 실행 중",
                message="원두 투입 단계만 1회 실행합니다.",
                busy=True,
            )
            run_protected_stage("bean_drop", bean_drop)
        elif stage == "grinder":
            status.set_grind_selection(
                None,
                f"테스트 설정 · {grind_turns}회전",
                grind_turns,
            )
            status.publish(
                phase="TEST_GRINDER",
                screen=4,
                progress=42,
                title="그라인더 테스트 실행 중",
                message=f"그라인더 단계를 {grind_turns}회전으로 실행합니다.",
                busy=True,
                grind_current_turns=0.0,
            )
            run_protected_stage("grinder", lambda: grinder(grind_turns))
        elif stage == "dripper_in":
            status.publish(
                phase="TEST_DRIPPER_IN",
                screen=5,
                progress=68,
                title="필터 투입 테스트 실행 중",
                message="필터 투입 단계만 1회 실행합니다.",
                busy=True,
            )
            run_protected_stage("dripper_in", dripper_in)
        elif stage == "spiral_pour":
            status.publish(
                phase="TEST_SPIRAL_POUR",
                screen=6,
                progress=75,
                title="스파이럴 드립 테스트 실행 중",
                message="주전자 파지부터 반환까지 1회 실행합니다.",
                busy=True,
                spiral_stage="시작",
                spiral_progress=0.0,
            )
            run_protected_stage("spiral_pour", spiral_pour)
        elif stage == "final_drip":
            status.publish(
                phase="TEST_FINAL_DRIP",
                screen=7,
                progress=89,
                title="최종 드립 테스트 실행 중",
                message="최종 드립 단계만 1회 실행합니다.",
                busy=True,
                final_drip_stage="시작",
                final_drip_progress=0.0,
            )
            run_protected_stage("final_drip", final_drip)
        elif stage == "gripper_open":
            opener = {
                "spoon_cup": spoon_cup_grip_open,
                "jar": jar_grip_open,
                "handle": handle_grip_open,
            }[open_mode]
            run_protected_stage("gripper_open", opener)
            node.get_logger().info(
                "그리퍼 열기 테스트 완료: "
                f"mode={open_mode}, name={TEST_GRIP_OPEN_MODES[open_mode]}"
            )
        elif stage == "gripper_close":
            run_protected_stage("gripper_close", grip_close)
            node.get_logger().info("그리퍼 닫기 테스트 완료")
        else:
            raise RuntimeError(f"지원하지 않는 테스트 단계: {stage}")

        status.finish_cycle("DONE")
        status.publish(
            phase="TEST_DONE",
            screen=1,
            progress=100,
            title=f"{stage_name} 테스트 완료",
            message=(
                "테스트가 끝났습니다. 잠시 후 테스트 대기 상태로 돌아가며 "
                "다른 테스트를 다시 선택할 수 있습니다."
            ),
            busy=True,
            spiral_stage=("완료" if stage in {"full_sequence", "spiral_pour"} else None),
            spiral_progress=(100.0 if stage in {"full_sequence", "spiral_pour"} else None),
            final_drip_stage=("완료" if stage in {"full_sequence", "final_drip"} else None),
            final_drip_progress=(100.0 if stage in {"full_sequence", "final_drip"} else None),
        )
        time.sleep(0.8)

    try:
        while rclpy.ok():
            control.set_busy(False)
            status.clear_test_context()
            status.reset_order_state()
            status.publish(
                phase="TEST_READY",
                screen=1,
                progress=0,
                title="원두 선택 또는 단계 테스트를 시작해 주세요",
                message=(
                    "DI 13~16으로 정상 전체 공정을 시작하거나 /test 페이지에서 "
                    "원하는 테스트를 선택하십시오. 입력할 때까지 시간 제한 없이 "
                    "기다립니다."
                ),
                busy=False,
                waiting_physical_button=True,
                error="",
            )

            source, action = buttons.wait_for_button_or_command(
                control.pop_test_command,
            )
            control.set_busy(True)

            if source == "command":
                command = dict(action)
                try:
                    execute_test(command)
                except Exception as error:
                    stage = str(command.get("stage", ""))
                    status.finish_cycle("ERROR")
                    node.get_logger().error(
                        f"단계 테스트 {stage} 실행 중 오류: {error}"
                    )
                    status.publish(
                        phase="TEST_ERROR",
                        screen=9,
                        progress=0,
                        title="단계 테스트 오류",
                        message=str(error),
                        busy=True,
                        error=f"{type(error).__name__}: {error}",
                    )
                    time.sleep(1.0)
                continue

            selected_button = int(action)
            selected_bean = BEAN_BY_BUTTON[selected_button]["name"]
            status.begin_cycle(test_mode=False)
            status.set_selection(selected_button, selected_bean)
            try:
                ensure_tool_tcp()
                status.publish(
                    phase="BEAN_SELECTED",
                    screen=1,
                    progress=3,
                    title=f"{selected_bean} 선택",
                    message=(
                        f"물리 버튼 {selected_button} 입력을 확인했습니다. "
                        + "커피 시스템을 시작합니다."
                    ),
                    busy=True,
                    button=selected_button,
                )
                run_full_sequence(None)
                status.finish_cycle("DONE")
                node.get_logger().info("커피 추출 전체 시퀀스 완료.")
                status.publish(
                    phase="WAIT_NEW_ORDER",
                    screen=8,
                    progress=100,
                    title="커피 추출이 완료되었습니다",
                    message=(
                        "새 커피를 주문하려면 물리 버튼 DI 13을 눌러주세요. "
                        "이 화면은 버튼을 누를 때까지 시간 제한 없이 유지됩니다."
                    ),
                    busy=True,
                    waiting_physical_button=True,
                    spiral_stage="완료",
                    spiral_progress=100.0,
                    final_drip_stage="완료",
                    final_drip_progress=100.0,
                )
                # 새 주문 버튼은 자동 선택 없이 DI13 입력을 무기한 기다린다.
                buttons.wait_for_button(allowed_buttons={13})
                node.get_logger().info(
                    "DI 13 커피 주문하기 입력 확인: 초기 화면으로 복귀"
                )
            except Exception as error:
                status.finish_cycle("ERROR")
                node.get_logger().error(f"전체 공정 실행 중 오류: {error}")
                status.publish(
                    phase="ERROR",
                    screen=9,
                    progress=0,
                    title="로봇 작업 오류",
                    message=str(error),
                    busy=True,
                    error=f"{type(error).__name__}: {error}",
                )
                time.sleep(1.0)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료 요청을 받았습니다.")
    finally:
        try:
            control_executor.shutdown(timeout_sec=1.0)
        except TypeError:
            control_executor.shutdown()
        control_spin_thread.join(timeout=2.0)
        control.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
