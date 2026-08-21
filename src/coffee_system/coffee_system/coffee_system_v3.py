#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
m0609_coffee_system.py

M0609 + OnRobot RG2 협동로봇 자동 커피 추출 시스템
원본 Task Writer(DRL) 프로그램(m0609_coffe_system.drl, System.drvar)을
ROS2 단일 파일 노드로 변환한 것입니다.

동작 순서 (원두 선택 -> bean_drop -> 분쇄 굵기 선택 -> grinder -> dripper_in)
============================================================================
1. 원두 선택   : DI 13~16 물리 버튼으로 원두 종류 선택
2. bean_drop   : 스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입
3. 분쇄 선택   : DI 13~16 물리 버튼으로 3/5/7/10회전 선택
4. grinder     : 선택된 회전 수만큼 원호 모션으로 분쇄
5. dripper_in  : 병을 잡고 move_periodic 후 10 N 외력 감지 대기

실행
====
    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609
    ros2 run rokey m0609_coffee_system
    (또는 패키지에 등록하지 않고 바로:  python3 m0609_coffee_system.py)

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
import time
from typing import Any, Callable, Optional

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import DR_init
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveJoint

# =============================================================================
# 사용자 설정
# =============================================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_coffee_system"

# 티치펜던트에 등록된 이름 (gear.py/move.py와 동일한 이름 사용)
TOOL_NAME = "Tool Weight_gripper"
TCP_NAME = "GripperDA_v1"

# 원본 DRL 하단 전역 설정값
VELJ_DEFAULT = 20.0
ACCJ_DEFAULT = 35.0
VELX_LIN_DEFAULT = 80.0
VELX_ROT_DEFAULT = 25.0
ACCX_LIN_DEFAULT = 300.0
ACCX_ROT_DEFAULT = 100.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =============================================================================
# 물리 버튼 / Web UI 상태 연동
# =============================================================================

STATUS_TOPIC = "/coffee_system/status"

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


class StatusReporter:
    """FastAPI Web UI가 구독할 JSON 상태를 발행한다."""

    def __init__(self, node) -> None:
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = node.create_publisher(String, STATUS_TOPIC, qos)
        self._selected_bean = ""
        self._selected_button: Optional[int] = None
        self._selected_grind = ""
        self._selected_grind_button: Optional[int] = None
        self._grind_turns = 0
        self._grind_current_turns = 0.0
        self._force_delta_n = 0.0
        self._force_peak_n = 0.0
        self._cycle_id = 1

    def set_selection(self, button: int, bean_name: str) -> None:
        self._selected_button = button
        self._selected_bean = bean_name

    def set_grind_selection(
        self,
        button: int,
        grind_name: str,
        turns: int,
    ) -> None:
        self._selected_grind_button = button
        self._selected_grind = grind_name
        self._grind_turns = turns
        self._grind_current_turns = 0.0

    def publish(
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

        grind_progress = 0.0
        if self._grind_turns > 0:
            grind_progress = (
                self._grind_current_turns / float(self._grind_turns) * 100.0
            )

        payload = {
            "phase": phase,
            "screen": screen,
            "progress": progress,
            "title": title,
            "message": message,
            "busy": busy,
            "waiting_physical_button": waiting_physical_button,
            "waiting_external_force": waiting_external_force,
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
            "button": button,
            "cycle_id": self._cycle_id,
            "error": error,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(msg)


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

    def _wait_for_source(self) -> None:
        deadline = time.monotonic() + BUTTON_SOURCE_WAIT_SEC
        self._discover_topics()

        while rclpy.ok():
            self._spin_once(0.05)
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

    def _stable_baseline(self) -> dict[int, int]:
        self._wait_for_source()
        baseline = None
        stable_since = time.monotonic()

        while rclpy.ok():
            current = self.read()
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

    def wait_for_button(self) -> int:
        baseline = self._stable_baseline()
        self._node.get_logger().info(
            "DI 13~16 버튼 입력 대기: "
            f"source={self._source}, baseline={baseline}"
        )

        while rclpy.ok():
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
                        released = self.read()
                        if released[index] == baseline[index]:
                            self._node.get_logger().info(
                                f"물리 버튼 DI {index} 해제 확인"
                            )
                            return index
                        time.sleep(BUTTON_POLL_SEC)

            time.sleep(BUTTON_POLL_SEC)

        raise KeyboardInterrupt


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
    DR_AVOID = dsr2.DR_AVOID
    # DR_OFF = dsr2.DR_OFF  # 원본에 있었으나 DSR_ROBOT2에 DR_OFF 상수 자체가 없어
    # AttributeError로 즉시 죽었음. set_velx() 3번째 인자로만 쓰였는데 그 인자
    # 자체가 실제 시그니처에 없어서 통째로 제거함 (아래 set_velx() 호출부 참고).
    ON = getattr(dsr2, "ON", 1)
    OFF = getattr(dsr2, "OFF", 0)

    status = StatusReporter(node)
    buttons = PhysicalButtonInput(node, get_digital_input)

    # =====================================================================
    # Teach 포즈 (System.drvar 원본 값)
    # =====================================================================
    System_spoon_j = posj(-69.6, 53.27, 96.56, 25.63, 114.79, -171.03)
    System_spoon_l = posx(251.01, -118.46, 40.17, 87.01, 92.73, -3.35)
    System_grinder_j = posj(-40.58, -9.25, 131.96, 78.44, 107.42, -125.46)
    System_grinder_l = posx(371.82, 168.64, 360.0, 61.05, 86.52, 56.67)
    System_handle_j = posj(15.05, 35.75, 18.15, -1.7, 124.03, 13.38)
    System_handle_l = posx(527.43, 129.24, 278.22, 121.84, -180.0, 120.1)
    System_bottle_j_1 = posj(-19.18, 34.8, 22.24, -2.53, 124.42, -13.34)
    System_bottle_j_2 = posj(-43.18, 55.32, 88.81, 53.44, 114.38, -60.57)
    System_bottle_l = posx(401.53, 167.78, 71.68, 92.22, 89.54, 89.25)
    System_bottle_l2 = posx(401.53, 167.78, 76.68, 92.22, 89.54, 89.25)
    System_drip_j = posj(-26.64, 44.1, 73.65, 89.85, 91.27, -26.21)
    System_drip_l = posx(751.71, 41.51, 238.65, 60.13, 91.27, -92.13)
    System_home = posj(-71.33, 47.91, 97.52, 18.17, 114.49, -168.37)
    System_spoon_l_2 = posx(251.01, -118.46, 50.17, 87.01, 92.73, -3.35)

    # =====================================================================
    # OnRobot RG2 그리퍼 제어 (DO 1, 2 접점 조합)
    # =====================================================================
    def grip_close() -> None:
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def jar_grip_open() -> None:
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def handle_grip_open() -> None:
        set_digital_output(1, ON)
        set_digital_output(2, ON)

    def spoon_cup_grip_open() -> None:
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

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
    ) -> float:
        """기준 힘 대비 외력 변화량이 threshold_n 이상일 때까지 기다린다.

        move_periodic 직후의 잔류 진동과 정적 하중을 오인하지 않도록 잠시 안정화한
        뒤 기준 힘을 평균 측정한다. 이후 3축 병진 힘 변화량의 벡터 크기가 연속
        EXTERNAL_FORCE_CONFIRM_SAMPLES회 임계값 이상이면 병을 친 것으로 판정한다.
        """
        node.get_logger().info(
            f"외력 감지 준비: {EXTERNAL_FORCE_SETTLE_SEC:.2f}초 안정화"
        )
        time.sleep(EXTERNAL_FORCE_SETTLE_SEC)

        baseline_samples: list[tuple[float, float, float]] = []
        for _ in range(EXTERNAL_FORCE_BASELINE_SAMPLES):
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
            current = read_external_force_xyz()
            delta = tuple(
                current[axis] - baseline[axis]
                for axis in range(3)
            )
            delta_norm = math.sqrt(sum(component * component for component in delta))
            peak_force = max(peak_force, delta_norm)

            now = time.monotonic()
            if now >= next_ui_update:
                status.publish(
                    phase="WAIT_EXTERNAL_FORCE",
                    screen=5,
                    progress=86,
                    title="병 바닥을 가볍게 쳐주세요",
                    message=(
                        f"현재 외력 변화량은 {delta_norm:.1f} N입니다. "
                        f"{threshold_n:.1f} N 이상 감지되면 자동으로 다음 단계로 진행합니다."
                    ),
                    busy=True,
                    waiting_external_force=True,
                    force_delta_n=delta_norm,
                    force_peak_n=peak_force,
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
        grip_close()
        wait(1.0)

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
        grip_close()

        # status.step("grinder", 1)  # UI 상태 발행용 — 비활성화
        movej(
            posj(13.43, 31.67, 32.98, 0.19, 115.35, 11.45),
            radius=0.0, ra=DR_MV_RA_DUPLICATE,
        )

        grind_via = posx(
            0.0, -124.5, -5.0, 90.0, -179.99, 88.27
        )
        grind_end = posx(
            -124.5, 0.0, -5.0, 90.0, -179.99, 88.27
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
        movej(System_bottle_j_1, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_bottle_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        grip_close()
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
            progress=86,
            title="병 바닥을 가볍게 쳐주세요",
            message=(
                f"기준 힘 대비 {EXTERNAL_FORCE_TRIGGER_N:.1f} N 이상의 외력이 "
                "감지되면 커피가 털린 것으로 확인하고 자동으로 다음 단계로 진행합니다."
            ),
            busy=True,
            waiting_external_force=True,
            force_delta_n=0.0,
            force_peak_n=0.0,
        )
        detected_force_n = wait_for_external_force(EXTERNAL_FORCE_TRIGGER_N)
        status.publish(
            phase="FILTER_FINISHING",
            screen=5,
            progress=92,
            title="커피 필터 투입을 마무리하는 중",
            message=(
                f"{detected_force_n:.1f} N의 외력을 감지했습니다. "
                "병을 제자리로 옮깁니다."
            ),
            busy=True,
            force_delta_n=detected_force_n,
            force_peak_n=detected_force_n,
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

    # =====================================================================
    # 전역 모션 파라미터 (원본 DRL 하단 설정과 동일)
    # =====================================================================
    set_singularity_handling(DR_AVOID)
    set_velj(VELJ_DEFAULT)
    set_accj(ACCJ_DEFAULT)
    # set_velx()는 실제 시그니처가 (vel1, vel2) 2개 인자까지만 받는데 원본은
    # 3번째 인자로 DR_OFF를 더 넘기고 있었음. DR_OFF는 DSR_ROBOT2에 존재하지도
    # 않는 상수라 AttributeError, 있었다 해도 TypeError가 났을 코드라 제거함.
    set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
    set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)

    status.publish(
        phase="READY_SELECT",
        screen=1,
        progress=0,
        title="물리 버튼으로 원두를 선택해 주세요",
        message=(
            "DI 13 에티오피아 · DI 14 콜롬비아 · "
            "DI 15 브라질 · DI 16 과테말라"
        ),
        busy=False,
        waiting_physical_button=True,
    )
    selected_button = buttons.wait_for_button()
    selected_bean = BEAN_BY_BUTTON[selected_button]["name"]
    status.set_selection(selected_button, selected_bean)
    status.publish(
        phase="BEAN_SELECTED",
        screen=1,
        progress=3,
        title=f"{selected_bean} 선택",
        message=(
            f"물리 버튼 {selected_button} 입력을 확인했습니다. "
            "커피 시스템을 시작합니다."
        ),
        busy=True,
        button=selected_button,
    )

    # =====================================================================
    # 메인 시퀀스 실행 (원본 while gLoop < 1: 1회 실행과 동일)
    # =====================================================================
    # current_stage = "bean_drop"  # UI 상태 발행(status.error)용 — 비활성화
    try:
        # 나머지 동작 전에 Tool/TCP부터 티치펜던트에 등록된 이름으로 맞춘다
        # (gear.py/move.py와 동일한 방식). 요청한 이름으로 실제 활성화됐는지
        # get_tool()/get_tcp()로 확인하고, 다르면 여기서 바로 멈춘다.
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

        node.get_logger().info("커피 추출 시스템 시작: bean_drop -> grinder -> dripper_in")

        node.get_logger().info("[1/3] bean_drop: 원두 투입 중...")
        status.publish(
            phase="BEAN_LOADING",
            screen=2,
            progress=10,
            title="원두를 그라인더에 넣는 중",
            message="스푼을 집어 원두를 그라인더 투입구로 옮기고 있습니다.",
            busy=True,
        )
        bean_drop()
        # status.stage_done("bean_drop")  # UI 상태 발행용 — 비활성화

        # current_stage = "grinder"  # UI 상태 발행(status.error)용 — 비활성화
        node.get_logger().info("[2/3] grinder: 분쇄 굵기 선택 대기...")
        status.publish(
            phase="GRIND_SELECT",
            screen=3,
            progress=34,
            title="원하는 분쇄 굵기를 선택해 주세요",
            message=(
                "DI 13 굵게(3회전) · DI 14 중간 굵게(5회전) · "
                "DI 15 중간 곱게(7회전) · DI 16 곱게(10회전)"
            ),
            busy=False,
            waiting_physical_button=True,
        )
        grind_button = buttons.wait_for_button()
        grind_option = GRIND_BY_BUTTON[grind_button]
        grind_name = grind_option["name"]
        grind_turns = int(grind_option["turns"])
        status.set_grind_selection(grind_button, grind_name, grind_turns)
        status.publish(
            phase="GRIND_SELECTED",
            screen=3,
            progress=37,
            title=f"{grind_name}를 선택했습니다",
            message=(
                f"물리 버튼 {grind_button} 입력을 확인했습니다. "
                f"그라인더를 {grind_turns}회전합니다."
            ),
            busy=True,
            button=grind_button,
        )

        node.get_logger().info(
            f"[2/3] grinder: {grind_name}, {grind_turns}회전 분쇄 중..."
        )
        status.publish(
            phase="GRINDER_MOVE",
            screen=4,
            progress=42,
            title="그라인더로 이동하는 중",
            message=(
                f"{grind_name} 설정으로 원두를 분쇄합니다. "
                f"그라인더 회전 수는 {grind_turns}회입니다."
            ),
            busy=True,
            grind_current_turns=0.0,
        )
        grinder(grind_turns)
        # status.stage_done("grinder")  # UI 상태 발행용 — 비활성화

        # current_stage = "dripper_in"  # UI 상태 발행(status.error)용 — 비활성화
        node.get_logger().info("[3/3] dripper_in: 드립 추출 중...")
        status.publish(
            phase="FILTER_LOADING",
            screen=5,
            progress=68,
            title="갈린 원두를 커피 필터에 넣는 중",
            message="분쇄 원두가 담긴 병을 집어 필터 위로 이동하고 있습니다.",
            busy=True,
        )
        dripper_in()
        # status.stage_done("dripper_in")  # UI 상태 발행용 — 비활성화

        node.get_logger().info("커피 추출 시퀀스 완료.")
        status.publish(
            phase="COMPLETE",
            screen=6,
            progress=100,
            title="커피 필터 투입이 완료되었습니다",
            message="커피 시스템 작업이 완료되었습니다.",
            busy=False,
        )
        # status.finished()  # UI 상태 발행용 — 비활성화

    except Exception as error:
        node.get_logger().error(f"실행 중 오류 발생: {error}")
        status.publish(
            phase="ERROR",
            screen=7,
            progress=0,
            title="로봇 작업 오류",
            message=str(error),
            busy=False,
            error=f"{type(error).__name__}: {error}",
        )
        # status.error(current_stage, str(error))  # UI 상태 발행용 — 비활성화
        raise

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()