# """
# m0609_coffee_system.py

# M0609 + OnRobot RG2 협동로봇 자동 커피 추출 시스템
# 원본 Task Writer(DRL) 프로그램(m0609_coffe_system.drl, System.drvar)을
# ROS2 단일 파일 노드로 변환한 것입니다.

# 동작 순서
# =========
# 1. 원두 선택       : DI 13~16 물리 버튼으로 원두 종류 선택
# 2. bean_drop       : 스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입
# 3. 분쇄 선택       : DI 13~16 물리 버튼으로 3/5/7/10회전 선택
# 4. grinder         : 선택된 회전 수만큼 원호 모션으로 분쇄
# 5. dripper_in      : 분쇄 원두가 든 병을 필터에 투입
# 6. spiral_pour     : 주전자를 파지하고 보상 내향 스파이럴 드립 후 원위치 반환

# 실행
# ====
#     ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609
#     ros2 run rokey m0609_coffee_system
#     (또는 패키지에 등록하지 않고 바로:  python3 m0609_coffee_system.py)

# 주의
# ====
# - 이 노드는 시작과 동시에 실제 로봇 모션을 수행합니다. 원본 좌표는 teach pendant에서
#   교시된 값(System.drvar)을 그대로 사용하므로, 실행 전 작업 공간(그라인더, 드리퍼,
#   병, 컵, 스푼 위치)이 교시 당시와 동일한지 반드시 확인하십시오.
# - OnRobot RG2 그리퍼는 컨트롤러 Digital Output 1, 2번 접점 조합으로 제어됩니다.
#   (그리퍼 컨트롤 박스가 해당 접점에 매핑되어 있어야 합니다.)
# - 그라인더 서브루틴은 task_compliance_ctrl() 진입 후 movec()으로 원호 모션을 수행합니다.
#   컴플라이언스 구간 진입 전 주변 장애물 여부를 확인하십시오.
# - 원본 DRL의 set_singular_handling(DR_AVOID)는 프로그래밍 매뉴얼에 공식 문서화된
#   set_singularity_handling(mode)로 치환하였습니다 (동일 기능, 특이점 자동회피).
# """

# from __future__ import annotations

# import json
# import math
# import sys
# import time
# from typing import Any, Callable, Optional

# import numpy as np
# import rclpy
# from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
# from std_msgs.msg import String

# import DR_init
# from dsr_msgs2.msg import RobotState
# from dsr_msgs2.srv import MoveJoint

# # =============================================================================
# # 사용자 설정
# # =============================================================================

# ROBOT_ID = "dsr01"
# ROBOT_MODEL = "m0609"
# NODE_NAME = "m0609_coffee_system"

# # 티치펜던트에 등록된 이름 (gear.py/move.py와 동일한 이름 사용)
# TOOL_NAME = "Tool Weight_gripper"
# TCP_NAME = "GripperDA_v1"

# # 원본 DRL 하단 전역 설정값
# VELJ_DEFAULT = 20.0
# ACCJ_DEFAULT = 35.0
# VELX_LIN_DEFAULT = 80.0
# VELX_ROT_DEFAULT = 25.0
# ACCX_LIN_DEFAULT = 300.0
# ACCX_ROT_DEFAULT = 100.0


# # =============================================================================
# # 스파이럴 드립 Sub 설정
# # =============================================================================

# SPOUT_TCP_NAME = "pot"

# SPIRAL_RADIUS_MM = 44.0
# SPIRAL_REVOLUTIONS = 5.0
# SPIRAL_DURATION_SEC = 15.0
# SPIRAL_J6_DELTA_DEG = -60.0
# SPIRAL_PATH_POINTS = 100

# CENTER_PIVOT_RETURN_DURATION_SEC = 3.0
# CENTER_PIVOT_RETURN_POINTS = 30

# POT_RELEASE_BASE_Z_OFFSET_MM = 10.0
# POT_RELEASE_SETTLE_SEC = 0.8

# MOVESX_MAX_LINEAR_STEP_MM = 15.0
# MOVESX_MAX_ANGULAR_STEP_DEG = 5.0
# MOVESX_LINEAR_VEL_MM_S = 120.0
# MOVESX_ANGULAR_VEL_DEG_S = 25.0
# MOVESX_LINEAR_ACC_MM_S2 = 300.0
# MOVESX_ANGULAR_ACC_DEG_S2 = 80.0

# SPIRAL_CENTER_OFFSET_SIGN_X = -1.0
# SPIRAL_ROTATION_SIGN = +1.0

# # 주전자 교시 좌표
# SYSTEM_POT_GRIP_VALUES = [831.36, -177.93, 108.26, 3.26, 90.23, 86.66]
# SYSTEM_POT_GRIP_JOINT_VALUES = [-28.73, 38.17, 112.55, 146.06, 65.17, -73.99]
# POUR_START_JOINT_VALUES = [-13.00, 23.41, 100.08, 144.04, 35.00, -122.57]
# PICKUP_APPROACH_REL_VALUES = [0.0, -80.0, 100.0, 0.0, 0.0, 0.0]
# PICKUP_LIFT_REL_VALUES = [-150.0, 0.0, 200.0, 0.0, 0.0, 0.0]

# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL


# # =============================================================================
# # 물리 버튼 / Web UI 상태 연동
# # =============================================================================

# STATUS_TOPIC = "/coffee_system/status"

# PHYSICAL_BUTTONS = (13, 14, 15, 16)
# BEAN_BY_BUTTON = {
#     13: {"id": "ethiopia", "name": "에티오피아 예가체프"},
#     14: {"id": "colombia", "name": "콜롬비아 수프리모"},
#     15: {"id": "brazil", "name": "브라질 산토스"},
#     16: {"id": "guatemala", "name": "과테말라 안티구아"},
# }

# # 분쇄 굵기 선택: 1회전은 movec angle 기준 360도입니다.
# GRIND_BY_BUTTON = {
#     13: {"id": "coarse", "name": "굵게 분쇄", "turns": 3},
#     14: {"id": "medium_coarse", "name": "중간 굵게 분쇄", "turns": 5},
#     15: {"id": "medium_fine", "name": "중간 곱게 분쇄", "turns": 7},
#     16: {"id": "fine", "name": "곱게 분쇄", "turns": 10},
# }
# DEGREES_PER_GRINDER_TURN = 360.0

# # 그라인더 실제 회전 진행률 측정 설정
# GRINDER_USER_COORD = 101
# GRINDER_PROGRESS_UPDATE_SEC = 0.10
# GRINDER_MOTION_START_TIMEOUT_SEC = 2.0
# MOTION_IDLE = 0

# # move_periodic 종료 후 사람이 병을 가볍게 쳤는지 확인하는 외력 조건입니다.
# # 대기 시작 시 측정한 기준 힘에서 10 N 이상 변화하면 완료로 판정합니다.
# EXTERNAL_FORCE_TRIGGER_N = 4.0
# EXTERNAL_FORCE_SETTLE_SEC = 0.60
# EXTERNAL_FORCE_BASELINE_SAMPLES = 20
# EXTERNAL_FORCE_SAMPLE_SEC = 0.02
# EXTERNAL_FORCE_CONFIRM_SAMPLES = 2
# EXTERNAL_FORCE_UI_UPDATE_SEC = 0.20

# BUTTON_POLL_SEC = 0.03
# BUTTON_DEBOUNCE_SEC = 0.05
# BUTTON_BASELINE_STABLE_SEC = 0.20
# BUTTON_STATE_STALE_SEC = 1.00
# BUTTON_SOURCE_WAIT_SEC = 3.00

# ROBOT_STATE_TYPE = "dsr_msgs2/msg/RobotState"
# ROBOT_STATE_TOPIC_CANDIDATES = (
#     "/dsr01/msg/robot_state",
#     "/dsr01/robot_state",
#     "/dsr01/dsr_controller2/robot_state",
# )


# def _pose6_from_dsr(value: Any) -> tuple[float, float, float, float, float, float]:
#     """get_current_posx() 반환값에서 6축 pose를 추출한다.

#     DSR_ROBOT2 버전에 따라 posx 자체 또는 (posx, solution_space) 형태로
#     반환될 수 있어 두 형식을 모두 처리한다.
#     """
#     candidate = value

#     if isinstance(value, (list, tuple)) and len(value) == 2:
#         first = value[0]
#         if hasattr(first, "__iter__") and not isinstance(first, (str, bytes)):
#             first_values = list(first)
#             if len(first_values) >= 6:
#                 candidate = first_values

#     if hasattr(candidate, "__iter__") and not isinstance(
#         candidate, (str, bytes)
#     ):
#         values = [float(item) for item in list(candidate)]
#     else:
#         raise RuntimeError(f"현재 TCP pose 형식이 올바르지 않습니다: {value!r}")

#     if len(values) < 6 or not all(math.isfinite(item) for item in values[:6]):
#         raise RuntimeError(f"현재 TCP pose 값이 올바르지 않습니다: {value!r}")

#     return tuple(values[:6])


# def _wrap_radians(angle: float) -> float:
#     """각도를 [-pi, pi) 범위로 정규화한다."""
#     return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


# def _json_turn_value(value: float) -> int | float:
#     """정수 회전은 JSON 정수로, 부분 회전은 소수로 반환한다."""
#     rounded = round(float(value), 3)
#     nearest = round(rounded)
#     if abs(rounded - nearest) < 1.0e-6:
#         return int(nearest)
#     return rounded



# # =============================================================================
# # 스파이럴 자세/경로 수학 유틸리티
# # =============================================================================

# def _clamp(value: float, low: float, high: float) -> float:
#     return max(low, min(high, float(value)))


# def _smoothstep5(progress: float) -> float:
#     u = _clamp(progress, 0.0, 1.0)
#     return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


# def _rot_y_deg(angle_deg: float) -> np.ndarray:
#     angle = math.radians(float(angle_deg))
#     c = math.cos(angle)
#     s = math.sin(angle)
#     return np.array(
#         [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
#         dtype=float,
#     )


# def _rot_z_deg(angle_deg: float) -> np.ndarray:
#     angle = math.radians(float(angle_deg))
#     c = math.cos(angle)
#     s = math.sin(angle)
#     return np.array(
#         [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
#         dtype=float,
#     )


# def _zyz_to_rotm(abc_deg) -> np.ndarray:
#     a_deg, b_deg, c_deg = [float(value) for value in abc_deg[:3]]
#     return _rot_z_deg(a_deg) @ _rot_y_deg(b_deg) @ _rot_z_deg(c_deg)


# def _project_rotation(rotation: np.ndarray) -> np.ndarray:
#     u_matrix, _, vt_matrix = np.linalg.svd(
#         np.asarray(rotation, dtype=float).reshape(3, 3)
#     )
#     output = u_matrix @ vt_matrix
#     if np.linalg.det(output) < 0.0:
#         u_matrix[:, -1] *= -1.0
#         output = u_matrix @ vt_matrix
#     return output


# def _rotvec_to_rotm(rotvec_rad) -> np.ndarray:
#     vector = np.asarray(rotvec_rad, dtype=float).reshape(3)
#     angle = float(np.linalg.norm(vector))
#     if angle < 1.0e-12:
#         return np.eye(3, dtype=float)

#     axis = vector / angle
#     x_axis, y_axis, z_axis = axis
#     skew = np.array(
#         [
#             [0.0, -z_axis, y_axis],
#             [z_axis, 0.0, -x_axis],
#             [-y_axis, x_axis, 0.0],
#         ],
#         dtype=float,
#     )
#     return (
#         np.eye(3, dtype=float)
#         + math.sin(angle) * skew
#         + (1.0 - math.cos(angle)) * (skew @ skew)
#     )


# def _rotm_to_rotvec(rotation: np.ndarray) -> np.ndarray:
#     matrix = _project_rotation(rotation)
#     cosine_angle = _clamp(
#         (float(np.trace(matrix)) - 1.0) * 0.5,
#         -1.0,
#         1.0,
#     )
#     angle = math.acos(cosine_angle)

#     if angle < 1.0e-10:
#         return np.zeros(3, dtype=float)

#     if abs(math.pi - angle) < 1.0e-6:
#         diagonal = np.maximum((np.diag(matrix) + 1.0) * 0.5, 0.0)
#         axis = np.sqrt(diagonal)
#         if matrix[2, 1] - matrix[1, 2] < 0.0:
#             axis[0] *= -1.0
#         if matrix[0, 2] - matrix[2, 0] < 0.0:
#             axis[1] *= -1.0
#         if matrix[1, 0] - matrix[0, 1] < 0.0:
#             axis[2] *= -1.0
#         axis_norm = float(np.linalg.norm(axis))
#         if axis_norm < 1.0e-9:
#             axis = np.array([1.0, 0.0, 0.0], dtype=float)
#         else:
#             axis = axis / axis_norm
#         return axis * angle

#     factor = angle / (2.0 * math.sin(angle))
#     return factor * np.array(
#         [
#             matrix[2, 1] - matrix[1, 2],
#             matrix[0, 2] - matrix[2, 0],
#             matrix[1, 0] - matrix[0, 1],
#         ],
#         dtype=float,
#     )


# def _wrapped_delta_deg(value: float, reference: float) -> float:
#     return (float(value) - float(reference) + 180.0) % 360.0 - 180.0


# def _rotm_to_zyz_candidates(rotation: np.ndarray) -> list[np.ndarray]:
#     matrix = _project_rotation(rotation)
#     b_angle = math.acos(_clamp(float(matrix[2, 2]), -1.0, 1.0))
#     sine_b = math.sin(b_angle)

#     if abs(sine_b) > 1.0e-8:
#         a_angle = math.atan2(float(matrix[1, 2]), float(matrix[0, 2]))
#         c_angle = math.atan2(float(matrix[2, 1]), -float(matrix[2, 0]))
#         first = np.degrees(
#             np.array([a_angle, b_angle, c_angle], dtype=float)
#         )
#         second = np.array(
#             [first[0] + 180.0, -first[1], first[2] + 180.0],
#             dtype=float,
#         )
#         return [first, second]

#     combined = math.degrees(
#         math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
#     )
#     b_deg = math.degrees(b_angle)
#     return [
#         np.array([combined, b_deg, 0.0], dtype=float),
#         np.array([0.0, b_deg, combined], dtype=float),
#     ]


# def _rotm_to_zyz_near(
#     rotation: np.ndarray,
#     reference_abc_deg,
# ) -> np.ndarray:
#     reference = np.asarray(reference_abc_deg, dtype=float).reshape(3)
#     best = None
#     best_cost = math.inf

#     for candidate in _rotm_to_zyz_candidates(rotation):
#         adjusted = candidate.copy()
#         for index in (0, 2):
#             adjusted[index] = reference[index] + _wrapped_delta_deg(
#                 adjusted[index],
#                 reference[index],
#             )

#         cost = float(
#             np.linalg.norm(
#                 np.array(
#                     [
#                         _wrapped_delta_deg(adjusted[0], reference[0]),
#                         adjusted[1] - reference[1],
#                         _wrapped_delta_deg(adjusted[2], reference[2]),
#                     ],
#                     dtype=float,
#                 )
#             )
#         )
#         if cost < best_cost:
#             best = adjusted
#             best_cost = cost

#     if best is None:
#         raise RuntimeError(
#             "회전행렬을 Doosan Z-Y-Z Euler로 변환하지 못했습니다."
#         )
#     return best


# def _numeric6(value: Any, *, label: str) -> np.ndarray:
#     candidate = value

#     if isinstance(candidate, tuple) and len(candidate) >= 1:
#         first = candidate[0]
#         if hasattr(first, "__iter__") and not isinstance(
#             first,
#             (str, bytes),
#         ):
#             candidate = first
#     elif isinstance(candidate, list) and len(candidate) == 2:
#         first = candidate[0]
#         if hasattr(first, "__iter__") and not isinstance(
#             first,
#             (str, bytes),
#         ):
#             first_values = list(first)
#             if len(first_values) >= 6:
#                 candidate = first_values

#     try:
#         values = np.asarray(
#             [float(item) for item in candidate],
#             dtype=float,
#         )
#     except Exception as exc:
#         raise RuntimeError(f"{label} 변환 실패: {value!r}") from exc

#     if values.size < 6:
#         raise RuntimeError(f"{label} 길이 오류: {values.tolist()!r}")

#     result = values[:6].copy()
#     if not np.all(np.isfinite(result)):
#         raise RuntimeError(
#             f"{label}에 유효하지 않은 값이 있습니다: {result!r}"
#         )
#     return result


# class StatusReporter:
#     """FastAPI Web UI가 구독할 JSON 상태를 발행한다."""

#     def __init__(self, node) -> None:
#         qos = QoSProfile(depth=10)
#         qos.reliability = ReliabilityPolicy.RELIABLE
#         qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
#         self._publisher = node.create_publisher(String, STATUS_TOPIC, qos)
#         self._selected_bean = ""
#         self._selected_button: Optional[int] = None
#         self._selected_grind = ""
#         self._selected_grind_button: Optional[int] = None
#         self._grind_turns = 0
#         self._grind_current_turns = 0.0
#         self._force_delta_n = 0.0
#         self._force_peak_n = 0.0
#         self._spiral_stage = "대기"
#         self._spiral_progress = 0.0
#         self._cycle_id = 1

#     def set_selection(self, button: int, bean_name: str) -> None:
#         self._selected_button = button
#         self._selected_bean = bean_name

#     def set_grind_selection(
#         self,
#         button: int,
#         grind_name: str,
#         turns: int,
#     ) -> None:
#         self._selected_grind_button = button
#         self._selected_grind = grind_name
#         self._grind_turns = turns
#         self._grind_current_turns = 0.0

#     def publish(
#         self,
#         *,
#         phase: str,
#         screen: int,
#         progress: int,
#         title: str,
#         message: str,
#         busy: bool,
#         waiting_physical_button: bool = False,
#         waiting_external_force: bool = False,
#         force_delta_n: Optional[float] = None,
#         force_peak_n: Optional[float] = None,
#         grind_current_turns: Optional[float] = None,
#         spiral_stage: Optional[str] = None,
#         spiral_progress: Optional[float] = None,
#         button: Optional[int] = None,
#         error: str = "",
#     ) -> None:
#         if force_delta_n is not None:
#             self._force_delta_n = float(force_delta_n)
#         if force_peak_n is not None:
#             self._force_peak_n = float(force_peak_n)
#         if grind_current_turns is not None:
#             current = max(0.0, float(grind_current_turns))
#             if self._grind_turns > 0:
#                 current = min(current, float(self._grind_turns))
#             self._grind_current_turns = current
#         if spiral_stage is not None:
#             self._spiral_stage = str(spiral_stage)
#         if spiral_progress is not None:
#             self._spiral_progress = max(
#                 0.0,
#                 min(100.0, float(spiral_progress)),
#             )

#         grind_progress = 0.0
#         if self._grind_turns > 0:
#             grind_progress = (
#                 self._grind_current_turns / float(self._grind_turns) * 100.0
#             )

#         payload = {
#             "phase": phase,
#             "screen": screen,
#             "progress": progress,
#             "title": title,
#             "message": message,
#             "busy": busy,
#             "waiting_physical_button": waiting_physical_button,
#             "waiting_external_force": waiting_external_force,
#             "force_threshold_n": EXTERNAL_FORCE_TRIGGER_N,
#             "force_delta_n": round(self._force_delta_n, 3),
#             "force_peak_n": round(self._force_peak_n, 3),
#             "selected_bean": self._selected_bean,
#             "selected_button": self._selected_button,
#             "selected_grind": self._selected_grind,
#             "selected_grind_button": self._selected_grind_button,
#             "grind_turns": self._grind_turns,
#             "grind_current_turns": _json_turn_value(
#                 self._grind_current_turns
#             ),
#             "grind_progress": round(grind_progress, 1),
#             "spiral_stage": self._spiral_stage,
#             "spiral_progress": round(self._spiral_progress, 1),
#             "spiral_radius_mm": SPIRAL_RADIUS_MM,
#             "spiral_revolutions": SPIRAL_REVOLUTIONS,
#             "spiral_duration_sec": SPIRAL_DURATION_SEC,
#             "spiral_j6_delta_deg": SPIRAL_J6_DELTA_DEG,
#             "button": button,
#             "cycle_id": self._cycle_id,
#             "error": error,
#             "timestamp": time.time(),
#         }
#         msg = String()
#         msg.data = json.dumps(payload, ensure_ascii=False)
#         self._publisher.publish(msg)


# class PhysicalButtonInput:
#     """DI 13~16을 RobotState 우선, get_digital_input() 보조로 감지한다.

#     버튼 해제 상태를 0으로 가정하지 않고 안정된 현재값을 기준값으로 사용하므로
#     Active-High와 Active-Low 배선을 모두 처리한다.
#     """

#     def __init__(
#         self,
#         node,
#         get_digital_input: Callable[[int], int],
#     ) -> None:
#         self._node = node
#         self._get_digital_input = get_digital_input
#         self._subscriptions = {}
#         self._robot_state_buttons = {}
#         self._robot_state_stamp = 0.0
#         self._source = "uninitialized"

#         for topic in ROBOT_STATE_TOPIC_CANDIDATES:
#             self._register_topic(topic)

#     def _register_topic(self, topic: str) -> None:
#         if not topic or topic in self._subscriptions:
#             return

#         subscription = self._node.create_subscription(
#             RobotState,
#             topic,
#             lambda msg, source=topic: self._robot_state_callback(msg, source),
#             10,
#         )
#         self._subscriptions[topic] = subscription
#         self._node.get_logger().info(f"RobotState 구독 등록: {topic}")

#     def _discover_topics(self) -> None:
#         try:
#             topics = self._node.get_topic_names_and_types()
#         except Exception as error:
#             self._node.get_logger().warning(f"RobotState 토픽 탐색 실패: {error}")
#             return

#         for topic, type_names in topics:
#             if ROBOT_STATE_TYPE in type_names:
#                 self._register_topic(topic)

#     def _robot_state_callback(self, msg: RobotState, source: str) -> None:
#         values = {}

#         if hasattr(msg, "controller_digital_input"):
#             mask = int(getattr(msg, "controller_digital_input"))
#             values = {
#                 index: (mask >> (index - 1)) & 0x1
#                 for index in PHYSICAL_BUTTONS
#             }
#         elif hasattr(msg, "ctrlbox_digital_input"):
#             raw = list(getattr(msg, "ctrlbox_digital_input"))
#             if len(raw) >= max(PHYSICAL_BUTTONS):
#                 values = {
#                     index: int(raw[index - 1])
#                     for index in PHYSICAL_BUTTONS
#                 }

#         if not values:
#             return

#         self._robot_state_buttons = values
#         self._robot_state_stamp = time.monotonic()
#         new_source = f"RobotState:{source}"

#         if self._source != new_source:
#             self._source = new_source
#             self._node.get_logger().info(
#                 f"물리 버튼 입력 소스 전환: {self._source}, raw={values}"
#             )

#     def _spin_once(self, timeout_sec: float = 0.0) -> None:
#         rclpy.spin_once(self._node, timeout_sec=max(0.0, timeout_sec))

#     def _robot_state_is_fresh(self) -> bool:
#         return (
#             bool(self._robot_state_buttons)
#             and time.monotonic() - self._robot_state_stamp
#             <= BUTTON_STATE_STALE_SEC
#         )

#     def _wait_for_source(self) -> None:
#         deadline = time.monotonic() + BUTTON_SOURCE_WAIT_SEC
#         self._discover_topics()

#         while rclpy.ok():
#             self._spin_once(0.05)
#             self._discover_topics()

#             if self._robot_state_is_fresh():
#                 return

#             if time.monotonic() >= deadline:
#                 self._node.get_logger().warning(
#                     "RobotState DI를 받지 못해 get_digital_input()으로 대체합니다."
#                 )
#                 return

#     def read(self) -> dict[int, int]:
#         self._spin_once(0.0)
#         self._discover_topics()

#         if self._robot_state_is_fresh():
#             return dict(self._robot_state_buttons)

#         values = {
#             index: int(self._get_digital_input(index))
#             for index in PHYSICAL_BUTTONS
#         }

#         if self._source != "DSR:get_digital_input":
#             self._source = "DSR:get_digital_input"
#             self._node.get_logger().warning(
#                 f"물리 버튼 입력 소스: {self._source}, raw={values}"
#             )

#         return values

#     def _stable_baseline(self) -> dict[int, int]:
#         self._wait_for_source()
#         baseline = None
#         stable_since = time.monotonic()

#         while rclpy.ok():
#             current = self.read()
#             now = time.monotonic()

#             if baseline != current:
#                 baseline = dict(current)
#                 stable_since = now
#                 self._node.get_logger().info(
#                     f"버튼 기준값 확인 ({self._source}): {baseline}"
#                 )

#             if now - stable_since >= BUTTON_BASELINE_STABLE_SEC:
#                 return dict(baseline)

#             time.sleep(BUTTON_POLL_SEC)

#         raise KeyboardInterrupt

#     def wait_for_button(self) -> int:
#         baseline = self._stable_baseline()
#         self._node.get_logger().info(
#             "DI 13~16 버튼 입력 대기: "
#             f"source={self._source}, baseline={baseline}"
#         )

#         while rclpy.ok():
#             current = self.read()
#             changed = [
#                 index
#                 for index in PHYSICAL_BUTTONS
#                 if current[index] != baseline[index]
#             ]

#             if changed:
#                 index = changed[0]
#                 time.sleep(BUTTON_DEBOUNCE_SEC)
#                 confirmed = self.read()

#                 if confirmed[index] != baseline[index]:
#                     self._node.get_logger().info(
#                         f"물리 버튼 DI {index} 감지: "
#                         f"{baseline[index]} -> {confirmed[index]}"
#                     )

#                     while rclpy.ok():
#                         released = self.read()
#                         if released[index] == baseline[index]:
#                             self._node.get_logger().info(
#                                 f"물리 버튼 DI {index} 해제 확인"
#                             )
#                             return index
#                         time.sleep(BUTTON_POLL_SEC)

#             time.sleep(BUTTON_POLL_SEC)

#         raise KeyboardInterrupt


# # =============================================================================
# # main
# # =============================================================================

# def main(args=None) -> None:
#     rclpy.init(args=args)

#     node = rclpy.create_node(
#         NODE_NAME,
#         namespace=ROBOT_ID,
#     )

#     DR_init.__dsr__node = node

#     try:
#         import DSR_ROBOT2 as dsr2
#     except ImportError as error:
#         node.get_logger().error(f"DSR_ROBOT2 모듈을 불러오지 못했습니다: {error}")
#         node.destroy_node()
#         rclpy.shutdown()
#         sys.exit(1)

#     # DSR_ROBOT2 import 시점에 생성되는 ~100개 서비스 클라이언트가 컨트롤러와 DDS
#     # discovery를 마칠 때까지 기다린다. set_singular_handling() 등 일부 API는
#     # movej()와 달리 wait_for_service() 없이 곧바로 call_async()를 던지므로,
#     # discovery가 끝나기 전에 첫 호출이 나가면 응답이 영영 오지 않아
#     # spin_until_future_complete()가 무한 대기에 빠진다. (고정 sleep은 시스템
#     # 부하에 따라 부족할 수 있어 실패 사례가 있었다 — 같은 노드로 대표 서비스
#     # 하나가 실제로 매칭될 때까지 기다려서 나머지 클라이언트도 함께 뜨게 한다.)
#     startup_probe = node.create_client(MoveJoint, "motion/move_joint")
#     while not startup_probe.wait_for_service(timeout_sec=1.0):
#         node.get_logger().info("컨트롤러 서비스 연결 대기 중...")
#     node.destroy_client(startup_probe)

#     # -------------------------------------------------------------------
#     # API 바인딩 (DRL 원본과 동일한 이름으로 사용)
#     # -------------------------------------------------------------------
#     movej = dsr2.movej
#     movel = dsr2.movel
#     movec = dsr2.movec
#     movesx = dsr2.movesx
#     amovec = getattr(dsr2, "amovec", None)
#     amovel = dsr2.amovel
#     move_periodic = dsr2.move_periodic
#     task_compliance_ctrl = dsr2.task_compliance_ctrl
#     release_compliance_ctrl = dsr2.release_compliance_ctrl
#     set_stiffnessx = dsr2.set_stiffnessx
#     set_digital_output = dsr2.set_digital_output
#     get_digital_input = dsr2.get_digital_input
#     get_tool_force = dsr2.get_tool_force
#     get_current_posx = dsr2.get_current_posx
#     get_current_posj = dsr2.get_current_posj
#     fkin = dsr2.fkin
#     check_motion = getattr(dsr2, "check_motion", None)
#     mwait = getattr(dsr2, "mwait", None)
#     set_tool = dsr2.set_tool
#     set_tcp = dsr2.set_tcp
#     get_tool = dsr2.get_tool
#     get_tcp = dsr2.get_tcp
#     drl_script_run = dsr2.drl_script_run
#     get_drl_state = dsr2.get_drl_state
#     get_robot_system = dsr2.get_robot_system
#     set_singularity_handling = dsr2.set_singularity_handling
#     set_velj = dsr2.set_velj
#     set_accj = dsr2.set_accj
#     set_velx = dsr2.set_velx
#     set_accx = dsr2.set_accx
#     wait = dsr2.wait
#     posj = dsr2.posj
#     posx = dsr2.posx

#     DR_BASE = dsr2.DR_BASE
#     DR_TOOL = dsr2.DR_TOOL
#     DR_MV_MOD_ABS = dsr2.DR_MV_MOD_ABS
#     DR_MV_MOD_REL = dsr2.DR_MV_MOD_REL
#     DR_MV_RA_DUPLICATE = dsr2.DR_MV_RA_DUPLICATE
#     DR_MV_RA_OVERRIDE = dsr2.DR_MV_RA_OVERRIDE
#     DR_MVS_VEL_NONE = getattr(dsr2, "DR_MVS_VEL_NONE", 0)
#     DR_AVOID = dsr2.DR_AVOID
#     # DR_OFF = dsr2.DR_OFF  # 원본에 있었으나 DSR_ROBOT2에 DR_OFF 상수 자체가 없어
#     # AttributeError로 즉시 죽었음. set_velx() 3번째 인자로만 쓰였는데 그 인자
#     # 자체가 실제 시그니처에 없어서 통째로 제거함 (아래 set_velx() 호출부 참고).
#     ON = getattr(dsr2, "ON", 1)
#     OFF = getattr(dsr2, "OFF", 0)

#     status = StatusReporter(node)
#     buttons = PhysicalButtonInput(node, get_digital_input)

#     # =====================================================================
#     # Teach 포즈 (System.drvar 원본 값)
#     # =====================================================================
#     System_spoon_j = posj(-69.6, 53.27, 96.56, 25.63, 114.79, -171.03)
#     System_spoon_l = posx(251.01, -118.46, 40.17, 87.01, 92.73, -3.35)
#     System_grinder_j = posj(-40.58, -9.25, 131.96, 78.44, 107.42, -125.46)
#     System_grinder_l = posx(371.82, 168.64, 360.0, 61.05, 86.52, 56.67)
#     System_handle_j = posj(15.05, 35.75, 18.15, -1.7, 124.03, 13.38)
#     System_handle_l = posx(527.43, 129.24, 278.22, 121.84, -180.0, 120.1)
#     System_bottle_j_1 = posj(-19.18, 34.8, 22.24, -2.53, 124.42, -13.34)
#     System_bottle_j_2 = posj(-43.18, 55.32, 88.81, 53.44, 114.38, -60.57)
#     System_bottle_l = posx(401.53, 167.78, 71.68, 92.22, 89.54, 89.25)
#     System_bottle_l2 = posx(401.53, 167.78, 76.68, 92.22, 89.54, 89.25)
#     System_drip_j = posj(-26.64, 44.1, 73.65, 89.85, 91.27, -26.21)
#     System_drip_l = posx(751.71, 41.51, 238.65, 60.13, 91.27, -92.13)
#     System_home = posj(-71.33, 47.91, 97.52, 18.17, 114.49, -168.37)
#     System_spoon_l_2 = posx(251.01, -118.46, 50.17, 87.01, 92.73, -3.35)

#     # 스파이럴 드립 Sub 교시 포즈
#     System_pot_grip = posx(*SYSTEM_POT_GRIP_VALUES)
#     System_pot_grip_joint = posj(*SYSTEM_POT_GRIP_JOINT_VALUES)
#     Pour_start_joint = posj(*POUR_START_JOINT_VALUES)
#     Pickup_approach_rel = posx(*PICKUP_APPROACH_REL_VALUES)
#     Pickup_lift_rel = posx(*PICKUP_LIFT_REL_VALUES)

#     # =====================================================================
#     # OnRobot RG2 그리퍼 제어 (DO 1, 2 접점 조합)
#     # =====================================================================
#     def grip_close() -> None:
#         set_digital_output(1, ON)
#         set_digital_output(2, OFF)

#     def jar_grip_open() -> None:
#         set_digital_output(1, OFF)
#         set_digital_output(2, ON)

#     def handle_grip_open() -> None:
#         set_digital_output(1, ON)
#         set_digital_output(2, ON)

#     def spoon_cup_grip_open() -> None:
#         set_digital_output(1, OFF)
#         set_digital_output(2, OFF)

#     def apply_tcp(name: str, timeout_sec: float = 6.0, poll_interval: float = 0.1) -> bool:
#         """set_tcp()를 DRL 프로그램으로 실행해서 TCP를 바꾼다.

#         set_tcp()를 ROS2 서비스로 직접 부르면 컨트롤러가 매번
#         "this command can only be used in manual mode"로 거부한다 (실측: 6초
#         재시도 내내 100% 거부, 반영된 적 0회). 티치펜던트 Task Writer 프로그램은
#         같은 명령이 통과되는 걸로 봐서, 로봇 모드(AUTONOMOUS/MANUAL) 자체보다는
#         "실행 중인 DRL 프로그램 컨텍스트에서 나온 호출이냐"가 실제 조건으로
#         보인다. drl_script_run()으로 set_tcp(...) 한 줄짜리 코드를 컨트롤러에
#         네이티브 프로그램처럼 로드해서 실행시켜 이 조건을 맞춘다.
#         """
#         code = f'set_tcp("{name}")'
#         start_ret = drl_script_run(get_robot_system(), code)
#         if start_ret != 0:
#             node.get_logger().error(f"drl_script_run 시작 실패: {code!r}, ret={start_ret}")
#             return False

#         deadline = time.monotonic() + timeout_sec
#         # get_drl_state(): 0=PLAY, 1=STOP, 2=HOLD, 3=LAST. PLAY를 벗어날 때까지 대기.
#         while get_drl_state() == 0:
#             if time.monotonic() >= deadline:
#                 node.get_logger().error(
#                     f"drl_script_run({code!r}) {timeout_sec:.0f}초 내에 끝나지 않음"
#                 )
#                 return False
#             time.sleep(poll_interval)

#         active = get_tcp()
#         if active != name:
#             node.get_logger().error(
#                 f"drl_script_run으로도 set_tcp('{name}') 반영 안 됨: 실제 활성={active}"
#             )
#             return False
#         return True

#     def read_external_force_xyz() -> tuple[float, float, float]:
#         """현재 TCP 외력의 병진 성분 Fx, Fy, Fz를 Tool 좌표계로 읽는다."""
#         value = get_tool_force(DR_TOOL)

#         if not isinstance(value, (list, tuple)) or len(value) < 3:
#             raise RuntimeError(
#                 f"get_tool_force() 반환값이 올바르지 않습니다: {value!r}"
#             )

#         force_xyz = tuple(float(value[index]) for index in range(3))
#         if not all(math.isfinite(component) for component in force_xyz):
#             raise RuntimeError(
#                 f"get_tool_force()에 유효하지 않은 값이 포함되어 있습니다: "
#                 f"{force_xyz!r}"
#             )

#         return force_xyz

#     def wait_for_external_force(
#         threshold_n: float = EXTERNAL_FORCE_TRIGGER_N,
#     ) -> float:
#         """기준 힘 대비 외력 변화량이 threshold_n 이상일 때까지 기다린다.

#         move_periodic 직후의 잔류 진동과 정적 하중을 오인하지 않도록 잠시 안정화한
#         뒤 기준 힘을 평균 측정한다. 이후 3축 병진 힘 변화량의 벡터 크기가 연속
#         EXTERNAL_FORCE_CONFIRM_SAMPLES회 임계값 이상이면 병을 친 것으로 판정한다.
#         """
#         node.get_logger().info(
#             f"외력 감지 준비: {EXTERNAL_FORCE_SETTLE_SEC:.2f}초 안정화"
#         )
#         time.sleep(EXTERNAL_FORCE_SETTLE_SEC)

#         baseline_samples: list[tuple[float, float, float]] = []
#         for _ in range(EXTERNAL_FORCE_BASELINE_SAMPLES):
#             baseline_samples.append(read_external_force_xyz())
#             time.sleep(EXTERNAL_FORCE_SAMPLE_SEC)

#         baseline = tuple(
#             sum(sample[axis] for sample in baseline_samples)
#             / len(baseline_samples)
#             for axis in range(3)
#         )

#         node.get_logger().info(
#             "외력 기준값 설정 완료: "
#             f"Fx={baseline[0]:.3f}, Fy={baseline[1]:.3f}, "
#             f"Fz={baseline[2]:.3f} N, threshold={threshold_n:.1f} N"
#         )

#         consecutive = 0
#         peak_force = 0.0
#         next_ui_update = 0.0

#         while rclpy.ok():
#             current = read_external_force_xyz()
#             delta = tuple(
#                 current[axis] - baseline[axis]
#                 for axis in range(3)
#             )
#             delta_norm = math.sqrt(sum(component * component for component in delta))
#             peak_force = max(peak_force, delta_norm)

#             now = time.monotonic()
#             if now >= next_ui_update:
#                 status.publish(
#                     phase="WAIT_EXTERNAL_FORCE",
#                     screen=5,
#                     progress=70,
#                     title="병 바닥을 가볍게 쳐주세요",
#                     message=(
#                         f"현재 외력 변화량은 {delta_norm:.1f} N입니다. "
#                         f"{threshold_n:.1f} N 이상 감지되면 자동으로 다음 단계로 진행합니다."
#                     ),
#                     busy=True,
#                     waiting_external_force=True,
#                     force_delta_n=delta_norm,
#                     force_peak_n=peak_force,
#                 )
#                 next_ui_update = now + EXTERNAL_FORCE_UI_UPDATE_SEC

#             if delta_norm >= threshold_n:
#                 consecutive += 1
#                 if consecutive >= EXTERNAL_FORCE_CONFIRM_SAMPLES:
#                     node.get_logger().info(
#                         f"외력 감지 완료: {delta_norm:.3f} N "
#                         f"(peak={peak_force:.3f} N)"
#                     )
#                     return delta_norm
#             else:
#                 consecutive = 0

#             time.sleep(EXTERNAL_FORCE_SAMPLE_SEC)

#         raise KeyboardInterrupt

#     # status = StatusReporter(node)  # UI 상태 발행용 — 비활성화

#     # 아래 movel()/amovel() 호출들: 원본에는 전부 app_type=DR_MV_APP_NONE 키워드가
#     # 붙어 있었으나, 실제 movel()/amovel() 시그니처에 app_type 파라미터가 없어
#     # TypeError가 나서 전부 제거함 (DRL 원본의 app 개념이 Python API 변환 과정에서
#     # 잘못 옮겨진 것으로 보임, movec()의 ori 제거와 같은 종류의 수정).
#     # =====================================================================
#     # 서브 루틴 (Task Writer 서브 프로그램 대응)
#     # =====================================================================
#     def bean_drop() -> None:
#         """스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입."""
#         # status.step("bean_drop", 0)  # UI 상태 발행용 — 비활성화
#         spoon_cup_grip_open()

#         # status.step("bean_drop", 1)  # UI 상태 발행용 — 비활성화
#         movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

#         # status.step("bean_drop", 2)  # UI 상태 발행용 — 비활성화
#         movej(System_spoon_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movel(
#             System_spoon_l, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )
#         grip_close()
#         wait(1.0)

#         # status.step("bean_drop", 3)  # UI 상태 발행용 — 비활성화
#         movel(
#             posx(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )
#         movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movel(
#             System_grinder_l, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )

#         # status.step("bean_drop", 4)  # UI 상태 발행용 — 비활성화
#         amovel(
#             posx(32.0, -15.0, 0.0, 0.0, 0.0, 0.0), time=1.0, ref=DR_BASE,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )
#         movej(
#             posj(0.0, 0.0, 0.0, 0.0, 0.0, 45.0), time=1.0,
#             radius=0.0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )
#         movel(
#             posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )

#         # status.step("bean_drop", 5)  # UI 상태 발행용 — 비활성화
#         movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)

#         # status.step("bean_drop", 6)  # UI 상태 발행용 — 비활성화
#         movel(
#             System_spoon_l_2, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )
#         movel(
#             posx(0.0, 0.0, -10.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )
#         spoon_cup_grip_open()
#         wait(1.0)

#         # status.step("bean_drop", 7)  # UI 상태 발행용 — 비활성화
#         movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

#     def publish_grinding_progress(
#         current_turns: float,
#         total_turns: int,
#     ) -> None:
#         """웹 UI에 실제 그라인더 회전량과 전체 회전량을 발행한다."""
#         total = max(1, int(total_turns))
#         current = max(0.0, min(float(current_turns), float(total)))
#         ratio = current / float(total)
#         percent = ratio * 100.0
#         overall_progress = int(round(42.0 + 25.0 * ratio))

#         status.publish(
#             phase="GRINDING",
#             screen=4,
#             progress=overall_progress,
#             title="원두를 갈고 있습니다",
#             message=(
#                 f"현재 {current:.2f} / {total}회전 "
#                 f"({percent:.0f}%) 진행했습니다."
#             ),
#             busy=True,
#             grind_current_turns=current,
#         )

#     def grind_with_continuous_progress(
#         grind_via,
#         grind_end,
#         grind_turns: int,
#     ) -> None:
#         """단일 amovec를 유지하며 현재 TCP 각도로 실제 회전량을 계산한다."""
#         total_angle = DEGREES_PER_GRINDER_TURN * float(grind_turns)
#         start_pose = _pose6_from_dsr(
#             get_current_posx(ref=GRINDER_USER_COORD)
#         )
#         start_angle = math.atan2(start_pose[1], start_pose[0])

#         # 현재점 -> 경유점 방향으로 원호 진행 방향을 결정한다.
#         via_angle = math.atan2(-124.5, 0.0)
#         direction_delta = _wrap_radians(via_angle - start_angle)
#         direction = -1.0 if direction_delta < 0.0 else 1.0

#         publish_grinding_progress(0.0, grind_turns)
#         amovec(
#             grind_via,
#             grind_end,
#             ref=GRINDER_USER_COORD,
#             angle=[total_angle, 0.0],
#             ra=DR_MV_RA_OVERRIDE,
#         )

#         last_angle = start_angle
#         accumulated_angle = 0.0
#         motion_seen = False
#         motion_start_deadline = (
#             time.monotonic() + GRINDER_MOTION_START_TIMEOUT_SEC
#         )
#         next_update = 0.0
#         monitor_warning_logged = False

#         while rclpy.ok():
#             now = time.monotonic()

#             try:
#                 motion_state = int(check_motion())
#             except Exception as error:
#                 if not monitor_warning_logged:
#                     node.get_logger().warning(
#                         "check_motion() 실패로 회전 중 실시간 측정을 중단하고 "
#                         f"모션 종료만 기다립니다: {error}"
#                     )
#                     monitor_warning_logged = True
#                 if callable(mwait):
#                     mwait(0)
#                 break

#             if motion_state != MOTION_IDLE:
#                 motion_seen = True

#             try:
#                 pose = _pose6_from_dsr(
#                     get_current_posx(ref=GRINDER_USER_COORD)
#                 )
#                 current_angle = math.atan2(pose[1], pose[0])
#                 angular_step = _wrap_radians(current_angle - last_angle)
#                 directed_step = direction * angular_step

#                 # 반대 방향의 미세 진동은 진행량에서 제외한다.
#                 if directed_step > 0.0:
#                     accumulated_angle += directed_step

#                 last_angle = current_angle
#                 current_turns = min(
#                     accumulated_angle / (2.0 * math.pi),
#                     float(grind_turns),
#                 )

#                 if now >= next_update:
#                     publish_grinding_progress(current_turns, grind_turns)
#                     next_update = now + GRINDER_PROGRESS_UPDATE_SEC
#             except Exception as error:
#                 if not monitor_warning_logged:
#                     node.get_logger().warning(
#                         "그라인더 현재 pose를 읽지 못해 마지막 진행률을 유지합니다: "
#                         f"{error}"
#                     )
#                     monitor_warning_logged = True

#             if motion_seen and motion_state == MOTION_IDLE:
#                 break

#             if not motion_seen and now >= motion_start_deadline:
#                 node.get_logger().warning(
#                     "amovec 시작 상태를 확인하지 못했습니다. 모션 종료를 기다립니다."
#                 )
#                 if callable(mwait):
#                     mwait(0)
#                 break

#             time.sleep(0.03)

#         if callable(mwait):
#             mwait(0)

#         publish_grinding_progress(float(grind_turns), grind_turns)

#     def grind_with_turn_steps(
#         grind_via,
#         grind_end,
#         grind_turns: int,
#     ) -> None:
#         """비동기 API 미지원 시 1회전 movec 단위로 진행률을 발행한다."""
#         publish_grinding_progress(0.0, grind_turns)

#         for completed_turns in range(1, grind_turns + 1):
#             movec(
#                 grind_via,
#                 grind_end,
#                 radius=0.0,
#                 ref=GRINDER_USER_COORD,
#                 angle=[DEGREES_PER_GRINDER_TURN, 0.0],
#                 ra=DR_MV_RA_OVERRIDE,
#             )
#             publish_grinding_progress(float(completed_turns), grind_turns)

#     def grinder(grind_turns: int) -> None:
#         """선택된 회전 수만큼 그라인더 손잡이를 돌려 원두를 분쇄."""
#         if grind_turns <= 0:
#             raise ValueError(f"그라인더 회전 수는 1 이상이어야 합니다: {grind_turns}")

#         # status.step("grinder", 0)  # UI 상태 발행용 — 비활성화
#         movej(System_handle_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         jar_grip_open()
#         movel(
#             System_handle_l, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )
#         grip_close()

#         # status.step("grinder", 1)  # UI 상태 발행용 — 비활성화
#         movej(
#             posj(13.43, 31.67, 32.98, 0.19, 115.35, 11.45),
#             radius=0.0, ra=DR_MV_RA_DUPLICATE,
#         )

#         grind_via = posx(
#             0.0, -124.5, -5.0, 90.0, -179.99, 88.27
#         )
#         grind_end = posx(
#             -124.5, 0.0, -5.0, 90.0, -179.99, 88.27
#         )

#         # status.step("grinder", 2)  # UI 상태 발행용 — 비활성화
#         task_compliance_ctrl()
#         set_stiffnessx(
#             [1500.0, 1500.0, 2500.0, 150.0, 150.0, 200.0],
#             time=0.5,
#         )

#         try:
#             # amovec + check_motion이 있으면 기존처럼 한 번의 연속 원호 모션을
#             # 유지하면서 좌표계 101의 현재 X/Y 각도로 실제 회전량을 측정한다.
#             if callable(amovec) and callable(check_motion):
#                 grind_with_continuous_progress(
#                     grind_via, grind_end, grind_turns
#                 )
#             else:
#                 node.get_logger().warning(
#                     "amovec/check_motion 미지원: 1회전 movec 단위 진행률로 대체합니다."
#                 )
#                 grind_with_turn_steps(grind_via, grind_end, grind_turns)
#         finally:
#             # status.step("grinder", 4)  # UI 상태 발행용 — 비활성화
#             release_compliance_ctrl()
#             jar_grip_open()

#     def dripper_in() -> None:
#         """드립 병을 잡고 주기 운동으로 커피를 추출."""
#         # status.step("dripper_in", 0)  # UI 상태 발행용 — 비활성화
#         movej(System_bottle_j_1, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movel(
#             System_bottle_l, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )
#         grip_close()
#         movel(
#             posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )

#         # status.step("dripper_in", 1)  # UI 상태 발행용 — 비활성화
#         movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movel(
#             posx(0.0, 0.0, 300.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )
#         movej(System_drip_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movel(
#             System_drip_l, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )
#         # 반영될 때까지 재시도 (apply_tcp 참고). 끝내 실패해도 move_periodic은
#         # 진행한다 — 병을 잡고 있는 상태라 여기서 멈추는 게 더 위험할 수 있다.
#         apply_tcp("joint4")

#         # status.step("dripper_in", 2)  # UI 상태 발행용 — 비활성화
#         move_periodic(
#             amp=[0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
#             period=[1.0, 1.0, 1.0, 0.3, 1.0, 1.0],
#             atime=0.0,
#             repeat=5,
#             ref=DR_TOOL,
#         )

#         status.publish(
#             phase="WAIT_EXTERNAL_FORCE",
#             screen=5,
#             progress=70,
#             title="병 바닥을 가볍게 쳐주세요",
#             message=(
#                 f"기준 힘 대비 {EXTERNAL_FORCE_TRIGGER_N:.1f} N 이상의 외력이 "
#                 "감지되면 커피가 털린 것으로 확인하고 자동으로 다음 단계로 진행합니다."
#             ),
#             busy=True,
#             waiting_external_force=True,
#             force_delta_n=0.0,
#             force_peak_n=0.0,
#         )
#         detected_force_n = wait_for_external_force(EXTERNAL_FORCE_TRIGGER_N)
#         status.publish(
#             phase="FILTER_FINISHING",
#             screen=5,
#             progress=74,
#             title="커피 필터 투입을 마무리하는 중",
#             message=(
#                 f"{detected_force_n:.1f} N의 외력을 감지했습니다. "
#                 "병을 제자리로 옮깁니다."
#             ),
#             busy=True,
#             force_delta_n=detected_force_n,
#             force_peak_n=detected_force_n,
#         )

#         # status.step("dripper_in", 3)  # UI 상태 발행용 — 비활성화
#         apply_tcp(TCP_NAME)
#         movel(
#             posx(0.0, 0.0, -150.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_TOOL,
#             mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
#         )
#         movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
#         movel(
#             System_bottle_l2, radius=0.0, ref=DR_BASE,
#             mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
#         )

#         # status.step("dripper_in", 4)  # UI 상태 발행용 — 비활성화
#         jar_grip_open()

#     def spiral_pour() -> None:
#         """주전자 파지부터 보상 스파이럴, 중심 회전, 반환까지 수행하는 Sub."""

#         def publish_spiral(
#             *,
#             phase: str,
#             progress: int,
#             spiral_progress: float,
#             stage: str,
#             title: str,
#             message: str,
#         ) -> None:
#             status.publish(
#                 phase=phase,
#                 screen=6,
#                 progress=progress,
#                 title=title,
#                 message=message,
#                 busy=True,
#                 spiral_stage=stage,
#                 spiral_progress=spiral_progress,
#             )

#         def current_pose_base() -> np.ndarray:
#             try:
#                 value = get_current_posx(ref=DR_BASE)
#             except TypeError:
#                 value = get_current_posx(DR_BASE)
#             return _numeric6(value, label="현재 TCP Base pose")

#         def current_joint() -> np.ndarray:
#             return _numeric6(
#                 get_current_posj(),
#                 label="현재 joint pose",
#             )

#         def forward_kinematics_base(joint_deg) -> np.ndarray:
#             joint_pose = posj(
#                 *[float(value) for value in joint_deg[:6]]
#             )
#             try:
#                 value = fkin(joint_pose, ref=DR_BASE)
#             except TypeError:
#                 value = fkin(joint_pose, DR_BASE)
#             return _numeric6(value, label="fkin 결과")

#         def rotation_distance_deg(first_abc, second_abc) -> float:
#             first_rotation = _zyz_to_rotm(first_abc)
#             second_rotation = _zyz_to_rotm(second_abc)
#             relative = _rotm_to_rotvec(
#                 first_rotation.T @ second_rotation
#             )
#             return math.degrees(float(np.linalg.norm(relative)))

#         def validate_path(
#             start_pose: np.ndarray,
#             numeric_path: list[np.ndarray],
#             *,
#             label: str,
#         ) -> None:
#             if not numeric_path:
#                 raise RuntimeError(f"{label} 경로가 비어 있습니다.")

#             previous = start_pose
#             max_linear_step = 0.0
#             max_angular_step = 0.0

#             for index, point in enumerate(numeric_path, start=1):
#                 if point.shape != (6,) or not np.all(np.isfinite(point)):
#                     raise RuntimeError(
#                         f"{label} 경유점 {index}가 올바르지 않습니다: "
#                         f"{point!r}"
#                     )

#                 linear_step = float(
#                     np.linalg.norm(point[:3] - previous[:3])
#                 )
#                 angular_step = rotation_distance_deg(
#                     previous[3:],
#                     point[3:],
#                 )
#                 max_linear_step = max(max_linear_step, linear_step)
#                 max_angular_step = max(
#                     max_angular_step,
#                     angular_step,
#                 )
#                 previous = point

#             node.get_logger().info(
#                 f"{label} 경로 검증: points={len(numeric_path)}, "
#                 f"max linear step={max_linear_step:.3f} mm, "
#                 f"max angular step={max_angular_step:.3f} deg"
#             )

#             if max_linear_step > MOVESX_MAX_LINEAR_STEP_MM:
#                 raise RuntimeError(
#                     f"{label} 경유점 병진 간격이 너무 큽니다: "
#                     f"{max_linear_step:.3f} mm"
#                 )
#             if max_angular_step > MOVESX_MAX_ANGULAR_STEP_DEG:
#                 raise RuntimeError(
#                     f"{label} 경유점 회전 간격이 너무 큽니다: "
#                     f"{max_angular_step:.3f} deg"
#                 )

#         def call_movesx(
#             path,
#             *,
#             duration_sec: float,
#         ):
#             kwargs = {
#                 "ref": DR_BASE,
#                 "mod": DR_MV_MOD_ABS,
#                 "vel_opt": DR_MVS_VEL_NONE,
#             }
#             attempts = (
#                 lambda: movesx(
#                     path,
#                     time=duration_sec,
#                     **kwargs,
#                 ),
#                 lambda: movesx(
#                     path,
#                     t=duration_sec,
#                     **kwargs,
#                 ),
#                 lambda: movesx(
#                     path,
#                     vel=[
#                         MOVESX_LINEAR_VEL_MM_S,
#                         MOVESX_ANGULAR_VEL_DEG_S,
#                     ],
#                     acc=[
#                         MOVESX_LINEAR_ACC_MM_S2,
#                         MOVESX_ANGULAR_ACC_DEG_S2,
#                     ],
#                     time=duration_sec,
#                     **kwargs,
#                 ),
#                 lambda: movesx(
#                     path,
#                     [
#                         MOVESX_LINEAR_VEL_MM_S,
#                         MOVESX_ANGULAR_VEL_DEG_S,
#                     ],
#                     [
#                         MOVESX_LINEAR_ACC_MM_S2,
#                         MOVESX_ANGULAR_ACC_DEG_S2,
#                     ],
#                     duration_sec,
#                     DR_BASE,
#                     DR_MV_MOD_ABS,
#                     DR_MVS_VEL_NONE,
#                 ),
#             )

#             last_type_error = None
#             for attempt in attempts:
#                 try:
#                     return attempt()
#                 except TypeError as error:
#                     last_type_error = error

#             raise RuntimeError(
#                 "현재 DSR_ROBOT2 movesx() 시그니처와 맞지 않습니다: "
#                 f"{last_type_error}"
#             )

#         # -------------------------------------------------------------
#         # 1. 주전자 접근 및 파지
#         # -------------------------------------------------------------
#         publish_spiral(
#             phase="SPIRAL_PICKUP",
#             progress=76,
#             spiral_progress=5,
#             stage="주전자 파지",
#             title="드립 주전자를 집는 중",
#             message="주전자를 파지한 뒤 스파이럴 시작 자세로 이동합니다.",
#         )
#         apply_tcp(TCP_NAME)

#         movej(
#             System_bottle_j_2,
#             radius=0.0,
#             ra=DR_MV_RA_DUPLICATE,
#         )
#         movel(
#             Pickup_approach_rel,
#             radius=0.0,
#             ref=DR_BASE,
#             mod=DR_MV_MOD_REL,
#             ra=DR_MV_RA_DUPLICATE,
#         )
#         movej(
#             System_pot_grip_joint,
#             radius=0.0,
#             ra=DR_MV_RA_DUPLICATE,
#         )

#         handle_grip_open()
#         time.sleep(0.30)

#         movel(
#             System_pot_grip,
#             radius=0.0,
#             ref=DR_BASE,
#             mod=DR_MV_MOD_ABS,
#             ra=DR_MV_RA_DUPLICATE,
#         )
#         grip_close()
#         time.sleep(0.50)

#         movel(
#             Pickup_lift_rel,
#             radius=0.0,
#             ref=DR_BASE,
#             mod=DR_MV_MOD_REL,
#             ra=DR_MV_RA_DUPLICATE,
#         )
#         movej(
#             Pour_start_joint,
#             radius=0.0,
#             ra=DR_MV_RA_DUPLICATE,
#         )

#         # -------------------------------------------------------------
#         # 2. pot TCP 기준 내향 스파이럴 경로 생성
#         # -------------------------------------------------------------
#         publish_spiral(
#             phase="SPIRAL_PREPARING",
#             progress=81,
#             spiral_progress=20,
#             stage="경로 준비",
#             title="스파이럴 드립 경로를 준비하는 중",
#             message=(
#                 f"반경 {SPIRAL_RADIUS_MM:.0f} mm, "
#                 f"{SPIRAL_REVOLUTIONS:.0f}회전 경로를 계산합니다."
#             ),
#         )
#         if not apply_tcp(SPOUT_TCP_NAME):
#             raise RuntimeError(
#                 f"주둥이 TCP '{SPOUT_TCP_NAME}' 적용 실패"
#             )

#         start_pose = current_pose_base()
#         start_joint = current_joint()

#         final_joint = start_joint.copy()
#         final_joint[5] += SPIRAL_J6_DELTA_DEG
#         final_virtual_pose = forward_kinematics_base(final_joint)

#         virtual_drift = final_virtual_pose[:3] - start_pose[:3]
#         required_compensation = -virtual_drift

#         center_x = (
#             start_pose[0]
#             + SPIRAL_CENTER_OFFSET_SIGN_X * SPIRAL_RADIUS_MM
#         )
#         center_y = start_pose[1]
#         hold_z = start_pose[2]

#         start_rotation = _zyz_to_rotm(start_pose[3:])
#         final_rotation = _zyz_to_rotm(final_virtual_pose[3:])
#         relative_rotvec = _rotm_to_rotvec(
#             start_rotation.T @ final_rotation
#         )

#         node.get_logger().info(
#             "Joint6 단독 가상 회전 시 주둥이 Base 변위 [mm]: "
#             f"dX={virtual_drift[0]:+.3f}, "
#             f"dY={virtual_drift[1]:+.3f}, "
#             f"dZ={virtual_drift[2]:+.3f}"
#         )
#         node.get_logger().info(
#             "로봇 본체의 반대 보상 [mm]: "
#             f"X={required_compensation[0]:+.3f}, "
#             f"Y={required_compensation[1]:+.3f}, "
#             f"Z={required_compensation[2]:+.3f}"
#         )

#         dense_count = max(4000, SPIRAL_PATH_POINTS * 80)
#         dense_s = np.linspace(
#             0.0,
#             1.0,
#             dense_count + 1,
#             dtype=float,
#         )
#         dense_radius = SPIRAL_RADIUS_MM * (1.0 - dense_s)
#         dense_theta = (
#             SPIRAL_ROTATION_SIGN
#             * 2.0
#             * math.pi
#             * SPIRAL_REVOLUTIONS
#             * dense_s
#         )
#         dense_x = center_x + dense_radius * np.cos(dense_theta)
#         dense_y = center_y + dense_radius * np.sin(dense_theta)
#         dense_xyz = np.column_stack(
#             (
#                 dense_x,
#                 dense_y,
#                 np.full_like(dense_x, hold_z),
#             )
#         )

#         segment_length = np.linalg.norm(
#             np.diff(dense_xyz, axis=0),
#             axis=1,
#         )
#         cumulative_length = np.concatenate(
#             (
#                 np.array([0.0], dtype=float),
#                 np.cumsum(segment_length),
#             )
#         )
#         total_path_length = float(cumulative_length[-1])
#         if (
#             not math.isfinite(total_path_length)
#             or total_path_length <= 0.0
#         ):
#             raise RuntimeError(
#                 f"스파이럴 호길이 계산 실패: {total_path_length}"
#             )

#         target_lengths = np.linspace(
#             total_path_length / float(SPIRAL_PATH_POINTS),
#             total_path_length,
#             SPIRAL_PATH_POINTS,
#             dtype=float,
#         )
#         spiral_s_samples = np.interp(
#             target_lengths,
#             cumulative_length,
#             dense_s,
#         )

#         numeric_path: list[np.ndarray] = []
#         path = []
#         previous_abc = start_pose[3:].copy()

#         for index, spiral_s in enumerate(
#             spiral_s_samples,
#             start=1,
#         ):
#             radius = SPIRAL_RADIUS_MM * (
#                 1.0 - float(spiral_s)
#             )
#             theta = (
#                 SPIRAL_ROTATION_SIGN
#                 * 2.0
#                 * math.pi
#                 * SPIRAL_REVOLUTIONS
#                 * float(spiral_s)
#             )

#             target_x = center_x + radius * math.cos(theta)
#             target_y = center_y + radius * math.sin(theta)
#             target_z = hold_z

#             orientation_progress = _smoothstep5(
#                 index / float(SPIRAL_PATH_POINTS)
#             )
#             target_rotation = start_rotation @ _rotvec_to_rotm(
#                 relative_rotvec * orientation_progress
#             )
#             target_abc = _rotm_to_zyz_near(
#                 target_rotation,
#                 previous_abc,
#             )
#             previous_abc = target_abc

#             target = np.array(
#                 [
#                     target_x,
#                     target_y,
#                     target_z,
#                     target_abc[0],
#                     target_abc[1],
#                     target_abc[2],
#                 ],
#                 dtype=float,
#             )
#             numeric_path.append(target)
#             path.append(posx(*[float(value) for value in target]))

#         validate_path(
#             start_pose,
#             numeric_path,
#             label="스파이럴",
#         )

#         # -------------------------------------------------------------
#         # 3. 내향 스파이럴 + Joint6 등가 기울임
#         # -------------------------------------------------------------
#         publish_spiral(
#             phase="SPIRAL_POURING",
#             progress=84,
#             spiral_progress=35,
#             stage="스파이럴 드립",
#             title="스파이럴 방식으로 물을 붓는 중",
#             message=(
#                 f"{SPIRAL_DURATION_SEC:.0f}초 동안 "
#                 f"{SPIRAL_REVOLUTIONS:.0f}회 내향 스파이럴을 수행합니다."
#             ),
#         )
#         node.get_logger().info(
#             "통합 워크플로우 스파이럴 Sub 시작: "
#             f"radius={SPIRAL_RADIUS_MM:.1f} mm, "
#             f"rev={SPIRAL_REVOLUTIONS:.1f}, "
#             f"duration={SPIRAL_DURATION_SEC:.1f} s, "
#             f"J6 equivalent={SPIRAL_J6_DELTA_DEG:.1f} deg"
#         )

#         result = call_movesx(
#             path,
#             duration_sec=SPIRAL_DURATION_SEC,
#         )
#         if isinstance(result, (int, float)) and result < 0:
#             raise RuntimeError(
#                 f"스파이럴 movesx 실패 반환값={result}"
#             )

#         end_pose = current_pose_base()
#         target_end = numeric_path[-1]
#         end_error = target_end[:3] - end_pose[:3]
#         node.get_logger().info(
#             "스파이럴 완료 주둥이 Base 오차 [mm]: "
#             f"X={end_error[0]:+.3f}, "
#             f"Y={end_error[1]:+.3f}, "
#             f"Z={end_error[2]:+.3f}"
#         )

#         # -------------------------------------------------------------
#         # 4. 스파이럴 중심에서 주둥이 XYZ 고정, 주전자 자세만 원복
#         # -------------------------------------------------------------
#         publish_spiral(
#             phase="SPIRAL_CENTER_RETURN",
#             progress=92,
#             spiral_progress=75,
#             stage="중심 고정 자세 복원",
#             title="주둥이를 고정한 채 주전자를 세우는 중",
#             message="스파이럴 중심에서 주둥이 위치를 유지하며 주전자 자세를 복원합니다.",
#         )

#         fixed_center_xyz = end_pose[:3].copy()
#         current_rotation = _zyz_to_rotm(end_pose[3:])
#         target_rotation = _zyz_to_rotm(start_pose[3:])
#         return_rotvec = _rotm_to_rotvec(
#             current_rotation.T @ target_rotation
#         )
#         total_return_deg = math.degrees(
#             float(np.linalg.norm(return_rotvec))
#         )

#         if total_return_deg >= 0.05:
#             return_numeric_path: list[np.ndarray] = []
#             return_path = []
#             previous_abc = end_pose[3:].copy()

#             for index in range(
#                 1,
#                 CENTER_PIVOT_RETURN_POINTS + 1,
#             ):
#                 progress_value = _smoothstep5(
#                     index / float(CENTER_PIVOT_RETURN_POINTS)
#                 )
#                 interpolated_rotation = (
#                     current_rotation
#                     @ _rotvec_to_rotm(
#                         return_rotvec * progress_value
#                     )
#                 )
#                 target_abc = _rotm_to_zyz_near(
#                     interpolated_rotation,
#                     previous_abc,
#                 )
#                 previous_abc = target_abc

#                 target = np.array(
#                     [
#                         fixed_center_xyz[0],
#                         fixed_center_xyz[1],
#                         fixed_center_xyz[2],
#                         target_abc[0],
#                         target_abc[1],
#                         target_abc[2],
#                     ],
#                     dtype=float,
#                 )
#                 return_numeric_path.append(target)
#                 return_path.append(
#                     posx(*[float(value) for value in target])
#                 )

#             validate_path(
#                 end_pose,
#                 return_numeric_path,
#                 label="중심 고정 자세복원",
#             )
#             result = call_movesx(
#                 return_path,
#                 duration_sec=CENTER_PIVOT_RETURN_DURATION_SEC,
#             )
#             if (
#                 isinstance(result, (int, float))
#                 and result < 0
#             ):
#                 raise RuntimeError(
#                     "중심 고정 자세복원 movesx 실패 "
#                     f"반환값={result}"
#                 )

#             pivot_end_pose = current_pose_base()
#             pivot_drift = (
#                 pivot_end_pose[:3] - fixed_center_xyz
#             )
#             node.get_logger().info(
#                 "중심 고정 자세복원 결과 [mm]: "
#                 f"dX={pivot_drift[0]:+.3f}, "
#                 f"dY={pivot_drift[1]:+.3f}, "
#                 f"dZ={pivot_drift[2]:+.3f}"
#             )

#         # -------------------------------------------------------------
#         # 5. 원래 파지 위치 + Base Z 10 mm로 이동 후 놓기
#         # -------------------------------------------------------------
#         publish_spiral(
#             phase="SPIRAL_RETURNING",
#             progress=96,
#             spiral_progress=90,
#             stage="주전자 반환",
#             title="주전자를 원래 위치로 옮기는 중",
#             message="원래 파지 위치의 Base +Z 10 mm 지점으로 이동합니다.",
#         )

#         if not apply_tcp(TCP_NAME):
#             raise RuntimeError(
#                 f"기본 TCP '{TCP_NAME}' 복원 실패"
#             )

#         movej(
#             System_pot_grip_joint,
#             radius=0.0,
#             ra=DR_MV_RA_DUPLICATE,
#         )

#         release_values = [
#             float(value) for value in SYSTEM_POT_GRIP_VALUES
#         ]
#         release_values[2] += POT_RELEASE_BASE_Z_OFFSET_MM
#         release_target = posx(*release_values)

#         movel(
#             release_target,
#             radius=0.0,
#             ref=DR_BASE,
#             mod=DR_MV_MOD_ABS,
#             ra=DR_MV_RA_DUPLICATE,
#         )

#         publish_spiral(
#             phase="SPIRAL_RELEASING",
#             progress=99,
#             spiral_progress=98,
#             stage="주전자 놓기",
#             title="주전자를 내려놓는 중",
#             message="그리퍼를 열어 주전자를 배치합니다.",
#         )
#         handle_grip_open()
#         time.sleep(POT_RELEASE_SETTLE_SEC)

#         publish_spiral(
#             phase="SPIRAL_DONE",
#             progress=99,
#             spiral_progress=100,
#             stage="완료",
#             title="스파이럴 드립 완료",
#             message="주전자를 원래 위치에 놓았습니다.",
#         )
#         node.get_logger().info(
#             "통합 워크플로우 스파이럴 Sub 완료"
#         )

#     # =====================================================================
#     # 전역 모션 파라미터 (원본 DRL 하단 설정과 동일)
#     # =====================================================================
#     set_singularity_handling(DR_AVOID)
#     set_velj(VELJ_DEFAULT)
#     set_accj(ACCJ_DEFAULT)
#     # set_velx()는 실제 시그니처가 (vel1, vel2) 2개 인자까지만 받는데 원본은
#     # 3번째 인자로 DR_OFF를 더 넘기고 있었음. DR_OFF는 DSR_ROBOT2에 존재하지도
#     # 않는 상수라 AttributeError, 있었다 해도 TypeError가 났을 코드라 제거함.
#     set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
#     set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)

#     status.publish(
#         phase="READY_SELECT",
#         screen=1,
#         progress=0,
#         title="물리 버튼으로 원두를 선택해 주세요",
#         message=(
#             "DI 13 에티오피아 · DI 14 콜롬비아 · "
#             "DI 15 브라질 · DI 16 과테말라"
#         ),
#         busy=False,
#         waiting_physical_button=True,
#     )
#     selected_button = buttons.wait_for_button()
#     selected_bean = BEAN_BY_BUTTON[selected_button]["name"]
#     status.set_selection(selected_button, selected_bean)
#     status.publish(
#         phase="BEAN_SELECTED",
#         screen=1,
#         progress=3,
#         title=f"{selected_bean} 선택",
#         message=(
#             f"물리 버튼 {selected_button} 입력을 확인했습니다. "
#             "커피 시스템을 시작합니다."
#         ),
#         busy=True,
#         button=selected_button,
#     )

#     # =====================================================================
#     # 메인 시퀀스 실행 (원본 while gLoop < 1: 1회 실행과 동일)
#     # =====================================================================
#     # current_stage = "bean_drop"  # UI 상태 발행(status.error)용 — 비활성화
#     try:
#         # 나머지 동작 전에 Tool/TCP부터 티치펜던트에 등록된 이름으로 맞춘다
#         # (gear.py/move.py와 동일한 방식). 요청한 이름으로 실제 활성화됐는지
#         # get_tool()/get_tcp()로 확인하고, 다르면 여기서 바로 멈춘다.
#         tool_ret = set_tool(TOOL_NAME)
#         tcp_ret = set_tcp(TCP_NAME)
#         active_tool = get_tool()
#         active_tcp = get_tcp()
#         node.get_logger().info(
#             f"set_tool ret={tool_ret}, 요청={TOOL_NAME}, 실제 활성={active_tool}"
#         )
#         node.get_logger().info(
#             f"set_tcp ret={tcp_ret}, 요청={TCP_NAME}, 실제 활성={active_tcp}"
#         )
#         if active_tool != TOOL_NAME or active_tcp != TCP_NAME:
#             raise RuntimeError(
#                 "Tool/TCP가 요청한 이름으로 활성화되지 않았습니다. "
#                 "티치펜던트에 해당 이름이 등록되어 있는지 확인하세요."
#             )

#         node.get_logger().info("커피 시스템 시작: bean_drop -> grinder -> dripper_in -> spiral_pour")

#         node.get_logger().info("[1/4] bean_drop: 원두 투입 중...")
#         status.publish(
#             phase="BEAN_LOADING",
#             screen=2,
#             progress=10,
#             title="원두를 그라인더에 넣는 중",
#             message="스푼을 집어 원두를 그라인더 투입구로 옮기고 있습니다.",
#             busy=True,
#         )
#         bean_drop()
#         # status.stage_done("bean_drop")  # UI 상태 발행용 — 비활성화

#         # current_stage = "grinder"  # UI 상태 발행(status.error)용 — 비활성화
#         node.get_logger().info("[2/4] grinder: 분쇄 굵기 선택 대기...")
#         status.publish(
#             phase="GRIND_SELECT",
#             screen=3,
#             progress=34,
#             title="원하는 분쇄 굵기를 선택해 주세요",
#             message=(
#                 "DI 13 굵게(3회전) · DI 14 중간 굵게(5회전) · "
#                 "DI 15 중간 곱게(7회전) · DI 16 곱게(10회전)"
#             ),
#             busy=False,
#             waiting_physical_button=True,
#         )
#         grind_button = buttons.wait_for_button()
#         grind_option = GRIND_BY_BUTTON[grind_button]
#         grind_name = grind_option["name"]
#         grind_turns = int(grind_option["turns"])
#         status.set_grind_selection(grind_button, grind_name, grind_turns)
#         status.publish(
#             phase="GRIND_SELECTED",
#             screen=3,
#             progress=37,
#             title=f"{grind_name}를 선택했습니다",
#             message=(
#                 f"물리 버튼 {grind_button} 입력을 확인했습니다. "
#                 f"그라인더를 {grind_turns}회전합니다."
#             ),
#             busy=True,
#             button=grind_button,
#         )

#         node.get_logger().info(
#             f"[2/4] grinder: {grind_name}, {grind_turns}회전 분쇄 중..."
#         )
#         status.publish(
#             phase="GRINDER_MOVE",
#             screen=4,
#             progress=42,
#             title="그라인더로 이동하는 중",
#             message=(
#                 f"{grind_name} 설정으로 원두를 분쇄합니다. "
#                 f"그라인더 회전 수는 {grind_turns}회입니다."
#             ),
#             busy=True,
#             grind_current_turns=0.0,
#         )
#         grinder(grind_turns)
#         # status.stage_done("grinder")  # UI 상태 발행용 — 비활성화

#         # current_stage = "dripper_in"  # UI 상태 발행(status.error)용 — 비활성화
#         node.get_logger().info("[3/4] dripper_in: 드립 추출 중...")
#         status.publish(
#             phase="FILTER_LOADING",
#             screen=5,
#             progress=68,
#             title="갈린 원두를 커피 필터에 넣는 중",
#             message="분쇄 원두가 담긴 병을 집어 필터 위로 이동하고 있습니다.",
#             busy=True,
#         )
#         dripper_in()
#         # status.stage_done("dripper_in")  # UI 상태 발행용 — 비활성화

#         node.get_logger().info(
#             "[4/4] spiral_pour: 보상 내향 스파이럴 드립 중..."
#         )
#         status.publish(
#             phase="SPIRAL_START",
#             screen=6,
#             progress=75,
#             title="스파이럴 드립을 시작합니다",
#             message="주전자를 집어 드리퍼 위에서 내향 스파이럴을 수행합니다.",
#             busy=True,
#             spiral_stage="시작",
#             spiral_progress=0.0,
#         )
#         spiral_pour()

#         node.get_logger().info("커피 추출 전체 시퀀스 완료.")
#         status.publish(
#             phase="COMPLETE",
#             screen=7,
#             progress=100,
#             title="커피 추출이 완료되었습니다",
#             message="원두 투입, 분쇄, 필터 투입, 스파이럴 드립 작업이 모두 완료되었습니다.",
#             busy=False,
#             spiral_stage="완료",
#             spiral_progress=100.0,
#         )
#         # status.finished()  # UI 상태 발행용 — 비활성화

#     except Exception as error:
#         node.get_logger().error(f"실행 중 오류 발생: {error}")
#         status.publish(
#             phase="ERROR",
#             screen=8,
#             progress=0,
#             title="로봇 작업 오류",
#             message=str(error),
#             busy=False,
#             error=f"{type(error).__name__}: {error}",
#         )
#         # status.error(current_stage, str(error))  # UI 상태 발행용 — 비활성화
#         raise

#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == "__main__":
#     main()


"""
m0609_coffee_system.py

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
7. final_drip      : 필터 홀더와 물컵을 배치한 뒤 주둥이 위치 보상 제어로 물 붓기

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

import numpy as np
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


# =============================================================================
# 스파이럴 드립 Sub 설정
# =============================================================================

SPOUT_TCP_NAME = "pot"

SPIRAL_RADIUS_MM = 44.0
SPIRAL_REVOLUTIONS = 5.0
SPIRAL_DURATION_SEC = 15.0
SPIRAL_J6_DELTA_DEG = -60.0
SPIRAL_PATH_POINTS = 100

CENTER_PIVOT_RETURN_DURATION_SEC = 3.0
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

# plate_outline(2).py의 기본값을 그대로 통합한다.
# System_final_j2 도달 후 현재 자세를 시작점으로 사용한다.
FINAL_POUR_TCP_NAME = "mug"
FINAL_POUR_SPOUT_OFFSET_TOOL_MM = np.array([1.0, 1.0, 1.0], dtype=float)
FINAL_POUR_J6_DELTA_DEG = 45.0
FINAL_POUR_DURATION_SEC = 8.0
FINAL_POUR_PATH_POINTS = 100

FINAL_POUR_MAX_LINEAR_STEP_MM = 5.0
FINAL_POUR_MAX_ANGULAR_STEP_DEG = 3.0
FINAL_POUR_MOVESX_LINEAR_VEL_MM_S = 40.0
FINAL_POUR_MOVESX_ANGULAR_VEL_DEG_S = 15.0
FINAL_POUR_MOVESX_LINEAR_ACC_MM_S2 = 120.0
FINAL_POUR_MOVESX_ANGULAR_ACC_DEG_S2 = 40.0

# m0609_final_drip(1).drl 하단 모션 설정. 마지막 Sub 진입 시에만 적용한다.
FINAL_DRIP_VELJ = 60.0
FINAL_DRIP_ACCJ = 100.0
FINAL_DRIP_VELX_LIN = 250.0
FINAL_DRIP_VELX_ROT = 80.625
FINAL_DRIP_ACCX_LIN = 1000.0
FINAL_DRIP_ACCX_ROT = 322.5

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
        self._spiral_stage = "대기"
        self._spiral_progress = 0.0
        self._final_drip_stage = "대기"
        self._final_drip_progress = 0.0
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
        spiral_stage: Optional[str] = None,
        spiral_progress: Optional[float] = None,
        final_drip_stage: Optional[str] = None,
        final_drip_progress: Optional[float] = None,
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
            "spiral_stage": self._spiral_stage,
            "spiral_progress": round(self._spiral_progress, 1),
            "spiral_radius_mm": SPIRAL_RADIUS_MM,
            "spiral_revolutions": SPIRAL_REVOLUTIONS,
            "spiral_duration_sec": SPIRAL_DURATION_SEC,
            "spiral_j6_delta_deg": SPIRAL_J6_DELTA_DEG,
            "final_drip_stage": self._final_drip_stage,
            "final_drip_progress": round(self._final_drip_progress, 1),
            "final_pour_duration_sec": FINAL_POUR_DURATION_SEC,
            "final_pour_j6_delta_deg": FINAL_POUR_J6_DELTA_DEG,
            "final_pour_tcp_name": FINAL_POUR_TCP_NAME,
            "final_pour_spout_offset_mm": [
                round(float(value), 3)
                for value in FINAL_POUR_SPOUT_OFFSET_TOOL_MM
            ],
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
    System_final_l = posx(699.92, -319.33, 332.35, 26.41, 87.7, 167.11)
    System_final_j2 = posj(-53.67, 53.18, 48.53, 96.63, 74.0, 146.0)
    System_final_approach_j = posj(
        -50.40, 31.33, 84.53, 98.98, 78.15, 40.42
    )

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
                    progress=70,
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
            progress=70,
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
            progress=74,
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
        grip_close()
        time.sleep(0.50)

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
            message="스파이럴 중심에서 주둥이 위치를 유지하며 주전자 자세를 복원합니다.",
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
            result = call_movesx(
                return_path,
                duration_sec=CENTER_PIVOT_RETURN_DURATION_SEC,
            )
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
                f"dZ={pivot_drift[2]:+.3f}"
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

    def final_drip() -> None:
        """DRL final_drip 동작 후 plate_outline 기반 물 붓기를 수행한다.

        순서
        ----
        1. 필터 홀더를 집어 지정 위치로 이동
        2. 물컵을 집어 최종 드립 시작 자세로 이동
        3. ``mug`` TCP를 활성화
        4. 현재 Joint6을 +45 deg 가상 회전한 자세를 기준으로, 주둥이 Base XYZ가
           고정되도록 TCP 병진을 보상한 MoveSX 경로 실행

        plate_outline(2).py의 기본 동작과 동일하게 물 붓기 후 시작 자세로
        자동 복귀하지 않으며, 최종 기울어진 자세가 전체 공정의 마지막 자세이다.
        """

        # final_drip DRL 파일에 기록된 모션 설정은 이 마지막 Sub부터만 적용한다.
        set_velj(FINAL_DRIP_VELJ)
        set_accj(FINAL_DRIP_ACCJ)
        set_velx(FINAL_DRIP_VELX_LIN, FINAL_DRIP_VELX_ROT)
        set_accx(FINAL_DRIP_ACCX_LIN, FINAL_DRIP_ACCX_ROT)

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

        def movej_with_time(target, duration_sec: float) -> Any:
            attempts = (
                lambda: movej(
                    target,
                    time=duration_sec,
                    radius=0.0,
                    ra=DR_MV_RA_DUPLICATE,
                ),
                lambda: movej(
                    target,
                    t=duration_sec,
                    radius=0.0,
                    ra=DR_MV_RA_DUPLICATE,
                ),
            )
            last_type_error: Optional[TypeError] = None
            for attempt in attempts:
                try:
                    return attempt()
                except TypeError as error:
                    last_type_error = error
            raise RuntimeError(
                "현재 DSR_ROBOT2 movej() 시간 인자 시그니처와 맞지 않습니다: "
                f"{last_type_error}"
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

        def spout_base_from_tcp_pose(tcp_pose: np.ndarray) -> np.ndarray:
            rotation = _zyz_to_rotm(tcp_pose[3:])
            return (
                tcp_pose[:3]
                + rotation @ FINAL_POUR_SPOUT_OFFSET_TOOL_MM
            )

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

        def call_final_movesx(path) -> Any:
            kwargs = {
                "ref": DR_BASE,
                "mod": DR_MV_MOD_ABS,
                "vel_opt": DR_MVS_VEL_NONE,
            }
            attempts = (
                lambda: movesx(
                    path,
                    time=FINAL_POUR_DURATION_SEC,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    t=FINAL_POUR_DURATION_SEC,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    vel=[
                        FINAL_POUR_MOVESX_LINEAR_VEL_MM_S,
                        FINAL_POUR_MOVESX_ANGULAR_VEL_DEG_S,
                    ],
                    acc=[
                        FINAL_POUR_MOVESX_LINEAR_ACC_MM_S2,
                        FINAL_POUR_MOVESX_ANGULAR_ACC_DEG_S2,
                    ],
                    time=FINAL_POUR_DURATION_SEC,
                    **kwargs,
                ),
                lambda: movesx(
                    path,
                    [
                        FINAL_POUR_MOVESX_LINEAR_VEL_MM_S,
                        FINAL_POUR_MOVESX_ANGULAR_VEL_DEG_S,
                    ],
                    [
                        FINAL_POUR_MOVESX_LINEAR_ACC_MM_S2,
                        FINAL_POUR_MOVESX_ANGULAR_ACC_DEG_S2,
                    ],
                    FINAL_POUR_DURATION_SEC,
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
                "현재 DSR_ROBOT2 movesx() 시그니처와 맞지 않습니다: "
                f"{last_type_error}"
            )

        def build_fixed_spout_path(
            start_pose: np.ndarray,
            start_joint: np.ndarray,
        ) -> tuple[list[Any], list[np.ndarray], np.ndarray]:
            start_rotation = _zyz_to_rotm(start_pose[3:])
            fixed_spout_base = (
                start_pose[:3]
                + start_rotation @ FINAL_POUR_SPOUT_OFFSET_TOOL_MM
            )

            virtual_final_joint = start_joint.copy()
            virtual_final_joint[5] += FINAL_POUR_J6_DELTA_DEG
            virtual_final_pose = forward_kinematics_base(virtual_final_joint)
            final_rotation = _zyz_to_rotm(virtual_final_pose[3:])
            relative_rotvec = _rotm_to_rotvec(
                start_rotation.T @ final_rotation
            )

            virtual_uncompensated_spout = (
                virtual_final_pose[:3]
                + final_rotation @ FINAL_POUR_SPOUT_OFFSET_TOOL_MM
            )
            uncompensated_drift = (
                virtual_uncompensated_spout - fixed_spout_base
            )
            node.get_logger().info(
                "가상 J6 단독 회전 시 주둥이 변위 [mm]: "
                f"dX={uncompensated_drift[0]:+.3f}, "
                f"dY={uncompensated_drift[1]:+.3f}, "
                f"dZ={uncompensated_drift[2]:+.3f}"
            )

            numeric_path: list[np.ndarray] = []
            path = []
            previous_abc = start_pose[3:].copy()

            for index in range(1, FINAL_POUR_PATH_POINTS + 1):
                progress_value = _smoothstep5(
                    index / float(FINAL_POUR_PATH_POINTS)
                )
                target_rotation = start_rotation @ _rotvec_to_rotm(
                    relative_rotvec * progress_value
                )
                target_abc = _rotm_to_zyz_near(
                    target_rotation,
                    previous_abc,
                )
                previous_abc = target_abc

                target_tcp_xyz = (
                    fixed_spout_base
                    - target_rotation @ FINAL_POUR_SPOUT_OFFSET_TOOL_MM
                )
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

                predicted_spout = (
                    target_tcp_xyz
                    + target_rotation @ FINAL_POUR_SPOUT_OFFSET_TOOL_MM
                )
                residual = predicted_spout - fixed_spout_base
                if float(np.linalg.norm(residual)) > 1.0e-6:
                    raise RuntimeError(
                        f"물 붓기 경유점 {index} 보상 계산 오류: "
                        f"residual={residual.tolist()}"
                    )

                numeric_path.append(target)
                path.append(
                    posx(*[float(value) for value in target])
                )

            validate_path(start_pose, numeric_path)
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
        grip_close()
        wait(0.50)
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
        grip_close()
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
            message="교시된 최종 드립 위치와 Joint6 시작 자세로 이동합니다.",
        )
        movej_with_time(System_final_approach_j, 9.33)
        movel(
            System_final_l,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_final_j2, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # -------------------------------------------------------------
        # 3. DRL의 tp_log("물 붓기") 위치: plate_outline 통합
        # -------------------------------------------------------------
        publish_final(
            phase="FINAL_DRIP_POUR_READY",
            overall_progress=96,
            final_progress=70,
            stage="물 붓기 준비",
            title="물컵 TCP와 보상 경로를 준비하는 중",
            message=(
                f"TCP '{FINAL_POUR_TCP_NAME}'를 적용하고 Joint6 등가 "
                f"{FINAL_POUR_J6_DELTA_DEG:+.0f}° 물 붓기 경로를 계산합니다."
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
        )

        node.get_logger().warning(
            "final_drip 물 붓기 시작: 고정 주둥이 Base XYZ="
            f"[{fixed_spout_base[0]:.3f}, "
            f"{fixed_spout_base[1]:.3f}, "
            f"{fixed_spout_base[2]:.3f}] mm"
        )
        publish_final(
            phase="FINAL_DRIP_POURING",
            overall_progress=98,
            final_progress=78,
            stage="물 붓기",
            title="주둥이 위치를 유지하며 물을 붓는 중",
            message=(
                f"{FINAL_POUR_DURATION_SEC:.0f}초 동안 주둥이 Base XYZ를 "
                f"고정하고 Joint6 등가 {FINAL_POUR_J6_DELTA_DEG:+.0f}°로 기울입니다."
            ),
        )

        start_time = time.monotonic()
        result = call_final_movesx(path)
        elapsed = time.monotonic() - start_time
        if isinstance(result, (int, float)) and result < 0:
            raise RuntimeError(
                f"final_drip 물 붓기 movesx 실패 반환값={result}"
            )

        end_pose = current_pose_base()
        end_joint = current_joint()
        measured_spout_base = spout_base_from_tcp_pose(end_pose)
        spout_error = measured_spout_base - fixed_spout_base
        final_pose_error = numeric_path[-1][:3] - end_pose[:3]
        joint_delta = end_joint - start_joint

        node.get_logger().info(
            "final_drip 물 붓기 완료: "
            f"ret={result}, elapsed={elapsed:.3f}s"
        )
        node.get_logger().info(
            "주둥이 고정 오차 [mm]: "
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
                "주둥이 위치 오차가 3 mm를 초과했습니다. "
                "mug TCP와 FINAL_POUR_SPOUT_OFFSET_TOOL_MM 값을 확인하십시오."
            )

        publish_final(
            phase="FINAL_DRIP_DONE",
            overall_progress=99,
            final_progress=100,
            stage="완료",
            title="최종 물 붓기 완료",
            message="물컵은 최종 기울어진 자세를 유지하며 전체 공정이 종료됩니다.",
        )
        node.get_logger().info("final_drip Sub 완료")

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

        node.get_logger().info("커피 시스템 시작: bean_drop -> grinder -> dripper_in -> spiral_pour -> final_drip")

        node.get_logger().info("[1/5] bean_drop: 원두 투입 중...")
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
        node.get_logger().info("[2/5] grinder: 분쇄 굵기 선택 대기...")
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
            f"[2/5] grinder: {grind_name}, {grind_turns}회전 분쇄 중..."
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
        node.get_logger().info("[3/5] dripper_in: 드립 추출 중...")
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

        node.get_logger().info(
            "[4/5] spiral_pour: 보상 내향 스파이럴 드립 중..."
        )
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
        spiral_pour()

        node.get_logger().info(
            "[5/5] final_drip: 필터 홀더 배치, 물컵 파지, 물 붓기 중..."
        )
        status.publish(
            phase="FINAL_DRIP_START",
            screen=7,
            progress=89,
            title="최종 드립 공정을 시작합니다",
            message="필터 홀더와 물컵을 배치한 뒤 주둥이 위치 보상 제어로 물을 붓습니다.",
            busy=True,
            final_drip_stage="시작",
            final_drip_progress=0.0,
        )
        final_drip()

        node.get_logger().info("커피 추출 전체 시퀀스 완료.")
        status.publish(
            phase="COMPLETE",
            screen=8,
            progress=100,
            title="커피 추출이 완료되었습니다",
            message=(
                "원두 투입, 분쇄, 필터 투입, 스파이럴 드립, "
                "필터 홀더 배치와 최종 물 붓기가 모두 완료되었습니다."
            ),
            busy=False,
            spiral_stage="완료",
            spiral_progress=100.0,
            final_drip_stage="완료",
            final_drip_progress=100.0,
        )
        # status.finished()  # UI 상태 발행용 — 비활성화

    except Exception as error:
        node.get_logger().error(f"실행 중 오류 발생: {error}")
        status.publish(
            phase="ERROR",
            screen=9,
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