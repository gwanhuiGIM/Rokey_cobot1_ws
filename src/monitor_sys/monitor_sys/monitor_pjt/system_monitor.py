# =============================================================
# system_monitor.py — M0609 시스템 모니터 노드 (GUI와는 완전히 다른 프로세스)
# -------------------------------------------------------------
# 실행: ros2 run monitor_sys system_monitor
#       ros2 launch monitor_sys web_admin.launch.py   (웹 관리자 UI와 함께 기동)
#       (자체 점검: python3 -m monitor_pjt.system_monitor --selftest — 로봇 불필요)
# 노드: /system_monitor  (실제로 dsr_msgs2/DRL 토픽·서비스를 만지는 유일한 노드)
#   구독: /dsr01/joint_states(JointState), /dsr01/error(RobotError),
#         /dsr01/robot_disconnection, /dsr01/io/ctrl_box_digital_input_state,
#         /rosout(Log), /coffee_process/state(공정 상태 — process_state.py 규약),
#         ~/cmd(String JSON, GUI가 보내는 제어 명령), OnRobotRGInput(그리퍼, 있을 때만)
#   발행: ~/status (String JSON 스냅샷 — snapshot.Snapshot, 10Hz),
#         ~/log (String, 이벤트 1줄씩)
#   서비스 클라이언트(폴링/제어): system/get_robot_state, get_robot_mode,
#         get_robot_speed_mode, set_robot_mode, set_robot_control,
#         aux_control/get_current_posx, get_current_velx, motion/move_stop, motion/jog
# 의존: rclpy, dsr_msgs2, sensor_msgs, std_msgs, rcl_interfaces
# -------------------------------------------------------------
# 아키텍처: dashboard.py(Qt)는 다른 프로세스다. 서로의 존재를 몰라도 되게
# 토픽 3개(~/status, ~/log, ~/cmd)로만 연결한다 — dashboard.py는 dsr_msgs2를
# import하지 않는다(로봇 SDK 없는 PC에서도 모니터 창을 띄울 수 있다).
#
# 알고리즘 요약
# - 통신 품질: 마지막 수신 후 경과시간으로 OK/STALE(>0.5s)/DISCONNECTED(>2s)
#   3단계 판정 (link_quality). Hz/latency/유실률은 LinkStats가 수신 간격의
#   "표본 중앙값"을 정상 주기로 보고 그보다 큰 gap을 유실로 추정한다 —
#   발행측 nominal Hz를 몰라도 되므로 doosan 버전이 바뀌어도 그대로 쓴다.
# - 안전 판정: 충돌 전용 토픽이 없어 evaluate_safety()가 SAFE_STOP 상태
#   진입 + SAFETY 그룹 에러를 "충돌 의심"으로 묶어 추정한다 (TODO(verify)
#   — 실기에서 실제 충돌 시 에러 코드를 확인해 좁힐 것).
# - 공정 상태머신 감시: /coffee_process/state로 들어오는 보고가
#   process_state.STEPS 순서를 건너뛰면 WARN 로그를 남긴다 (자동화 노드
#   버그로 스텝 보고를 빠뜨렸는지 조기에 드러내기 위함).
# - 제어 명령: ~/cmd로 들어온 JSON({"cmd": "...", ...})을 그대로 대응
#   메서드(estop/stop/start/set_mode/jog/...)에 위임한다. 게이트
#   (control_enabled)는 이 노드 안, 메서드 레벨에서 검사한다 — GUI의
#   체크박스는 표시일 뿐이고, 실제 차단은 항상 서버(이 노드) 쪽이다.
# =============================================================

import json
import math
import time
from collections import deque
from dataclasses import asdict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String, UInt8MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log

from dsr_msgs2.msg import RobotError, RobotDisconnection
from dsr_msgs2.srv import (
    GetRobotState,
    GetRobotMode,
    GetRobotSpeedMode,
    SetRobotMode,
    SetRobotControl,
    GetCurrentPosx,
    GetCurrentVelx,
    MoveStop,
    MoveLine,
    MoveJoint,
    Jog,
)

try:
    from monitor_sys.monitor_pjt import process_state as ps
    from monitor_sys.monitor_pjt.snapshot import Snapshot
except ImportError:  # 소스 파일 직접 실행 호환
    try:
        from monitor_pjt import process_state as ps
        from monitor_pjt.snapshot import Snapshot
    except ImportError:
        import process_state as ps
        from snapshot import Snapshot

# --- 튜닝 상수 (매직넘버 금지: 전부 여기 또는 ROS 파라미터) -------------
DEFAULT_ROBOT_ID = "dsr01"
PUBLISH_HZ = 10.0            # 상태 스냅샷 발행 주기 [Hz]
POLL_HZ = 5.0                # 서비스 폴링 주기 [Hz] (state/mode/velx)
STALE_SEC = 0.5              # 이보다 오래된 데이터 = STALE (CLAUDE.md §4.1)
DISCONNECTED_SEC = 2.0       # 이보다 오래되면 DISCONNECTED
LINK_WINDOW = 100            # 링크 품질(hz/latency/loss) 계산 표본 수
LOG_MAX_LINES = 2000         # 로그 링버퍼 상한 (무한 append 금지)
MAX_JOG_SPEED_PCT = 20.0     # UI에서 낼 수 있는 최대 jog 속도 [%] 하드 리밋
DEFAULT_LINEAR_SPEED = 20.0   # dashboard 1회 상대 moveL 선속도 [mm/s]
DEFAULT_LINEAR_ACC = 50.0     # dashboard 1회 상대 moveL 선가속도 [mm/s²]
JOG_ROT_VEL_DPS = 20.0        # dashboard A/B/C 회전 속도 [deg/s]
JOG_ROT_ACC_DPS2 = 40.0       # dashboard A/B/C 회전 가속도 [deg/s²]
JOG_JOINT_VEL_DPS = 5.0       # dashboard J6 회전 속도 [deg/s]
JOG_JOINT_ACC_DPS2 = 10.0     # dashboard J6 회전 가속도 [deg/s²]
GRIPPER_SERVICE = "/onrobot/sendCommand"
DR_BASE = 0                  # get_current_velx/posx, jog의 기준 좌표계
JOG_AXIS_TASK_X = 6          # Jog.srv: 0~5 = J1~J6, 6~11 = X,Y,Z,rx,ry,rz

# MoveStop.srv stop_mode
STOP_QSTOP_STO = 0           # STO 포함 급정지 — 소프트 E-Stop에 사용
STOP_SSTOP = 2               # 소프트 정지 — 일반 STOP 버튼

# SetRobotControl.srv
CONTROL_RESET_SAFE_STOP = 2  # SAFE_STOP -> STANDBY (START 버튼 = 재개)

ROBOT_STATE_NAMES = {
    0: "INITIALIZING", 1: "STANDBY", 2: "MOVING", 3: "SAFE_OFF",
    4: "TEACHING", 5: "SAFE_STOP", 6: "EMERGENCY_STOP", 7: "HOMMING",
    8: "RECOVERY", 9: "SAFE_STOP2", 10: "SAFE_OFF2", 15: "NOT_READY",
}
ROBOT_MODE_NAMES = {0: "MANUAL", 1: "AUTO", 2: "MEASURE"}
ERROR_LEVEL_NAMES = {1: "INFO", 2: "WARN", 3: "ERROR"}
ERROR_GROUP_SAFETY = 5       # RobotError.group: SAFETY_CONTROLLER
STATES_SAFETY_STOP = (5, 9)  # SAFE_STOP, SAFE_STOP2 -> 충돌/안전정지 의심
STATES_ESTOP = (6,)


# --- 순수 로직 (로봇 없이 pytest/selftest 가능) --------------------------

def link_quality(state_age_sec):
    """마지막 수신 후 경과 시간 -> 연결 등급 (CLAUDE.md §4.1)."""
    if state_age_sec is None or state_age_sec > DISCONNECTED_SEC:
        return "DISCONNECTED"
    if state_age_sec > STALE_SEC:
        return "STALE"
    return "OK"


class LinkStats:
    """토픽 수신 통계: 실측 Hz, latency(헤더 stamp 대비), 유실률 추정.

    유실률은 "정상 간격의 정수배만큼 벌어진 gap = 그만큼 빠졌다"로 추정한다.
    정상 간격은 표본 중앙값(스파이크에 강함). 발행측 nominal Hz를 몰라도
    되므로 doosan 버전이 바뀌어도 그대로 쓸 수 있다.
    """

    def __init__(self, window=LINK_WINDOW):
        self.intervals = deque(maxlen=window)
        self.latencies = deque(maxlen=window)
        self.last_recv = None

    def record(self, recv_sec, stamp_sec=None):
        if self.last_recv is not None:
            self.intervals.append(recv_sec - self.last_recv)
        self.last_recv = recv_sec
        if stamp_sec:
            self.latencies.append(max(0.0, recv_sec - stamp_sec))

    def age(self, now_sec):
        return None if self.last_recv is None else now_sec - self.last_recv

    def hz(self):
        if not self.intervals:
            return 0.0
        median = sorted(self.intervals)[len(self.intervals) // 2]
        return 1.0 / median if median > 0 else 0.0

    def latency_ms(self):
        if not self.latencies:
            return 0.0
        return 1000.0 * sum(self.latencies) / len(self.latencies)

    def loss_rate(self):
        """0.0~1.0. gap이 정상 간격의 2배 이상이면 그 배수만큼 유실로 센다."""
        if len(self.intervals) < 3:
            return 0.0
        median = sorted(self.intervals)[len(self.intervals) // 2]
        if median <= 0:
            return 0.0
        dropped = sum(max(0, round(i / median) - 1) for i in self.intervals)
        return dropped / (len(self.intervals) + dropped)


def norm3(values):
    return math.sqrt(sum(v * v for v in values[:3]))


def _mat_vec(matrix, vector):
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def euler_zyz_matrix(a_deg, b_deg, c_deg):
    """Doosan posx [A,B,C] (intrinsic Z-Y-Z)를 3x3 회전행렬로 변환."""
    a, b, c = map(math.radians, (a_deg, b_deg, c_deg))
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    return [
        [ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
        [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
        [-sb * cc, sb * sc, cb],
    ]


def rotate_pose_about_tool_offset(pose, axis, delta_deg, offset):
    """TOOL 오프셋 지점의 BASE 위치를 유지하며 A/B/C 자세 목표를 계산."""
    if len(pose) != 6 or len(offset) != 3 or axis not in (3, 4, 5):
        raise ValueError("invalid pose, offset, or rotation axis")
    values = [float(value) for value in (*pose, *offset, delta_deg)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose and offset must contain finite values")
    current_rotation = euler_zyz_matrix(*pose[3:6])
    current_offset = _mat_vec(current_rotation, offset)
    pivot = [pose[i] + current_offset[i] for i in range(3)]

    target = [float(value) for value in pose]
    target[axis] += float(delta_deg)
    target_offset = _mat_vec(euler_zyz_matrix(*target[3:6]), offset)
    target[:3] = [pivot[i] - target_offset[i] for i in range(3)]
    return target


def evaluate_safety(robot_state, last_error_level, last_error_group):
    """상태/에러 -> (estop, safety_stop, collision_suspected, fault).

    충돌 전용 토픽이 없으므로 SAFE_STOP 진입 + SAFETY 그룹 에러를 충돌
    의심으로 본다. TODO(verify): 실기에서 충돌을 유발해 실제로 뜨는
    RobotError code를 확인하고 코드 기반 판정으로 좁힐 것.
    """
    estop = robot_state in STATES_ESTOP
    safety_stop = robot_state in STATES_SAFETY_STOP
    collision = safety_stop or last_error_group == ERROR_GROUP_SAFETY
    fault = estop or safety_stop or last_error_level == 3
    return estop, safety_stop, collision, fault


class SystemMonitor(Node):

    def __init__(self):
        super().__init__("system_monitor")

        robot_id = self.declare_parameter("robot_id", DEFAULT_ROBOT_ID).value
        self._publish_hz = self.declare_parameter("publish_hz", PUBLISH_HZ).value
        self._poll_hz = self.declare_parameter("poll_hz", POLL_HZ).value
        self._control_enabled = self.declare_parameter("control_enabled", False).value
        ns = f"/{robot_id}"

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.snap = Snapshot()
        self.snap.control_enabled = bool(self._control_enabled)
        self.stats = LinkStats()
        self.logs = deque(maxlen=LOG_MAX_LINES)
        self._last_error_group = 0
        self._last_error_level = 0
        self._step_started = None
        self._motion_pending = False

        # --- 구독 ---
        self.create_subscription(
            JointState, f"{ns}/joint_states", self._on_joint_states, sensor_qos)
        self.create_subscription(
            RobotError, f"{ns}/error", self._on_error, event_qos)
        self.create_subscription(
            RobotDisconnection, f"{ns}/robot_disconnection",
            self._on_disconnection, event_qos)
        self.create_subscription(
            UInt8MultiArray, f"{ns}/io/ctrl_box_digital_input_state",
            self._on_io, sensor_qos)
        self.create_subscription(
            String, ps.TOPIC, self._on_process, ps.qos())
        self.create_subscription(String, "~/cmd", self._on_cmd, event_qos)
        # /rosout를 그대로 이벤트 로그로 쓴다 (별도 로그 파이프라인 불필요)
        self.create_subscription(
            Log, "/rosout", self._on_rosout,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=50))
        self._subscribe_gripper()

        # --- 발행 ---
        self.status_pub = self.create_publisher(String, "~/status", event_qos)
        self.log_pub = self.create_publisher(String, "~/log", event_qos)

        # --- 서비스 클라이언트 (폴링 + 제어) ---
        self.cli = {
            "state": self.create_client(GetRobotState, f"{ns}/system/get_robot_state"),
            "mode": self.create_client(GetRobotMode, f"{ns}/system/get_robot_mode"),
            "speed_mode": self.create_client(
                GetRobotSpeedMode, f"{ns}/system/get_robot_speed_mode"),
            "set_mode": self.create_client(SetRobotMode, f"{ns}/system/set_robot_mode"),
            "set_control": self.create_client(
                SetRobotControl, f"{ns}/system/set_robot_control"),
            "posx": self.create_client(GetCurrentPosx, f"{ns}/aux_control/get_current_posx"),
            "velx": self.create_client(GetCurrentVelx, f"{ns}/aux_control/get_current_velx"),
            "stop": self.create_client(MoveStop, f"{ns}/motion/move_stop"),
            "move_line": self.create_client(MoveLine, f"{ns}/motion/move_line"),
            "move_joint": self.create_client(MoveJoint, f"{ns}/motion/move_joint"),
            "jog": self.create_client(Jog, f"{ns}/motion/jog"),
        }
        self._gripper_type = None
        try:
            from onrobot_rg_msgs.srv import SetCommand
            self._gripper_type = SetCommand
            self.cli["gripper"] = self.create_client(SetCommand, GRIPPER_SERVICE)
        except ImportError:
            self.get_logger().warn("onrobot_rg_msgs 없음 - 그리퍼 제어 비활성")

        self.create_timer(1.0 / self._poll_hz, self._poll)
        self.create_timer(1.0 / self._publish_hz, self._publish)
        self.get_logger().info(
            f"system_monitor started (ns={ns}, control_enabled={self._control_enabled})")

    # ---------- 구독 콜백 (블로킹 금지) ----------

    def _on_joint_states(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stats.record(self._now(), stamp)
        self.snap.joint_pos_deg = [math.degrees(p) for p in msg.position[:6]]

    def _on_error(self, msg):
        self._last_error_level = msg.level
        self._last_error_group = msg.group
        self.snap.last_error_level = ERROR_LEVEL_NAMES.get(msg.level, str(msg.level))
        self.snap.last_error = f"[{msg.code}] {msg.msg1} {msg.msg2} {msg.msg3}".strip()
        self._log(self.snap.last_error_level, f"robot error {self.snap.last_error}")

    def _on_disconnection(self, _msg):
        self.snap.service_ready = False
        self._log("ERROR", "robot connection lost")

    def _on_io(self, msg):
        data = list(msg.data)
        self.snap.di, self.snap.do = data[:16], data[16:32]

    def _on_process(self, msg):
        """자동화 노드의 공정 보고. 규약·상태머신은 process_state.py."""
        try:
            payload = ps.decode(msg.data)
        except (ValueError, TypeError) as error:
            self._log("WARN", f"bad {ps.TOPIC} payload: {error}")
            return

        step = payload["step"]
        if step != self.snap.step:
            if not ps.is_expected_transition(self.snap.step or None, step):
                # 상태머신 감시: 정의된 순서를 건너뛰면 로그에 남긴다
                self._log("WARN", f"unexpected step {self.snap.step} -> {step}")
            self._step_started = self._now()
            self._log("INFO", f"step {step} ({payload['label']})")
        self.snap.step = step
        self.snap.step_label = payload["label"]
        self.snap.step_index = payload["index"]
        self.snap.step_status = payload["status"]
        self.snap.step_message = payload["message"] or payload["hint"]
        self.snap.task_progress = payload["progress"]
        if payload["status"] == "ERROR":
            self._log("ERROR", f"process error at {step}: {payload['message']}")

    def _on_cmd(self, msg):
        """GUI(dashboard, 별도 프로세스)가 ~/cmd로 보낸 제어 명령 디스패치.
        게이트(control_enabled)는 각 메서드 안에서 검사하므로 여기서는
        cmd 이름만 known 메서드에 그대로 연결한다."""
        try:
            payload = json.loads(msg.data)
            cmd = payload["cmd"]
        except (ValueError, KeyError, TypeError) as error:
            self._log("WARN", f"bad ~/cmd payload: {error}")
            return

        actions = {
            "estop": lambda: self.emergency_stop(),
            "stop": lambda: self.stop(),
            "start": lambda: self.start(),
            "set_control_enabled": lambda: self.set_control_enabled(
                payload.get("enabled", False)),
            "set_mode": lambda: self.set_mode(payload.get("mode", "")),
            "jog": lambda: self.jog(payload.get("axis", 0), payload.get("speed", 0.0)),
            "jog_stop": lambda: self.jog_stop(payload.get("axis", 0)),
            "move_task": lambda: self.move_task(
                payload.get("axis", 0),
                payload.get("distance", 0.0),
                payload.get("linear_speed", DEFAULT_LINEAR_SPEED),
                payload.get("tcp_offset", [0.0, 0.0, 0.0])),
            "move_joint6": lambda: self.move_joint6(payload.get("delta", 0.0)),
            "gripper": lambda: self.gripper(payload.get("command", "")),
        }
        action = actions.get(cmd)
        if action is None:
            self._log("WARN", f"unknown cmd: {cmd}")
            return
        action()

    def _on_rosout(self, msg):
        if msg.name == self.get_name():
            return  # 자기 로그 되먹임 방지
        level = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        self._log(level.get(msg.level, str(msg.level)), f"{msg.name}: {msg.msg}")

    def _subscribe_gripper(self):
        # ponytail: OnRobot 패키지가 없는 환경에서도 모니터가 떠야 하므로 선택적.
        try:
            from onrobot_rg_msgs.msg import OnRobotRGInput
        except ImportError:
            self.get_logger().warn("onrobot_rg_msgs 없음 - 그리퍼 패널 비활성")
            return

        def on_gripper(msg):
            self.snap.gripper_connected = True
            self.snap.gripper_width_mm = msg.gwdf / 10.0
            self.snap.gripper_busy = bool(msg.gsta & 0x01)
            self.snap.gripper_grip_detected = bool(msg.gsta & 0x02)

        self.create_subscription(OnRobotRGInput, "OnRobotRGInput", on_gripper, 10)

    # ---------- 폴링 ----------

    def _poll(self):
        self._call("state", GetRobotState.Request(), self._got_state)
        self._call("mode", GetRobotMode.Request(), self._got_mode)
        self._call("speed_mode", GetRobotSpeedMode.Request(), self._got_speed_mode)
        self._call("velx", GetCurrentVelx.Request(ref=DR_BASE), self._got_velx)
        self._call("posx", GetCurrentPosx.Request(ref=DR_BASE), self._got_posx)

    def _call(self, key, request, on_result):
        """비동기 호출만. 콜백에서 대기하면 GUI가 얼어붙는다 (§5.3)."""
        client = self.cli[key]
        if not client.service_is_ready():
            if key == "state":
                self.snap.service_ready = False
            return
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._done(key, f, on_result))

    def _done(self, key, future, on_result):
        try:
            result = future.result()
        except Exception as error:            # 서비스 예외를 삼키지 않는다 (§5.4)
            if key in ("move_line", "move_joint"):
                self._motion_pending = False
            self._log("ERROR", f"{key} service failed: {error}")
            return
        if result is None or not getattr(result, "success", True):
            if key in ("move_line", "move_joint"):
                self._motion_pending = False
            self._log("WARN", f"{key} service returned failure")
            return
        on_result(result)

    def _got_state(self, res):
        self.snap.service_ready = True
        self.snap.robot_state = int(res.robot_state)
        self.snap.robot_state_str = ROBOT_STATE_NAMES.get(
            self.snap.robot_state, f"UNKNOWN({res.robot_state})")
        (self.snap.estop, self.snap.safety_stop,
         self.snap.collision_suspected, self.snap.fault) = evaluate_safety(
            self.snap.robot_state, self._last_error_level, self._last_error_group)

    def _got_mode(self, res):
        self.snap.robot_mode = ROBOT_MODE_NAMES.get(res.robot_mode, str(res.robot_mode))

    def _got_speed_mode(self, res):
        self.snap.speed_mode = "REDUCED" if res.speed_mode == 1 else "NORMAL"

    def _got_velx(self, res):
        self.snap.linear_speed = norm3(res.vel[:3])
        self.snap.angular_speed = norm3(res.vel[3:])

    def _got_posx(self, res):
        if res.task_pos_info:
            self.snap.tcp_posx = list(res.task_pos_info[0].data[:6])

    # ---------- 발행 ----------

    def _publish(self):
        now = self._now()
        age = self.stats.age(now)
        self.snap.stamp = now
        self.snap.link_state = link_quality(age)
        self.snap.link_age_ms = -1.0 if age is None else age * 1000.0
        self.snap.link_hz = self.stats.hz()
        self.snap.link_latency_ms = self.stats.latency_ms()
        self.snap.link_loss_rate = self.stats.loss_rate()
        self.snap.step_elapsed = (
            0.0 if self._step_started is None else now - self._step_started)
        self.status_pub.publish(String(data=json.dumps(asdict(self.snap))))

    def snapshot(self):
        """GUI가 토픽 대신 직접 읽고 싶을 때 (같은 프로세스일 때만)."""
        return self.snap

    def _log(self, level, text):
        line = f"{time.strftime('%H:%M:%S')} [{level}] {text}"
        self.logs.append(line)
        self.log_pub.publish(String(data=line))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 제어 (기본 비활성, §5.1) ----------

    def _guard(self, what):
        if not self.snap.control_enabled:
            self._log("WARN", f"{what} ignored: control disabled")
            return False
        return True

    def set_control_enabled(self, enabled):
        self.snap.control_enabled = bool(enabled)
        self._log("INFO", f"control {'enabled' if enabled else 'disabled'}")

    def emergency_stop(self):
        """소프트 E-Stop. control_enabled와 무관하게 항상 동작한다 (§5.2).
        물리 E-Stop의 대체재가 아니다."""
        self._log("ERROR", "EMERGENCY STOP requested (software)")
        self._call("stop", MoveStop.Request(stop_mode=STOP_QSTOP_STO), lambda r: None)

    def stop(self):
        self._log("INFO", "soft stop requested")
        self._call("stop", MoveStop.Request(stop_mode=STOP_SSTOP), lambda r: None)

    def start(self):
        """SAFE_STOP 해제 후 STANDBY 복귀 (= 재개)."""
        if not self._guard("start"):
            return
        self._call("set_control",
                   SetRobotControl.Request(robot_control=CONTROL_RESET_SAFE_STOP),
                   lambda r: self._log("INFO", "safe stop reset"))

    def set_mode(self, mode_name):
        """'AUTO' / 'MANUAL'."""
        if not self._guard("set_mode"):
            return
        mode = {"MANUAL": 0, "AUTO": 1}.get(mode_name.upper())
        if mode is None:
            self._log("WARN", f"unknown mode {mode_name}")
            return
        self._call("set_mode", SetRobotMode.Request(robot_mode=mode),
                   lambda r: self._log("INFO", f"mode -> {mode_name}"))

    def jog(self, axis, speed_pct):
        """방향 버튼(WASD/화살표)용. axis: 0~5=J1~J6, 6~11=X,Y,Z,rx,ry,rz.
        speed_pct=0이면 정지. 버튼 press/release에 걸어 쓰고, 슬라이더
        valueChanged에는 절대 연결하지 말 것 (§5.1)."""
        if not self._guard("jog"):
            return
        speed = max(-MAX_JOG_SPEED_PCT, min(MAX_JOG_SPEED_PCT, float(speed_pct)))
        self._call("jog", Jog.Request(jog_axis=int(axis), move_reference=DR_BASE,
                                      speed=speed), lambda r: None)

    def jog_stop(self, axis):
        self.jog(axis, 0.0)

    def move_task(self, axis, distance, linear_speed, tcp_offset):
        """control_GUI의 1회 상대 moveL 및 TOOL-offset pivot 회전을 수행."""
        if not self._guard("move_task") or self._motion_pending:
            if self._motion_pending:
                self._log("WARN", "move_task ignored: previous motion is pending")
            return
        try:
            axis = int(axis)
            distance = float(distance)
            linear_speed = max(1.0, min(300.0, float(linear_speed)))
            offset = [float(value) for value in tcp_offset]
            if axis not in range(6) or len(offset) != 3:
                raise ValueError
            if not all(math.isfinite(value) for value in [distance, *offset]):
                raise ValueError
            if any(abs(value) > 1000.0 for value in offset):
                raise ValueError
        except (TypeError, ValueError):
            self._log("WARN", "invalid move_task payload")
            return

        request = MoveLine.Request()
        if axis < 3:
            target = [0.0] * 6
            target[axis] = distance
            request.mode = 1                 # DR_MV_MOD_REL
            description = f"REL/BASE {'XYZ'[axis]} {distance:+.1f} mm"
        else:
            if len(self.snap.tcp_posx) < 6:
                self._log("WARN", "rotation ignored: current TCP pose unavailable")
                return
            try:
                target = rotate_pose_about_tool_offset(
                    self.snap.tcp_posx[:6], axis, distance, offset)
            except ValueError as error:
                self._log("WARN", f"rotation target failed: {error}")
                return
            request.mode = 0                 # DR_MV_MOD_ABS
            description = (
                f"OFFSET-PIVOT {'ABC'[axis - 3]} {distance:+.1f} deg "
                f"offset={offset}")

        request.pos = target
        request.vel = [linear_speed, JOG_ROT_VEL_DPS]
        request.acc = [
            max(DEFAULT_LINEAR_ACC, min(400.0, linear_speed * 2.0)),
            JOG_ROT_ACC_DPS2,
        ]
        request.time = 0.0
        request.radius = 0.0
        request.ref = DR_BASE
        request.blend_type = 0
        request.sync_type = 0
        if not self.cli["move_line"].service_is_ready():
            self._log("WARN", "moveL ignored: motion/move_line service unavailable")
            return
        self._motion_pending = True
        self._log("INFO", f"moveL command: {description}")
        self._call("move_line", request, self._motion_done)

    def move_joint6(self, delta_deg):
        """현재 joint state 기준 J6 절대 목표를 만들어 moveJ로 보낸다."""
        if not self._guard("move_joint6") or self._motion_pending:
            if self._motion_pending:
                self._log("WARN", "move_joint6 ignored: previous motion is pending")
            return
        if len(self.snap.joint_pos_deg) < 6:
            self._log("WARN", "J6 rotation ignored: current joint pose unavailable")
            return
        try:
            delta = float(delta_deg)
            if not math.isfinite(delta):
                raise ValueError
        except (TypeError, ValueError):
            self._log("WARN", "invalid J6 delta")
            return
        target = list(self.snap.joint_pos_deg[:6])
        target[5] += delta
        request = MoveJoint.Request()
        request.pos = target
        request.vel = JOG_JOINT_VEL_DPS
        request.acc = JOG_JOINT_ACC_DPS2
        request.time = 0.0
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 0
        if not self.cli["move_joint"].service_is_ready():
            self._log("WARN", "moveJ ignored: motion/move_joint service unavailable")
            return
        self._motion_pending = True
        self._log("INFO", f"moveJ command: J6 {delta:+.1f} deg")
        self._call("move_joint", request, self._motion_done)

    def _motion_done(self, _result):
        self._motion_pending = False
        self._log("INFO", "discrete motion done")

    def gripper(self, command):
        if not self._guard("gripper"):
            return
        if command not in ("o", "c") or self._gripper_type is None:
            self._log("WARN", "invalid or unavailable gripper command")
            return
        request = self._gripper_type.Request()
        request.command = command
        label = "open" if command == "o" else "close"
        self._log("INFO", f"gripper {label} requested")
        self._call("gripper", request, lambda _result: self._log(
            "INFO", f"gripper {label} done"))


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# --- self-check: 로봇 없이 순수 로직만 검증 -----------------------------

def _demo():
    assert link_quality(None) == "DISCONNECTED"
    assert link_quality(0.1) == "OK"
    assert link_quality(STALE_SEC + 0.01) == "STALE"
    assert link_quality(DISCONNECTED_SEC + 0.01) == "DISCONNECTED"

    s = LinkStats()
    for i in range(20):                      # 10ms 간격 = 100Hz, 유실 없음
        s.record(i * 0.01, i * 0.01 - 0.002)
    assert abs(s.hz() - 100.0) < 1e-6, s.hz()
    assert abs(s.latency_ms() - 2.0) < 1e-6, s.latency_ms()
    assert s.loss_rate() == 0.0
    assert abs(s.age(0.25) - 0.06) < 1e-9

    s2 = LinkStats()                         # 4개 간격 중 하나만 3배 gap -> 2개 유실
    for t in (0.0, 0.01, 0.02, 0.05, 0.06):
        s2.record(t)
    assert abs(s2.loss_rate() - 2 / 6) < 1e-9, s2.loss_rate()

    assert evaluate_safety(1, 0, 0) == (False, False, False, False)
    assert evaluate_safety(6, 0, 0) == (True, False, False, True)       # E-Stop
    assert evaluate_safety(5, 0, 0) == (False, True, True, True)        # 충돌 의심
    assert evaluate_safety(2, 3, 2)[3] is True                          # ERROR 레벨
    assert evaluate_safety(2, 1, ERROR_GROUP_SAFETY)[2] is True

    assert abs(norm3([3.0, 4.0, 0.0, 9.0]) - 5.0) < 1e-9
    pose = [100.0, 200.0, 300.0, 10.0, 20.0, 30.0]
    rotated = rotate_pose_about_tool_offset(pose, 3, 5.0, [0.0, 0.0, 0.0])
    assert rotated[:3] == pose[:3] and rotated[3] == 15.0
    offset = [10.0, -4.0, 7.0]
    before = _mat_vec(euler_zyz_matrix(*pose[3:]), offset)
    after = _mat_vec(euler_zyz_matrix(*rotated[3:]), offset)
    # offset=0 이외의 피벗도 위 함수 자체에서 유한한 6D 목표를 내야 한다.
    pivoted = rotate_pose_about_tool_offset(pose, 5, -5.0, offset)
    assert len(pivoted) == 6 and all(math.isfinite(value) for value in pivoted)
    assert before != after
    json.dumps(asdict(Snapshot()))           # GUI로 나가는 JSON 직렬화 확인
    print("system_monitor self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo()
    else:
        main()
