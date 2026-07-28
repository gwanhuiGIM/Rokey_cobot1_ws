# =============================================================
# dashboard.py — 시스템 모니터 + 핸드드립 공정 상태를 보여주는 PyQt5 창
# -------------------------------------------------------------
# 실행: ros2 run monitor_pjt dashboard
#       ros2 launch monitor_pjt monitor.launch.py   (system_monitor와 함께 기동)
#       레이아웃만 확인: python3 dashboard.py — ROS/로봇 불필요 (node=None)
# 노드: /monitor_dashboard — system_monitor(별도 프로세스)와는 토픽으로만
#       통신한다(MonitorClient). dsr_msgs2 등 로봇 SDK는 전혀 import하지
#       않으므로 로봇 SDK가 없는 PC에서도 이 창만 띄워 원격 모니터링할 수 있다.
#   구독: /system_monitor/status(String JSON -> snapshot.Snapshot),
#         /system_monitor/log(String, 로그 1줄)
#   발행: /system_monitor/cmd(String JSON, 제어 명령 — system_monitor._on_cmd 참고)
# 의존: PyQt5, rclpy, std_msgs, monitor_pjt.{snapshot, process_state}
# -------------------------------------------------------------
# 설계 (CLAUDE.md §5)
# - .ui 파일 없이 코드로 레이아웃 구성 -> 절대경로/objectName 중복 문제 없음.
# - rclpy.spin_once는 Qt 메인스레드 QTimer(20ms)에서만 호출 -> 단일 스레드.
# - 슬라이더 valueChanged에 모션 명령을 걸지 않는다. jog는 버튼 press/release.
# - "제어 활성화" 체크가 꺼져 있으면 모든 모션 위젯이 비활성. E-Stop만 예외.
#   (실제 차단은 이 창이 아니라 system_monitor 쪽 — 여기 게이트는 UI일 뿐.)
# - 소프트웨어 E-Stop은 물리 E-Stop의 대체재가 아니다 (화면에도 명시).
#
# 화면 갱신 알고리즘
# - refresh()는 10Hz로 MonitorClient.snapshot()(마지막으로 받은 Snapshot)을
#   읽어 위젯에 그대로 반영한다 — 위젯쪽에 상태를 들고 있지 않는다.
# - 배너 우선순위: E-Stop > 충돌의심 > 일반 fault > 공정 ERROR > 통신이상
#   > 정상. 항상 "가장 위험한 것"이 위에 뜨게 순서대로 검사한다.
# - 공정 칩(chip) 색상은 step_index만으로 결정: 현재=강조색(상태별),
#   지난 스텝=회색, 아직=어두움 — 상태머신 자체는 process_state.py가 정의.
# =============================================================

import json
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from monitor_sys.monitor_pjt import process_state as ps
from monitor_sys.monitor_pjt.snapshot import Snapshot

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QProgressBar, QPushButton, QSlider, QTextEdit,
    QVBoxLayout, QWidget,
)

STATUS_TOPIC = "/system_monitor/status"
LOG_TOPIC = "/system_monitor/log"
CMD_TOPIC = "/system_monitor/cmd"

GUI_HZ = 10.0                 # 화면 갱신 [Hz]
SPIN_MS = 20                  # rclpy.spin_once 주기 [ms]
LATENCY_HISTORY = 200         # latency 그래프 표본 수
LOG_VIEW_MAX_BLOCKS = 2000    # QTextEdit 줄 상한 (무한 append 금지)
JOG_AXES = [("X", 6), ("Y", 7), ("Z", 8)]   # Jog.srv: 6~11 = X,Y,Z,rx,ry,rz
DEFAULT_JOG_SPEED_PCT = 5
JOINT_LIMITS_DEG = [360, 360, 150, 360, 360, 360]  # m0609.urdf 실측
LOG_LEVELS = ["ALL", "INFO", "WARN", "ERROR"]

COLOR_OK = "#1e7d32"
COLOR_WARN = "#b26a00"
COLOR_BAD = "#c62828"
COLOR_IDLE = "#555555"
COLOR_DONE = "#37474f"        # 지나간 공정 스텝
PROCESS_COLUMNS = 4           # 공정 스텝 칩 배치 열 수


def led(color):
    return f"border-radius:7px; min-width:14px; max-width:14px; min-height:14px; max-height:14px; background:{color};"


class Sparkline(QWidget):
    """latency 시계열 미니 그래프. ponytail: pyqtgraph 대신 paintEvent 20줄."""

    def __init__(self, maxlen=LATENCY_HISTORY):
        super().__init__()
        self.values = deque(maxlen=maxlen)
        self.setMinimumHeight(60)

    def push(self, value):
        self.values.append(float(value))
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111111"))
        if len(self.values) < 2:
            return
        top = max(max(self.values), 1.0)
        w, h = self.width(), self.height()
        step = w / (len(self.values) - 1)
        painter.setPen(QPen(QColor("#4fc3f7"), 1))
        points = [
            (i * step, h - (v / top) * (h - 4) - 2)
            for i, v in enumerate(self.values)
        ]
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            painter.drawLine(int(x0), int(y0), int(x1), int(y1))
        painter.setPen(QColor("#888888"))
        painter.drawText(4, 12, f"max {top:.1f} ms")


class MonitorClient(Node):
    """system_monitor(별도 프로세스)와 토픽으로만 말하는 얇은 클라이언트.

    Dashboard 위젯이 기대하는 인터페이스(snapshot()/logs/제어 메서드들)를
    그대로 구현해서, system_monitor.SystemMonitor를 직접 들고 있던 예전
    구조와 위젯 코드 쪽 변경 없이 갈아끼울 수 있게 했다. dsr_msgs2는
    import하지 않는다 — 로봇 SDK 없이도 이 창을 띄울 수 있어야 하므로.
    """

    def __init__(self):
        super().__init__("monitor_dashboard")
        self.snap = Snapshot()
        self.logs = deque(maxlen=LOG_VIEW_MAX_BLOCKS)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, qos)
        self.create_subscription(String, LOG_TOPIC, self._on_log, qos)
        self.cmd_pub = self.create_publisher(String, CMD_TOPIC, qos)

    def _on_status(self, msg):
        try:
            self.snap = Snapshot(**json.loads(msg.data))
        except (ValueError, TypeError) as error:
            self.get_logger().warn(f"bad status payload: {error}")

    def _on_log(self, msg):
        self.logs.append(msg.data)

    def snapshot(self):
        return self.snap

    def _cmd(self, **payload):
        self.cmd_pub.publish(String(data=json.dumps(payload)))

    def set_control_enabled(self, enabled):
        self._cmd(cmd="set_control_enabled", enabled=bool(enabled))

    def emergency_stop(self):
        self._cmd(cmd="estop")

    def stop(self):
        self._cmd(cmd="stop")

    def start(self):
        self._cmd(cmd="start")

    def set_mode(self, mode):
        self._cmd(cmd="set_mode", mode=mode)

    def jog(self, axis, speed):
        self._cmd(cmd="jog", axis=int(axis), speed=float(speed))

    def jog_stop(self, axis):
        self._cmd(cmd="jog_stop", axis=int(axis))


class Dashboard(QMainWindow):

    def __init__(self, node=None):
        super().__init__()
        self.node = node
        self.setWindowTitle("M0609 System Monitor")
        self.resize(1100, 760)
        self._log_cursor = 0

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.addWidget(self._build_banner())

        outer.addWidget(self._build_process_panel())

        columns = QHBoxLayout()
        left = QVBoxLayout()
        left.addWidget(self._build_state_panel())
        left.addWidget(self._build_position_panel())
        left.addWidget(self._build_comm_panel())
        right = QVBoxLayout()
        right.addWidget(self._build_control_panel())
        right.addWidget(self._build_io_panel())
        columns.addLayout(left, 3)
        columns.addLayout(right, 2)
        outer.addLayout(columns)
        outer.addWidget(self._build_log_panel(), 1)

        self._apply_control_gate(False)
        timer = QTimer(self)
        timer.timeout.connect(self.refresh)
        timer.start(int(1000 / GUI_HZ))

    # ---------- 패널 구성 ----------

    def _build_banner(self):
        self.lbl_banner = QLabel("--")
        self.lbl_banner.setAlignment(Qt.AlignCenter)
        self.lbl_banner.setStyleSheet(
            f"background:{COLOR_IDLE}; color:white; font-size:20px; padding:10px;")
        return self.lbl_banner

    def _build_state_panel(self):
        box = QGroupBox("1. 로봇 상태")
        grid = QGridLayout(box)
        self.lbl_state = QLabel("--")
        self.lbl_state.setStyleSheet("font-size:22px; font-weight:bold;")
        self.lbl_mode = QLabel("--")
        self.lbl_speed_mode = QLabel("--")
        self.lbl_speed = QLabel("--")

        grid.addWidget(QLabel("상태"), 0, 0)
        grid.addWidget(self.lbl_state, 0, 1, 1, 3)
        grid.addWidget(QLabel("운전 모드"), 1, 0)
        grid.addWidget(self.lbl_mode, 1, 1)
        grid.addWidget(QLabel("속도 모드"), 1, 2)
        grid.addWidget(self.lbl_speed_mode, 1, 3)
        grid.addWidget(QLabel("TCP 속도"), 2, 0)
        grid.addWidget(self.lbl_speed, 2, 1, 1, 3)
        return box

    def _build_process_panel(self):
        """핸드드립 공정 상태머신 (process_state.STEPS가 유일한 정의)."""
        box = QGroupBox("핸드드립 공정 진행")
        layout = QVBoxLayout(box)

        head = QHBoxLayout()
        self.lbl_step = QLabel("(공정 보고 없음)")
        self.lbl_step.setStyleSheet("font-size:20px; font-weight:bold;")
        self.lbl_step_msg = QLabel("")
        self.lbl_step_elapsed = QLabel("")
        head.addWidget(self.lbl_step)
        head.addWidget(self.lbl_step_msg, 1)
        head.addWidget(self.lbl_step_elapsed)
        layout.addLayout(head)

        self.bar_task = QProgressBar()
        self.bar_task.setRange(0, 100)
        layout.addWidget(self.bar_task)

        chips = QGridLayout()
        self.chips = []
        for i, (key, label, hint) in enumerate(ps.STEPS):
            trigger = ps.DI_TRIGGER.get(key)
            text = f"{i + 1}. {label}" + (f"\n▶ DI{trigger}" if trigger else "")
            chip = QLabel(text)
            chip.setAlignment(Qt.AlignCenter)
            chip.setToolTip(hint or key)
            chips.addWidget(chip, i // PROCESS_COLUMNS, i % PROCESS_COLUMNS)
            self.chips.append(chip)
        layout.addLayout(chips)
        return box

    def _build_position_panel(self):
        box = QGroupBox("2. 로봇 위치")
        grid = QGridLayout(box)
        self.bars_joint, self.lbls_joint = [], []
        for i, limit in enumerate(JOINT_LIMITS_DEG):
            bar = QProgressBar()
            bar.setRange(-limit, limit)      # 관절 한계 대비 비율이 그대로 보임
            bar.setFormat("")
            label = QLabel("--")
            grid.addWidget(QLabel(f"J{i + 1}"), i, 0)
            grid.addWidget(bar, i, 1)
            grid.addWidget(label, i, 2)
            self.bars_joint.append(bar)
            self.lbls_joint.append(label)
        self.lbl_posx = QLabel("--")
        self.lbl_posx.setStyleSheet("font-family:monospace;")
        grid.addWidget(QLabel("TCP"), 6, 0)
        grid.addWidget(self.lbl_posx, 6, 1, 1, 2)
        return box

    def _build_comm_panel(self):
        box = QGroupBox("3. 통신 상태")
        grid = QGridLayout(box)
        self.led_ros = QLabel()
        self.led_srv = QLabel()
        self.led_gripper = QLabel()
        self.lbl_link = QLabel("--")
        self.lbl_link_stat = QLabel("--")
        self.graph_latency = Sparkline()

        for col, (name, dot) in enumerate(
                [("ROS 토픽", self.led_ros), ("드라이버 서비스", self.led_srv),
                 ("RG2 그리퍼", self.led_gripper)]):
            cell = QHBoxLayout()
            cell.addWidget(dot)
            cell.addWidget(QLabel(name))
            cell.addStretch()
            wrap = QWidget()
            wrap.setLayout(cell)
            grid.addWidget(wrap, 0, col)
        grid.addWidget(self.lbl_link, 1, 0, 1, 3)
        grid.addWidget(self.lbl_link_stat, 2, 0, 1, 3)
        grid.addWidget(self.graph_latency, 3, 0, 1, 3)
        return box

    def _build_control_panel(self):
        box = QGroupBox("4. 제어")
        layout = QVBoxLayout(box)

        self.btn_estop = QPushButton("E-STOP")
        self.btn_estop.setMinimumHeight(70)
        self.btn_estop.setStyleSheet(
            f"background:{COLOR_BAD}; color:white; font-size:24px; font-weight:bold;")
        self.btn_estop.clicked.connect(self.on_estop)
        layout.addWidget(self.btn_estop)
        note = QLabel("※ 소프트웨어 E-Stop은 물리 E-Stop의 대체재가 아닙니다.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color:{COLOR_BAD};")
        layout.addWidget(note)

        self.chk_control = QCheckBox("제어 활성화 (Enable Control)")
        self.chk_control.toggled.connect(self.on_control_toggled)
        layout.addWidget(self.chk_control)

        row = QHBoxLayout()
        self.btn_start = QPushButton("START (안전정지 해제)")
        self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.clicked.connect(self.on_stop)
        row.addWidget(self.btn_start)
        row.addWidget(self.btn_stop)
        layout.addLayout(row)

        mode_row = QHBoxLayout()
        self.btn_manual = QPushButton("MANUAL")
        self.btn_manual.clicked.connect(lambda: self.on_set_mode("MANUAL"))
        self.btn_auto = QPushButton("AUTO")
        self.btn_auto.clicked.connect(lambda: self.on_set_mode("AUTO"))
        mode_row.addWidget(self.btn_manual)
        mode_row.addWidget(self.btn_auto)
        layout.addLayout(mode_row)

        # jog 속도: 슬라이더는 값만 보관하고, 명령은 버튼 press에서만 나간다
        speed_row = QHBoxLayout()
        self.sld_speed = QSlider(Qt.Horizontal)
        self.sld_speed.setRange(1, 20)       # 상한은 노드의 MAX_JOG_SPEED_PCT와 동일
        self.sld_speed.setValue(DEFAULT_JOG_SPEED_PCT)
        self.lbl_speed_set = QLabel(f"{DEFAULT_JOG_SPEED_PCT} %")
        self.sld_speed.valueChanged.connect(
            lambda v: self.lbl_speed_set.setText(f"{v} %"))
        speed_row.addWidget(QLabel("Jog 속도"))
        speed_row.addWidget(self.sld_speed)
        speed_row.addWidget(self.lbl_speed_set)
        layout.addLayout(speed_row)

        self.jog_buttons = []
        jog_grid = QGridLayout()
        for col, (name, axis) in enumerate(JOG_AXES):
            for row_i, sign in enumerate((+1, -1)):
                button = QPushButton(f"{name}{'+' if sign > 0 else '-'}")
                button.setAutoRepeat(False)
                button.pressed.connect(
                    lambda a=axis, s=sign: self.on_jog(a, s))
                button.released.connect(lambda a=axis: self.on_jog_stop(a))
                jog_grid.addWidget(button, row_i, col)
                self.jog_buttons.append(button)
        layout.addLayout(jog_grid)
        layout.addStretch()
        return box

    def _build_io_panel(self):
        box = QGroupBox("5. IO / 그리퍼")
        grid = QGridLayout(box)
        self.leds_di, self.leds_do = [], []
        grid.addWidget(QLabel("DI"), 0, 0)
        grid.addWidget(QLabel("DO"), 1, 0)
        for i in range(16):
            for row_i, store in ((0, self.leds_di), (1, self.leds_do)):
                dot = QLabel()
                dot.setStyleSheet(led(COLOR_IDLE))
                grid.addWidget(dot, row_i, i + 1)
                store.append(dot)
        # 공정 버튼 (DRL이 get_digital_input(n)으로 읽는 그 신호).
        # 토픽 배열 인덱스 = n - 1 (data[0] = DI1).
        self.leds_button, self.lbls_button = {}, {}
        for row_i, (number, label) in enumerate(sorted(ps.DI_LABELS.items())):
            dot = QLabel()
            dot.setStyleSheet(led(COLOR_IDLE))
            name = QLabel(f"DI{number}  {label}")
            grid.addWidget(dot, 2 + row_i, 0)
            grid.addWidget(name, 2 + row_i, 1, 1, 16)
            self.leds_button[number] = dot
            self.lbls_button[number] = name

        self.lbl_gripper = QLabel("--")
        grid.addWidget(QLabel("RG2"), 2 + len(ps.DI_LABELS), 0)
        grid.addWidget(self.lbl_gripper, 2 + len(ps.DI_LABELS), 1, 1, 16)
        return box

    def _build_log_panel(self):
        box = QGroupBox("6. 로그")
        layout = QVBoxLayout(box)
        row = QHBoxLayout()
        self.cmb_log_level = QComboBox()
        self.cmb_log_level.addItems(LOG_LEVELS)
        btn_clear = QPushButton("지우기")
        btn_clear.clicked.connect(lambda: self.txt_log.clear())
        row.addWidget(QLabel("필터"))
        row.addWidget(self.cmb_log_level)
        row.addStretch()
        row.addWidget(btn_clear)
        layout.addLayout(row)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.document().setMaximumBlockCount(LOG_VIEW_MAX_BLOCKS)
        self.txt_log.setStyleSheet("font-family:monospace;")
        layout.addWidget(self.txt_log)
        return box

    # ---------- 제어 핸들러 ----------

    def _apply_control_gate(self, enabled):
        for widget in ([self.btn_start, self.btn_stop, self.btn_manual,
                        self.btn_auto, self.sld_speed] + self.jog_buttons):
            widget.setEnabled(enabled)      # E-Stop은 절대 비활성화하지 않는다

    def on_control_toggled(self, enabled):
        self._apply_control_gate(enabled)
        if self.node:
            self.node.set_control_enabled(enabled)

    def on_estop(self):
        if self.node:
            self.node.emergency_stop()

    def on_stop(self):
        if self.node:
            self.node.stop()

    def on_start(self):
        if self.node and self._confirm("START", "안전정지를 해제하고 재개합니다."):
            self.node.start()

    def on_set_mode(self, mode):
        if self.node and self._confirm(f"모드 전환 -> {mode}", "로봇 운전 모드를 바꿉니다."):
            self.node.set_mode(mode)

    def on_jog(self, axis, sign):
        if self.node:
            self.node.jog(axis, sign * self.sld_speed.value())

    def on_jog_stop(self, axis):
        if self.node:
            self.node.jog_stop(axis)

    def _confirm(self, title, detail):
        from PyQt5.QtWidgets import QMessageBox
        answer = QMessageBox.question(
            self, title, f"{detail}\n\n실행할까요?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return answer == QMessageBox.Yes

    # ---------- 화면 갱신 ----------

    def refresh(self):
        if self.node is None:
            return
        snap = self.node.snapshot()

        if snap.estop:
            self._set_banner("EMERGENCY STOP", COLOR_BAD)
        elif snap.collision_suspected:
            self._set_banner("충돌/안전정지 감지", COLOR_BAD)
        elif snap.fault:
            self._set_banner(f"FAULT — {snap.last_error}", COLOR_BAD)
        elif snap.step_status == "ERROR":
            self._set_banner(f"공정 오류 — {snap.step_label}: {snap.step_message}", COLOR_BAD)
        elif snap.link_state != "OK":
            self._set_banner(f"통신 {snap.link_state}", COLOR_WARN)
        else:
            self._set_banner(f"정상 — {snap.robot_state_str}", COLOR_OK)

        self.lbl_state.setText(snap.robot_state_str)
        self.lbl_mode.setText(snap.robot_mode)
        self.lbl_speed_mode.setText(snap.speed_mode)
        self.lbl_speed.setText(
            f"linear {snap.linear_speed:7.1f} mm/s   angular {snap.angular_speed:6.1f} deg/s")
        self._refresh_process(snap)

        for i, bar in enumerate(self.bars_joint):
            value = snap.joint_pos_deg[i] if i < len(snap.joint_pos_deg) else 0.0
            bar.setValue(int(value))
            self.lbls_joint[i].setText(f"{value:+8.2f}°")
        if snap.tcp_posx:
            self.lbl_posx.setText(" ".join(f"{v:+8.1f}" for v in snap.tcp_posx))

        self.led_ros.setStyleSheet(led(
            {"OK": COLOR_OK, "STALE": COLOR_WARN}.get(snap.link_state, COLOR_BAD)))
        self.led_srv.setStyleSheet(led(COLOR_OK if snap.service_ready else COLOR_BAD))
        self.led_gripper.setStyleSheet(
            led(COLOR_OK if snap.gripper_connected else COLOR_IDLE))
        self.lbl_link.setText(
            f"{snap.link_state}  |  age {snap.link_age_ms:.0f} ms  |  {snap.link_hz:.1f} Hz")
        self.lbl_link_stat.setText(
            f"latency {snap.link_latency_ms:.1f} ms  |  loss {snap.link_loss_rate * 100:.1f} %")
        self.graph_latency.push(snap.link_latency_ms)

        for dots, values in ((self.leds_di, snap.di), (self.leds_do, snap.do)):
            for i, dot in enumerate(dots):
                on = i < len(values) and values[i]
                dot.setStyleSheet(led(COLOR_OK if on else COLOR_IDLE))
        for number, dot in self.leds_button.items():
            index = number - 1                    # DRL 1-based -> 배열 0-based
            on = index < len(snap.di) and snap.di[index]
            waiting = ps.DI_TRIGGER.get(snap.step) == number
            dot.setStyleSheet(led(COLOR_OK if on else
                                  COLOR_WARN if waiting else COLOR_IDLE))
            self.lbls_button[number].setStyleSheet(
                f"color:{COLOR_WARN}; font-weight:bold;" if waiting else "")
        self.lbl_gripper.setText(
            f"width {snap.gripper_width_mm:5.1f} mm | "
            f"{'BUSY' if snap.gripper_busy else 'idle'} | "
            f"{'파지 감지' if snap.gripper_grip_detected else '비어있음'}"
            if snap.gripper_connected else "미연결")

        self._drain_logs()

    def _refresh_process(self, snap):
        self.lbl_step.setText(
            f"{snap.step_label}" + (f"  [{snap.step_status}]" if snap.step_status else ""))
        self.lbl_step_msg.setText(snap.step_message)
        self.lbl_step_elapsed.setText(
            f"{snap.step_elapsed:.1f} s" if snap.step_index >= 0 else "")
        self.bar_task.setValue(int(snap.task_progress))

        current_color = {"ERROR": COLOR_BAD, "WAITING": COLOR_WARN}.get(
            snap.step_status, COLOR_OK)
        for i, chip in enumerate(self.chips):
            if i == snap.step_index:
                style = f"background:{current_color}; color:white; font-weight:bold;"
            elif snap.step_index >= 0 and i < snap.step_index:
                style = f"background:{COLOR_DONE}; color:#aaaaaa;"
            else:
                style = "background:#222222; color:#777777;"
            chip.setStyleSheet(style + " padding:6px; border-radius:4px;")

    def _set_banner(self, text, color):
        self.lbl_banner.setText(text)
        self.lbl_banner.setStyleSheet(
            f"background:{color}; color:white; font-size:20px; padding:10px;")

    def _drain_logs(self):
        """노드의 링버퍼에서 새 줄만 가져온다 (매 프레임 전체 재출력 금지)."""
        logs = self.node.logs
        new_lines = list(logs)[self._log_cursor:]
        self._log_cursor = len(logs)
        level = self.cmb_log_level.currentText()
        for line in new_lines:
            if level != "ALL" and f"[{level}]" not in line:
                continue
            color = (COLOR_BAD if "[ERROR]" in line or "[FATAL]" in line
                     else COLOR_WARN if "[WARN]" in line else "#dddddd")
            self.txt_log.append(f'<span style="color:{color}">{line}</span>')

    def closeEvent(self, event):
        if self.node:
            self.node.set_control_enabled(False)
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = MonitorClient()

    app = QApplication(sys.argv)
    window = Dashboard(node)
    window.show()

    spin = QTimer()
    spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin.start(SPIN_MS)

    exit_code = app.exec_()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    # ROS 없이 레이아웃만 확인 (node=None이면 refresh가 즉시 return).
    # 실제 실행은 ros2 run monitor_pjt dashboard (-> main()).
    app = QApplication(sys.argv)
    Dashboard(None).show()
    sys.exit(app.exec_())
