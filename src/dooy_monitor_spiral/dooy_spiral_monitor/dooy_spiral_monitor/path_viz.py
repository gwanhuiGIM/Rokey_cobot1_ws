#!/usr/bin/env python3
"""핸드드립 주둥이 실시간 경로 시각화 패널.

`kettle_circle_pour.py --viz`에서 별도 프로세스로 실행되며, 두산 로봇의
GetCurrentPosx 서비스를 주기적으로 읽는다. 측정 TCP에 주둥이 오프셋을 적용한
실제 주둥이 경로와 필터 상단 경계를 표시한다. 이 모듈은 읽기 전용이며 로봇 명령을
보내지 않는다.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from collections import deque
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np
import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import GetCurrentPosx

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.font_manager import FontProperties


def _load_korean_plot_font() -> FontProperties:
    """Matplotlib 자체 폰트 캐시를 거치지 않고 한글 TTC를 직접 읽는다."""
    candidates = (
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"),
    )
    for font_path in candidates:
        if font_path.is_file():
            return FontProperties(fname=str(font_path))
    # Noto CJK가 없는 환경에서도 패널 자체는 실행되도록 기본 폰트로 폴백.
    return FontProperties(family="sans-serif")


KOREAN_PLOT_FONT = _load_korean_plot_font()


DEFAULT_NAMESPACE = "dsr01"
DEFAULT_SPOUT_OFFSET_MM = [0.0, 150.0, 10.0]
DEFAULT_FILTER_TOP_RADIUS_MM = 60.0
DEFAULT_FILTER_BOTTOM_RADIUS_MM = 15.0
DEFAULT_FILTER_DEPTH_MM = 70.0
DEFAULT_POUR_RADIUS_MM = 30.0
DEFAULT_SAMPLE_HZ = 20.0
DEFAULT_MAX_POINTS = 3000


def _rz(deg: float) -> np.ndarray:
    rad = math.radians(deg)
    cos_value, sin_value = math.cos(rad), math.sin(rad)
    return np.array([
        [cos_value, -sin_value, 0.0],
        [sin_value, cos_value, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _ry(deg: float) -> np.ndarray:
    rad = math.radians(deg)
    cos_value, sin_value = math.cos(rad), math.sin(rad)
    return np.array([
        [cos_value, 0.0, sin_value],
        [0.0, 1.0, 0.0],
        [-sin_value, 0.0, cos_value],
    ])


def zyz_to_matrix(a: float, b: float, c: float) -> np.ndarray:
    """Doosan posx ZYZ 자세를 BASE←TOOL 회전행렬로 변환한다."""
    return _rz(a) @ _ry(b) @ _rz(c)


def spout_position(pose: Sequence[float],
                   spout_offset_mm: Sequence[float]) -> np.ndarray:
    """활성 TCP posx와 TOOL 오프셋으로 주둥이 끝의 BASE XYZ를 계산한다."""
    pose_array = np.asarray(pose, dtype=float)
    offset = np.asarray(spout_offset_mm, dtype=float)
    return pose_array[:3] + zyz_to_matrix(*pose_array[3:6]) @ offset


class TcpPosePoller(Node):
    """GetCurrentPosx를 비동기로 폴링하는 읽기 전용 ROS2 노드."""

    def __init__(
            self,
            namespace: str,
            sample_hz: float,
            pose_callback: Callable[[np.ndarray, float], None],
            status_callback: Callable[[str, bool], None]):
        super().__init__("hand_drip_path_viz", namespace=namespace)
        self._pose_callback = pose_callback
        self._status_callback = status_callback
        self._pending = False
        self._service_announced = False
        self._client = self.create_client(
            GetCurrentPosx,
            f"/{namespace}/aux_control/get_current_posx",
        )
        self._timer = self.create_timer(
            1.0 / max(1.0, sample_hz),
            self._request_pose,
        )

    def _request_pose(self) -> None:
        if self._pending:
            return
        if not self._client.service_is_ready():
            self._status_callback(
                "TCP 서비스 대기 중: "
                f"/{self.get_namespace().strip('/')}/aux_control/"
                "get_current_posx",
                False,
            )
            return
        if not self._service_announced:
            self._service_announced = True
            self._status_callback("TCP 서비스 연결됨 · 실시간 추적 중", True)

        request = GetCurrentPosx.Request()
        request.ref = 0  # DR_BASE
        self._pending = True
        future = self._client.call_async(request)
        future.add_done_callback(self._on_pose)

    def _on_pose(self, future) -> None:
        self._pending = False
        try:
            response = future.result()
            if not response.task_pos_info:
                raise RuntimeError("응답에 task_pos_info가 없습니다")
            values = np.asarray(
                response.task_pos_info[0].data[:6],
                dtype=float,
            )
            if values.shape != (6,) or not np.all(np.isfinite(values)):
                raise RuntimeError("유효하지 않은 TCP 포즈입니다")
            self._pose_callback(values, time.monotonic())
        except Exception as error:
            self._status_callback(f"TCP 읽기 실패: {error}", False)


class PathVizWindow(QMainWindow):
    """필터 형상과 주둥이 실시간 경로를 표시하는 Qt 패널."""

    def __init__(
            self,
            spout_offset_mm: Sequence[float],
            filter_center_mm: Optional[Sequence[float]],
            filter_top_radius_mm: float,
            filter_bottom_radius_mm: float,
            filter_depth_mm: float,
            pour_radius_mm: float,
            max_points: int):
        super().__init__()
        self.setWindowTitle("HAND DRIP · REAL-TIME SPOUT PATH")
        self.resize(1280, 760)

        self.spout_offset = np.asarray(spout_offset_mm, dtype=float)
        self.filter_center = (
            None if filter_center_mm is None
            else np.asarray(filter_center_mm, dtype=float)
        )
        self.filter_top_radius = float(filter_top_radius_mm)
        self.filter_bottom_radius = float(filter_bottom_radius_mm)
        self.filter_depth = float(filter_depth_mm)
        self.pour_radius = float(pour_radius_mm)

        self.spout_path = deque(maxlen=max_points)
        self.timestamps = deque(maxlen=max_points)
        self.start_spout: Optional[np.ndarray] = None
        self.last_pose: Optional[np.ndarray] = None
        self.path_length_mm = 0.0
        self.current_speed_mm_s = 0.0
        self.sample_count = 0
        self._plot_dirty = False

        self._build_ui()
        self._draw_static_plot()

        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self._redraw)
        self.redraw_timer.start(66)  # 약 15 FPS

    def _build_ui(self) -> None:
        central = QWidget()
        central.setStyleSheet(
            "QWidget { background: #111827; color: #e5e7eb; }"
            "QLabel#title { color: #f9fafb; font-size: 19px; "
            "font-weight: 700; }"
            "QLabel#status { padding: 6px 10px; border-radius: 5px; }"
            "QLabel[class=\"value\"] { background: #1f2937; "
            "border: 1px solid #374151; "
            "padding: 7px; border-radius: 4px; font-family: monospace; }"
            "QPushButton { background: #374151; padding: 7px 14px; "
            "border-radius: 4px; }"
            "QPushButton:hover { background: #4b5563; }"
        )
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        header = QHBoxLayout()
        title = QLabel("HAND DRIP · SPOUT PATH")
        title.setObjectName("title")
        header.addWidget(title)
        header.addStretch()
        self.status_label = QLabel("ROS 연결 준비 중")
        self.status_label.setObjectName("status")
        header.addWidget(self.status_label)
        clear_button = QPushButton("경로 초기화")
        clear_button.clicked.connect(self.clear_path)
        header.addWidget(clear_button)
        layout.addLayout(header)

        self.figure = Figure(facecolor="#111827", constrained_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax_3d = self.figure.add_subplot(1, 2, 1, projection="3d")
        self.ax_top = self.figure.add_subplot(1, 2, 2)
        layout.addWidget(self.canvas, stretch=1)

        values = QGridLayout()
        self.spout_value = self._value_label()
        self.delta_value = self._value_label()
        self.filter_value = self._value_label()
        self.motion_value = self._value_label()
        self.orientation_value = self._value_label()
        value_items = (
            ("계산 주둥이 BASE XYZ (mm)", self.spout_value),
            ("시작점 대비 계산 주둥이 ΔXYZ (mm)", self.delta_value),
            ("필터 중심 기준 · 반경 / 상단 여유", self.filter_value),
            ("주둥이 이동량 / 속도 / 샘플링", self.motion_value),
            ("TCP 자세 A/B/C (deg)", self.orientation_value),
        )
        for index, (name, widget) in enumerate(value_items):
            row, column = divmod(index, 3)
            box = QVBoxLayout()
            caption = QLabel(name)
            caption.setStyleSheet("color: #9ca3af; font-size: 11px;")
            box.addWidget(caption)
            box.addWidget(widget)
            values.addLayout(box, row, column)
        layout.addLayout(values)

    @staticmethod
    def _value_label() -> QLabel:
        label = QLabel("-")
        label.setProperty("class", "value")
        label.setFont(QFont("DejaVu Sans Mono", 10))
        return label

    def set_status(self, text: str, connected: bool) -> None:
        self.status_label.setText(text)
        color = "#065f46" if connected else "#7c2d12"
        self.status_label.setStyleSheet(
            f"background: {color}; padding: 6px 10px; border-radius: 5px;"
        )

    def add_pose(self, pose: np.ndarray, timestamp: float) -> None:
        spout = spout_position(pose, self.spout_offset)
        if self.filter_center is None:
            self.filter_center = spout.copy()
        if self.start_spout is None:
            self.start_spout = spout.copy()

        if self.spout_path:
            distance = float(np.linalg.norm(spout - self.spout_path[-1]))
            dt = max(timestamp - self.timestamps[-1], 1e-6)
            self.current_speed_mm_s = distance / dt
            # 센서의 정지 노이즈가 누적 이동량을 부풀리지 않도록 0.05 mm 게이트.
            if distance >= 0.05:
                self.path_length_mm += distance

        self.spout_path.append(spout)
        self.timestamps.append(timestamp)
        self.last_pose = pose.copy()
        self.sample_count += 1
        self._plot_dirty = True
        self._update_values(spout, pose)

    def _update_values(
            self, spout: np.ndarray, pose: np.ndarray) -> None:
        delta = spout - self.start_spout
        filter_delta = spout - self.filter_center
        radial = float(np.linalg.norm(filter_delta[:2]))
        filter_margin = self.filter_top_radius - radial
        self.spout_value.setText(self._xyz_text(spout))
        self.delta_value.setText(self._xyz_text(delta, signed=True))
        self.filter_value.setText(
            f"dX {filter_delta[0]:+7.2f}  dY {filter_delta[1]:+7.2f}  "
            f"R {radial:6.2f}/{self.filter_top_radius:.2f}  "
            f"Margin {filter_margin:+6.2f} mm"
        )
        sample_hz = 0.0
        if len(self.timestamps) >= 2:
            sample_window = min(len(self.timestamps), 31)
            sample_times = list(self.timestamps)[-sample_window:]
            elapsed = sample_times[-1] - sample_times[0]
            if elapsed > 0.0:
                sample_hz = (len(sample_times) - 1) / elapsed
        self.motion_value.setText(
            f"Length {self.path_length_mm:8.2f} mm  ·  "
            f"Speed {self.current_speed_mm_s:7.2f} mm/s  ·  "
            f"{sample_hz:5.1f} Hz / {self.sample_count} samples"
        )
        self.orientation_value.setText(
            f"A {pose[3]:+8.2f}  B {pose[4]:+8.2f}  C {pose[5]:+8.2f}"
        )

    @staticmethod
    def _xyz_text(values: np.ndarray, signed: bool = False) -> str:
        spec = "+8.2f" if signed else "8.2f"
        return (
            f"X {values[0]:{spec}}  Y {values[1]:{spec}}  "
            f"Z {values[2]:{spec}}"
        )

    def clear_path(self) -> None:
        self.spout_path.clear()
        self.timestamps.clear()
        self.start_spout = None
        self.last_pose = None
        self.path_length_mm = 0.0
        self.current_speed_mm_s = 0.0
        self.sample_count = 0
        self._plot_dirty = True

    def _style_axis(self, axis, title: str) -> None:
        axis.set_facecolor("#111827")
        axis.set_title(
            title,
            color="#f9fafb",
            pad=12,
            fontproperties=KOREAN_PLOT_FONT,
        )
        axis.tick_params(colors="#9ca3af")
        for spine in axis.spines.values():
            spine.set_color("#4b5563")
        axis.grid(True, color="#374151", alpha=0.55)

    def _draw_static_plot(self) -> None:
        self.ax_3d.clear()
        self.ax_top.clear()
        self._style_axis(self.ax_3d, "계산 주둥이 경로 · DR_BASE")
        self._style_axis(self.ax_top, "필터 내부 주둥이 경로 · Top View")
        self.ax_3d.set_xlabel("X (mm)", color="#9ca3af")
        self.ax_3d.set_ylabel("Y (mm)", color="#9ca3af")
        self.ax_3d.set_zlabel("Z (mm)", color="#9ca3af")
        self.ax_top.set_xlabel("Filter X (mm)", color="#9ca3af")
        self.ax_top.set_ylabel("Filter Y (mm)", color="#9ca3af")
        self.ax_top.set_aspect("equal", adjustable="box")

    def _draw_filter(self) -> None:
        if self.filter_center is None:
            return
        center = self.filter_center
        angles = np.linspace(0.0, 2.0 * math.pi, 72)
        top_x = center[0] + self.filter_top_radius * np.cos(angles)
        top_y = center[1] + self.filter_top_radius * np.sin(angles)
        top_z = np.full_like(angles, center[2])
        bottom_x = center[0] + self.filter_bottom_radius * np.cos(angles)
        bottom_y = center[1] + self.filter_bottom_radius * np.sin(angles)
        bottom_z = np.full_like(angles, center[2] - self.filter_depth)

        self.ax_3d.plot(
            top_x,
            top_y,
            top_z,
            color="#34d399",
            alpha=0.95,
            linewidth=1.8,
            label="필터 상단",
        )
        self.ax_3d.plot(
            bottom_x, bottom_y, bottom_z,
            color="#6b7280", alpha=0.65, linewidth=1.0,
        )
        for index in range(0, len(angles), 9):
            self.ax_3d.plot(
                [top_x[index], bottom_x[index]],
                [top_y[index], bottom_y[index]],
                [top_z[index], bottom_z[index]],
                color="#6b7280",
                alpha=0.45,
                linewidth=0.8,
            )

        self.ax_top.plot(
            self.filter_top_radius * np.cos(angles),
            self.filter_top_radius * np.sin(angles),
            color="#34d399",
            linewidth=2.0,
            label="필터 상단",
        )
        self.ax_top.fill(
            self.filter_top_radius * np.cos(angles),
            self.filter_top_radius * np.sin(angles),
            color="#34d399",
            alpha=0.08,
        )
        self.ax_top.scatter([0.0], [0.0], color="#f9fafb", s=24, marker="+")

    @staticmethod
    def _set_equal_3d(axis, points: np.ndarray) -> None:
        minimum = np.min(points, axis=0)
        maximum = np.max(points, axis=0)
        center = (minimum + maximum) * 0.5
        radius = max(float(np.max(maximum - minimum)) * 0.55, 40.0)
        axis.set_xlim(center[0] - radius, center[0] + radius)
        axis.set_ylim(center[1] - radius, center[1] + radius)
        axis.set_zlim(center[2] - radius, center[2] + radius)

    def _redraw(self) -> None:
        if not self._plot_dirty:
            return
        self._plot_dirty = False
        self._draw_static_plot()
        self._draw_filter()

        if self.spout_path:
            spout = np.asarray(self.spout_path)
            self.ax_3d.plot(
                spout[:, 0], spout[:, 1], spout[:, 2],
                color="#fb923c", linewidth=2.0, label="계산 주둥이",
            )
            self.ax_3d.scatter(
                [spout[-1, 0]], [spout[-1, 1]], [spout[-1, 2]],
                color="#fed7aa", s=34,
            )

            relative_spout = spout - self.filter_center
            self.ax_top.plot(
                relative_spout[:, 0],
                relative_spout[:, 1],
                color="#fb923c",
                linewidth=2.0,
                label="계산 주둥이",
            )
            self.ax_top.scatter(
                [relative_spout[-1, 0]],
                [relative_spout[-1, 1]],
                color="#fed7aa",
                s=36,
            )

            bounds = [spout]
            if self.filter_center is not None:
                filter_bounds = np.array([
                    self.filter_center
                    + [-self.filter_top_radius, -self.filter_top_radius, 0.0],
                    self.filter_center
                    + [self.filter_top_radius, self.filter_top_radius, 0.0],
                    self.filter_center
                    + [0.0, 0.0, -self.filter_depth],
                ])
                bounds.append(filter_bounds)
            self._set_equal_3d(self.ax_3d, np.vstack(bounds))

        # 실패 재현에서는 주둥이가 필터 밖으로 크게 이탈할 수 있다. 필터
        # 반지름으로 축을 고정하면 MoveC 원 전체가 화면 밖으로 사라지므로,
        # 필터 중심(0, 0)을 유지한 채 지금까지의 경로를 모두 포함해 확장한다.
        top_extent = self.filter_top_radius
        if self.spout_path and self.filter_center is not None:
            relative_spout = (
                np.asarray(self.spout_path) - self.filter_center
            )
            top_extent = max(
                top_extent,
                float(np.max(np.abs(relative_spout[:, :2]))),
            )
        top_limit = max(top_extent * 1.15, self.filter_top_radius * 1.15)
        self.ax_top.set_xlim(-top_limit, top_limit)
        self.ax_top.set_ylim(-top_limit, top_limit)
        self.ax_3d.legend(
            loc="upper left",
            facecolor="#1f2937",
            labelcolor="#e5e7eb",
            prop=KOREAN_PLOT_FONT,
        )
        self.ax_top.legend(
            loc="upper right",
            facecolor="#1f2937",
            labelcolor="#e5e7eb",
            prop=KOREAN_PLOT_FONT,
        )
        self.canvas.draw_idle()


def launch_path_viz(
        namespace: str,
        spout_offset_mm: Sequence[float],
        filter_center_mm: Sequence[float],
        filter_top_radius_mm: float = DEFAULT_FILTER_TOP_RADIUS_MM,
        pour_radius_mm: float = DEFAULT_POUR_RADIUS_MM,
        ) -> subprocess.Popen:
    """붓기 프로그램에서 시각화 모듈을 별도 프로세스로 실행한다."""
    command = [
        sys.executable,
        "-m",
        "dooy_spiral_monitor.path_viz",
        "--namespace",
        namespace,
        "--spout-offset",
        *[str(float(value)) for value in spout_offset_mm],
        "--filter-center",
        *[str(float(value)) for value in filter_center_mm],
        "--filter-radius",
        str(float(filter_top_radius_mm)),
        "--pour-radius",
        str(float(pour_radius_mm)),
    ]
    return subprocess.Popen(command)


def stop_path_viz(process: Optional[subprocess.Popen]) -> None:
    """시각화 프로세스를 정상 종료하고, 지연되면 강제 종료한다."""
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="핸드드립 실시간 주둥이 경로 패널")
    parser.add_argument("--namespace", default=DEFAULT_NAMESPACE)
    parser.add_argument(
        "--spout-offset",
        nargs=3,
        type=float,
        default=DEFAULT_SPOUT_OFFSET_MM,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--filter-center",
        nargs=3,
        type=float,
        default=None,
        metavar=("X", "Y", "Z"),
        help="필터 상단 중심의 DR_BASE XYZ(mm). 생략하면 첫 주둥이 위치 사용",
    )
    parser.add_argument(
        "--filter-radius",
        type=float,
        default=DEFAULT_FILTER_TOP_RADIUS_MM,
    )
    parser.add_argument(
        "--filter-bottom-radius",
        type=float,
        default=DEFAULT_FILTER_BOTTOM_RADIUS_MM,
    )
    parser.add_argument(
        "--filter-depth",
        type=float,
        default=DEFAULT_FILTER_DEPTH_MM,
    )
    parser.add_argument(
        "--pour-radius",
        type=float,
        default=DEFAULT_POUR_RADIUS_MM,
        help="이전 호출부 호환용 붓기 반지름(mm, 목표 원은 표시하지 않음)",
    )
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ)
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    return parser


def main(args=None) -> int:
    parser = build_parser()
    parsed_args, ros_args = parser.parse_known_args(args=args)
    if parsed_args.filter_radius <= 0.0:
        parser.error("--filter-radius는 0보다 커야 합니다")
    if parsed_args.filter_bottom_radius < 0.0:
        parser.error("--filter-bottom-radius는 0 이상이어야 합니다")
    if parsed_args.filter_depth <= 0.0:
        parser.error("--filter-depth는 0보다 커야 합니다")
    if parsed_args.pour_radius <= 0.0:
        parser.error("--pour-radius는 0보다 커야 합니다")
    if parsed_args.sample_hz <= 0.0:
        parser.error("--sample-hz는 0보다 커야 합니다")
    if parsed_args.max_points < 10:
        parser.error("--max-points는 10 이상이어야 합니다")

    rclpy.init(args=ros_args)
    application = QApplication.instance() or QApplication(sys.argv[:1])
    window = PathVizWindow(
        spout_offset_mm=parsed_args.spout_offset,
        filter_center_mm=parsed_args.filter_center,
        filter_top_radius_mm=parsed_args.filter_radius,
        filter_bottom_radius_mm=parsed_args.filter_bottom_radius,
        filter_depth_mm=parsed_args.filter_depth,
        pour_radius_mm=parsed_args.pour_radius,
        max_points=parsed_args.max_points,
    )
    poller = TcpPosePoller(
        namespace=parsed_args.namespace.strip("/"),
        sample_hz=parsed_args.sample_hz,
        pose_callback=window.add_pose,
        status_callback=window.set_status,
    )

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(poller, timeout_sec=0.0))
    ros_timer.start(10)
    window.show()

    try:
        return application.exec_()
    finally:
        ros_timer.stop()
        poller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
