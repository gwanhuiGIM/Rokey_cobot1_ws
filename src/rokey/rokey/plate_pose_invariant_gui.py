#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BMW M styled GUI for the pose-invariant plate load monitor.

This GUI is read-only with respect to robot motion.  It captures empty/center
calibration, displays the estimated load point in plate and BASE coordinates,
and visualizes the plate's actual tilt relative to the calibration pose.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import tkinter.font as tkfont

try:
    from .plate_pose_invariant_monitor import (
        CALIBRATION_DURATION_SEC,
        DEFAULT_LOG_DIR,
        DISPLAY_HZ,
        MAX_CALIBRATION_ORIENTATION_DRIFT_DEG,
        MAX_CALIBRATION_POSITION_DRIFT_MM,
        MAX_TCP_ANGULAR_SPEED_DEG_S,
        MAX_TCP_LINEAR_SPEED_MM_S,
        MEDIAN_WINDOW,
        PLATE_RADIUS_MM,
        SAMPLE_HZ,
        StaticWrenchModel,
        WRENCH_FILTER_TIME_CONSTANT_SEC,
        object_coordinate_frames,
        orientation_difference_deg,
        read_tcp_pose_base,
        read_tcp_velocity_base,
        read_wrench_base,
        robust_mean,
        rotation_matrix_zyz_deg,
        save_calibration,
    )
except ImportError:
    from plate_pose_invariant_monitor import (
        CALIBRATION_DURATION_SEC,
        DEFAULT_LOG_DIR,
        DISPLAY_HZ,
        MAX_CALIBRATION_ORIENTATION_DRIFT_DEG,
        MAX_CALIBRATION_POSITION_DRIFT_MM,
        MAX_TCP_ANGULAR_SPEED_DEG_S,
        MAX_TCP_LINEAR_SPEED_MM_S,
        MEDIAN_WINDOW,
        PLATE_RADIUS_MM,
        SAMPLE_HZ,
        StaticWrenchModel,
        WRENCH_FILTER_TIME_CONSTANT_SEC,
        object_coordinate_frames,
        orientation_difference_deg,
        read_tcp_pose_base,
        read_tcp_velocity_base,
        read_wrench_base,
        robust_mean,
        rotation_matrix_zyz_deg,
        save_calibration,
    )


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "plate_pose_invariant_gui"
GUI_REFRESH_MS = int(round(1000.0 / max(DISPLAY_HZ, 1.0)))

# BMW M design tokens adapted from DESIGN-bmw-m.md.
BMW = {
    "canvas": "#000000",
    "surface_soft": "#0d0d0d",
    "surface": "#1a1a1a",
    "elevated": "#262626",
    "hairline": "#3c3c3c",
    "white": "#ffffff",
    "body": "#bbbbbb",
    "strong": "#e6e6e6",
    "muted": "#7e7e7e",
    "blue_light": "#0066b1",
    "blue": "#1c69d4",
    "red": "#e22718",
    "warning": "#f4b400",
    "success": "#0fa336",
}


def relative_tilt_tool_deg(
    reference_pose: Sequence[float],
    current_pose: Sequence[float],
) -> Tuple[float, float, float, float, float]:
    """Return local Tool-Y/Z tilt and the low-side direction in the plate plane."""
    reference = np.asarray(reference_pose, dtype=float)
    current = np.asarray(current_pose, dtype=float)
    rotation_reference = rotation_matrix_zyz_deg(reference[3:])
    rotation_current = rotation_matrix_zyz_deg(current[3:])
    relative = rotation_reference.T @ rotation_current
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1.0e-9:
        rotation_vector_deg = np.zeros(3)
    else:
        axis = np.asarray(
            [
                relative[2, 1] - relative[1, 2],
                relative[0, 2] - relative[2, 0],
                relative[1, 0] - relative[0, 1],
            ]
        ) / (2.0 * math.sin(angle))
        rotation_vector_deg = axis * math.degrees(angle)

    tilt_y = float(rotation_vector_deg[1])
    tilt_z = float(rotation_vector_deg[2])
    magnitude = math.hypot(tilt_y, tilt_z)
    # For plate normal +X, delta_normal = omega x X = (0, omega_z, -omega_y).
    low_y = tilt_z
    low_z = -tilt_y
    return tilt_y, tilt_z, magnitude, low_y, low_z


class MonitorBackend:
    def __init__(self, node: Any, dsr: Any, events: queue.Queue) -> None:
        self.node = node
        self.dsr = dsr
        self.events = events
        self.commands: queue.Queue = queue.Queue()
        self.running = True
        self.monitoring = False

        self.empty_wrench: Optional[np.ndarray] = None
        self.empty_pose: Optional[np.ndarray] = None
        self.center_wrench: Optional[np.ndarray] = None
        self.reference_pose: Optional[np.ndarray] = None
        self.model: Optional[StaticWrenchModel] = None
        self.calibration_path: Optional[Path] = None

        self.wrench_history: deque = deque(maxlen=MEDIAN_WINDOW)
        self.filtered_wrench: Optional[np.ndarray] = None
        self.previous_time: Optional[float] = None

    def emit(self, event: str, **payload: Any) -> None:
        self.events.put({"event": event, **payload})

    def log(self, text: str, level: str = "info") -> None:
        self.emit("log", text=text, level=level, timestamp=time.time())

    def capture(self, label: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        count = max(10, int(round(CALIBRATION_DURATION_SEC * SAMPLE_HZ)))
        samples = []
        period = 1.0 / SAMPLE_HZ
        self.emit("busy", value=True, text=f"{label.upper()} CAPTURE")
        self.log(f"{label} 측정을 시작합니다. 로봇과 물체를 움직이지 마세요.")
        for index in range(count):
            if not self.running:
                raise RuntimeError("종료 요청")
            start = time.monotonic()
            samples.append(read_wrench_base(self.dsr))
            self.emit("progress", value=(index + 1) / count)
            remaining = period - (time.monotonic() - start)
            if remaining > 0.0:
                time.sleep(remaining)
        array = np.asarray(samples, dtype=float)
        mean = robust_mean(array)
        std = np.std(array, axis=0, ddof=1)
        pose = read_tcp_pose_base(self.dsr)
        self.emit("busy", value=False, text="READY")
        self.emit("progress", value=0.0)
        return mean, std, pose

    def capture_empty(self) -> None:
        self.monitoring = False
        mean, std, pose = self.capture("empty plate")
        self.empty_wrench = mean
        self.empty_pose = pose
        self.center_wrench = None
        self.reference_pose = None
        self.model = None
        self.log("빈 plate 기준 측정 완료.", "success")
        self.emit(
            "calibration",
            empty_ready=True,
            center_ready=False,
            model_ready=False,
            empty_std=std.tolist(),
        )

    def capture_center(self) -> None:
        if self.empty_wrench is None or self.empty_pose is None:
            raise RuntimeError("먼저 EMPTY PLATE를 측정하세요.")
        self.monitoring = False
        center, std, center_pose = self.capture("center object")
        position_drift = float(np.linalg.norm(center_pose[:3] - self.empty_pose[:3]))
        orientation_drift = orientation_difference_deg(self.empty_pose, center_pose)
        if (
            position_drift > MAX_CALIBRATION_POSITION_DRIFT_MM
            or orientation_drift > MAX_CALIBRATION_ORIENTATION_DRIFT_DEG
        ):
            raise RuntimeError(
                "캘리브레이션 사이 로봇 자세가 바뀌었습니다: "
                f"{position_drift:.2f} mm / {orientation_drift:.2f} deg"
            )
        self.center_wrench = center
        self.reference_pose = self.empty_pose.copy()
        self.model = StaticWrenchModel.calibrate(
            self.empty_wrench,
            self.center_wrench,
            self.reference_pose,
        )
        self.calibration_path = save_calibration(
            self.model,
            self.empty_wrench,
            self.center_wrench,
            self.reference_pose,
            None,
        )
        self.filtered_wrench = None
        self.wrench_history.clear()
        self.log(
            f"자세 불변 모델 생성 완료 · {self.calibration_path.name}",
            "success",
        )
        self.emit(
            "calibration",
            empty_ready=True,
            center_ready=True,
            model_ready=True,
            center_std=std.tolist(),
            path=str(self.calibration_path),
        )

    def load_model(self, path_string: str) -> None:
        data = json.loads(Path(path_string).read_text(encoding="utf-8"))
        self.model = StaticWrenchModel.from_dict(data["model"])
        self.reference_pose = np.asarray(data["reference_pose"], dtype=float)
        self.empty_wrench = np.asarray(data["empty_wrench_base"], dtype=float)
        self.center_wrench = np.asarray(data["center_wrench_base"], dtype=float)
        self.empty_pose = self.reference_pose.copy()
        self.calibration_path = Path(path_string)
        self.filtered_wrench = None
        self.wrench_history.clear()
        self.log(f"캘리브레이션 로드 · {self.calibration_path.name}", "success")
        self.emit(
            "calibration",
            empty_ready=True,
            center_ready=True,
            model_ready=True,
            path=str(self.calibration_path),
        )

    def process_command(self, command: Tuple[str, Any]) -> None:
        name, payload = command
        if name == "empty":
            self.capture_empty()
        elif name == "center":
            self.capture_center()
        elif name == "load":
            self.load_model(str(payload))
        elif name == "start":
            if self.model is None:
                raise RuntimeError("EMPTY/CENTER 캘리브레이션 또는 모델 로드가 필요합니다.")
            self.monitoring = True
            self.log("실시간 모니터링 시작.", "success")
            self.emit("monitoring", value=True)
        elif name == "pause":
            self.monitoring = False
            self.log("모니터링 일시정지.")
            self.emit("monitoring", value=False)
        elif name == "shutdown":
            self.running = False
        else:
            raise RuntimeError(f"알 수 없는 명령: {name}")

    def sample_state(self) -> Dict[str, Any]:
        assert self.model is not None
        assert self.reference_pose is not None
        now = time.monotonic()
        wrench = read_wrench_base(self.dsr)
        pose = read_tcp_pose_base(self.dsr)
        velocity = read_tcp_velocity_base(self.dsr)
        self.wrench_history.append(wrench)
        median = np.median(np.asarray(self.wrench_history), axis=0)
        dt = now - self.previous_time if self.previous_time is not None else 1.0 / SAMPLE_HZ
        alpha = 1.0 - math.exp(-max(dt, 1.0e-4) / WRENCH_FILTER_TIME_CONSTANT_SEC)
        self.filtered_wrench = (
            median.copy()
            if self.filtered_wrench is None
            else alpha * median + (1.0 - alpha) * self.filtered_wrench
        )
        self.previous_time = now

        estimate = self.model.estimate(self.filtered_wrench, pose)
        linear_speed = float(np.linalg.norm(velocity[:3]))
        angular_speed = float(np.linalg.norm(velocity[3:]))
        stationary = (
            linear_speed <= MAX_TCP_LINEAR_SPEED_MM_S
            and angular_speed <= MAX_TCP_ANGULAR_SPEED_DEG_S
        )
        plate_xyz = np.full(3, math.nan)
        tool_xyz = np.full(3, math.nan)
        base_xyz = np.full(3, math.nan)
        if estimate.valid:
            plate_xyz, tool_xyz, base_xyz = object_coordinate_frames(estimate, pose)

        tilt_y, tilt_z, tilt_mag, low_y, low_z = relative_tilt_tool_deg(
            self.reference_pose,
            pose,
        )
        return {
            "valid": estimate.valid,
            "reason": estimate.reason,
            "stationary": stationary,
            "plate_xyz": plate_xyz.tolist(),
            "tool_xyz": tool_xyz.tolist(),
            "base_xyz": base_xyz.tolist(),
            "radius_mm": estimate.radius_mm,
            "angle_deg": estimate.angle_deg,
            "normal_force_ratio": estimate.normal_force_ratio,
            "force_residual_n": estimate.force_residual_n,
            "wrench_base": self.filtered_wrench.tolist(),
            "tcp_pose": pose.tolist(),
            "linear_speed": linear_speed,
            "angular_speed": angular_speed,
            "tilt_y_deg": tilt_y,
            "tilt_z_deg": tilt_z,
            "tilt_magnitude_deg": tilt_mag,
            "low_y": low_y,
            "low_z": low_z,
            "timestamp": time.time(),
        }

    def run(self) -> None:
        self.log("MONITOR BACKEND ONLINE · READ-ONLY")
        period = 1.0 / SAMPLE_HZ
        while self.running:
            start = time.monotonic()
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                command = None
            if command is not None:
                try:
                    self.process_command(command)
                except Exception as error:
                    self.emit("busy", value=False, text="FAULT")
                    self.emit("error", text=str(error))
                    self.log(str(error), "error")

            if self.monitoring and self.model is not None:
                try:
                    self.emit("state", state=self.sample_state())
                except Exception as error:
                    self.monitoring = False
                    self.emit("monitoring", value=False)
                    self.emit("error", text=f"모니터링 오류: {error}")
                    self.log(f"모니터링 오류: {error}", "error")

            remaining = period - (time.monotonic() - start)
            if remaining > 0.0:
                time.sleep(remaining)


class BMWMonitorApp(tk.Tk):
    def __init__(self, backend: MonitorBackend, events: queue.Queue) -> None:
        super().__init__()
        self.backend = backend
        self.events = events
        self.latest_state: Optional[Dict[str, Any]] = None
        self.model_ready = False
        self.empty_ready = False
        self.center_ready = False
        self.monitoring = False
        self.busy = False
        self.closing = False

        self.title("PLATE LOAD MONITOR / M0609")
        self.geometry("1480x900")
        self.minsize(1180, 760)
        self.configure(bg=BMW["canvas"])
        self.font_family = self._select_font()
        self._configure_style()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(50, self._poll_events)

    def _select_font(self) -> str:
        families = set(tkfont.families(self))
        for family in ("BMW Type Next Latin", "Inter", "DejaVu Sans"):
            if family in families:
                return family
        return "TkDefaultFont"

    def _configure_style(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=BMW["canvas"], foreground=BMW["body"], font=(self.font_family, 10))
        style.configure("TFrame", background=BMW["canvas"])
        style.configure("Card.TFrame", background=BMW["surface"], bordercolor=BMW["hairline"], relief="solid", borderwidth=1)
        style.configure("TLabel", background=BMW["canvas"], foreground=BMW["body"])
        style.configure("Card.TLabel", background=BMW["surface"], foreground=BMW["body"])
        style.configure("CardTitle.TLabel", background=BMW["surface"], foreground=BMW["white"], font=(self.font_family, 12, "bold"))
        style.configure("Metric.TLabel", background=BMW["surface"], foreground=BMW["white"], font=(self.font_family, 19, "bold"))
        style.configure("Muted.TLabel", background=BMW["surface"], foreground=BMW["muted"], font=(self.font_family, 9))
        style.configure("TProgressbar", background=BMW["blue"], troughcolor=BMW["elevated"], borderwidth=0)

    def _stripe(self, parent: tk.Widget) -> tk.Frame:
        stripe = tk.Frame(parent, bg=BMW["canvas"], height=4)
        stripe.pack(fill="x")
        stripe.pack_propagate(False)
        for color in (BMW["blue_light"], BMW["blue"], BMW["red"]):
            tk.Frame(stripe, bg=color, width=76).pack(side="left", fill="y")
        tk.Frame(stripe, bg=BMW["hairline"]).pack(side="left", fill="both", expand=True)
        return stripe

    def _button(self, parent: tk.Widget, text: str, command: Any, danger: bool = False) -> tk.Button:
        color = BMW["red"] if danger else BMW["white"]
        return tk.Button(
            parent,
            text=text.upper(),
            command=command,
            bg=BMW["surface"],
            fg=color,
            activebackground=BMW["elevated"],
            activeforeground=BMW["white"],
            disabledforeground=BMW["muted"],
            relief="solid",
            bd=1,
            highlightthickness=0,
            font=(self.font_family, 10, "bold"),
            padx=16,
            pady=11,
            cursor="hand2",
        )

    def _build_ui(self) -> None:
        header = tk.Frame(self, bg=BMW["canvas"], height=78)
        header.pack(fill="x", padx=24, pady=(18, 0))
        title_col = tk.Frame(header, bg=BMW["canvas"])
        title_col.pack(side="left", fill="y")
        tk.Label(
            title_col,
            text="PLATE LOAD MONITOR",
            bg=BMW["canvas"], fg=BMW["white"],
            font=(self.font_family, 25, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_col,
            text="POSE-INVARIANT WRENCH ANALYSIS  /  DOOSAN M0609 + RG2",
            bg=BMW["canvas"], fg=BMW["muted"],
            font=(self.font_family, 9),
        ).pack(anchor="w", pady=(5, 0))

        status_col = tk.Frame(header, bg=BMW["canvas"])
        status_col.pack(side="right", fill="y")
        self.status_badge = tk.Label(
            status_col, text="STANDBY", bg=BMW["elevated"], fg=BMW["white"],
            font=(self.font_family, 10, "bold"), padx=18, pady=10,
        )
        self.status_badge.pack(side="right", pady=9)
        self.connection_label = tk.Label(
            status_col, text="READ-ONLY / NO MOTION", bg=BMW["canvas"], fg=BMW["body"],
            font=(self.font_family, 9), padx=18,
        )
        self.connection_label.pack(side="right")

        self._stripe(self)

        main = tk.Frame(self, bg=BMW["canvas"])
        main.pack(fill="both", expand=True, padx=24, pady=20)
        main.grid_columnconfigure(0, weight=5, uniform="main")
        main.grid_columnconfigure(1, weight=5, uniform="main")
        main.grid_columnconfigure(2, weight=4, uniform="main")
        main.grid_rowconfigure(0, weight=1)

        self._build_position_card(main)
        self._build_tilt_card(main)
        self._build_side_card(main)

        footer = tk.Frame(self, bg=BMW["surface_soft"], height=34)
        footer.pack(fill="x")
        self.footer_label = tk.Label(
            footer,
            text=f"SAMPLE {SAMPLE_HZ:.0f} HZ  ·  DISPLAY {DISPLAY_HZ:.0f} HZ  ·  PLATE Ø {PLATE_RADIUS_MM * 2:.0f} MM",
            bg=BMW["surface_soft"], fg=BMW["muted"], font=(self.font_family, 8),
        )
        self.footer_label.pack(side="left", padx=24, pady=9)

    def _card_header(self, parent: tk.Widget, index: str, title: str, caption: str) -> None:
        header = tk.Frame(parent, bg=BMW["surface"])
        header.pack(fill="x", padx=20, pady=(18, 10))
        tk.Label(header, text=index, bg=BMW["surface"], fg=BMW["blue"], font=(self.font_family, 10, "bold")).pack(anchor="w")
        tk.Label(header, text=title.upper(), bg=BMW["surface"], fg=BMW["white"], font=(self.font_family, 17, "bold")).pack(anchor="w", pady=(2, 0))
        tk.Label(header, text=caption, bg=BMW["surface"], fg=BMW["muted"], font=(self.font_family, 9)).pack(anchor="w", pady=(4, 0))

    def _build_position_card(self, parent: tk.Widget) -> None:
        card = tk.Frame(parent, bg=BMW["surface"], highlightbackground=BMW["hairline"], highlightthickness=1)
        card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self._card_header(card, "01", "Load position", "PLATE-CENTER FRAME / X NORMAL · Y–Z SURFACE")
        self.position_canvas = tk.Canvas(card, bg=BMW["surface_soft"], highlightthickness=0)
        self.position_canvas.pack(fill="both", expand=True, padx=20, pady=8)
        metrics = tk.Frame(card, bg=BMW["surface"])
        metrics.pack(fill="x", padx=20, pady=(8, 18))
        self.pos_vars = {axis: tk.StringVar(value="--") for axis in ("X", "Y", "Z", "R")}
        for axis in ("X", "Y", "Z", "R"):
            cell = tk.Frame(metrics, bg=BMW["surface_soft"], highlightbackground=BMW["hairline"], highlightthickness=1)
            cell.pack(side="left", fill="x", expand=True, padx=(0, 6) if axis != "R" else 0)
            tk.Label(cell, text=axis, bg=BMW["surface_soft"], fg=BMW["muted"], font=(self.font_family, 8, "bold")).pack(anchor="w", padx=10, pady=(8, 0))
            tk.Label(cell, textvariable=self.pos_vars[axis], bg=BMW["surface_soft"], fg=BMW["white"], font=(self.font_family, 14, "bold")).pack(anchor="w", padx=10, pady=(2, 8))

    def _build_tilt_card(self, parent: tk.Widget) -> None:
        card = tk.Frame(parent, bg=BMW["surface"], highlightbackground=BMW["hairline"], highlightthickness=1)
        card.grid(row=0, column=1, sticky="nsew", padx=8)
        self._card_header(card, "02", "Plate attitude", "ACTUAL TILT FROM CALIBRATION POSE / LOW-SIDE VECTOR")
        self.tilt_canvas = tk.Canvas(card, bg=BMW["surface_soft"], highlightthickness=0)
        self.tilt_canvas.pack(fill="both", expand=True, padx=20, pady=8)
        metrics = tk.Frame(card, bg=BMW["surface"])
        metrics.pack(fill="x", padx=20, pady=(8, 18))
        self.tilt_vars = {axis: tk.StringVar(value="--") for axis in ("TOOL Y", "TOOL Z", "MAG")}
        for axis in ("TOOL Y", "TOOL Z", "MAG"):
            cell = tk.Frame(metrics, bg=BMW["surface_soft"], highlightbackground=BMW["hairline"], highlightthickness=1)
            cell.pack(side="left", fill="x", expand=True, padx=(0, 6) if axis != "MAG" else 0)
            tk.Label(cell, text=axis, bg=BMW["surface_soft"], fg=BMW["muted"], font=(self.font_family, 8, "bold")).pack(anchor="w", padx=10, pady=(8, 0))
            tk.Label(cell, textvariable=self.tilt_vars[axis], bg=BMW["surface_soft"], fg=BMW["white"], font=(self.font_family, 14, "bold")).pack(anchor="w", padx=10, pady=(2, 8))

    def _build_side_card(self, parent: tk.Widget) -> None:
        card = tk.Frame(parent, bg=BMW["surface"], highlightbackground=BMW["hairline"], highlightthickness=1)
        card.grid(row=0, column=2, sticky="nsew", padx=(8, 0))
        self._card_header(card, "03", "System control", "CALIBRATION · MONITOR · DIAGNOSTICS")

        buttons = tk.Frame(card, bg=BMW["surface"])
        buttons.pack(fill="x", padx=18, pady=(0, 10))
        self.empty_button = self._button(buttons, "01 Empty plate", lambda: self._command("empty"))
        self.empty_button.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=4)
        self.center_button = self._button(buttons, "02 Center object", lambda: self._command("center"))
        self.center_button.grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=4)
        self.load_button = self._button(buttons, "Load model", self._load_model)
        self.load_button.grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=4)
        self.start_button = self._button(buttons, "Start monitor", lambda: self._command("start"))
        self.start_button.grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=4)
        self.pause_button = self._button(buttons, "Pause", lambda: self._command("pause"))
        self.pause_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=4)
        buttons.grid_columnconfigure(0, weight=1)
        buttons.grid_columnconfigure(1, weight=1)

        self.progress = ttk.Progressbar(card, mode="determinate", maximum=1.0)
        self.progress.pack(fill="x", padx=18, pady=(2, 14))

        state_box = tk.Frame(card, bg=BMW["surface_soft"], highlightbackground=BMW["hairline"], highlightthickness=1)
        state_box.pack(fill="x", padx=18, pady=(0, 12))
        self.state_vars = {
            "MODEL": tk.StringVar(value="NOT READY"),
            "ESTIMATE": tk.StringVar(value="--"),
            "MOTION": tk.StringVar(value="--"),
            "BASE XYZ": tk.StringVar(value="--"),
            "FORCE Δ": tk.StringVar(value="--"),
        }
        for row, (label, variable) in enumerate(self.state_vars.items()):
            tk.Label(state_box, text=label, bg=BMW["surface_soft"], fg=BMW["muted"], font=(self.font_family, 8, "bold")).grid(row=row, column=0, sticky="w", padx=12, pady=7)
            tk.Label(state_box, textvariable=variable, bg=BMW["surface_soft"], fg=BMW["strong"], font=(self.font_family, 9), anchor="e").grid(row=row, column=1, sticky="e", padx=12, pady=7)
        state_box.grid_columnconfigure(1, weight=1)

        tk.Label(card, text="EVENT LOG", bg=BMW["surface"], fg=BMW["white"], font=(self.font_family, 9, "bold")).pack(anchor="w", padx=18, pady=(2, 6))
        self.log_list = tk.Listbox(
            card,
            bg=BMW["surface_soft"], fg=BMW["body"],
            selectbackground=BMW["elevated"], selectforeground=BMW["white"],
            highlightbackground=BMW["hairline"], highlightthickness=1,
            relief="flat", font=(self.font_family, 8), height=9,
        )
        self.log_list.pack(fill="both", expand=True, padx=18, pady=(0, 18))
        self._update_buttons()

    def _command(self, name: str, payload: Any = None) -> None:
        self.backend.commands.put((name, payload))

    def _load_model(self) -> None:
        path = filedialog.askopenfilename(
            title="LOAD CALIBRATION",
            initialdir=str(DEFAULT_LOG_DIR),
            filetypes=[("Calibration JSON", "*.json")],
        )
        if path:
            self._command("load", path)

    def _update_buttons(self) -> None:
        self.empty_button.configure(state=tk.DISABLED if self.busy else tk.NORMAL)
        self.center_button.configure(state=tk.NORMAL if self.empty_ready and not self.busy else tk.DISABLED)
        self.start_button.configure(state=tk.NORMAL if self.model_ready and not self.busy else tk.DISABLED)
        self.pause_button.configure(state=tk.NORMAL if self.monitoring and not self.busy else tk.DISABLED)
        self.load_button.configure(state=tk.DISABLED if self.busy else tk.NORMAL)

    def _draw_position(self, state: Optional[Dict[str, Any]]) -> None:
        canvas = self.position_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 360)
        height = max(canvas.winfo_height(), 360)
        cx, cy = width / 2.0, height / 2.0
        radius_px = max(80.0, min(width, height) / 2.0 - 38.0)
        canvas.create_oval(cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px, outline=BMW["white"], width=2)
        for ratio in (0.5, 0.8):
            r = radius_px * ratio
            canvas.create_oval(cx-r, cy-r, cx+r, cy+r, outline=BMW["hairline"], width=1)
        canvas.create_line(cx-radius_px, cy, cx+radius_px, cy, fill=BMW["hairline"], width=1)
        canvas.create_line(cx, cy-radius_px, cx, cy+radius_px, fill=BMW["hairline"], width=1)
        canvas.create_text(cx+radius_px-4, cy-12, text="+Y", fill=BMW["body"], anchor="e", font=(self.font_family, 9, "bold"))
        canvas.create_text(cx+10, cy-radius_px+4, text="+Z", fill=BMW["body"], anchor="nw", font=(self.font_family, 9, "bold"))
        canvas.create_oval(cx-4, cy-4, cx+4, cy+4, fill=BMW["muted"], outline="")
        if not state or not state["valid"]:
            canvas.create_text(cx, cy, text="NO VALID LOAD POSITION", fill=BMW["muted"], font=(self.font_family, 11, "bold"))
            return
        y_mm, z_mm = state["plate_xyz"][1], state["plate_xyz"][2]
        scale = radius_px / max(PLATE_RADIUS_MM, 1.0)
        px = cx + max(-PLATE_RADIUS_MM * 1.2, min(PLATE_RADIUS_MM * 1.2, y_mm)) * scale
        py = cy - max(-PLATE_RADIUS_MM * 1.2, min(PLATE_RADIUS_MM * 1.2, z_mm)) * scale
        color = BMW["red"] if state["radius_mm"] > PLATE_RADIUS_MM else BMW["blue"]
        canvas.create_line(cx, cy, px, py, fill=color, width=2, arrow=tk.LAST)
        canvas.create_oval(px-11, py-11, px+11, py+11, fill=color, outline=BMW["white"], width=2)
        canvas.create_text(px, py-21, text="LOAD", fill=BMW["white"], font=(self.font_family, 8, "bold"))

    def _draw_tilt(self, state: Optional[Dict[str, Any]]) -> None:
        canvas = self.tilt_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 360)
        height = max(canvas.winfo_height(), 360)
        cx, cy = width / 2.0, height / 2.0
        radius = max(80.0, min(width, height) / 2.0 - 44.0)
        canvas.create_oval(cx-radius, cy-radius, cx+radius, cy+radius, outline=BMW["white"], width=2)
        canvas.create_line(cx-radius, cy, cx+radius, cy, fill=BMW["hairline"])
        canvas.create_line(cx, cy-radius, cx, cy+radius, fill=BMW["hairline"])
        canvas.create_text(cx+radius-4, cy-12, text="+Y", fill=BMW["body"], anchor="e", font=(self.font_family, 9, "bold"))
        canvas.create_text(cx+10, cy-radius+4, text="+Z", fill=BMW["body"], anchor="nw", font=(self.font_family, 9, "bold"))
        if not state:
            canvas.create_text(cx, cy, text="REFERENCE POSE REQUIRED", fill=BMW["muted"], font=(self.font_family, 11, "bold"))
            return
        low_y, low_z = state["low_y"], state["low_z"]
        magnitude = state["tilt_magnitude_deg"]
        norm = math.hypot(low_y, low_z)
        if norm < 1.0e-5:
            canvas.create_oval(cx-7, cy-7, cx+7, cy+7, outline=BMW["success"], width=2)
            canvas.create_text(cx, cy+28, text="LEVEL", fill=BMW["success"], font=(self.font_family, 10, "bold"))
            return
        ux, uz = low_y / norm, low_z / norm
        arrow_len = radius * min(1.0, 0.25 + magnitude / 5.0)
        low_x, low_screen_y = cx + ux * arrow_len, cy - uz * arrow_len
        high_x, high_screen_y = cx - ux * arrow_len * 0.72, cy + uz * arrow_len * 0.72
        canvas.create_line(high_x, high_screen_y, low_x, low_screen_y, fill=BMW["red"], width=5, arrow=tk.LAST, arrowshape=(14, 18, 7))
        canvas.create_text(low_x, low_screen_y-16, text="LOW", fill=BMW["red"], font=(self.font_family, 9, "bold"))
        canvas.create_text(high_x, high_screen_y+16, text="HIGH", fill=BMW["blue_light"], font=(self.font_family, 9, "bold"))
        # A compressed ellipse reinforces that this panel represents attitude.
        squash = max(0.28, 1.0 - min(magnitude, 12.0) / 18.0)
        canvas.create_oval(cx-radius*0.55, cy-radius*0.22*squash, cx+radius*0.55, cy+radius*0.22*squash, outline=BMW["body"], width=1)

    def _update_state(self, state: Dict[str, Any]) -> None:
        self.latest_state = state
        valid = bool(state["valid"])
        stationary = bool(state["stationary"])
        plate = state["plate_xyz"]
        base = state["base_xyz"]
        for axis, value in zip(("X", "Y", "Z"), plate):
            self.pos_vars[axis].set(f"{value:+.1f} mm" if math.isfinite(value) else "--")
        self.pos_vars["R"].set(f"{state['radius_mm']:.1f} mm" if math.isfinite(state["radius_mm"]) else "--")
        self.tilt_vars["TOOL Y"].set(f"{state['tilt_y_deg']:+.2f}°")
        self.tilt_vars["TOOL Z"].set(f"{state['tilt_z_deg']:+.2f}°")
        self.tilt_vars["MAG"].set(f"{state['tilt_magnitude_deg']:.2f}°")
        self.state_vars["ESTIMATE"].set("VALID" if valid else state["reason"])
        self.state_vars["MOTION"].set("STATIONARY" if stationary else "MOVING")
        self.state_vars["BASE XYZ"].set(
            " / ".join(f"{v:.1f}" for v in base) if all(math.isfinite(v) for v in base) else "--"
        )
        self.state_vars["FORCE Δ"].set(f"{state['force_residual_n']:.2f} N" if math.isfinite(state["force_residual_n"]) else "--")
        self.status_badge.configure(
            text="MONITORING" if valid else "CHECK SIGNAL",
            bg=BMW["success"] if valid else BMW["warning"],
            fg=BMW["canvas"] if not valid else BMW["white"],
        )
        self._draw_position(state)
        self._draw_tilt(state)

    def _handle_event(self, item: Dict[str, Any]) -> None:
        event = item["event"]
        if event == "state":
            self._update_state(item["state"])
        elif event == "log":
            stamp = datetime.fromtimestamp(item["timestamp"]).strftime("%H:%M:%S")
            self.log_list.insert(tk.END, f"{stamp}  {item['text']}")
            self.log_list.yview_moveto(1.0)
        elif event == "busy":
            self.busy = bool(item["value"])
            self.status_badge.configure(text=item["text"], bg=BMW["blue"] if item["value"] else BMW["elevated"], fg=BMW["white"])
        elif event == "progress":
            self.progress["value"] = item["value"]
        elif event == "calibration":
            self.empty_ready = item["empty_ready"]
            self.center_ready = item["center_ready"]
            self.model_ready = item["model_ready"]
            self.state_vars["MODEL"].set("READY" if self.model_ready else "NOT READY")
            if self.model_ready:
                self.status_badge.configure(text="READY", bg=BMW["success"], fg=BMW["white"])
        elif event == "monitoring":
            self.monitoring = bool(item["value"])
            self.status_badge.configure(text="MONITORING" if self.monitoring else "PAUSED", bg=BMW["success"] if self.monitoring else BMW["elevated"])
        elif event == "error":
            self.status_badge.configure(text="FAULT", bg=BMW["red"], fg=BMW["white"])
            messagebox.showerror("PLATE MONITOR", item["text"])
        self._update_buttons()

    def _poll_events(self) -> None:
        if self.closing:
            return
        latest_state = None
        for _ in range(200):
            try:
                item = self.events.get_nowait()
            except queue.Empty:
                break
            if item["event"] == "state":
                latest_state = item
            else:
                self._handle_event(item)
        if latest_state is not None:
            self._handle_event(latest_state)
        self.after(50, self._poll_events)

    def _on_close(self) -> None:
        self.closing = True
        self._command("shutdown")
        self.destroy()


def main(args=None) -> None:
    import rclpy
    import DR_init

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    import DSR_ROBOT2 as dsr

    events: queue.Queue = queue.Queue()
    backend = MonitorBackend(node, dsr, events)
    app = BMWMonitorApp(backend, events)
    backend_thread = threading.Thread(target=backend.run, daemon=True)
    backend_thread.start()
    try:
        app.mainloop()
    finally:
        backend.running = False
        backend_thread.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
