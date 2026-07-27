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
    from tkinter import messagebox, ttk
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
NODE_NAME = "plate_position_gui"

GRIPPER_MODEL = "OnRobot RG2"
PLATE_RADIUS_MM = 100.0
CALIBRATION_RADIUS_MM = 60.0

SAMPLE_HZ = 10.0
SAMPLE_PERIOD_SEC = 1.0 / SAMPLE_HZ

BASELINE_DURATION_SEC = 5.0
CENTER_REFERENCE_DURATION_SEC = 4.0
DIRECTION_REFERENCE_DURATION_SEC = 3.0

EXTERNAL_TORQUE_EMA_ALPHA = 0.20
JOINT_TORQUE_EMA_ALPHA = 0.20
DERIVATIVE_EMA_ALPHA = 0.25

EXTERNAL_TORQUE_NOISE_FLOOR_NM = 0.02

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

GUI_POLL_MS = 50
GUI_HISTORY_SEC = 20.0

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

        self.noise_sigma: Optional[np.ndarray] = None
        self.center_threshold_z = MIN_CENTER_THRESHOLD_Z
        self.object_load_delta_norm_nm = math.nan

        self.direction_points: Dict[str, Dict[str, Any]] = {}
        self.position_model: Optional[PositionModel] = None

        self.filtered_external: Optional[np.ndarray] = None
        self.filtered_joint: Optional[np.ndarray] = None
        self.filtered_external_derivative: Optional[np.ndarray] = None
        self.filtered_joint_derivative: Optional[np.ndarray] = None
        self.previous_filtered_external: Optional[np.ndarray] = None
        self.previous_filtered_joint: Optional[np.ndarray] = None
        self.previous_time: Optional[float] = None

        self.score_history: deque = deque()
        self.center_latched = True
        self.center_enter_count = 0
        self.center_exit_count = 0

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
            monitor_enabled=self.monitor_enabled,
            center_threshold_z=self.center_threshold_z,
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
    ) -> Tuple[Measurement, Measurement]:
        sample_count = max(
            5,
            int(round(duration_sec * SAMPLE_HZ)),
        )

        external_samples = []
        joint_samples = []

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
        )

        self.send_ui("measurement_finished")

        return result

    def capture_empty(self) -> None:
        self.monitor_enabled = False

        (
            self.empty_external,
            self.empty_joint,
        ) = self.collect_measurement(
            BASELINE_DURATION_SEC,
            "빈 판 external/joint torque 측정",
        )

        self.reference_pose = self.get_current_posx_base()

        self.center_external = None
        self.center_joint = None
        self.noise_sigma = None
        self.direction_points.clear()
        self.position_model = None
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

        self.send_ui(
            "empty_captured",
            mean=self.empty_external.mean.tolist(),
            std=self.empty_external.std.tolist(),
            pose=self.reference_pose.tolist(),
        )
        self.send_reference_state()

    def capture_center(self) -> None:
        if self.empty_external is None:
            raise RuntimeError("먼저 빈 판 기준을 측정하십시오.")

        self.monitor_enabled = False

        (
            self.center_external,
            self.center_joint,
        ) = self.collect_measurement(
            CENTER_REFERENCE_DURATION_SEC,
            "물체 중심 위치 external/joint torque 측정",
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

        self.direction_points.clear()
        self.position_model = None
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

        external, joint = self.collect_measurement(
            DIRECTION_REFERENCE_DURATION_SEC,
            f"{name} 위치 external/joint torque 측정",
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
            "center_error": center_error,
            "center_error_z": center_error_z,
            "center_error_z_norm": center_error_z_norm,
        }

        self.position_model = None

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

        feature_rows = []
        target_rows = []

        calibration_points: Dict[
            str,
            Dict[str, Any],
        ] = {}

        for sample in self.center_external.samples:
            normalized = (
                sample
                - self.center_external.mean
            ) / self.noise_sigma

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
            "external_mean": self.center_external.mean.tolist(),
            "center_error_z_norm": 0.0,
        }

        edge_z_norms = []

        for name in (
            "Y_PLUS",
            "Y_MINUS",
            "Z_PLUS",
            "Z_MINUS",
        ):
            point = self.direction_points[name]

            for sample in point["external"].samples:
                normalized = (
                    sample
                    - self.center_external.mean
                ) / self.noise_sigma

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

            calibration_points[name] = {
                "y_mm": point["y_mm"],
                "z_mm": point["z_mm"],
                "description": point["description"],
                "external_mean": point["external"].mean.tolist(),
                "external_std": point["external"].std.tolist(),
                "center_error": point["center_error"].tolist(),
                "center_error_z": point["center_error_z"].tolist(),
                "center_error_z_norm": point["center_error_z_norm"],
            }

            edge_z_norms.append(
                point["center_error_z_norm"]
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

        self.position_model = PositionModel(
            coefficients=coefficients,
            calibration_rmse_mm=rmse,
            calibration_points=calibration_points,
            edge_reference_z=edge_reference_z,
            condition_number=condition_number,
        )

        self.send_log(
            f"위치 모델 생성 완료: RMSE={rmse:.2f} mm, "
            f"condition={condition_number:.2f}, "
            f"edge z={edge_reference_z:.2f}"
        )

        if rmse > MAX_POSITION_MODEL_RMSE_MM:
            self.send_log(
                "[경고] 위치 회귀 RMSE가 큽니다. "
                "방향 보정 위치와 손 접촉 상태를 확인하십시오."
            )

        self.send_ui(
            "model_built",
            rmse_mm=rmse,
            condition_number=condition_number,
            edge_reference_z=edge_reference_z,
            coefficients=coefficients.tolist(),
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
        self.score_history.clear()

    def update_center_latch(self, center_error_z_norm: float) -> bool:
        enter_threshold = (
            self.center_threshold_z
            * CENTER_ENTER_RATIO
        )
        exit_threshold = (
            self.center_threshold_z
            * CENTER_EXIT_RATIO
        )

        if self.center_latched:
            if center_error_z_norm > exit_threshold:
                self.center_exit_count += 1
            else:
                self.center_exit_count = 0

            if self.center_exit_count >= CENTER_CONFIRM_CYCLES:
                self.center_latched = False
                self.center_exit_count = 0
                self.center_enter_count = 0

        else:
            if center_error_z_norm < enter_threshold:
                self.center_enter_count += 1
            else:
                self.center_enter_count = 0

            if self.center_enter_count >= CENTER_CONFIRM_CYCLES:
                self.center_latched = True
                self.center_enter_count = 0
                self.center_exit_count = 0

        return self.center_latched

    def estimate_position(
        self,
        center_error_z: np.ndarray,
    ) -> Tuple[float, float, float, float, str]:
        if self.position_model is None:
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
            @ self.position_model.coefficients
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

        self.filtered_external = ema_array(
            external_raw,
            self.filtered_external,
            EXTERNAL_TORQUE_EMA_ALPHA,
        )
        self.filtered_joint = ema_array(
            joint_raw,
            self.filtered_joint,
            JOINT_TORQUE_EMA_ALPHA,
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
            DERIVATIVE_EMA_ALPHA,
        )
        self.filtered_joint_derivative = ema_array(
            joint_derivative_raw,
            self.filtered_joint_derivative,
            DERIVATIVE_EMA_ALPHA,
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
            center_error_z
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
            ]
        )

        message = Float64MultiArray()
        message.data = values.tolist()
        self.feature_publisher.publish(message)

    def publish_status(self, state: Dict[str, Any]) -> None:
        payload = {
            "label": self.current_label,
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

            "feature_topic": FEATURE_TOPIC,
            "status_topic": STATUS_TOPIC,
            "command_topic": COMMAND_TOPIC,

            "direction_points": {},
            "position_model": None,
        }

        for name, point in self.direction_points.items():
            data["direction_points"][name] = {
                "y_mm": point["y_mm"],
                "z_mm": point["z_mm"],
                "description": point["description"],
                "external_mean": point["external"].mean.tolist(),
                "external_std": point["external"].std.tolist(),
                "center_error": point["center_error"].tolist(),
                "center_error_z": point["center_error_z"].tolist(),
                "center_error_z_norm": point["center_error_z_norm"],
            }

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

        self.model_path.write_text(
            json.dumps(
                data,
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

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

        elif name == "shutdown":
            self.running = False

        else:
            raise RuntimeError(
                f"알 수 없는 명령: {name}"
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
                        raw = self.read_robot_state()
                        state = self.build_state(
                            raw,
                            loop_start,
                        )

                        self.publish_features(state)
                        self.publish_status(state)
                        self.write_csv(state)

                        self.send_ui(
                            "state",
                            state=self.serialize_state(state),
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
        self.direction_points: set[str] = set()

        self.latest_state: Optional[
            Dict[str, Any]
        ] = None

        self.history: deque = deque()
        self.csv_path = ""
        self.model_path = ""

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
        style = ttk.Style()

        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(
            "Title.TLabel",
            font=("Sans", 18, "bold"),
        )
        style.configure(
            "Section.TLabel",
            font=("Sans", 12, "bold"),
        )
        style.configure(
            "MetricName.TLabel",
            font=("Sans", 10),
        )
        style.configure(
            "MetricValue.TLabel",
            font=("Monospace", 13, "bold"),
        )
        style.configure(
            "Status.TLabel",
            font=("Sans", 17, "bold"),
            anchor="center",
        )
        style.configure(
            "Big.TButton",
            font=("Sans", 11, "bold"),
            padding=(8, 8),
        )
        style.configure(
            "Direction.TButton",
            font=("Sans", 10, "bold"),
            padding=(6, 6),
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

        # 판 표시
        plate_frame = ttk.LabelFrame(
            left,
            text="판 위 위치",
            padding=6,
        )
        plate_frame.grid(
            row=1,
            column=0,
            sticky="nsew",
        )
        plate_frame.rowconfigure(
            0,
            weight=1,
        )
        plate_frame.columnconfigure(
            0,
            weight=1,
        )

        self.plate_canvas = tk.Canvas(
            plate_frame,
            background="#111827",
            highlightthickness=0,
        )
        self.plate_canvas.grid(
            row=0,
            column=0,
            sticky="nsew",
        )
        self.plate_canvas.bind(
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

        # 히스토리
        history_frame = ttk.LabelFrame(
            left,
            text="최근 Z 신호",
            padding=6,
        )
        history_frame.grid(
            row=3,
            column=0,
            sticky="nsew",
            pady=(8, 0),
        )
        history_frame.rowconfigure(
            0,
            weight=1,
        )
        history_frame.columnconfigure(
            0,
            weight=1,
        )

        self.history_canvas = tk.Canvas(
            history_frame,
            background="#ffffff",
            highlightthickness=0,
        )
        self.history_canvas.grid(
            row=0,
            column=0,
            sticky="nsew",
        )
        self.history_canvas.bind(
            "<Configure>",
            lambda _event: self.draw_history(),
        )

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
                f"선택적 4방향 위치 보정 "
                f"(중심에서 {CALIBRATION_RADIUS_MM:.0f} mm)"
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
                "+Y 60 mm",
                +CALIBRATION_RADIUS_MM,
                0.0,
                "중심에서 Tool +Y 방향",
            ),
            (
                "Y_MINUS",
                "-Y 60 mm",
                -CALIBRATION_RADIUS_MM,
                0.0,
                "중심에서 Tool -Y 방향",
            ),
            (
                "Z_PLUS",
                "+Z 60 mm",
                0.0,
                +CALIBRATION_RADIUS_MM,
                "중심에서 Tool +Z 방향",
            ),
            (
                "Z_MINUS",
                "-Z 60 mm",
                0.0,
                -CALIBRATION_RADIUS_MM,
                "중심에서 Tool -Z 방향",
            ),
        ]

        for index, (
            name,
            text,
            y_mm,
            z_mm,
            description,
        ) in enumerate(direction_definitions):
            button = ttk.Button(
                control_frame,
                text=text,
                style="Direction.TButton",
                command=lambda n=name, y=y_mm, z=z_mm, d=description: (
                    self.request_direction_capture(
                        n,
                        y,
                        z,
                        d,
                    )
                ),
            )
            button.grid(
                row=3 + index // 2,
                column=index % 2,
                sticky="ew",
                padx=(
                    0 if index % 2 == 0 else 4,
                    4 if index % 2 == 0 else 0,
                ),
                pady=3,
            )
            self.direction_buttons[
                name
            ] = button

        self.model_button = ttk.Button(
            control_frame,
            text="위치 모델 생성",
            style="Big.TButton",
            command=self.request_build_model,
        )
        self.model_button.grid(
            row=5,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(6, 3),
        )

        ttk.Separator(
            control_frame,
            orient=tk.HORIZONTAL,
        ).grid(
            row=6,
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
            row=7,
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
            row=7,
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
            text="중심 대비 external torque — 축별 z-score",
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
                background="#f3f4f6",
                highlightthickness=1,
                highlightbackground="#d1d5db",
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
            ("복귀 힌트", "hint"),
            ("물체", "object"),
            ("로봇 정지", "stationary"),
            ("자세 유지", "pose"),
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

        if name.startswith("capture_") or name == "build_model":
            self.busy = True
            self.update_button_states()

    # -------------------------------------------------------------------------
    # UI 큐
    # -------------------------------------------------------------------------

    def poll_ui_queue(self) -> None:
        try:
            while True:
                event = self.ui_queue.get_nowait()
                self.handle_ui_event(event)
        except queue.Empty:
            pass

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

        elif name == "direction_captured":
            self.append_log(
                f"{event['name']} 보정 저장 완료 · "
                f"z-norm={event['center_error_z_norm']:.2f}"
            )

        elif name == "model_built":
            self.append_log(
                "위치 모델 생성 완료 · "
                f"RMSE={event['rmse_mm']:.2f} mm · "
                f"condition={event['condition_number']:.2f}"
            )
            self.metric_variables[
                "position"
            ].set(
                "4방향 위치 모델 준비 완료"
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

        self.draw_plate()
        self.draw_axis_bars()

    # -------------------------------------------------------------------------
    # 상태 업데이트
    # -------------------------------------------------------------------------

    def update_state(
        self,
        state: Dict[str, Any],
    ) -> None:
        self.latest_state = state

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

        now = time.monotonic()
        self.history.append(
            (
                now,
                float(
                    state["center_error_z"]
                ),
            )
        )

        while (
            self.history
            and now - self.history[0][0]
            > GUI_HISTORY_SEC
        ):
            self.history.popleft()

        self.draw_plate()
        self.draw_axis_bars()
        self.draw_history()

    # -------------------------------------------------------------------------
    # 판
    # -------------------------------------------------------------------------

    def draw_plate(self) -> None:
        canvas = self.plate_canvas
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

        if self.latest_state is None:
            canvas.create_text(
                center_x,
                center_y + radius_px * 0.65,
                text="빈 판 및 중심 기준을 측정하십시오.",
                fill="#9ca3af",
                font=("Sans", 12),
            )
            return

        state = self.latest_state

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
    # 히스토리
    # -------------------------------------------------------------------------

    def draw_history(self) -> None:
        canvas = self.history_canvas
        canvas.delete("all")

        width = max(
            canvas.winfo_width(),
            100,
        )
        height = max(
            canvas.winfo_height(),
            80,
        )

        left = 45
        right = width - 10
        top = 10
        bottom = height - 24

        canvas.create_rectangle(
            left,
            top,
            right,
            bottom,
            outline="#d1d5db",
        )

        threshold = (
            float(
                self.latest_state[
                    "center_error_z"
                ]
            )
            if False
            else 4.0
        )

        maximum_y = 8.0

        if self.latest_state is not None:
            maximum_y = max(
                8.0,
                float(
                    self.latest_state[
                        "center_error_z"
                    ]
                )
                * 1.2,
            )

        def y_to_pixel(value: float) -> float:
            ratio = min(
                max(
                    value / maximum_y,
                    0.0,
                ),
                1.0,
            )
            return bottom - ratio * (
                bottom - top
            )

        threshold_y = y_to_pixel(
            threshold
        )

        canvas.create_line(
            left,
            threshold_y,
            right,
            threshold_y,
            fill="#ef4444",
            dash=(5, 3),
        )
        canvas.create_text(
            left - 4,
            threshold_y,
            text=f"{threshold:.1f}",
            anchor="e",
            fill="#ef4444",
        )

        canvas.create_text(
            left - 4,
            top,
            text=f"{maximum_y:.1f}",
            anchor="e",
            fill="#6b7280",
        )
        canvas.create_text(
            left - 4,
            bottom,
            text="0",
            anchor="e",
            fill="#6b7280",
        )

        if len(self.history) < 2:
            return

        first_time = self.history[0][0]
        last_time = self.history[-1][0]
        time_span = max(
            last_time - first_time,
            1.0,
        )

        points = []

        for timestamp, value in self.history:
            x = (
                left
                + (
                    timestamp - first_time
                )
                / time_span
                * (right - left)
            )
            y = y_to_pixel(value)
            points.extend([x, y])

        canvas.create_line(
            *points,
            fill="#2563eb",
            width=2,
            smooth=True,
        )

        canvas.create_text(
            (left + right) / 2.0,
            height - 10,
            text=f"최근 {GUI_HISTORY_SEC:.0f}초",
            fill="#6b7280",
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

    backend_thread = threading.Thread(
        target=backend.run,
        daemon=True,
    )
    backend_thread.start()

    root = tk.Tk()
    gui = PlatePositionGUI(
        root=root,
        command_queue=command_queue,
        ui_queue=ui_queue,
    )

    try:
        root.mainloop()

    finally:
        command_queue.put(
            ("shutdown", None)
        )
        backend_thread.join(
            timeout=2.0,
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()