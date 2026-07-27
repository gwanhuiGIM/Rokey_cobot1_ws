#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Joint-pose-independent plate position monitor (terminal test version).

By default this program sends no robot motion commands.  With the explicit
``--enable-tilt`` option it can send conservative ``servol`` commands.  Position
estimation uses Doosan's estimated TCP wrench in BASE coordinates, rather than
directly regressing six joint torques.

Workflow
--------
1. Keep the robot still, remove the object, and press Enter.
2. Put the same object at the plate center and press Enter.
3. Move the object and/or change the robot joint pose slowly.  Position is
   printed after the robot stops.

Assumptions
-----------
* Plate plane is Tool Y-Z, and its normal is Tool +X (same convention as
  plate_monitor_v2).
* The robot/object are quasi-static while a position is accepted.
* Empty and center calibration are captured at the same TCP pose.
* Doosan TCP Euler angles follow Z-Y-Z notation.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import select
import shutil
import sys
import time
from typing import Any, Optional, Sequence, Tuple

import numpy as np


# =============================================================================
# User settings
# =============================================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "plate_pose_invariant_monitor"

SAMPLE_HZ = 20.0
DISPLAY_HZ = 5.0
CALIBRATION_DURATION_SEC = 4.0
WRENCH_FILTER_TIME_CONSTANT_SEC = 0.20
MEDIAN_WINDOW = 5

PLATE_RADIUS_MM = 100.0
# control_GUI.py의 PhysicalParams 기본값과 동일하다. Tool 원점에서 plate
# 표면 중심까지 Tool +X 방향 거리다.
TCP_TO_PLATE_OFFSET_MM = 10.0
MAX_ESTIMATED_RADIUS_MM = PLATE_RADIUS_MM * 1.2
MIN_OBJECT_FORCE_N = 0.5
MIN_NORMAL_FORCE_RATIO = 0.15
MAX_FORCE_RESIDUAL_N = 5.0

MAX_CALIBRATION_POSITION_DRIFT_MM = 2.0
MAX_CALIBRATION_ORIENTATION_DRIFT_DEG = 1.0
MAX_TCP_LINEAR_SPEED_MM_S = 1.0
MAX_TCP_ANGULAR_SPEED_DEG_S = 0.5

# --- Optional balancing tilt controller -------------------------------------
# PLATE +Y load -> lower -Y side -> local -Z rotation.
# PLATE +Z load -> lower -Z side -> local +Y rotation.
TILT_KP_DEG_PER_MM = 0.020
TILT_DEADBAND_MM = 3.0
MAX_TILT_DEG = 2.0
MAX_TILT_VEL_DEG_S = 0.5
MAX_TILT_ACC_DEG_S2 = 1.0
SERVO_LINEAR_VEL_MM_S = 5.0
SERVO_LINEAR_ACC_MM_S2 = 10.0

# A single static wrench cannot observe COM displacement parallel to gravity.
# At a horizontal reference pose that missing direction is normally Tool X.
# If large tilt angles create a repeatable bias, enter measured COM heights.
EMPTY_LOAD_COM_TOOL_X_MM = 0.0
OBJECT_COM_TOOL_X_MM = 0.0

# plate_monitor_v2 defines the plate surface as Tool Y-Z.
PLATE_NORMAL_TOOL = np.asarray([1.0, 0.0, 0.0], dtype=float)
PLATE_Y_TOOL = np.asarray([0.0, 1.0, 0.0], dtype=float)
PLATE_Z_TOOL = np.asarray([0.0, 0.0, 1.0], dtype=float)

DEFAULT_LOG_DIR = Path.home() / "plate_balance_logs"


# =============================================================================
# Pure math
# =============================================================================

def fixed_array(value: Any, length: int, name: str) -> np.ndarray:
    """Normalize the several list/tuple shapes returned by DSR_ROBOT2."""
    if value is None:
        raise RuntimeError(f"{name}: robot API returned None")

    if isinstance(value, (list, tuple)) and len(value) == 2:
        first = value[0]
        if hasattr(first, "__iter__") and not isinstance(first, (str, bytes)):
            first_values = list(first)
            if len(first_values) >= length:
                value = first_values

    result = np.asarray(list(value), dtype=float).reshape(-1)
    if result.size < length:
        raise RuntimeError(
            f"{name}: expected {length} values, received {result.size}: {value}"
        )
    result = result[:length]
    if not np.all(np.isfinite(result)):
        raise RuntimeError(f"{name}: non-finite value: {result}")
    return result


def rotation_matrix_zyz_deg(angles_deg: Sequence[float]) -> np.ndarray:
    a, b, c = np.radians(np.asarray(angles_deg, dtype=float))
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    rz_a = np.asarray(
        [[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]], dtype=float
    )
    ry_b = np.asarray(
        [[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]], dtype=float
    )
    rz_c = np.asarray(
        [[cc, -sc, 0.0], [sc, cc, 0.0], [0.0, 0.0, 1.0]], dtype=float
    )
    return rz_a @ ry_b @ rz_c


def orientation_difference_deg(pose_a: np.ndarray, pose_b: np.ndarray) -> float:
    rotation_a = rotation_matrix_zyz_deg(pose_a[3:])
    rotation_b = rotation_matrix_zyz_deg(pose_b[3:])
    cosine = (float(np.trace(rotation_a.T @ rotation_b)) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def rotation_vector_matrix(rotation_vector_deg: Sequence[float]) -> np.ndarray:
    """Rodrigues rotation for a TOOL-local rotation vector in degrees."""
    vector = np.radians(np.asarray(rotation_vector_deg, dtype=float))
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-12:
        return np.eye(3)
    axis = vector / angle
    x, y, z = axis
    skew = np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def matrix_to_zyz_deg_near(
    rotation: np.ndarray,
    reference_angles_deg: Sequence[float],
) -> np.ndarray:
    """Convert rotation to the equivalent Z-Y-Z angles nearest a reference."""
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    reference = np.asarray(reference_angles_deg, dtype=float)
    b = math.acos(max(-1.0, min(1.0, float(rotation[2, 2]))))
    if abs(math.sin(b)) < 1.0e-8:
        # ZYZ is singular here; preserve C and solve the observable A+C term.
        c = math.radians(reference[2])
        total = math.atan2(rotation[1, 0], rotation[0, 0])
        candidates = [np.asarray([total - c, b, c])]
    else:
        a = math.atan2(rotation[1, 2], rotation[0, 2])
        c = math.atan2(rotation[2, 1], -rotation[2, 0])
        candidates = [
            np.asarray([a, b, c]),
            np.asarray([a + math.pi, -b, c + math.pi]),
        ]

    best = None
    best_cost = math.inf
    for candidate in candidates:
        degrees = np.degrees(candidate)
        nearest = reference + (degrees - reference + 180.0) % 360.0 - 180.0
        cost = float(np.sum(np.square(nearest - reference)))
        if cost < best_cost:
            best = nearest
            best_cost = cost
    assert best is not None
    return best


def robust_mean(samples: Sequence[Sequence[float]]) -> np.ndarray:
    values = np.asarray(samples, dtype=float)
    median = np.median(values, axis=0)
    mad = np.median(np.abs(values - median), axis=0)
    scale = 1.4826 * np.maximum(mad, 1.0e-12)
    accepted = np.abs(values - median) <= 4.5 * scale
    filtered = np.where(accepted, values, np.nan)
    mean = np.nanmean(filtered, axis=0)
    fallback = np.mean(values, axis=0)
    mean[~np.isfinite(mean)] = fallback[~np.isfinite(mean)]
    return mean


def infer_com_tool(
    wrench_base: np.ndarray,
    rotation_base_tool: np.ndarray,
    tool_x_m: float,
) -> np.ndarray:
    """Infer point-load COM and constrain its unobservable Tool-X coordinate."""
    force = wrench_base[:3]
    moment = wrench_base[3:]
    force_squared = float(force @ force)
    if force_squared <= 1.0e-8:
        raise RuntimeError("Cartesian force is too small to infer COM")

    # For M = r x F, F x M / |F|^2 is the closest point on the force line.
    com_base = np.cross(force, moment) / force_squared
    com_tool = rotation_base_tool.T @ com_base
    force_tool = rotation_base_tool.T @ force

    normal_force = float(PLATE_NORMAL_TOOL @ force_tool)
    if abs(normal_force) > 1.0e-8:
        line_parameter = (
            tool_x_m - float(PLATE_NORMAL_TOOL @ com_tool)
        ) / normal_force
        com_tool = com_tool + line_parameter * force_tool
    return com_tool


@dataclass
class Estimate:
    valid: bool
    reason: str
    y_mm: float = math.nan
    z_mm: float = math.nan
    radius_mm: float = math.nan
    angle_deg: float = math.nan
    normal_force_ratio: float = math.nan
    force_residual_n: float = math.nan


@dataclass
class StaticWrenchModel:
    empty_force_base: np.ndarray
    empty_com_tool: np.ndarray
    object_force_base: np.ndarray
    object_center_com_tool: np.ndarray

    @classmethod
    def calibrate(
        cls,
        empty_wrench: np.ndarray,
        center_wrench: np.ndarray,
        reference_pose: np.ndarray,
    ) -> "StaticWrenchModel":
        rotation = rotation_matrix_zyz_deg(reference_pose[3:])
        object_wrench = center_wrench - empty_wrench
        object_force_norm = float(np.linalg.norm(object_wrench[:3]))
        if object_force_norm < MIN_OBJECT_FORCE_N:
            raise RuntimeError(
                "Object force is too small. "
                f"|F_center-F_empty|={object_force_norm:.3f} N"
            )

        return cls(
            empty_force_base=empty_wrench[:3].copy(),
            empty_com_tool=infer_com_tool(
                empty_wrench,
                rotation,
                EMPTY_LOAD_COM_TOOL_X_MM / 1000.0,
            ),
            object_force_base=object_wrench[:3].copy(),
            object_center_com_tool=infer_com_tool(
                object_wrench,
                rotation,
                OBJECT_COM_TOOL_X_MM / 1000.0,
            ),
        )

    @property
    def center_force_base(self) -> np.ndarray:
        return self.empty_force_base + self.object_force_base

    def predict_center_wrench(self, tcp_pose: np.ndarray) -> np.ndarray:
        rotation = rotation_matrix_zyz_deg(tcp_pose[3:])
        empty_com_base = rotation @ self.empty_com_tool
        object_com_base = rotation @ self.object_center_com_tool
        moment = (
            np.cross(empty_com_base, self.empty_force_base)
            + np.cross(object_com_base, self.object_force_base)
        )
        return np.concatenate([self.center_force_base, moment])

    def estimate(self, measured_wrench: np.ndarray, tcp_pose: np.ndarray) -> Estimate:
        rotation = rotation_matrix_zyz_deg(tcp_pose[3:])
        object_force = self.object_force_base
        object_force_norm = float(np.linalg.norm(object_force))
        plate_normal_base = rotation @ PLATE_NORMAL_TOOL
        normal_force_ratio = abs(float(plate_normal_base @ object_force)) / max(
            object_force_norm, 1.0e-12
        )

        if normal_force_ratio < MIN_NORMAL_FORCE_RATIO:
            return Estimate(
                False,
                "PLATE_NEAR_VERTICAL",
                normal_force_ratio=normal_force_ratio,
            )

        predicted_center = self.predict_center_wrench(tcp_pose)
        force_residual = float(
            np.linalg.norm(measured_wrench[:3] - predicted_center[:3])
        )
        if force_residual > MAX_FORCE_RESIDUAL_N:
            return Estimate(
                False,
                "NON_STATIC_FORCE",
                normal_force_ratio=normal_force_ratio,
                force_residual_n=force_residual,
            )

        # Moving the same object does not change gravity force; it changes only
        # moment by delta_M = delta_r x F_object.
        delta_moment = measured_wrench[3:] - predicted_center[3:]
        closest = np.cross(object_force, delta_moment) / float(
            object_force @ object_force
        )
        normal_dot_force = float(plate_normal_base @ object_force)
        displacement_base = closest - (
            float(plate_normal_base @ closest) / normal_dot_force
        ) * object_force
        displacement_tool = rotation.T @ displacement_base

        y_mm = 1000.0 * float(PLATE_Y_TOOL @ displacement_tool)
        z_mm = 1000.0 * float(PLATE_Z_TOOL @ displacement_tool)
        radius_mm = math.hypot(y_mm, z_mm)
        angle_deg = math.degrees(math.atan2(z_mm, y_mm))

        if radius_mm > MAX_ESTIMATED_RADIUS_MM:
            return Estimate(
                False,
                "POSITION_OUT_OF_RANGE",
                y_mm,
                z_mm,
                radius_mm,
                angle_deg,
                normal_force_ratio,
                force_residual,
            )
        return Estimate(
            True,
            "OK",
            y_mm,
            z_mm,
            radius_mm,
            angle_deg,
            normal_force_ratio,
            force_residual,
        )

    def to_dict(self) -> dict:
        return {
            "empty_force_base": self.empty_force_base.tolist(),
            "empty_com_tool": self.empty_com_tool.tolist(),
            "object_force_base": self.object_force_base.tolist(),
            "object_center_com_tool": self.object_center_com_tool.tolist(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "StaticWrenchModel":
        return cls(
            empty_force_base=np.asarray(data["empty_force_base"], dtype=float),
            empty_com_tool=np.asarray(data["empty_com_tool"], dtype=float),
            object_force_base=np.asarray(data["object_force_base"], dtype=float),
            object_center_com_tool=np.asarray(
                data["object_center_com_tool"], dtype=float
            ),
        )


# =============================================================================
# Robot sampling and terminal program
# =============================================================================

def read_wrench_base(dsr: Any) -> np.ndarray:
    # Deliberately request BASE.  The vendored DR_TOOL service rotates force but
    # does not rotate all moment components consistently.
    try:
        value = dsr.get_tool_force(ref=dsr.DR_BASE)
    except TypeError:
        value = dsr.get_tool_force(dsr.DR_BASE)
    return fixed_array(value, 6, "get_tool_force(DR_BASE)")


def read_tcp_pose_base(dsr: Any) -> np.ndarray:
    try:
        value = dsr.get_current_posx(ref=dsr.DR_BASE)
    except TypeError:
        value = dsr.get_current_posx(dsr.DR_BASE)
    return fixed_array(value, 6, "get_current_posx(DR_BASE)")


def read_tcp_velocity_base(dsr: Any) -> np.ndarray:
    try:
        value = dsr.get_current_velx(ref=dsr.DR_BASE)
    except TypeError:
        value = dsr.get_current_velx(dsr.DR_BASE)
    return fixed_array(value, 6, "get_current_velx(DR_BASE)")


def capture_wrench(dsr: Any, title: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    count = max(10, int(round(CALIBRATION_DURATION_SEC * SAMPLE_HZ)))
    samples = []
    period = 1.0 / SAMPLE_HZ
    print(f"\n{title}: {CALIBRATION_DURATION_SEC:.1f}초 측정 중...", flush=True)
    for _index in range(count):
        start = time.monotonic()
        samples.append(read_wrench_base(dsr))
        remaining = period - (time.monotonic() - start)
        if remaining > 0.0:
            time.sleep(remaining)
    print(f"{title}: 측정 완료")
    array = np.asarray(samples, dtype=float)
    return robust_mean(array), np.std(array, axis=0, ddof=1), read_tcp_pose_base(dsr)


def save_calibration(
    model: StaticWrenchModel,
    empty_wrench: np.ndarray,
    center_wrench: np.ndarray,
    reference_pose: np.ndarray,
    requested_path: Optional[str],
) -> Path:
    DEFAULT_LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = (
        Path(requested_path).expanduser()
        if requested_path
        else DEFAULT_LOG_DIR
        / f"pose_invariant_calibration_{datetime.now():%Y%m%d_%H%M%S}.json"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "format_version": 1,
        "created_at": datetime.now().isoformat(),
        "robot_id": ROBOT_ID,
        "robot_model": ROBOT_MODEL,
        "reference_pose": reference_pose.tolist(),
        "empty_wrench_base": empty_wrench.tolist(),
        "center_wrench_base": center_wrench.tolist(),
        "model": model.to_dict(),
        "settings": {
            "plate_radius_mm": PLATE_RADIUS_MM,
            "tcp_to_plate_offset_mm": TCP_TO_PLATE_OFFSET_MM,
            "empty_load_com_tool_x_mm": EMPTY_LOAD_COM_TOOL_X_MM,
            "object_com_tool_x_mm": OBJECT_COM_TOOL_X_MM,
        },
    }
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def load_calibration(path_string: str) -> StaticWrenchModel:
    path = Path(path_string).expanduser()
    data = json.loads(path.read_text(encoding="utf-8"))
    print(f"캘리브레이션 불러옴: {path}")
    return StaticWrenchModel.from_dict(data["model"])


def object_coordinate_frames(
    estimate: Estimate,
    tcp_pose_base: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return contact position in plate-center, TOOL/TCP, and BASE frames.

    control_GUI's physical convention is used exactly: plate normal is Tool +X,
    the surface is Tool Y-Z, and its center is TCP_TO_PLATE_OFFSET_MM along +X.
    The estimated point lies on that surface, so plate-local X is identically 0.
    """
    plate_xyz_mm = np.asarray([0.0, estimate.y_mm, estimate.z_mm], dtype=float)
    tool_xyz_mm = np.asarray(
        [TCP_TO_PLATE_OFFSET_MM, estimate.y_mm, estimate.z_mm], dtype=float
    )
    rotation = rotation_matrix_zyz_deg(tcp_pose_base[3:])
    base_xyz_mm = tcp_pose_base[:3] + rotation @ tool_xyz_mm
    return plate_xyz_mm, tool_xyz_mm, base_xyz_mm


class SnapshotCsv:
    """Save one valid XYZ sample whenever Enter is pressed."""

    def __init__(self) -> None:
        DEFAULT_LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.path = DEFAULT_LOG_DIR / f"pose_invariant_positions_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.file = self.path.open("w", newline="", encoding="utf-8", buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "timestamp",
                "plate_x_mm", "plate_y_mm", "plate_z_mm",
                "tool_x_mm", "tool_y_mm", "tool_z_mm",
                "base_x_mm", "base_y_mm", "base_z_mm",
                "radius_mm", "angle_deg",
                "tilt_tool_y_deg", "tilt_tool_z_deg",
                "tcp_x_mm", "tcp_y_mm", "tcp_z_mm",
                "tcp_a_deg", "tcp_b_deg", "tcp_c_deg",
                "normal_force_ratio", "force_residual_n",
            ]
        )

    def save(
        self,
        estimate: Estimate,
        tcp_pose: np.ndarray,
        plate_xyz: np.ndarray,
        tool_xyz: np.ndarray,
        base_xyz: np.ndarray,
        tilt_deg: np.ndarray,
    ) -> None:
        self.writer.writerow(
            [
                datetime.now().isoformat(timespec="milliseconds"),
                *plate_xyz.tolist(),
                *tool_xyz.tolist(),
                *base_xyz.tolist(),
                estimate.radius_mm,
                estimate.angle_deg,
                *tilt_deg.tolist(),
                *tcp_pose.tolist(),
                estimate.normal_force_ratio,
                estimate.force_residual_n,
            ]
        )

    def close(self) -> None:
        self.file.close()


def clear_live_line() -> None:
    if sys.stdout.isatty():
        print("\r\033[2K", end="", flush=True)


def show_live_line(text: str) -> None:
    """Update one terminal row without wrapping and producing log floods."""
    if not sys.stdout.isatty():
        print(text, flush=True)
        return
    width = max(20, shutil.get_terminal_size(fallback=(100, 24)).columns - 1)
    print("\r\033[2K" + text[:width], end="", flush=True)


def enter_pressed() -> bool:
    if not sys.stdin.isatty():
        return False
    readable, _writable, _exceptional = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return False
    sys.stdin.readline()
    return True


class LocalTiltController:
    """P controller that tilts the plate downhill opposite the measured load."""

    def __init__(
        self,
        dsr: Any,
        neutral_pose: np.ndarray,
        actual_motion: bool,
    ) -> None:
        self.dsr = dsr
        self.neutral_pose = neutral_pose.copy()
        self.neutral_rotation = rotation_matrix_zyz_deg(neutral_pose[3:])
        self.actual_motion = actual_motion
        # [rotation about local Tool Y, rotation about local Tool Z]
        self.commanded_tilt_deg = np.zeros(2, dtype=float)

    def update(
        self,
        estimate: Estimate,
        estimator_valid: bool,
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if estimator_valid and estimate.radius_mm > TILT_DEADBAND_MM:
            desired = np.asarray(
                [
                    +TILT_KP_DEG_PER_MM * estimate.z_mm,
                    -TILT_KP_DEG_PER_MM * estimate.y_mm,
                ],
                dtype=float,
            )
            magnitude = float(np.linalg.norm(desired))
            if magnitude > MAX_TILT_DEG:
                desired *= MAX_TILT_DEG / magnitude
        else:
            desired = np.zeros(2, dtype=float)

        max_step = MAX_TILT_VEL_DEG_S * max(dt, 1.0e-3)
        delta = np.clip(
            desired - self.commanded_tilt_deg,
            -max_step,
            max_step,
        )
        self.commanded_tilt_deg += delta

        local_rotation_vector = np.asarray(
            [0.0, self.commanded_tilt_deg[0], self.commanded_tilt_deg[1]],
            dtype=float,
        )
        target_rotation = (
            self.neutral_rotation
            @ rotation_vector_matrix(local_rotation_vector)
        )
        target_pose = self.neutral_pose.copy()
        target_pose[3:] = matrix_to_zyz_deg_near(
            target_rotation,
            self.neutral_pose[3:],
        )

        if self.actual_motion:
            self.dsr.servol(
                target_pose.tolist(),
                vel=[SERVO_LINEAR_VEL_MM_S, MAX_TILT_VEL_DEG_S],
                acc=[SERVO_LINEAR_ACC_MM_S2, MAX_TILT_ACC_DEG_S2],
                time=max(dt, 0.05),
            )
        return self.commanded_tilt_deg.copy(), target_pose


def run_monitor(
    dsr: Any,
    model: StaticWrenchModel,
    tilt_enabled: bool,
    actual_motion: bool,
) -> None:
    wrench_history: deque[np.ndarray] = deque(maxlen=MEDIAN_WINDOW)
    filtered_wrench: Optional[np.ndarray] = None
    previous_time: Optional[float] = None
    previous_display_time = 0.0
    period = 1.0 / SAMPLE_HZ
    display_period = 1.0 / DISPLAY_HZ
    snapshots = SnapshotCsv()
    neutral_pose = read_tcp_pose_base(dsr)
    tilt_controller = (
        LocalTiltController(dsr, neutral_pose, actual_motion)
        if tilt_enabled
        else None
    )

    print("\n모니터 시작. 로봇을 움직이는 동안은 MOVING으로 표시됩니다.")
    print("정지 후 위치가 안정되면 서로 다른 joint 자세에서도 Y/Z를 비교하세요.")
    print("PLATE 좌표: 중심 원점, +X=법선, Y/Z=plate 면(control_GUI와 동일)")
    print(f"Enter: 현재 XYZ 저장 ({snapshots.path})")
    if tilt_controller is None:
        print("TILT: OFF")
    elif actual_motion:
        print("TILT: ACTUAL MOTION — 실제 servol 명령 전송")
    else:
        print("TILT: DRY-RUN — 목표 기울기만 계산, 모션 명령 없음")
    print("종료: Ctrl+C\n")

    try:
        while True:
            start = time.monotonic()
            wrench = read_wrench_base(dsr)
            pose = read_tcp_pose_base(dsr)
            velocity = read_tcp_velocity_base(dsr)

            wrench_history.append(wrench)
            median_wrench = np.median(np.asarray(wrench_history), axis=0)
            dt = start - previous_time if previous_time is not None else period
            alpha = 1.0 - math.exp(-max(dt, 1.0e-4) / WRENCH_FILTER_TIME_CONSTANT_SEC)
            filtered_wrench = (
                median_wrench.copy()
                if filtered_wrench is None
                else alpha * median_wrench + (1.0 - alpha) * filtered_wrench
            )
            previous_time = start

            linear_speed = float(np.linalg.norm(velocity[:3]))
            angular_speed = float(np.linalg.norm(velocity[3:]))
            stationary = (
                linear_speed <= MAX_TCP_LINEAR_SPEED_MM_S
                and angular_speed <= MAX_TCP_ANGULAR_SPEED_DEG_S
            )
            estimate = model.estimate(filtered_wrench, pose)
            coordinates = (
                object_coordinate_frames(estimate, pose)
                if estimate.valid
                else None
            )

            tilt_deg = np.zeros(2, dtype=float)
            if tilt_controller is not None:
                tilt_deg, _target_pose = tilt_controller.update(
                    estimate,
                    estimator_valid=estimate.valid,
                    dt=dt,
                )

            if not stationary:
                text = (
                    f"MOVING v={linear_speed:.1f}mm/s w={angular_speed:.1f}deg/s"
                )
            elif estimate.valid and coordinates is not None:
                plate_xyz, _tool_xyz, base_xyz = coordinates
                edge = " EDGE" if estimate.radius_mm > PLATE_RADIUS_MM else ""
                text = (
                    f"P({plate_xyz[0]:+.1f},{plate_xyz[1]:+.1f},{plate_xyz[2]:+.1f}) "
                    f"B({base_xyz[0]:.1f},{base_xyz[1]:.1f},{base_xyz[2]:.1f}) "
                    f"T(y,z)=({tilt_deg[0]:+.2f},{tilt_deg[1]:+.2f}){edge}"
                )
            else:
                text = (
                    f"INVALID {estimate.reason} "
                    f"normal={estimate.normal_force_ratio:.2f} dF={estimate.force_residual_n:.2f}N"
                )

            if start - previous_display_time >= display_period:
                show_live_line(text)
                previous_display_time = start

            if enter_pressed():
                clear_live_line()
                if stationary and estimate.valid and coordinates is not None:
                    plate_xyz, tool_xyz, base_xyz = coordinates
                    snapshots.save(
                        estimate, pose, plate_xyz, tool_xyz, base_xyz, tilt_deg
                    )
                    print(
                        "저장: "
                        f"PLATE=({plate_xyz[0]:+.1f}, {plate_xyz[1]:+.1f}, {plate_xyz[2]:+.1f}) mm, "
                        f"BASE=({base_xyz[0]:.1f}, {base_xyz[1]:.1f}, {base_xyz[2]:.1f}) mm"
                    )
                else:
                    print("저장 안 함: 로봇 이동 중이거나 추정값이 INVALID입니다.")

            remaining = period - (time.monotonic() - start)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        clear_live_line()
        snapshots.close()
        print(f"좌표 로그 닫음: {snapshots.path}")


def calibrate_interactively(dsr: Any, save_path: Optional[str]) -> StaticWrenchModel:
    input("\n[1/2] plate에서 물체를 제거하고 로봇을 정지한 뒤 Enter: ")
    empty_wrench, empty_std, empty_pose = capture_wrench(dsr, "빈 plate")
    print("빈 plate wrench:", np.array2string(empty_wrench, precision=4))
    print("표준편차         :", np.array2string(empty_std, precision=4))

    input("\n[2/2] 같은 물체를 plate 정중앙에 놓고 손을 뗀 뒤 Enter: ")
    center_wrench, center_std, center_pose = capture_wrench(dsr, "중앙 물체")
    print("중앙 wrench      :", np.array2string(center_wrench, precision=4))
    print("표준편차         :", np.array2string(center_std, precision=4))

    position_drift = float(np.linalg.norm(center_pose[:3] - empty_pose[:3]))
    orientation_drift = orientation_difference_deg(empty_pose, center_pose)
    if (
        position_drift > MAX_CALIBRATION_POSITION_DRIFT_MM
        or orientation_drift > MAX_CALIBRATION_ORIENTATION_DRIFT_DEG
    ):
        raise RuntimeError(
            "빈 plate와 중앙 측정 사이에 로봇 자세가 바뀌었습니다: "
            f"position={position_drift:.2f} mm, orientation={orientation_drift:.2f} deg"
        )

    model = StaticWrenchModel.calibrate(empty_wrench, center_wrench, empty_pose)
    object_force = model.object_force_base
    print(
        "물체 force       :",
        np.array2string(object_force, precision=4),
        f"|F|={np.linalg.norm(object_force):.3f} N",
    )
    print(
        "추정 물체 중심 COM(Tool, m):",
        np.array2string(model.object_center_com_tool, precision=5),
    )
    path = save_calibration(
        model, empty_wrench, center_wrench, empty_pose, save_path
    )
    print(f"캘리브레이션 저장: {path}")
    return model


# =============================================================================
# Offline math self-test
# =============================================================================

def synthetic_wrench(rotation: np.ndarray, force: np.ndarray, com_tool: np.ndarray) -> np.ndarray:
    return np.concatenate([force, np.cross(rotation @ com_tool, force)])


def run_self_test() -> None:
    empty_force = np.asarray([0.0, 0.0, -19.62])
    object_force = np.asarray([0.0, 0.0, -4.905])
    empty_com = np.asarray([0.0, -0.015, 0.035])
    object_com = np.asarray([0.0, 0.0, 0.090])
    reference_pose = np.asarray([500.0, 0.0, 400.0, 0.0, 90.0, 0.0])
    reference_rotation = rotation_matrix_zyz_deg(reference_pose[3:])
    empty = synthetic_wrench(reference_rotation, empty_force, empty_com)
    center_object = synthetic_wrench(reference_rotation, object_force, object_com)
    model = StaticWrenchModel.calibrate(empty, empty + center_object, reference_pose)

    expected = np.asarray([42.0, -31.0])
    displacement = np.asarray([0.0, expected[0] / 1000.0, expected[1] / 1000.0])
    for angles in ([0.0, 90.0, 0.0], [25.0, 65.0, -40.0], [-70.0, 80.0, 35.0]):
        pose = np.asarray([700.0, -120.0, 250.0, *angles])
        rotation = rotation_matrix_zyz_deg(angles)
        measured = synthetic_wrench(rotation, empty_force, empty_com)
        measured += synthetic_wrench(rotation, object_force, object_com + displacement)
        estimate = model.estimate(measured, pose)
        if not estimate.valid:
            raise AssertionError(estimate.reason)
        np.testing.assert_allclose([estimate.y_mm, estimate.z_mm], expected, atol=1e-8)
        plate_xyz, tool_xyz, base_xyz = object_coordinate_frames(estimate, pose)
        np.testing.assert_allclose(plate_xyz, [0.0, *expected], atol=1e-8)
        np.testing.assert_allclose(tool_xyz, [TCP_TO_PLATE_OFFSET_MM, *expected], atol=1e-8)
        expected_base = pose[:3] + rotation @ tool_xyz
        np.testing.assert_allclose(base_xyz, expected_base, atol=1e-8)

        recovered_angles = matrix_to_zyz_deg_near(rotation, angles)
        recovered_rotation = rotation_matrix_zyz_deg(recovered_angles)
        np.testing.assert_allclose(recovered_rotation, rotation, atol=1e-10)

    class NoMotionDsr:
        pass

    tilt_controller = LocalTiltController(
        NoMotionDsr(), reference_pose, actual_motion=False
    )
    positive_y = Estimate(
        True, "OK", y_mm=20.0, z_mm=0.0, radius_mm=20.0
    )
    tilt, _target = tilt_controller.update(positive_y, True, dt=1.0)
    assert abs(tilt[0]) < 1.0e-12
    assert tilt[1] < 0.0  # +Y load commands local -Z rotation.
    print("SELF TEST PASS: 서로 다른 3개 TCP 자세에서 동일한 PLATE/TOOL/BASE XYZ가 복원됐습니다.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--load", help="저장된 calibration JSON을 불러옵니다.")
    parser.add_argument("--save", help="새 calibration JSON 저장 경로입니다.")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="로봇 없이 자세 불변 수학 계산만 검증합니다.",
    )
    tilt_group = parser.add_mutually_exclusive_group()
    tilt_group.add_argument(
        "--tilt-dry-run",
        action="store_true",
        help="반대 방향 기울기를 계산해 표시하지만 로봇에는 명령하지 않습니다.",
    )
    tilt_group.add_argument(
        "--enable-tilt",
        action="store_true",
        help="반대 방향 기울기 servol 명령을 실제 로봇에 전송합니다.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_test()
        return

    # Import ROS only for the real-robot path, so --self-test works offline.
    import rclpy
    import DR_init

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    rclpy.init()
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    import DSR_ROBOT2 as dsr

    print(
        "자세 불변 plate monitor — "
        + (
            "실제 tilt 모션 요청됨"
            if args.enable_tilt
            else "모션 명령 없음"
        )
    )
    try:
        model = (
            load_calibration(args.load)
            if args.load
            else calibrate_interactively(dsr, args.save)
        )
        if args.enable_tilt:
            print(
                "\n[경고] 실제 로봇에 연속 servol 기울기 명령을 전송합니다.\n"
                f"최대 {MAX_TILT_DEG:.1f} deg, 속도 {MAX_TILT_VEL_DEG_S:.1f} deg/s.\n"
                "비상정지 버튼을 준비하고, plate 주변에 사람이 없는지 확인하십시오."
            )
            confirmation = input("실제 구동을 시작하려면 MOVE를 입력: ").strip()
            if confirmation != "MOVE":
                print("실제 구동을 취소했습니다.")
                return
        run_monitor(
            dsr,
            model,
            tilt_enabled=args.tilt_dry_run or args.enable_tilt,
            actual_motion=args.enable_tilt,
        )
    except KeyboardInterrupt:
        print("\n종료합니다.")
    except Exception as error:
        print(f"\n오류: {error}", file=sys.stderr)
        raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
