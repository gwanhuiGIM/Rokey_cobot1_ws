#!/usr/bin/env python3
"""Vision-free ball-on-plate balancing controller for a Doosan M0609 + RG2 gripper.

Single-file ROS2 (rclpy) node with an embedded Tkinter GUI. Run with:

    ros2 run ball_balance_control ball_balance_node

Prerequisite (per project spec): the robot driver stack is already up, e.g.

    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609

Method notes (see project CLAUDE.md for full references):
  - External force/torque at the TCP is read from dsr_msgs2 aux_control/get_tool_force,
    which the driver derives from joint-torque sensing + the robot's dynamic model
    (the M0609 has built-in per-joint torque sensors) -- conceptually the same family
    of technique as generalized-momentum external-force observers (De Luca & Mattone,
    ICRA 2003; De Luca et al., IROS 2006), just implemented inside the Doosan
    controller rather than by us.
  - The ball's (x, y) contact point on the plate is recovered from that wrench with
    the classic single-point-contact / center-of-pressure back-solve (Bicchi,
    "Intrinsic Contact Sensing for Soft Fingers", ICRA 1990; Bicchi/Salisbury/Brock,
    IJRR 1993; same formula underlies force-plate CoP computation in biomechanics).
    This rig's geometry is confirmed as: the RG2 grasps the plate at its rim with the
    ball-bearing face spanning the tool frame's local Y-Z plane, and local +X is the
    plate's outward normal (the axis gravity acts along for a resting ball) -- so the
    formula is axis-permuted from the more commonly-quoted Z-normal form to
    y = -Mz/Fx, z = My/Fx. This is only well-conditioned away from Fx=0, so a
    minimum-|Fx| gate is applied before trusting (y, z).
  - Control is translation-priority: the plate is translated horizontally (base-frame
    X/Y) to roll the ball back to center, which is dynamically the "cart-table" /
    non-prehensile-tray-transport mechanism (Kajita et al., ICRA 2003 ZMP preview
    control; Selvaggio et al., IEEE T-CST 2023 non-prehensile transport) -- moving the
    support point is interchangeable with tilting it, in the ball's equation of
    motion. Tilt (Rx/Ry) is added only as a small secondary correction.
  - Noise handling follows standard sensor-fusion practice: outlier gating (robust
    median/MAD) and a dead-zone on the raw wrench (since division by a noisy near-zero
    Fx is the dominant error source), then moving-average + low-pass + a constant-
    velocity Kalman filter on the resulting (x, y) track (cf. Van Damme et al., ICRA
    2011; the Kalman-over-momentum-observer approach in IEEE MFI 2015).

Interface-verification note: every dsr_msgs2 service/message used below was checked
directly against this workspace's own DoosanRobotics/doosan-robot2 source -- both the
.srv/.msg field lists AND the dsr_controller2.cpp service-callback implementations --
not just the message definitions, because GetCurrentPosx is a trap: unlike
GetCurrentPosj (flat `float64[6] pos`), GetCurrentPosx's response is
`std_msgs/Float64MultiArray[] task_pos_info`, and get_current_posx_cb in
dsr_controller2.cpp always pushes exactly one element whose `.data` holds
[x,y,z,a,b,c,solution_space]. _on_posx_response() below indexes task_pos_info[0].data
accordingly. Field names should still be spot-checked once against the deployed
driver version (e.g. `ros2 interface show dsr_msgs2/srv/GetCurrentPosx`), since the
safe_set()/safe_get() helpers below degrade to a one-time printed warning (visible in
the GUI's event log) instead of crashing the node if a field name turns out to differ.

Notably, dsr_msgs2's RobotState/RobotStateRt messages exist as types but are not
actually published by the driver (verified against dsr_controller2 source -- no
create_publisher<RobotState> call exists). So, unlike a literal reading of "subscribe
to a state topic", this node polls robot state via the aux_control/get_current_posx,
aux_control/get_current_posj, aux_control/get_tool_force and system/get_robot_state
*services* on a timer instead -- the only mechanism that actually exists in the
driver for this data. The GUI-configurable "sampling Hz" controls the period of the
get_tool_force poll specifically (task/joint pose and robot status are polled on
their own separate, fixed-rate timers -- see POSE_POLL_HZ / ROBOT_STATUS_POLL_HZ).
"""

import json
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Optional, Any

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
    MoveStop,
    GetToolForce,
    GetCurrentPosx,
    GetCurrentPosj,
    GetRobotState,
)
from dsr_msgs2.msg import ServolStream

# OnRobot RG2 gripper is driven over a ROS service (see plate_move.py). Guard the
# import so the GUI still runs if the onrobot messages are not built.
try:
    from onrobot_rg_msgs.srv import SetCommand
    _HAVE_GRIPPER_SRV = True
except ImportError:
    SetCommand = None
    _HAVE_GRIPPER_SRV = False

GRIPPER_SERVICE = '/onrobot/sendCommand'

# Manual rotation-jog rate. The balancing tilt limits (max_tilt_vel_dps ~0.2)
# are far too slow for hand jogging, so discrete Rx/Ry/Rz jogs use these.
JOG_ROT_VEL_DPS = 20.0
JOG_ROT_ACC_DPS2 = 40.0

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import tkinter.font as tkfont


# --------------------------------------------------------------------------------
# Module-level constants
# --------------------------------------------------------------------------------

DEFAULT_NAMESPACE = 'dsr01'
CONTROL_LOOP_HZ = 10.0                 # balancing/ServolStream loop; manual moveL is event-driven
GUI_REFRESH_HZ = 30.0
POSE_POLL_HZ = 10.0                    # current task/joint pose refresh rate (decoupled from sample_hz)
ROBOT_STATUS_POLL_HZ = 2.0             # slow poll for system/get_robot_state
GRAVITY_MPS2 = 9.80665
DEFAULT_HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
PRESET_DIR = os.path.expanduser('~/.ball_balance_control/presets')
LOG_DIR_DEFAULT = os.path.expanduser('~/.ball_balance_control/logs')


# --------------------------------------------------------------------------------
# Defensive field access for dsr_msgs2 requests/responses/messages
#
# We cannot execute against the live driver from this development environment, so
# field names for a few less-central services/messages were inferred from a strong,
# consistently-observed naming pattern rather than a byte-for-byte source read. These
# helpers make that uncertainty fail soft (one printed warning, degraded feature)
# instead of hard (unhandled AttributeError killing the control loop).
# --------------------------------------------------------------------------------

_warned_fields: set = set()


def safe_set(obj: Any, **kwargs) -> Any:
    for key, value in kwargs.items():
        try:
            setattr(obj, key, value)
        except (AttributeError, ValueError, TypeError) as exc:
            warn_key = f'{type(obj).__module__}.{type(obj).__name__}.{key}'
            if warn_key not in _warned_fields:
                _warned_fields.add(warn_key)
                print(f'[ball_balance_control][WARN] could not set field '
                      f'{warn_key} = {value!r} ({exc}); verify with '
                      f"'ros2 interface show {type(obj).__module__.split('.')[0]}"
                      f"/{'srv' if 'srv' in type(obj).__module__ else 'msg'}/"
                      f"{type(obj).__name__.replace('_Request', '').replace('_Response', '')}'")
    return obj


def safe_get(obj: Any, *names: str, default=None):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    warn_key = f'{type(obj).__module__}.{type(obj).__name__}.<{"|".join(names)}>'
    if warn_key not in _warned_fields:
        _warned_fields.add(warn_key)
        print(f'[ball_balance_control][WARN] none of fields {names} found on '
              f'{type(obj).__name__}; using default {default!r}')
    return default


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else (hi if value > hi else value)


# --------------------------------------------------------------------------------
# Configuration dataclasses -- every field here is exposed and live-editable in the
# GUI (Section 8 of the spec). Plain dataclasses (not frozen) so GUI callbacks can
# mutate fields directly; access from the ROS thread is through SharedState's lock.
# --------------------------------------------------------------------------------

@dataclass
class PhysicalParams:
    plate_radius_mm: float = 100.0
    ball_mass_g: float = 50.0
    ball_radius_mm: float = 20.0
    tcp_to_plate_offset_mm: float = 10.0  # +X distance from TCP/flange to plate surface center
                                           # (local +X is the plate's outward/normal axis on this rig)


@dataclass
class FilterParams:
    sample_hz: float = 30.0            # GUI slider: 5-100 Hz, preprocessing/state-poll rate
    outlier_mad_k: float = 5.0         # reject wrench samples beyond k * MAD of recent window
    deadzone_n: float = 0.05           # N, dead-zone applied to Fz before CoP division
    deadzone_nm: float = 0.005         # Nm, dead-zone applied to Mx/My before CoP division
    moving_avg_window_ms: float = 150.0
    lpf_cutoff_hz: float = 3.0
    kalman_process_noise: float = 5.0       # mm^2/s^2-ish process noise for constant-velocity model
    kalman_measurement_noise: float = 15.0  # mm^2 measurement noise
    min_fx_n: float = 0.15             # |Fx| (plate-normal force) below this -> "no ball" / CoP not trusted


@dataclass
class SafetyLimits:
    # Workspace box is expressed as an offset (mm) from the pose captured at connect
    # time, not absolute machine coordinates (we don't know the cell layout).
    ws_x_min_mm: float = -150.0
    ws_x_max_mm: float = 150.0
    ws_y_min_mm: float = -150.0
    ws_y_max_mm: float = 150.0
    ws_z_min_mm: float = -100.0
    ws_z_max_mm: float = 100.0
    max_tilt_deg: float = 12.0
    joint_margin_deg: float = 30.0     # allowed joint travel from the pose captured at connect time
    no_ball_fx_n: float = 0.12          # |Fx| (plate-normal force) threshold for ball-present detection
    departure_hold_s: float = 0.6      # sustained no-ball duration before declaring departure
    comm_timeout_s: float = 1.0
    # Manual moveL defaults: clearly visible while remaining moderate for setup.
    max_lin_vel_mms: float = 20.0
    max_lin_acc_mms2: float = 50.0
    max_tilt_vel_dps: float = 0.2
    max_tilt_acc_dps2: float = 0.4
    joint_move_vel_dps: float = 5.0
    joint_move_acc_dps2: float = 10.0


@dataclass
class ControlParams:
    trans_kp: float = 0.6
    trans_ki: float = 0.02
    trans_kd: float = 0.15
    tilt_kp: float = 0.05
    tilt_ki: float = 0.001
    tilt_kd: float = 0.01
    tilt_weight: float = 0.3           # 0..1, secondary-tilt contribution scale
    auto_search_enabled: bool = False
    auto_search_gain: float = 0.0006
    max_auto_bias_deg: float = 3.0
    # Disabled by default. It must never start automatically when the HMI is
    # opened for manual TCP positioning or A/B capture.
    balancing_enabled: bool = False


@dataclass
class ManualTarget:
    # Base-frame task pose from GUI sliders; balancing correction is superposed on this.
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    rx_deg: float = 0.0
    ry_deg: float = 0.0
    rz_deg: float = 0.0


# --------------------------------------------------------------------------------
# Signal-processing building blocks. Each is a small, independently testable class;
# GUI sliders feed their tunable parameters in every update() call so a parameter
# change takes effect on the very next sample with no restart needed.
# --------------------------------------------------------------------------------

class OutlierGate:
    """Robust median/MAD gate: rejects samples too far from the recent robust
    center and substitutes the last accepted value instead of a raw outlier."""

    def __init__(self, window: int = 15):
        self.window = max(5, window)
        self._buf = deque(maxlen=self.window)
        self._last_good = 0.0

    def update(self, value: Optional[float], k: float) -> tuple:
        if value is None or not math.isfinite(value):
            return self._last_good, True
        if len(self._buf) >= 5:
            arr = np.fromiter(self._buf, dtype=float)
            med = float(np.median(arr))
            mad = float(np.median(np.abs(arr - med))) * 1.4826 + 1e-6
            if abs(value - med) > k * mad:
                return self._last_good, True
        self._buf.append(value)
        self._last_good = value
        return value, False


class MovingAverage:
    def __init__(self):
        self._buf = deque(maxlen=1)

    def update(self, value: float, window_ms: float, hz: float) -> float:
        n = max(1, round(window_ms / 1000.0 * hz))
        if n != self._buf.maxlen:
            self._buf = deque(list(self._buf)[-n:], maxlen=n)
        self._buf.append(value)
        return sum(self._buf) / len(self._buf)


class LowPassFilter:
    """First-order IIR (RC) filter; alpha is recomputed from cutoff/hz on every
    call so GUI changes to either take effect immediately and consistently."""

    def __init__(self):
        self._y: Optional[float] = None

    def reset(self):
        self._y = None

    def update(self, value: float, cutoff_hz: float, hz: float) -> float:
        if self._y is None:
            self._y = value
            return value
        dt = 1.0 / max(hz, 1e-6)
        rc = 1.0 / (2.0 * math.pi * max(cutoff_hz, 1e-6))
        alpha = dt / (rc + dt)
        self._y = self._y + alpha * (value - self._y)
        return self._y


class KalmanFilter2D:
    """Constant-velocity Kalman filter on the final (x, y) ball estimate."""

    def __init__(self):
        self._x = np.zeros(4)
        self._P = np.eye(4) * 1e3
        self._initialized = False

    def reset(self):
        self._x = np.zeros(4)
        self._P = np.eye(4) * 1e3
        self._initialized = False

    def update(self, meas_x: float, meas_y: float, dt: float, q: float, r: float) -> tuple:
        if not self._initialized:
            self._x = np.array([meas_x, meas_y, 0.0, 0.0])
            self._initialized = True
            return meas_x, meas_y
        dt = max(dt, 1e-3)
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        G = np.array([0.5 * dt * dt, 0.5 * dt * dt, dt, dt])
        Q = np.outer(G, G) * q
        x_pred = F @ self._x
        P_pred = F @ self._P @ F.T + Q
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        R = np.eye(2) * r
        innov = np.array([meas_x, meas_y]) - H @ x_pred
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)
        self._x = x_pred + K @ innov
        self._P = (np.eye(4) - K @ H) @ P_pred
        return float(self._x[0]), float(self._x[1])


class SlewLimiter:
    """Velocity+acceleration-limited follower used to turn a possibly-jumpy PID
    output into the bounded, smooth trajectory the safety spec requires."""

    def __init__(self, n: int):
        self._val = np.zeros(n)
        self._vel = np.zeros(n)
        self._init = False

    def reset(self, value):
        self._val = np.array(value, dtype=float)
        self._vel = np.zeros_like(self._val)
        self._init = True

    def update(self, target, dt: float, max_vel: float, max_acc: float):
        target = np.array(target, dtype=float)
        if not self._init:
            self.reset(target)
            return self._val.copy()
        dt = max(dt, 1e-6)
        max_dvel = max_acc * dt
        desired_vel = np.clip((target - self._val) / dt, -max_vel, max_vel)
        dvel = np.clip(desired_vel - self._vel, -max_dvel, max_dvel)
        self._vel = np.clip(self._vel + dvel, -max_vel, max_vel)
        self._val = self._val + self._vel * dt
        return self._val.copy()


class PID:
    def __init__(self):
        self._i = 0.0
        self._prev_err: Optional[float] = None

    def reset(self):
        self._i = 0.0
        self._prev_err = None

    def update(self, err: float, dt: float, kp: float, ki: float, kd: float,
               i_limit: Optional[float] = None) -> float:
        dt = max(dt, 1e-6)
        self._i += err * dt
        if i_limit is not None:
            self._i = clamp(self._i, -i_limit, i_limit)
        d = 0.0 if self._prev_err is None else (err - self._prev_err) / dt
        self._prev_err = err
        return kp * err + ki * self._i + kd * d


class PerformanceTracker:
    """Rolling-window settling time / RMSE / overshoot, for the Section 7 readout."""

    def __init__(self, window_s: float = 20.0, settle_tol_mm: float = 8.0):
        self.window_s = window_s
        self.settle_tol_mm = settle_tol_mm
        self._samples = deque()
        self._entered_tol_at: Optional[float] = None
        self._last_settle_time: Optional[float] = None

    def reset(self):
        self._samples.clear()
        self._entered_tol_at = None
        self._last_settle_time = None

    def update(self, t: float, error_mm: float):
        self._samples.append((t, error_mm))
        while self._samples and t - self._samples[0][0] > self.window_s:
            self._samples.popleft()
        if error_mm <= self.settle_tol_mm:
            if self._entered_tol_at is None:
                self._entered_tol_at = t
            self._last_settle_time = t - self._entered_tol_at
        else:
            self._entered_tol_at = None

    def rmse(self) -> float:
        if not self._samples:
            return 0.0
        arr = np.fromiter((e for _, e in self._samples), dtype=float)
        return float(np.sqrt(np.mean(arr ** 2)))

    def overshoot_mm(self) -> float:
        if not self._samples:
            return 0.0
        return float(max(e for _, e in self._samples))

    def settling_time_s(self) -> Optional[float]:
        return self._last_settle_time


# --------------------------------------------------------------------------------
# Coordinate transform + center-of-pressure estimation (Section 1 of the spec)
# --------------------------------------------------------------------------------

def euler_zyz_to_matrix(a_deg: float, b_deg: float, c_deg: float) -> np.ndarray:
    """posx orientation [A,B,C]: intrinsic Z-Y-Z per the DRL manual (A about base Z,
    B about the once-rotated Y, C about the twice-rotated Z) -> R = Rz(A) Ry(B) Rz(C).
    """
    a, b, c = math.radians(a_deg), math.radians(b_deg), math.radians(c_deg)
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    rz_a = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
    ry_b = np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]])
    rz_c = np.array([[cc, -sc, 0.0], [sc, cc, 0.0], [0.0, 0.0, 1.0]])
    return rz_a @ ry_b @ rz_c


def rotate_wrench_to_plate(wrench_base: np.ndarray, r_base_to_tcp: np.ndarray,
                            plate_x_offset_mm: float) -> tuple:
    """Rotate a base-frame wrench into the plate-local frame and shift the moment
    reference from the TCP origin to the plate-surface center. Pose-independent:
    only the *relative* base->TCP rotation is used, so results stay consistent as
    the arm re-poses (Section 1's core requirement). All 6 raw components must be
    gated/cleaned *before* calling this -- once rotated, a noisy Fy/Fz/Mx can leak
    into the local Fx/My/Mz whenever the plate is tilted, since rotation mixes all
    three axes together.

    Rig geometry (confirmed against hardware, NOT the more common Z-normal
    convention): the RG2 grasps the plate at its rim with the ball-bearing face
    spanning the tool frame's local Y-Z plane, and the plate's outward normal --
    the direction gravity acts on a resting ball -- is local +X. So the plate
    surface center sits an offset along +X from the TCP origin, not +Z."""
    f_base, m_base = wrench_base[0:3], wrench_base[3:6]
    f_plate = r_base_to_tcp.T @ f_base
    m_tcp = r_base_to_tcp.T @ m_base
    r_offset = np.array([plate_x_offset_mm / 1000.0, 0.0, 0.0])
    m_plate_center = m_tcp - np.cross(r_offset, f_plate)
    return f_plate, m_plate_center


def cop_solve(fx: float, my: float, mz: float, min_fx_n: float) -> tuple:
    """Center-of-pressure back-solve for a single point contact on the plate's
    local Y-Z face (local +X is the plate normal / gravity direction -- see
    rotate_wrench_to_plate's docstring). Derivation: contact at r=(0,y,z) under a
    purely-normal force F=(fx,0,0) gives M = r x F = (0, z*fx, -y*fx), so
    y=-Mz/Fx, z=My/Fx (this is the Bicchi et al. CoP formula, axis-permuted for
    this rig's X-normal geometry instead of the more common Z-normal one). Valid
    only away from Fx=0 -- see module docstring. Returned as (plate_x_mm,
    plate_y_mm) == (physical y, physical z) to match the rest of the pipeline's
    generic 2D plate-coordinate naming."""
    if abs(fx) < max(min_fx_n, 1e-6):
        return 0.0, 0.0, False
    return (-mz / fx) * 1000.0, (my / fx) * 1000.0, True


# --------------------------------------------------------------------------------
# CSV logging and JSON parameter presets (Sections 7 and 8-5)
# --------------------------------------------------------------------------------

_CSV_FIELDS = ['timestamp', 'raw_x_mm', 'raw_y_mm', 'filt_x_mm', 'filt_y_mm', 'fx_n', 'my_nm',
               'mz_nm', 'task_x', 'task_y', 'task_z', 'task_rx', 'task_ry', 'task_rz',
               'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'ctrl_dx', 'ctrl_dy', 'ctrl_rx', 'ctrl_ry',
               'confidence', 'event']


class CsvLogger:
    def __init__(self):
        self._fh = None
        self.path: Optional[str] = None

    @property
    def active(self) -> bool:
        return self._fh is not None

    def start(self, path: str):
        self.stop()
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self._fh = open(path, 'w', newline='')
        self.path = path
        self._fh.write(','.join(_CSV_FIELDS) + '\n')

    def stop(self):
        if self._fh is not None:
            self._fh.close()
            self._fh = None
            self.path = None

    def write_row(self, **kw):
        if self._fh is None:
            return
        row = ','.join(str(kw.get(f, '')) for f in _CSV_FIELDS)
        self._fh.write(row + '\n')
        self._fh.flush()


class PresetManager:
    def __init__(self, directory: str = PRESET_DIR):
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def list_presets(self):
        return sorted(f[:-5] for f in os.listdir(self.directory) if f.endswith('.json'))

    def save(self, name: str, payload: dict):
        with open(os.path.join(self.directory, f'{name}.json'), 'w') as fh:
            json.dump(payload, fh, indent=2)

    def load(self, name: str) -> dict:
        with open(os.path.join(self.directory, f'{name}.json')) as fh:
            return json.load(fh)

    def delete(self, name: str):
        path = os.path.join(self.directory, f'{name}.json')
        if os.path.exists(path):
            os.remove(path)


# --------------------------------------------------------------------------------
# Dry-run plant model (Section 6): a minimal rolling-ball-on-plate integrator so the
# full raw-wrench -> filter -> control -> GUI pipeline is exercised identically to
# real hardware, without needing the robot connected.
# --------------------------------------------------------------------------------

class DrySimPhysics:
    ROLLING_FACTOR = 5.0 / 7.0  # solid sphere, rolling without slipping (I = 2/5 m r^2)
    DAMPING = 0.6

    def __init__(self):
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.vx_mms = 0.0
        self.vy_mms = 0.0
        self.departed = False
        self._prev_t: Optional[float] = None
        self._prev_plate_pos = (0.0, 0.0)
        self._prev_plate_vel = (0.0, 0.0)
        self._rng = np.random.default_rng(12345)

    def reset(self, x_mm: float = 0.0, y_mm: float = 0.0):
        self.x_mm, self.y_mm = x_mm, y_mm
        self.vx_mms = self.vy_mms = 0.0
        self.departed = False
        self._prev_t = None

    def step(self, t: float, plate_x_mm: float, plate_y_mm: float,
              rx_deg: float, ry_deg: float, plate_radius_mm: float):
        if self._prev_t is None:
            self._prev_t = t
            self._prev_plate_pos = (plate_x_mm, plate_y_mm)
            self._prev_plate_vel = (0.0, 0.0)
            return
        dt = clamp(t - self._prev_t, 1e-3, 0.1)
        self._prev_t = t

        plate_vx = (plate_x_mm - self._prev_plate_pos[0]) / dt
        plate_vy = (plate_y_mm - self._prev_plate_pos[1]) / dt
        plate_ax = (plate_vx - self._prev_plate_vel[0]) / dt
        plate_ay = (plate_vy - self._prev_plate_vel[1]) / dt
        self._prev_plate_pos, self._prev_plate_vel = (plate_x_mm, plate_y_mm), (plate_vx, plate_vy)

        g_mms2 = GRAVITY_MPS2 * 1000.0
        tilt_ax = self.ROLLING_FACTOR * g_mms2 * math.sin(math.radians(ry_deg))
        tilt_ay = -self.ROLLING_FACTOR * g_mms2 * math.sin(math.radians(rx_deg))
        fictitious_ax = -self.ROLLING_FACTOR * plate_ax
        fictitious_ay = -self.ROLLING_FACTOR * plate_ay

        ax = tilt_ax + fictitious_ax - self.DAMPING * self.vx_mms
        ay = tilt_ay + fictitious_ay - self.DAMPING * self.vy_mms
        self.vx_mms += ax * dt
        self.vy_mms += ay * dt
        self.x_mm += self.vx_mms * dt
        self.y_mm += self.vy_mms * dt

        r = math.hypot(self.x_mm, self.y_mm)
        if r > plate_radius_mm * 1.15:
            self.departed = True
        elif r <= plate_radius_mm:
            self.departed = False

    def synth_wrench_base(self, ball_mass_g: float, plate_x_offset_mm: float,
                            r_base_to_tcp: np.ndarray, noise_n: float, noise_nm: float) -> np.ndarray:
        """Inverse of _process_wrench_sample()'s decode path, for dry-run. Must use
        the same X-normal/Y-Z-plate-face convention as cop_solve()/
        rotate_wrench_to_plate(), or dry-run would silently validate against a
        geometry the real hardware doesn't have (this is exactly the class of bug
        that hid the GetCurrentPosx issue -- see project memory)."""
        # +mg, not -mg: gravity pulls the ball toward local +X (per the rig's
        # geometry), so the plate holds it up with a -X reaction on the ball, and
        # by Newton's third law the ball pushes back on the plate/tool along +X --
        # that reaction is what get_tool_force reports as the external force. (The
        # CoP ratios below are actually sign-invariant to this either way, since
        # both M and F scale together -- but the sign should still match reality.)
        fx_local = 0.0 if self.departed else (ball_mass_g / 1000.0) * GRAVITY_MPS2
        y_m, z_m = self.x_mm / 1000.0, self.y_mm / 1000.0
        f_local = np.array([fx_local, 0.0, 0.0])
        m_plate_center = np.array([0.0, z_m * fx_local, -y_m * fx_local])
        r_offset = np.array([plate_x_offset_mm / 1000.0, 0.0, 0.0])
        # r_offset and f_local are collinear (both along local X) so this cross
        # product is always exactly zero -- kept only for structural symmetry with
        # rotate_wrench_to_plate()'s general-case offset correction.
        m_tcp_local = m_plate_center + np.cross(r_offset, f_local)
        wrench_local = np.concatenate([f_local, m_tcp_local])
        f_base = r_base_to_tcp @ wrench_local[0:3]
        m_base = r_base_to_tcp @ wrench_local[3:6]
        wrench_base = np.concatenate([f_base, m_base])
        noise = self._rng.normal(0.0, 1.0, 6) * np.array([noise_n] * 3 + [noise_nm] * 3)
        return wrench_base + noise


# --------------------------------------------------------------------------------
# Thread-safe blackboard shared between the rclpy spin thread and the Tkinter main
# thread (exactly two threads touch this: every node timer/service callback stays
# on the node's default mutually-exclusive callback group -- see main() -- so they
# never race each other, only the separate Tkinter thread). snapshot()/update() are
# still locked for atomic multi-field reads/writes (e.g. filt_x_mm and filt_y_mm
# read together as one consistent pair). The plain config dataclasses above are
# mutated directly by GUI callbacks and read directly by the control loop without a
# lock, relying on CPython's GIL for atomic single-attribute access -- adequate
# since no field is ever read-modify-written across threads.
# --------------------------------------------------------------------------------

class SharedState:
    def __init__(self):
        self._lock = threading.RLock()
        self.connected = False
        self.dry_run = False
        self.robot_state_code = -1
        self.busy_discrete_move = False

        self.raw_x_mm = 0.0
        self.raw_y_mm = 0.0
        self.filt_x_mm = 0.0
        self.filt_y_mm = 0.0
        self.ball_valid = False
        self.ball_present = False
        self.confidence = 'red'

        self.wrench_plate = np.zeros(6)  # Fx,Fy,Fz,Mx,My,Mz in the plate-local frame, post gate/deadzone
        self.fx_n = 0.0  # plate-normal force (local +X on this rig -- see cop_solve() docstring)

        self.task_pose = np.zeros(6)
        self.joint_pose = np.zeros(6)

        self.ctrl_translation_mm = (0.0, 0.0)
        self.ctrl_tilt_deg = (0.0, 0.0)
        self.auto_bias_deg = (0.0, 0.0)
        self.error_mm = 0.0
        self.rmse_mm = 0.0
        self.overshoot_mm = 0.0
        self.settling_s: Optional[float] = None

        self.estop_active = False
        self.comm_error = False
        self.ball_departed = False
        self.limit_hit = False
        self.alerts = deque(maxlen=300)

        self.zero_bias = np.zeros(6)

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def add_alert(self, level: str, message: str):
        with self._lock:
            self.alerts.append((time.time(), level, message))

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'connected': self.connected, 'dry_run': self.dry_run,
                'robot_state_code': self.robot_state_code,
                'busy_discrete_move': self.busy_discrete_move,
                'raw_x_mm': self.raw_x_mm, 'raw_y_mm': self.raw_y_mm,
                'filt_x_mm': self.filt_x_mm, 'filt_y_mm': self.filt_y_mm,
                'ball_valid': self.ball_valid, 'ball_present': self.ball_present,
                'confidence': self.confidence,
                'wrench_plate': self.wrench_plate.copy(), 'fx_n': self.fx_n,
                'task_pose': self.task_pose.copy(), 'joint_pose': self.joint_pose.copy(),
                'ctrl_translation_mm': self.ctrl_translation_mm,
                'ctrl_tilt_deg': self.ctrl_tilt_deg, 'auto_bias_deg': self.auto_bias_deg,
                'error_mm': self.error_mm, 'rmse_mm': self.rmse_mm,
                'overshoot_mm': self.overshoot_mm, 'settling_s': self.settling_s,
                'estop_active': self.estop_active, 'comm_error': self.comm_error,
                'ball_departed': self.ball_departed, 'limit_hit': self.limit_hit,
                'alerts': list(self.alerts),
            }


# --------------------------------------------------------------------------------
# The ROS2 node. Owns every service client/publisher/timer, the estimation +
# control math, and the safety state machine. The Tkinter GUI (further below)
# only ever touches this through SharedState + the plain config dataclasses, plus
# a handful of thread-safe command methods (estop, home, zero, jog, reconnect).
# --------------------------------------------------------------------------------

class BallBalanceNode(Node):
    def __init__(self):
        super().__init__('ball_balance_node')
        self.declare_parameter('robot_namespace', DEFAULT_NAMESPACE)
        self.ns = self.get_parameter('robot_namespace').get_parameter_value().string_value or DEFAULT_NAMESPACE

        self.physical = PhysicalParams()
        self.filters = FilterParams()
        self.safety = SafetyLimits()
        self.control = ControlParams()
        self.manual = ManualTarget()
        self.home_joints_deg = list(DEFAULT_HOME_JOINTS_DEG)

        self.shared = SharedState()
        # The operator HMI controls the physical robot only. Simulation can no
        # longer be selected accidentally from the GUI.
        self.shared.dry_run = False

        self._wrench_gates = [OutlierGate() for _ in range(6)]  # Fx,Fy,Fz,Mx,My,Mz
        self._ma_x, self._ma_y = MovingAverage(), MovingAverage()
        self._lpf_x, self._lpf_y = LowPassFilter(), LowPassFilter()
        self._kf = KalmanFilter2D()

        self._trans_pid_x, self._trans_pid_y = PID(), PID()
        self._tilt_pid_x, self._tilt_pid_y = PID(), PID()
        self._slew = SlewLimiter(6)
        self._perf = PerformanceTracker()
        self._csv = CsvLogger()
        self._presets = PresetManager()
        self._sim = DrySimPhysics()
        self._auto_bias_rx = 0.0
        self._auto_bias_ry = 0.0

        self._r_base_to_tcp = np.eye(3)
        self._reference_task_pose: Optional[np.ndarray] = None
        self._reference_joint_pose: Optional[np.ndarray] = None
        self._commanded_pose = np.zeros(6)
        self._pending_discrete_move = False
        self._ball_lost_since: Optional[float] = None
        self._departed_alert_sent = False
        self._last_raw_wrench = np.zeros(6)
        self._last_tool_force_ok_t = time.time()
        self._last_posx_ok_t = time.time()
        self._active_sample_hz = self.filters.sample_hz
        self._requested_ns: Optional[str] = None
        # Motion output is opt-in. The simplified HMI's START button enables
        # streaming; STOP/E-STOP disable it immediately.
        self.motion_enabled = False

        self.cli_move_joint = self.create_client(MoveJoint, f'/{self.ns}/motion/move_joint')
        self.cli_move_line = self.create_client(MoveLine, f'/{self.ns}/motion/move_line')
        self.cli_move_stop = self.create_client(MoveStop, f'/{self.ns}/motion/move_stop')
        self.cli_get_tool_force = self.create_client(GetToolForce, f'/{self.ns}/aux_control/get_tool_force')
        self.cli_get_posx = self.create_client(GetCurrentPosx, f'/{self.ns}/aux_control/get_current_posx')
        self.cli_get_posj = self.create_client(GetCurrentPosj, f'/{self.ns}/aux_control/get_current_posj')
        self.cli_get_robot_state = self.create_client(GetRobotState, f'/{self.ns}/system/get_robot_state')
        # OnRobot gripper service lives outside the /dsr01 namespace, so it is
        # namespace-independent and created only once here.
        self.cli_gripper = (
            self.create_client(SetCommand, GRIPPER_SERVICE)
            if _HAVE_GRIPPER_SRV else None
        )

        # dsr_controller2.cpp subscribes with create_subscription<ServolStream>("servol_stream",
        # 20, ...) -- a plain-integer QoS arg, which resolves to rclcpp's default RELIABLE +
        # VOLATILE + KEEP_LAST. A BEST_EFFORT publisher is incompatible with that (DDS drops
        # the match entirely -- "requesting incompatible QoS... Last incompatible policy:
        # RELIABILITY" -- so every ServolStream message was being silently discarded and the
        # robot never moved), so this must match RELIABLE, not BEST_EFFORT.
        stream_qos = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub_servol_stream = self.create_publisher(ServolStream, f'/{self.ns}/servol_stream', stream_qos)

        self.timer_control = self.create_timer(1.0 / CONTROL_LOOP_HZ, self._on_control_tick)
        self.timer_pose = self.create_timer(1.0 / POSE_POLL_HZ, self._on_pose_tick)
        self.timer_status = self.create_timer(1.0 / ROBOT_STATUS_POLL_HZ, self._on_status_tick)
        self.timer_sample = self.create_timer(1.0 / self._active_sample_hz, self._on_sample_tick)

        self.get_logger().info(
            f'ball_balance_node up, namespace=/{self.ns}, dry_run={self.shared.dry_run}. '
            f'Verify dsr_msgs2 interface names with e.g. '
            f"'ros2 interface show dsr_msgs2/srv/GetToolForce' if service calls warn."
        )

    def set_namespace(self, ns: str):
        """Rebuild every client/publisher under a new namespace (Section 8 requires
        the ROS2 connection itself, not just parameters, to be GUI-editable)."""
        self.ns = ns or DEFAULT_NAMESPACE
        self.cli_move_joint = self.create_client(MoveJoint, f'/{self.ns}/motion/move_joint')
        self.cli_move_line = self.create_client(MoveLine, f'/{self.ns}/motion/move_line')
        self.cli_move_stop = self.create_client(MoveStop, f'/{self.ns}/motion/move_stop')
        self.cli_get_tool_force = self.create_client(GetToolForce, f'/{self.ns}/aux_control/get_tool_force')
        self.cli_get_posx = self.create_client(GetCurrentPosx, f'/{self.ns}/aux_control/get_current_posx')
        self.cli_get_posj = self.create_client(GetCurrentPosj, f'/{self.ns}/aux_control/get_current_posj')
        self.cli_get_robot_state = self.create_client(GetRobotState, f'/{self.ns}/system/get_robot_state')
        self.destroy_publisher(self.pub_servol_stream)
        stream_qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                                 history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.VOLATILE)
        self.pub_servol_stream = self.create_publisher(ServolStream, f'/{self.ns}/servol_stream', stream_qos)
        self._reference_task_pose = None
        self._reference_joint_pose = None
        self.shared.add_alert('info', f'Namespace changed to /{self.ns}')

    # ---------------------------------------------------------------- helpers

    def _note_comm_issue(self, msg: str):
        self.shared.add_alert('warn', msg)

    def _recreate_sample_timer(self, hz: float):
        hz = clamp(hz, 1.0, 200.0)
        self.destroy_timer(self.timer_sample)
        self.timer_sample = self.create_timer(1.0 / hz, self._on_sample_tick)
        self._active_sample_hz = hz

    # ---------------------------------------------------------- state polling
    # RobotState/RobotStateRt exist as dsr_msgs2 types but the driver never
    # actually publishes them (verified against dsr_controller2 source) -- so,
    # unlike a literal "subscribe to a state topic", state here comes from these
    # three services, each on its own timer per the required Hz separation.

    def _on_pose_tick(self):
        if self.shared.dry_run:
            return  # dry-run pose comes from our own commanded/simulated trajectory
        if self.cli_get_posx.service_is_ready():
            req = GetCurrentPosx.Request()
            safe_set(req, ref=0)  # DR_BASE -- matches the base-frame assumption used throughout
            fut = self.cli_get_posx.call_async(req)
            fut.add_done_callback(self._on_posx_response)
        else:
            self._note_comm_issue('aux_control/get_current_posx not available')
        if self.cli_get_posj.service_is_ready():
            fut2 = self.cli_get_posj.call_async(GetCurrentPosj.Request())
            fut2.add_done_callback(self._on_posj_response)

    def _on_posx_response(self, fut):
        try:
            resp = fut.result()
        except Exception as exc:
            self._note_comm_issue(f'get_current_posx failed: {exc}')
            return
        # GetCurrentPosx.srv's response is `std_msgs/Float64MultiArray[] task_pos_info`,
        # NOT a flat 'pos' field -- task_pos_info[0].data == [x,y,z,a,b,c,solution_space]
        # (see module docstring's interface-verification note). Getting this wrong means
        # _r_base_to_tcp and _reference_task_pose never leave their startup defaults on
        # real hardware, silently disabling both the pose-independent CoP transform and
        # the control loop (which refuses to run while _reference_task_pose is None).
        info = safe_get(resp, 'task_pos_info', default=None)
        if not info or len(info[0].data) < 6:
            return
        pose = np.array(list(info[0].data)[:6], dtype=float)
        self._r_base_to_tcp = euler_zyz_to_matrix(pose[3], pose[4], pose[5])
        if self._reference_task_pose is None:
            self._reference_task_pose = pose.copy()
            self._commanded_pose = pose.copy()
            self._slew.reset(pose)
            self.manual.x_mm, self.manual.y_mm, self.manual.z_mm = 0.0, 0.0, 0.0
            self.manual.rx_deg, self.manual.ry_deg, self.manual.rz_deg = 0.0, 0.0, 0.0
        self.shared.update(task_pose=pose)
        self._last_posx_ok_t = time.time()

    def _on_posj_response(self, fut):
        try:
            resp = fut.result()
        except Exception as exc:
            self._note_comm_issue(f'get_current_posj failed: {exc}')
            return
        pos = safe_get(resp, 'pos', 'joint_pos', 'posj', default=None)
        if pos is None or len(pos) < 6:
            return
        joints = np.array(list(pos)[:6], dtype=float)
        if self._reference_joint_pose is None:
            self._reference_joint_pose = joints.copy()
        self.shared.update(joint_pose=joints)

    def request_namespace_change(self, ns: str):
        """Thread-safe entry point for the GUI thread: only sets a flag. The actual
        rclpy client/publisher rebuild happens on the executor thread in
        _on_status_tick, since mutating a Node's clients/publishers concurrently
        with the executor spinning it is not something rclpy is documented safe for."""
        self._requested_ns = ns

    def _on_status_tick(self):
        if self._requested_ns is not None:
            ns, self._requested_ns = self._requested_ns, None
            self.set_namespace(ns)
        if abs(self.filters.sample_hz - self._active_sample_hz) > 1e-6:
            self._recreate_sample_timer(self.filters.sample_hz)
        if self.shared.dry_run:
            self.shared.update(connected=True, robot_state_code=-1)
            return
        if not self.cli_get_robot_state.service_is_ready():
            self.shared.update(connected=False)
            return
        fut = self.cli_get_robot_state.call_async(GetRobotState.Request())
        fut.add_done_callback(self._on_robot_state_response)

    def _on_robot_state_response(self, fut):
        try:
            resp = fut.result()
        except Exception as exc:
            self._note_comm_issue(f'get_robot_state failed: {exc}')
            self.shared.update(connected=False)
            return
        code = safe_get(resp, 'robot_state', 'state', default=-1)
        self.shared.update(connected=True, robot_state_code=int(code))

    def _on_sample_tick(self):
        t = time.time()
        if self.shared.dry_run:
            self._sim.step(t, self._commanded_pose[0], self._commanded_pose[1],
                            self._commanded_pose[3], self._commanded_pose[4],
                            self.physical.plate_radius_mm)
            self._r_base_to_tcp = euler_zyz_to_matrix(
                self._commanded_pose[3], self._commanded_pose[4], self._commanded_pose[5])
            wrench = self._sim.synth_wrench_base(
                self.physical.ball_mass_g, self.physical.tcp_to_plate_offset_mm,
                self._r_base_to_tcp, noise_n=0.03, noise_nm=0.003)
            self.shared.update(task_pose=self._commanded_pose.copy(), connected=True)
            self._process_wrench_sample(t, wrench)
            return
        if not self.cli_get_tool_force.service_is_ready():
            self._note_comm_issue('aux_control/get_tool_force not available')
            return
        req = GetToolForce.Request()
        safe_set(req, ref=0)
        fut = self.cli_get_tool_force.call_async(req)
        fut.add_done_callback(lambda f: self._on_tool_force_response(f, t))

    def _on_tool_force_response(self, fut, t: float):
        try:
            resp = fut.result()
        except Exception as exc:
            self._note_comm_issue(f'get_tool_force failed: {exc}')
            return
        raw = safe_get(resp, 'tool_force', 'force', 'data', default=None)
        if raw is None or len(raw) < 6:
            return
        wrench = np.array(list(raw)[:6], dtype=float)
        if not np.all(np.isfinite(wrench)):
            self._note_comm_issue('get_tool_force returned NaN/Inf')
            return
        self._last_tool_force_ok_t = t
        self._process_wrench_sample(t, wrench)

    # ------------------------------------------------------- estimation + filtering
    # Section 1 (pose-independent CoP estimate) + Section 2 (noise pipeline) live
    # here: outlier-gate + dead-zone protect the CoP division (most noise-sensitive
    # step), then moving-average -> low-pass -> Kalman smooth the resulting (x, y).

    def _process_wrench_sample(self, t: float, wrench_base: np.ndarray):
        self._last_raw_wrench = wrench_base.copy()
        wrench = wrench_base - self.shared.zero_bias

        # Gate all 6 raw components *before* rotating -- once rotated, a noisy
        # Fy/Fz/Mx leaks into the plate-local Fx/My/Mz whenever the plate is
        # tilted (rotation mixes axes), so gating only the post-rotation channels
        # would miss that coupling.
        gated = np.empty(6)
        bad_flags = [False] * 6
        for i in range(6):
            gated[i], bad_flags[i] = self._wrench_gates[i].update(float(wrench[i]), self.filters.outlier_mad_k)

        f_plate, m_plate_center = rotate_wrench_to_plate(
            gated, self._r_base_to_tcp, self.physical.tcp_to_plate_offset_mm)
        # Local +X is this rig's plate normal (Y-Z is the plate face) -- see
        # rotate_wrench_to_plate()/cop_solve()'s docstrings.
        fx, my, mz = float(f_plate[0]), float(m_plate_center[1]), float(m_plate_center[2])
        if abs(fx) < self.filters.deadzone_n:
            fx = 0.0
        if abs(my) < self.filters.deadzone_nm:
            my = 0.0
        if abs(mz) < self.filters.deadzone_nm:
            mz = 0.0
        raw_x, raw_y, valid = cop_solve(fx, my, mz, self.filters.min_fx_n)
        wrench_clean = np.array([fx, f_plate[1], f_plate[2], m_plate_center[0], my, mz])
        fx_bad, my_bad, mz_bad = bad_flags[0], bad_flags[4], bad_flags[5]

        hz = max(self.filters.sample_hz, 1e-3)
        ma_x = self._ma_x.update(raw_x, self.filters.moving_avg_window_ms, hz)
        ma_y = self._ma_y.update(raw_y, self.filters.moving_avg_window_ms, hz)
        lp_x = self._lpf_x.update(ma_x, self.filters.lpf_cutoff_hz, hz)
        lp_y = self._lpf_y.update(ma_y, self.filters.lpf_cutoff_hz, hz)
        filt_x, filt_y = self._kf.update(lp_x, lp_y, 1.0 / hz,
                                          self.filters.kalman_process_noise,
                                          self.filters.kalman_measurement_noise)

        expected_fx = (self.physical.ball_mass_g / 1000.0) * GRAVITY_MPS2
        ball_present = valid and abs(fx) >= self.safety.no_ball_fx_n
        radius_ok = math.hypot(filt_x, filt_y) <= self.physical.plate_radius_mm * 1.2
        plausible = abs(fx) <= expected_fx * 4.0 + 1.0

        if not ball_present:
            if self._ball_lost_since is None:
                self._ball_lost_since = t
        else:
            self._ball_lost_since = None
        departed = (self._ball_lost_since is not None
                    and (t - self._ball_lost_since) > self.safety.departure_hold_s)

        noisy = fx_bad or my_bad or mz_bad
        if not valid or not plausible or departed:
            confidence = 'red'
        elif noisy or not radius_ok:
            confidence = 'yellow'
        else:
            confidence = 'green'

        self.shared.update(
            raw_x_mm=raw_x, raw_y_mm=raw_y, filt_x_mm=filt_x, filt_y_mm=filt_y,
            ball_valid=valid, ball_present=ball_present, confidence=confidence,
            wrench_plate=wrench_clean, fx_n=fx, ball_departed=departed,
        )
        if departed and not self._departed_alert_sent:
            self.shared.add_alert('warn', 'Ball departure detected -- balancing paused')
            self._departed_alert_sent = True
        elif not departed:
            self._departed_alert_sent = False

        if self._csv.active:
            tp, jp = self.shared.task_pose, self.shared.joint_pose
            dx, dy = self.shared.ctrl_translation_mm
            crx, cry = self.shared.ctrl_tilt_deg
            self._csv.write_row(
                timestamp=datetime.now().isoformat(),
                raw_x_mm=f'{raw_x:.2f}', raw_y_mm=f'{raw_y:.2f}',
                filt_x_mm=f'{filt_x:.2f}', filt_y_mm=f'{filt_y:.2f}',
                fx_n=f'{fx:.3f}', my_nm=f'{my:.4f}', mz_nm=f'{mz:.4f}',
                task_x=f'{tp[0]:.2f}', task_y=f'{tp[1]:.2f}', task_z=f'{tp[2]:.2f}',
                task_rx=f'{tp[3]:.2f}', task_ry=f'{tp[4]:.2f}', task_rz=f'{tp[5]:.2f}',
                j1=f'{jp[0]:.2f}', j2=f'{jp[1]:.2f}', j3=f'{jp[2]:.2f}',
                j4=f'{jp[3]:.2f}', j5=f'{jp[4]:.2f}', j6=f'{jp[5]:.2f}',
                ctrl_dx=f'{dx:.2f}', ctrl_dy=f'{dy:.2f}', ctrl_rx=f'{crx:.3f}', ctrl_ry=f'{cry:.3f}',
                confidence=confidence, event='',
            )

    # ------------------------------------------------------------- control loop
    # Section 3: fixed-rate, translation-priority PID with tilt as a weighted
    # secondary correction, an optional slow adaptive tilt-bias search, manual+auto
    # superposition, then hard safety clamps and a vel/acc slew limiter before
    # publishing. Runs continuously regardless of manual/auto mode (spec 3).

    def _on_control_tick(self):
        # One locked snapshot for the whole tick: the GUI thread can call
        # cmd_set_dry_run()/cmd_estop()/cmd_zero() (which write SharedState)
        # concurrently with this timer callback, so reading several related fields
        # (e.g. filt_x_mm and filt_y_mm as one pair) needs a single consistent view
        # rather than piecemeal unlocked attribute reads.
        snap = self.shared.snapshot()
        t = time.time()
        dt = 1.0 / CONTROL_LOOP_HZ

        if not self.motion_enabled:
            return

        # Manual XYZ jog is issued as one relative MoveLine service request per
        # button click.  ServolStream is reserved for the balancing controller;
        # publishing both command types would make them fight over the robot.
        if not self.control.balancing_enabled:
            return

        if snap['dry_run'] and self._reference_task_pose is None:
            self._reference_task_pose = np.zeros(6)
            self._reference_joint_pose = np.array(self.home_joints_deg, dtype=float)
            self._commanded_pose = np.zeros(6)
            self._slew.reset(self._commanded_pose)

        if snap['estop_active'] or self._pending_discrete_move or self._reference_task_pose is None:
            return

        comm_error = False
        if not snap['dry_run']:
            comm_error = ((t - self._last_tool_force_ok_t > self.safety.comm_timeout_s)
                          or (t - self._last_posx_ok_t > self.safety.comm_timeout_s))
        if comm_error != snap['comm_error']:
            self.shared.update(comm_error=comm_error)
            if comm_error:
                self.shared.add_alert('error', 'Communication timeout -- holding last commanded pose')
            else:
                self.shared.add_alert('info', 'Communication recovered')
        if comm_error:
            return

        apply_balancing = (self.control.balancing_enabled and snap['ball_present']
                            and not snap['ball_departed'] and snap['confidence'] != 'red')
        ex, ey = snap['filt_x_mm'], snap['filt_y_mm']
        err_mm = math.hypot(ex, ey)
        self._perf.update(t, err_mm)
        self.shared.update(error_mm=err_mm, rmse_mm=self._perf.rmse(),
                            overshoot_mm=self._perf.overshoot_mm(), settling_s=self._perf.settling_time_s())

        if apply_balancing:
            dx = self._trans_pid_x.update(ex, dt, self.control.trans_kp, self.control.trans_ki,
                                           self.control.trans_kd, i_limit=50.0)
            dy = self._trans_pid_y.update(ey, dt, self.control.trans_kp, self.control.trans_ki,
                                           self.control.trans_kd, i_limit=50.0)
            # Cross-axis coupling (classic ball-and-plate layout): an X offset is
            # corrected by tilting about Y, a Y offset by tilting about X.
            tilt_from_ex = self._tilt_pid_x.update(ex, dt, self.control.tilt_kp, self.control.tilt_ki,
                                                    self.control.tilt_kd, i_limit=5.0)
            tilt_from_ey = self._tilt_pid_y.update(ey, dt, self.control.tilt_kp, self.control.tilt_ki,
                                                    self.control.tilt_kd, i_limit=5.0)
            d_ry = -tilt_from_ex * self.control.tilt_weight
            d_rx = tilt_from_ey * self.control.tilt_weight
            if self.control.auto_search_enabled:
                self._auto_bias_ry = clamp(
                    self._auto_bias_ry - self.control.auto_search_gain * ex * dt,
                    -self.control.max_auto_bias_deg, self.control.max_auto_bias_deg)
                self._auto_bias_rx = clamp(
                    self._auto_bias_rx + self.control.auto_search_gain * ey * dt,
                    -self.control.max_auto_bias_deg, self.control.max_auto_bias_deg)
        else:
            dx = dy = d_rx = d_ry = 0.0
            self._trans_pid_x.reset()
            self._trans_pid_y.reset()
            self._tilt_pid_x.reset()
            self._tilt_pid_y.reset()

        self.shared.update(ctrl_translation_mm=(dx, dy), ctrl_tilt_deg=(d_rx, d_ry),
                            auto_bias_deg=(self._auto_bias_rx, self._auto_bias_ry))

        ref = self._reference_task_pose
        pre_clamp = np.array([
            ref[0] + self.manual.x_mm + dx,
            ref[1] + self.manual.y_mm + dy,
            ref[2] + self.manual.z_mm,
            ref[3] + self.manual.rx_deg + d_rx + self._auto_bias_rx,
            ref[4] + self.manual.ry_deg + d_ry + self._auto_bias_ry,
            ref[5] + self.manual.rz_deg,
        ])
        target = pre_clamp.copy()
        target[0] = clamp(target[0], ref[0] + self.safety.ws_x_min_mm, ref[0] + self.safety.ws_x_max_mm)
        target[1] = clamp(target[1], ref[1] + self.safety.ws_y_min_mm, ref[1] + self.safety.ws_y_max_mm)
        target[2] = clamp(target[2], ref[2] + self.safety.ws_z_min_mm, ref[2] + self.safety.ws_z_max_mm)
        target[3] = clamp(target[3], ref[3] - self.safety.max_tilt_deg, ref[3] + self.safety.max_tilt_deg)
        target[4] = clamp(target[4], ref[4] - self.safety.max_tilt_deg, ref[4] + self.safety.max_tilt_deg)
        limit_hit = not np.allclose(target, pre_clamp, atol=1e-6)
        if limit_hit != snap['limit_hit']:
            self.shared.update(limit_hit=limit_hit)
            if limit_hit:
                self.shared.add_alert('warn', 'Workspace/tilt limit reached -- command clamped')

        max_vel = np.array([self.safety.max_lin_vel_mms] * 3 + [self.safety.max_tilt_vel_dps] * 3)
        max_acc = np.array([self.safety.max_lin_acc_mms2] * 3 + [self.safety.max_tilt_acc_dps2] * 3)
        self._commanded_pose = self._slew.update(target, dt, max_vel, max_acc)

        if snap['dry_run']:
            return  # DrySimPhysics consumes self._commanded_pose on the next sample tick

        msg = ServolStream()
        safe_set(msg, pos=[float(v) for v in self._commanded_pose],
                 vel=[float(self.safety.max_lin_vel_mms), float(self.safety.max_tilt_vel_dps)],
                 acc=[float(self.safety.max_lin_acc_mms2), float(self.safety.max_tilt_acc_dps2)],
                 time=dt)
        self.pub_servol_stream.publish(msg)

    # ------------------------------------------------------------ GUI-facing commands

    def cmd_estop(self):
        self.motion_enabled = False
        self.shared.update(estop_active=True)
        self.shared.add_alert('error', 'E-STOP engaged')
        if not self.shared.dry_run and self.cli_move_stop.service_is_ready():
            req = MoveStop.Request()
            safe_set(req, stop_mode=0)  # DR_QSTOP_STO -- immediate Safe-Torque-Off stop
            self.cli_move_stop.call_async(req)

    def cmd_clear_estop(self):
        self.shared.update(estop_active=False)
        self._trans_pid_x.reset()
        self._trans_pid_y.reset()
        self._tilt_pid_x.reset()
        self._tilt_pid_y.reset()
        self._slew.reset(self._commanded_pose)
        self.shared.add_alert('info', 'E-STOP cleared')

    def cmd_start_motion(self):
        snap = self.shared.snapshot()
        if snap['estop_active']:
            self.shared.add_alert('error', 'E-STOP을 먼저 해제하세요')
            return
        if not snap['dry_run'] and not snap['connected']:
            self.shared.add_alert('error', '로봇이 연결되지 않아 START할 수 없습니다')
            return
        self.manual.x_mm = self.manual.y_mm = self.manual.z_mm = 0.0
        self.manual.rx_deg = self.manual.ry_deg = self.manual.rz_deg = 0.0
        # The simplified HMI START button arms manual TCP jog only. Automatic
        # balancing is a separate experiment and must not alter the taught pose.
        self.control.balancing_enabled = False
        self._reference_task_pose = None
        self.motion_enabled = True
        self.shared.add_alert(
            'info',
            'MANUAL moveL START -- XYZ 버튼은 DR_BASE 기준 상대 직선이동',
        )

    def cmd_stop_motion(self):
        self.motion_enabled = False
        if not self.shared.dry_run and self.cli_move_stop.service_is_ready():
            req = MoveStop.Request()
            safe_set(req, stop_mode=2)  # DR_SSTOP: controlled soft stop
            self.cli_move_stop.call_async(req)
        self.shared.add_alert('warn', 'Motion STOP -- 현재 위치를 유지합니다')

    def cmd_zero(self):
        self.shared.update(zero_bias=self._last_raw_wrench.copy())
        self.shared.add_alert('info', 'Zeroing applied (no-ball baseline captured)')

    def cmd_set_dry_run(self, enabled: bool):
        self.motion_enabled = False
        self.control.balancing_enabled = False
        self.shared.update(dry_run=enabled)
        self._reference_task_pose = None
        self._reference_joint_pose = None
        self._trans_pid_x.reset()
        self._trans_pid_y.reset()
        self._tilt_pid_x.reset()
        self._tilt_pid_y.reset()
        self._sim.reset()
        self.shared.add_alert('info', f"Dry-run {'enabled' if enabled else 'disabled'}")

    def cmd_home(self):
        if self.shared.estop_active or self._pending_discrete_move:
            return
        self.manual.x_mm = self.manual.y_mm = self.manual.z_mm = 0.0
        self.manual.rx_deg = self.manual.ry_deg = self.manual.rz_deg = 0.0
        self._auto_bias_rx = self._auto_bias_ry = 0.0
        if self.shared.dry_run:
            self._sim.reset()
            self._reference_task_pose = None
            self.shared.add_alert('info', 'Home (simulated reset)')
            return
        if not self.cli_move_joint.service_is_ready():
            self.shared.add_alert('error', 'motion/move_joint service not available')
            return
        self._pending_discrete_move = True
        self.shared.update(busy_discrete_move=True)
        req = MoveJoint.Request()
        safe_set(req, pos=[float(v) for v in self.home_joints_deg],
                 vel=float(self.safety.joint_move_vel_dps),
                 acc=float(self.safety.joint_move_acc_dps2),
                 time=0.0, radius=0.0, mode=0, blend_type=0, sync_type=0)
        fut = self.cli_move_joint.call_async(req)
        fut.add_done_callback(self._on_discrete_move_done)

    def cmd_set_home_from_current(self):
        """Section 6 requires both a home 'set' and a home 'return' -- cmd_home() above
        is the return half; this captures the robot's current joint pose as the new
        home target, so the safe-return pose isn't stuck at DEFAULT_HOME_JOINTS_DEG."""
        if self.shared.estop_active or self._pending_discrete_move:
            return
        current = self.shared.joint_pose
        if not np.all(np.isfinite(current)):
            self.shared.add_alert('error', '현재 조인트 값이 유효하지 않아 홈으로 저장할 수 없습니다')
            return
        self.home_joints_deg = [float(v) for v in current]
        joints_txt = '  '.join(f'{v:.1f}' for v in self.home_joints_deg)
        self.shared.add_alert('info', f'홈 포지션을 현재 자세로 저장: {joints_txt}')

    def cmd_jog_joints(self, joints_deg):
        if self.shared.estop_active or self._pending_discrete_move:
            return
        if self.shared.dry_run:
            self._reference_joint_pose = np.array(joints_deg, dtype=float)
            self.shared.update(joint_pose=self._reference_joint_pose.copy())
            self.shared.add_alert('info', 'Joint jog (simulated)')
            return
        if not self.cli_move_joint.service_is_ready():
            self.shared.add_alert('error', 'motion/move_joint service not available')
            return
        self._pending_discrete_move = True
        self.shared.update(busy_discrete_move=True)
        req = MoveJoint.Request()
        safe_set(req, pos=[float(v) for v in joints_deg],
                 vel=float(self.safety.joint_move_vel_dps),
                 acc=float(self.safety.joint_move_acc_dps2),
                 time=0.0, radius=0.0, mode=0, blend_type=0, sync_type=0)
        fut = self.cli_move_joint.call_async(req)
        fut.add_done_callback(self._on_discrete_move_done)

    def cmd_jog_task(self, axis: int, distance: float):
        """Jog the active TCP once along a DR_BASE axis using relative moveL.

        Axes 0/1/2 are X/Y/Z translation (mm); 3/4/5 are Rx/Ry/Rz rotation (deg).
        """
        snap = self.shared.snapshot()
        axis_names = ('X', 'Y', 'Z', 'Rx', 'Ry', 'Rz')
        axis_units = ('mm', 'mm', 'mm', 'deg', 'deg', 'deg')
        if axis not in range(6):
            self.shared.add_alert('error', f'Invalid task axis: {axis}')
            return
        if not self.motion_enabled:
            self.shared.add_alert('warn', 'START를 누른 뒤 수동 이동을 사용하세요')
            return
        if snap['estop_active']:
            self.shared.add_alert('error', 'E-STOP을 먼저 해제하세요')
            return
        if self._pending_discrete_move:
            self.shared.add_alert('warn', '이전 moveL 동작이 끝날 때까지 기다리세요')
            return

        axis_name = axis_names[axis]
        unit = axis_units[axis]
        if snap['dry_run']:
            simulated = np.array(snap['task_pose'], dtype=float)
            simulated[axis] += float(distance)
            self.shared.update(task_pose=simulated)
            self.shared.add_alert(
                'info',
                f'[DRY-RUN] moveL REL/DR_BASE {axis_name} {distance:+.1f} {unit}',
            )
            return
        if not snap['connected']:
            self.shared.add_alert('error', '로봇이 연결되지 않아 moveL을 실행할 수 없습니다')
            return
        if not self.cli_move_line.service_is_ready():
            self.shared.add_alert('error', 'motion/move_line service not available')
            return

        target = [0.0] * 6
        target[axis] = float(distance)
        req = MoveLine.Request()
        safe_set(
            req,
            pos=target,
            vel=[
                float(self.safety.max_lin_vel_mms),
                # Rotation jog needs a usable angular rate, not the tiny
                # balancing tilt limit; harmless for pure-translation jogs.
                JOG_ROT_VEL_DPS,
            ],
            acc=[
                float(self.safety.max_lin_acc_mms2),
                JOG_ROT_ACC_DPS2,
            ],
            time=0.0,
            radius=0.0,
            ref=0,          # DR_BASE
            mode=1,         # DR_MV_MOD_REL
            blend_type=0,
            sync_type=0,
        )
        self._pending_discrete_move = True
        self.shared.update(busy_discrete_move=True)
        self.shared.add_alert(
            'info',
            f'moveL command: REL/DR_BASE {axis_name} {distance:+.1f} {unit}',
        )
        fut = self.cli_move_line.call_async(req)
        fut.add_done_callback(self._on_discrete_move_done)

    def cmd_jog_joint6(self, delta_deg: float):
        """Rotate the gripper by jogging joint 6 relative to the current pose."""
        if not self.motion_enabled:
            self.shared.add_alert('warn', 'START를 누른 뒤 그리퍼 회전을 사용하세요')
            return
        current = np.array(self.shared.joint_pose, dtype=float)
        if current.shape[0] < 6:
            self.shared.add_alert('error', '현재 조인트 값을 읽을 수 없습니다')
            return
        target = current.copy()
        target[5] += float(delta_deg)
        self.shared.add_alert('info', f'그리퍼 회전 J6 {delta_deg:+.1f} deg')
        self.cmd_jog_joints(target.tolist())

    def cmd_gripper(self, command: str):
        """Open ('o') or close ('c') the OnRobot gripper via its ROS service."""
        label = '열기' if command == 'o' else '닫기'
        snap = self.shared.snapshot()
        if snap['dry_run']:
            self.shared.add_alert('info', f'[DRY-RUN] 그리퍼 {label}')
            return
        if self.cli_gripper is None:
            self.shared.add_alert(
                'error', 'onrobot_rg_msgs 미설치: 그리퍼 서비스 사용 불가'
            )
            return
        if not self.cli_gripper.service_is_ready():
            self.shared.add_alert(
                'error',
                f"그리퍼 서비스 '{GRIPPER_SERVICE}' 없음 "
                '(onrobot_rg_control 실행 확인)',
            )
            return
        req = SetCommand.Request()
        req.command = command
        self.shared.add_alert('info', f'그리퍼 {label} 명령')
        fut = self.cli_gripper.call_async(req)

        def _done(f):
            try:
                resp = f.result()
                ok = safe_get(resp, 'success', default=True)
                if not ok:
                    self.shared.add_alert('error', f'그리퍼 {label} 실패')
            except Exception as exc:
                self.shared.add_alert('error', f'그리퍼 {label} 예외: {exc}')

        fut.add_done_callback(_done)

    def _on_discrete_move_done(self, fut):
        self._pending_discrete_move = False
        self._reference_task_pose = None
        self._reference_joint_pose = None
        self.shared.update(busy_discrete_move=False)
        try:
            resp = fut.result()
            ok = safe_get(resp, 'success', default=True)
            self.shared.add_alert('info' if ok else 'error', f"Discrete move {'done' if ok else 'failed'}")
        except Exception as exc:
            self.shared.add_alert('error', f'Discrete move failed: {exc}')

    def cmd_start_logging(self, path: str):
        try:
            self._csv.start(path)
            self.shared.add_alert('info', f'Logging started: {path}')
        except OSError as exc:
            self.shared.add_alert('error', f'Could not start logging: {exc}')

    def cmd_stop_logging(self):
        self._csv.stop()
        self.shared.add_alert('info', 'Logging stopped')

    @property
    def logging_active(self) -> bool:
        return self._csv.active

    @property
    def logging_path(self) -> Optional[str]:
        return self._csv.path

    def _all_params_dict(self) -> dict:
        return {
            'physical': asdict(self.physical), 'filters': asdict(self.filters),
            'safety': asdict(self.safety), 'control': asdict(self.control),
            'home_joints_deg': list(self.home_joints_deg),
        }

    def _apply_params_dict(self, d: dict):
        for key, obj in (('physical', self.physical), ('filters', self.filters),
                          ('safety', self.safety), ('control', self.control)):
            for k, v in d.get(key, {}).items():
                if hasattr(obj, k):
                    setattr(obj, k, v)
        if 'home_joints_deg' in d and len(d['home_joints_deg']) == 6:
            self.home_joints_deg = list(d['home_joints_deg'])

    def list_presets(self):
        return self._presets.list_presets()

    def cmd_delete_preset(self, name: str):
        self._presets.delete(name)
        self.shared.add_alert('info', f'Preset deleted: {name}')

    def cmd_save_preset(self, name: str):
        self._presets.save(name, self._all_params_dict())
        self.shared.add_alert('info', f'Preset saved: {name}')

    def cmd_load_preset(self, name: str):
        try:
            self._apply_params_dict(self._presets.load(name))
            self.shared.add_alert('info', f'Preset loaded: {name}')
        except (OSError, json.JSONDecodeError) as exc:
            self.shared.add_alert('error', f'Could not load preset {name}: {exc}')


# --------------------------------------------------------------------------------
# Tkinter GUI (Section 8). Runs on the main thread; talks to the node only through
# SharedState.snapshot() (locked) and the plain config dataclasses (GIL-atomic
# field access) plus the node's cmd_*()/request_*() methods, never by touching
# rclpy entities directly -- see the comment on request_namespace_change() above.
# --------------------------------------------------------------------------------

BMW = {
    'canvas': '#000000',
    'surface_soft': '#0d0d0d',
    'surface': '#1a1a1a',
    'elevated': '#262626',
    'hairline': '#3c3c3c',
    'white': '#ffffff',
    'body': '#bbbbbb',
    'strong': '#e6e6e6',
    'muted': '#7e7e7e',
    'blue_light': '#0066b1',
    'blue': '#1c69d4',
    'red': '#e22718',
    'warning': '#f4b400',
    'success': '#0fa336',
}

CONF_COLORS = {'green': BMW['success'], 'yellow': BMW['warning'], 'red': BMW['red']}
CONF_LABELS = {'green': '신뢰도: 정상', 'yellow': '신뢰도: 주의', 'red': '신뢰도: 이상'}


class App(tk.Tk):
    def __init__(self, node: BallBalanceNode):
        super().__init__()
        self.node = node
        self.title('BALL BALANCE CONTROL / DOOSAN M0609 + RG2')
        self.geometry('1480x980')
        self.minsize(1180, 760)
        self.configure(bg=BMW['canvas'])
        families = set(tkfont.families(self))
        self.font_family = next(
            (name for name in ('BMW Type Next', 'Inter', 'Arial', 'DejaVu Sans') if name in families),
            'TkDefaultFont',
        )
        self._configure_bmw_style()
        self._sync_widgets = []      # (widget, get_fn) reconciled every refresh
        self._error_history = deque(maxlen=200)
        self._log_path_var = tk.StringVar(value=os.path.join(LOG_DIR_DEFAULT, 'session.csv'))

        self._build_top_bar()
        self._build_plates()
        self._build_tabs()
        self._style_surface_descendants(self)

        self.protocol('WM_DELETE_WINDOW', self._on_close)
        self.after(int(1000 / GUI_REFRESH_HZ), self._refresh)

    def _configure_bmw_style(self):
        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure('.', background=BMW['canvas'], foreground=BMW['body'],
                        font=(self.font_family, 9))
        style.configure('TFrame', background=BMW['canvas'])
        style.configure('Surface.TFrame', background=BMW['surface'])
        style.configure('TLabel', background=BMW['canvas'], foreground=BMW['body'])
        style.configure('Surface.TLabel', background=BMW['surface'], foreground=BMW['body'])
        style.configure('Section.TLabel', background=BMW['canvas'], foreground=BMW['white'],
                        font=(self.font_family, 10, 'bold'))
        style.configure('TLabelframe', background=BMW['surface'], foreground=BMW['white'],
                        bordercolor=BMW['hairline'], borderwidth=1, relief='solid')
        style.configure('TLabelframe.Label', background=BMW['surface'], foreground=BMW['white'],
                        font=(self.font_family, 10, 'bold'))
        style.configure('TButton', background=BMW['elevated'], foreground=BMW['white'],
                        bordercolor=BMW['hairline'], borderwidth=1, padding=(12, 7),
                        font=(self.font_family, 9, 'bold'))
        style.map('TButton', background=[('active', BMW['hairline']), ('pressed', BMW['blue'])],
                  foreground=[('disabled', BMW['muted']), ('active', BMW['white'])])
        style.configure('TCheckbutton', background=BMW['canvas'], foreground=BMW['body'],
                        indicatorbackground=BMW['surface'], indicatorforeground=BMW['blue'],
                        padding=4)
        style.map('TCheckbutton', background=[('active', BMW['canvas'])],
                  foreground=[('active', BMW['white'])])
        style.configure('Surface.TCheckbutton', background=BMW['surface'], foreground=BMW['body'],
                        indicatorbackground=BMW['surface_soft'], indicatorforeground=BMW['blue'],
                        padding=4)
        style.map('Surface.TCheckbutton', background=[('active', BMW['surface'])],
                  foreground=[('active', BMW['white'])])
        style.configure('TEntry', fieldbackground=BMW['surface_soft'], foreground=BMW['white'],
                        insertcolor=BMW['white'], bordercolor=BMW['hairline'], padding=5)
        style.configure('TNotebook', background=BMW['canvas'], bordercolor=BMW['hairline'],
                        tabmargins=(0, 0, 0, 0))
        style.configure('TNotebook.Tab', background=BMW['surface'], foreground=BMW['muted'],
                        padding=(18, 10), font=(self.font_family, 9, 'bold'),
                        bordercolor=BMW['hairline'])
        style.map('TNotebook.Tab', background=[('selected', BMW['elevated'])],
                  foreground=[('selected', BMW['white']), ('active', BMW['white'])])
        style.configure('Vertical.TScrollbar', background=BMW['elevated'],
                        troughcolor=BMW['surface_soft'], bordercolor=BMW['hairline'],
                        arrowcolor=BMW['body'])

    def _m_stripe(self, parent):
        stripe = tk.Frame(parent, bg=BMW['canvas'], height=4)
        stripe.pack(fill='x')
        stripe.pack_propagate(False)
        for color in (BMW['blue_light'], BMW['blue'], BMW['red']):
            tk.Frame(stripe, bg=color, width=76).pack(side='left', fill='y')
        tk.Frame(stripe, bg=BMW['hairline']).pack(side='left', fill='both', expand=True)

    def _style_surface_descendants(self, parent, in_surface=False):
        for child in parent.winfo_children():
            child_is_surface = in_surface or isinstance(child, ttk.LabelFrame)
            if child_is_surface:
                if isinstance(child, ttk.Label):
                    child.configure(style='Surface.TLabel')
                elif isinstance(child, ttk.Checkbutton):
                    child.configure(style='Surface.TCheckbutton')
                elif isinstance(child, ttk.Frame):
                    child.configure(style='Surface.TFrame')
            self._style_surface_descendants(child, child_is_surface)

    # ------------------------------------------------------------------- top bar

    def _build_top_bar(self):
        header = tk.Frame(self, bg=BMW['canvas'], padx=24, pady=16)
        header.pack(side='top', fill='x')
        title = tk.Frame(header, bg=BMW['canvas'])
        title.pack(side='left', fill='y')
        tk.Label(title, text='BALL BALANCE', bg=BMW['canvas'], fg=BMW['white'],
                 font=(self.font_family, 24, 'bold')).pack(anchor='w')
        tk.Label(title, text='CONTROL / DOOSAN M0609 + RG2', bg=BMW['canvas'], fg=BMW['muted'],
                 font=(self.font_family, 9, 'bold')).pack(anchor='w', pady=(2, 0))

        safety = tk.Frame(header, bg=BMW['canvas'])
        safety.pack(side='right', fill='y')
        tk.Button(safety, text='E-STOP', bg=BMW['red'], fg=BMW['white'],
                  activebackground='#b51d13', activeforeground=BMW['white'],
                  relief='flat', bd=0, highlightthickness=0,
                  font=(self.font_family, 12, 'bold'), width=12, height=2,
                  command=self._on_estop).pack(side='right', padx=(8, 0))
        tk.Button(safety, text='E-STOP 해제', bg=BMW['elevated'], fg=BMW['white'],
                  activebackground=BMW['hairline'], activeforeground=BMW['white'],
                  relief='flat', bd=0, highlightthickness=1,
                  highlightbackground=BMW['hairline'], font=(self.font_family, 9, 'bold'),
                  command=self._on_clear_estop).pack(side='right', padx=4, ipady=8)
        ttk.Button(safety, text='홈 복귀', command=self.node.cmd_home).pack(side='right', padx=4, ipady=2)

        self._m_stripe(self)

        bar = tk.Frame(self, bg=BMW['surface'], padx=20, pady=10,
                       highlightthickness=1, highlightbackground=BMW['hairline'])
        bar.pack(side='top', fill='x', padx=24, pady=(14, 6))

        self.lbl_conn = tk.Label(bar, text='CONNECTING', bg=BMW['muted'], fg=BMW['white'], width=14,
                                  font=(self.font_family, 9, 'bold'), padx=6, pady=6)
        self.lbl_conn.pack(side='left', padx=(0, 12))

        tk.Label(bar, text='NAMESPACE  /', bg=BMW['surface'], fg=BMW['muted'],
                 font=(self.font_family, 8, 'bold')).pack(side='left')
        self.var_ns = tk.StringVar(value=self.node.ns)
        ttk.Entry(bar, textvariable=self.var_ns, width=10).pack(side='left')
        ttk.Button(bar, text='재연결', command=self._on_reconnect).pack(side='left', padx=(2, 10))

        self.var_dry = tk.BooleanVar(value=self.node.shared.dry_run)
        dry = tk.Checkbutton(bar, text='드라이런 / SIMULATION', variable=self.var_dry,
                             command=self._on_toggle_dry_run, bg=BMW['surface'], fg=BMW['body'],
                             activebackground=BMW['surface'], activeforeground=BMW['white'],
                             selectcolor=BMW['surface_soft'], font=(self.font_family, 9),
                             highlightthickness=0, bd=0)
        dry.pack(side='left', padx=6)

        self.lbl_confidence = tk.Label(bar, text='신뢰도: -', bg=BMW['muted'], fg=BMW['white'], width=13,
                                        font=(self.font_family, 9, 'bold'), padx=6, pady=6)
        self.lbl_confidence.pack(side='left', padx=10)

        self.lbl_ball = tk.Label(bar, text='공: -', width=10, bg=BMW['surface'], fg=BMW['body'],
                                 font=(self.font_family, 9))
        self.lbl_ball.pack(side='left', padx=4)
        self.lbl_comm = tk.Label(bar, text='통신: -', width=9, bg=BMW['surface'], fg=BMW['body'],
                                 font=(self.font_family, 9))
        self.lbl_comm.pack(side='left', padx=4)
        self.lbl_limit = tk.Label(bar, text='리미트: -', width=9, bg=BMW['surface'], fg=BMW['body'],
                                  font=(self.font_family, 9))
        self.lbl_limit.pack(side='left', padx=4)
        self.lbl_busy = tk.Label(bar, text='', width=22, bg=BMW['surface'], fg=BMW['warning'],
                                 font=(self.font_family, 9, 'bold'))
        self.lbl_busy.pack(side='left', padx=4)

    def _on_reconnect(self):
        self.node.request_namespace_change(self.var_ns.get().strip())

    def _on_toggle_dry_run(self):
        self.node.cmd_set_dry_run(bool(self.var_dry.get()))

    def _on_estop(self):
        self.node.cmd_estop()

    def _on_clear_estop(self):
        self.node.cmd_clear_estop()

    # -------------------------------------------------------------- dual canvases

    def _build_plates(self):
        row = ttk.Frame(self, padding=(24, 10), style='TFrame')
        row.pack(side='top', fill='x')

        left = ttk.LabelFrame(row, text='01  RAW POSITION / 전처리 전', padding=14)
        left.pack(side='left', fill='both', expand=True, padx=(0, 6))
        self.canvas_raw = tk.Canvas(left, width=360, height=300, bg=BMW['surface_soft'], highlightthickness=1,
                                     highlightbackground=BMW['hairline'])
        self.canvas_raw.pack(fill='x')
        self.lbl_raw_xy = ttk.Label(left, text='y=0.0 mm  z=0.0 mm',
                                    style='Surface.TLabel', font=(self.font_family, 13, 'bold'))
        self.lbl_raw_xy.pack(pady=(10, 2))
        self.lbl_raw_wrench = ttk.Label(left, text='Fx=0.00 N  My=0.000 Nm  Mz=0.000 Nm',
                                        style='Surface.TLabel')
        self.lbl_raw_wrench.pack()

        right = ttk.LabelFrame(row, text='02  FILTERED POSITION / 제어 입력', padding=14)
        right.pack(side='left', fill='both', expand=True, padx=(6, 0))
        self.canvas_filt = tk.Canvas(right, width=360, height=300, bg=BMW['surface_soft'], highlightthickness=1,
                                      highlightbackground=BMW['hairline'])
        self.canvas_filt.pack(fill='x')
        self.lbl_filt_xy = ttk.Label(right, text='y=0.0 mm  z=0.0 mm',
                                     style='Surface.TLabel', font=(self.font_family, 13, 'bold'))
        self.lbl_filt_xy.pack(pady=(10, 2))
        self.lbl_filt_err = ttk.Label(right, text='중심오차=0.0 mm', style='Surface.TLabel')
        self.lbl_filt_err.pack()

    def _draw_plate(self, canvas: tk.Canvas, x_mm: float, y_mm: float, radius_mm: float, color: str):
        canvas.delete('all')
        w, h = int(canvas['width']), int(canvas['height'])
        cx, cy = w / 2, h / 2
        px_r = min(cx, cy) - 14
        radius_mm = max(radius_mm, 1e-3)
        scale = px_r / radius_mm
        canvas.create_oval(cx - px_r, cy - px_r, cx + px_r, cy + px_r,
                           outline=BMW['strong'], width=2)
        for fraction in (0.5,):
            rr = px_r * fraction
            canvas.create_oval(cx - rr, cy - rr, cx + rr, cy + rr,
                               outline=BMW['hairline'], width=1)
        canvas.create_line(cx - px_r, cy, cx + px_r, cy, fill=BMW['hairline'])
        canvas.create_line(cx, cy - px_r, cx, cy + px_r, fill=BMW['hairline'])
        canvas.create_text(cx + px_r - 4, cy - 12, text='+Y', anchor='e',
                           fill=BMW['muted'], font=(self.font_family, 8, 'bold'))
        canvas.create_text(cx + 8, cy - px_r + 4, text='+Z', anchor='nw',
                           fill=BMW['muted'], font=(self.font_family, 8, 'bold'))
        canvas.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=BMW['white'], outline='')
        bx = cx + clamp(x_mm, -radius_mm * 1.4, radius_mm * 1.4) * scale
        by = cy - clamp(y_mm, -radius_mm * 1.4, radius_mm * 1.4) * scale
        r = 9
        canvas.create_line(cx, cy, bx, by, fill=BMW['blue'], width=2)
        canvas.create_oval(bx - r, by - r, bx + r, by + r,
                           fill=color, outline=BMW['white'], width=2)

    # ------------------------------------------------------------- slider helper

    def _labeled_slider(self, parent, row, text, desc, from_, to_, resolution, get_fn, set_fn):
        ttk.Label(parent, text=text, font=(self.font_family, 9, 'bold'),
                  style='Surface.TLabel').grid(
            row=row, column=0, sticky='w', padx=6, pady=(8, 0))

        def _on_move(v):
            set_fn(float(v))

        scale = tk.Scale(parent, from_=from_, to=to_, resolution=resolution, orient='horizontal',
                          length=300, showvalue=True, command=_on_move,
                          bg=BMW['surface'], fg=BMW['white'], activebackground=BMW['blue'],
                          troughcolor=BMW['surface_soft'], highlightthickness=0, bd=0,
                          sliderrelief='flat', font=(self.font_family, 8))
        scale.set(get_fn())
        scale.grid(row=row, column=1, sticky='ew', padx=6, pady=(8, 0))
        ttk.Label(parent, text=desc, foreground=BMW['muted'], wraplength=660, justify='left',
                  style='Surface.TLabel', font=(self.font_family, 8)).grid(
                      row=row + 1, column=0, columnspan=2, sticky='w', padx=6, pady=(0, 4))
        self._sync_widgets.append((scale, get_fn))
        return scale

    def _labeled_entry(self, parent, row, col, text, get_fn, set_fn, width=8):
        ttk.Label(parent, text=text, style='Surface.TLabel').grid(
            row=row, column=col, sticky='w', padx=4, pady=3)
        var = tk.StringVar(value=f'{get_fn():.3g}')
        entry = ttk.Entry(parent, textvariable=var, width=width)
        entry.grid(row=row, column=col + 1, sticky='w', padx=4, pady=3)

        def _commit(_evt=None):
            try:
                set_fn(float(var.get()))
            except ValueError:
                var.set(f'{get_fn():.3g}')

        entry.bind('<Return>', _commit)
        entry.bind('<FocusOut>', _commit)
        self._sync_widgets.append((entry, get_fn))
        return entry

    # ------------------------------------------------------------------- tabs

    def _make_scrollable(self, tab: ttk.Frame) -> ttk.Frame:
        """Each tab below packs more sliders/controls than fit in one window (the
        preprocessing tab alone has 12 sliders x 2 rows). Tkinter has no built-in
        scrollable frame, so wrap the tab in a Canvas+Scrollbar and hand callers an
        inner Frame to build into -- otherwise controls below the fold are simply
        unreachable, which would break the "every function must be operable from the
        GUI" requirement (spec 8-0)."""
        canvas = tk.Canvas(tab, bg=BMW['canvas'], highlightthickness=0)
        vscroll = ttk.Scrollbar(tab, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=vscroll.set)
        vscroll.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)

        inner = ttk.Frame(canvas, padding=8)
        window_id = canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda _e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfig(window_id, width=e.width))

        def _wheel(event):
            if event.num == 4:
                canvas.yview_scroll(-1, 'units')
            elif event.num == 5:
                canvas.yview_scroll(1, 'units')
            else:
                canvas.yview_scroll(-1 if event.delta > 0 else 1, 'units')

        # Bind/unbind on hover (rather than bind_all permanently) so the wheel only
        # ever scrolls whichever tab's canvas the mouse is currently over.
        canvas.bind('<Enter>', lambda _e: (canvas.bind_all('<MouseWheel>', _wheel),
                                            canvas.bind_all('<Button-4>', _wheel),
                                            canvas.bind_all('<Button-5>', _wheel)))
        canvas.bind('<Leave>', lambda _e: (canvas.unbind_all('<MouseWheel>'),
                                            canvas.unbind_all('<Button-4>'),
                                            canvas.unbind_all('<Button-5>')))
        return inner

    def _build_tabs(self):
        nb = ttk.Notebook(self)
        nb.pack(side='top', fill='both', expand=True, padx=24, pady=(4, 18))
        tab_pre = ttk.Frame(nb)
        tab_ctrl = ttk.Frame(nb)
        tab_safety = ttk.Frame(nb)
        tab_preset = ttk.Frame(nb)
        nb.add(tab_pre, text='01  SIGNAL / PHYSICAL')
        nb.add(tab_ctrl, text='02  ROBOT CONTROL')
        nb.add(tab_safety, text='03  SAFETY / SYSTEM')
        nb.add(tab_preset, text='04  PRESETS')
        self._build_tab_preprocessing(self._make_scrollable(tab_pre))
        self._build_tab_control(self._make_scrollable(tab_ctrl))
        self._build_tab_safety(self._make_scrollable(tab_safety))
        self._build_tab_presets(self._make_scrollable(tab_preset))

    def _build_tab_preprocessing(self, parent):
        phys = ttk.LabelFrame(parent, text='물리 파라미터 (공/플레이트)', padding=6)
        phys.pack(side='top', fill='x', pady=(0, 8))
        p = self.node.physical
        self._labeled_slider(phys, 0, '플레이트 반지름 (mm)',
                              '공 위치 추정과 이탈 판정, 원판 시각화 스케일의 기준이 되는 플레이트 반지름입니다.',
                              20, 300, 1, lambda: p.plate_radius_mm, lambda v: setattr(p, 'plate_radius_mm', v))
        self._labeled_slider(phys, 2, '공 질량 (g)',
                              '드라이런 시뮬레이션 및 이상치(신뢰도) 판정에 쓰이는 예상 Fz 크기 계산에 사용됩니다.',
                              1, 500, 1, lambda: p.ball_mass_g, lambda v: setattr(p, 'ball_mass_g', v))
        self._labeled_slider(phys, 4, '공 반지름 (mm)', '원판 시각화에서 공 마커 참고 크기로 사용됩니다.',
                              5, 60, 1, lambda: p.ball_radius_mm, lambda v: setattr(p, 'ball_radius_mm', v))
        self._labeled_slider(phys, 6, 'TCP->플레이트 표면 X 오프셋 (mm)',
                              '그리퍼 TCP 원점에서 플레이트 표면 중심까지의 +X 거리입니다(이 리그에서는 +X가 플레이트 '
                              '법선 방향). 이 값으로 모멘트 기준점을 TCP에서 플레이트 중심으로 이동시켜 CoP(y=-Mz/Fx, '
                              'z=My/Fx) 역산의 기준점을 맞춥니다.',
                              0, 100, 1, lambda: p.tcp_to_plate_offset_mm,
                              lambda v: setattr(p, 'tcp_to_plate_offset_mm', v))

        filt = ttk.LabelFrame(parent, text='노이즈 전처리 필터', padding=6)
        filt.pack(side='top', fill='x')
        f = self.node.filters
        self._labeled_slider(filt, 0, '수신 Hz (샘플링 주기)',
                              '로봇으로부터 힘/토크 데이터를 읽어오는 샘플링 주기입니다. 높을수록 반응은 빠르지만 '
                              '노이즈에 민감해지고, 낮을수록 부드럽지만 반응이 느려집니다. (제어 루프 주기와는 별개입니다)',
                              5, 100, 1, lambda: f.sample_hz, lambda v: setattr(f, 'sample_hz', v))
        self._labeled_slider(filt, 2, '이상치 제거 강도 (MAD 배수)',
                              '최근 값들의 중앙값에서 이 배수 이상 벗어난 튐값을 무시합니다. 작을수록 엄격하게 걸러냅니다.',
                              1, 10, 0.5, lambda: f.outlier_mad_k, lambda v: setattr(f, 'outlier_mad_k', v))
        self._labeled_slider(filt, 4, '데드존 -- 힘 (N)',
                              '이 값보다 작은 Fz 변화는 0으로 취급해 division-by-noise를 줄입니다.',
                              0, 1.0, 0.01, lambda: f.deadzone_n, lambda v: setattr(f, 'deadzone_n', v))
        self._labeled_slider(filt, 6, '데드존 -- 모멘트 (Nm)',
                              '이 값보다 작은 Mx/My 변화는 0으로 취급합니다.',
                              0, 0.05, 0.001, lambda: f.deadzone_nm, lambda v: setattr(f, 'deadzone_nm', v))
        self._labeled_slider(filt, 8, '이동평균 윈도우 (ms)',
                              '최근 이 시간 동안의 샘플을 평균해 공 위치를 부드럽게 만듭니다. 길수록 부드럽지만 반응이 느려집니다.',
                              0, 1000, 10, lambda: f.moving_avg_window_ms, lambda v: setattr(f, 'moving_avg_window_ms', v))
        self._labeled_slider(filt, 10, '저역통과 필터 컷오프 (Hz)',
                              '이 주파수보다 빠른 변화는 억제합니다. 낮을수록 부드럽지만 반응이 느려집니다.',
                              0.1, 20, 0.1, lambda: f.lpf_cutoff_hz, lambda v: setattr(f, 'lpf_cutoff_hz', v))
        self._labeled_slider(filt, 12, '칼만필터 프로세스 노이즈',
                              '공이 스스로 얼마나 빠르게 움직일 수 있다고 가정할지의 정도입니다. 클수록 측정값을 더 신뢰합니다.',
                              0.1, 50, 0.1, lambda: f.kalman_process_noise,
                              lambda v: setattr(f, 'kalman_process_noise', v))
        self._labeled_slider(filt, 14, '칼만필터 측정 노이즈',
                              '측정값 자체를 얼마나 신뢰할지의 정도입니다. 클수록 필터 출력이 더 부드러워집니다.',
                              1, 100, 1, lambda: f.kalman_measurement_noise,
                              lambda v: setattr(f, 'kalman_measurement_noise', v))
        self._labeled_slider(filt, 16, '최소 |Fx| 임계값 (N, 공 감지 기준)',
                              '이 값보다 플레이트 법선방향 힘 |Fx|가 작으면 공이 없다고 보고 CoP 역산을 신뢰하지 않습니다.',
                              0.02, 2.0, 0.01, lambda: f.min_fx_n, lambda v: setattr(f, 'min_fx_n', v))

        zero_row = ttk.Frame(parent, padding=(0, 8))
        zero_row.pack(side='top', fill='x')
        ttk.Button(zero_row, text='Zeroing (공 없음 기준값 재설정)', command=self.node.cmd_zero).pack(side='left')
        ttk.Label(zero_row, text='  전원 인가 후 또는 드리프트가 의심될 때, 플레이트에 공이 없는 상태에서 누르세요.',
                  foreground=BMW['muted']).pack(side='left')

    def _set_speed_profile(self, profile: str):
        profiles = {
            'very_slow': (1.0, 2.0, 0.2, 0.4, 5.0, 10.0),
            'slow': (3.0, 6.0, 0.5, 1.0, 10.0, 20.0),
        }
        values = profiles[profile]
        s = self.node.safety
        (
            s.max_lin_vel_mms,
            s.max_lin_acc_mms2,
            s.max_tilt_vel_dps,
            s.max_tilt_acc_dps2,
            s.joint_move_vel_dps,
            s.joint_move_acc_dps2,
        ) = values
        self.node.shared.add_alert(
            'info',
            'Motion profile: VERY SLOW' if profile == 'very_slow' else 'Motion profile: SLOW',
        )

    def _build_motion_speed_panel(self, parent, show_profiles=False):
        s = self.node.safety
        speed = ttk.LabelFrame(
            parent,
            text='ROBOT MOTION SPEED / 속도 제한',
            padding=8,
        )
        speed.pack(side='top', fill='x', pady=(0, 8))
        if show_profiles:
            profile_row = ttk.Frame(speed)
            profile_row.grid(row=0, column=0, columnspan=2, sticky='ew', padx=6, pady=(2, 6))
            ttk.Label(
                profile_row,
                text='권장 시작값: VERY SLOW · 실제 동작 중에도 아래 제한값을 낮출 수 있습니다.',
                foreground=BMW['muted'],
            ).pack(side='left')
            ttk.Button(
                profile_row,
                text='VERY SLOW 적용',
                command=lambda: self._set_speed_profile('very_slow'),
            ).pack(side='right', padx=(4, 0))
            ttk.Button(
                profile_row,
                text='SLOW 적용',
                command=lambda: self._set_speed_profile('slow'),
            ).pack(side='right', padx=4)
            start_row = 1
        else:
            start_row = 0

        self._labeled_slider(
            speed, start_row, 'TCP 최대 선속도 (mm/s)',
            '수평·수직 Task Space 이동 속도입니다. VERY SLOW 기본값은 1.0 mm/s입니다.',
            0.1, 20.0, 0.1,
            lambda: s.max_lin_vel_mms,
            lambda v: setattr(s, 'max_lin_vel_mms', v),
        )
        self._labeled_slider(
            speed, start_row + 2, 'TCP 최대 선가속도 (mm/s²)',
            '선속도가 변하는 속도입니다. 낮을수록 출발과 정지가 부드럽습니다.',
            0.1, 50.0, 0.1,
            lambda: s.max_lin_acc_mms2,
            lambda v: setattr(s, 'max_lin_acc_mms2', v),
        )
        self._labeled_slider(
            speed, start_row + 4, 'Tilting 최대 각속도 (deg/s)',
            '원판 기울기 변화 속도입니다. VERY SLOW 기본값은 0.2 deg/s입니다.',
            0.05, 5.0, 0.05,
            lambda: s.max_tilt_vel_dps,
            lambda v: setattr(s, 'max_tilt_vel_dps', v),
        )
        self._labeled_slider(
            speed, start_row + 6, 'Tilting 최대 각가속도 (deg/s²)',
            '기울기 속도가 변하는 속도입니다. 낮을수록 틸팅 시작과 정지가 부드럽습니다.',
            0.05, 10.0, 0.05,
            lambda: s.max_tilt_acc_dps2,
            lambda v: setattr(s, 'max_tilt_acc_dps2', v),
        )
        self._labeled_slider(
            speed, start_row + 8, 'Joint/Home 최대 속도 (deg/s)',
            '조인트 이동 적용과 홈 복귀에 공통으로 사용하는 관절 속도입니다.',
            0.5, 30.0, 0.5,
            lambda: s.joint_move_vel_dps,
            lambda v: setattr(s, 'joint_move_vel_dps', v),
        )
        self._labeled_slider(
            speed, start_row + 10, 'Joint/Home 최대 가속도 (deg/s²)',
            '조인트 이동 적용과 홈 복귀의 관절 가속도입니다.',
            0.5, 60.0, 0.5,
            lambda: s.joint_move_acc_dps2,
            lambda v: setattr(s, 'joint_move_acc_dps2', v),
        )
        return speed

    def _build_tab_control(self, parent):
        self._build_motion_speed_panel(parent, show_profiles=True)

        task = ttk.LabelFrame(parent, text='Task Space 수동 조작 (X/Y/Z/Rx/Ry/Rz) -- 자동 밸런싱과 동시 반영', padding=6)
        task.pack(side='top', fill='x', pady=(0, 8))
        m = self.node.manual
        specs = [('X (mm)', 'x_mm', -200, 200), ('Y (mm)', 'y_mm', -200, 200), ('Z (mm)', 'z_mm', -100, 100),
                 ('Rx (deg)', 'rx_deg', -20, 20), ('Ry (deg)', 'ry_deg', -20, 20), ('Rz (deg)', 'rz_deg', -45, 45)]
        for i, (label, attr, lo, hi) in enumerate(specs):
            self._labeled_slider(task, i * 2, label,
                                  '실제 이동량은 안전 탭의 워크스페이스/틸트 리미트로 다시 한 번 제한됩니다.',
                                  lo, hi, 0.5, (lambda a=attr: getattr(m, a)),
                                  (lambda v, a=attr: setattr(m, a, v)))

        joint = ttk.LabelFrame(parent, text='Joint Space 이동 (Joint1~6, deg) -- 이산 이동', padding=6)
        joint.pack(side='top', fill='x', pady=(0, 8))
        ttk.Label(joint, text='현재:').grid(row=0, column=0, sticky='w', padx=4)
        self.lbl_joint_current = ttk.Label(joint, text='- - - - - -')
        self.lbl_joint_current.grid(row=0, column=1, columnspan=6, sticky='w')
        self._joint_vars = []
        for j in range(6):
            ttk.Label(joint, text=f'J{j + 1}').grid(row=1, column=j, padx=4)
            var = tk.StringVar(value=f'{self.node.home_joints_deg[j]:.1f}')
            ttk.Entry(joint, textvariable=var, width=7).grid(row=2, column=j, padx=4)
            self._joint_vars.append(var)
        ttk.Button(joint, text='조인트 이동 적용 (이산 이동)', command=self._on_apply_joints).grid(
            row=3, column=0, columnspan=3, pady=6, sticky='w')
        ttk.Button(joint, text='현재 자세를 홈으로 저장', command=self._on_set_home).grid(
            row=3, column=3, columnspan=3, pady=6, sticky='w')
        ttk.Label(joint, text='주의: 이 이동 중에는 스트리밍 밸런싱 명령이 잠시 정지되었다가 완료 후 재개됩니다.',
                  foreground=BMW['muted'], style='Surface.TLabel').grid(
                      row=4, column=0, columnspan=6, sticky='w')

        mode = ttk.LabelFrame(parent, text='제어 모드', padding=6)
        mode.pack(side='top', fill='x', pady=(0, 8))
        c = self.node.control
        self.var_balancing = tk.BooleanVar(value=c.balancing_enabled)
        ttk.Checkbutton(mode, text='자동 밸런싱 제어 On/Off', variable=self.var_balancing,
                         command=lambda: setattr(c, 'balancing_enabled', bool(self.var_balancing.get()))
                         ).grid(row=0, column=0, sticky='w', padx=6, pady=4)
        self.var_autosearch = tk.BooleanVar(value=c.auto_search_enabled)
        ttk.Checkbutton(mode, text='자동 최적 자세 탐색 On/Off', variable=self.var_autosearch,
                         command=lambda: setattr(c, 'auto_search_enabled', bool(self.var_autosearch.get()))
                         ).grid(row=0, column=1, sticky='w', padx=6, pady=4)

        pid = ttk.LabelFrame(parent, text='제어 게인', padding=6)
        pid.pack(side='top', fill='x', pady=(0, 8))
        self._labeled_slider(pid, 0, '수평이동 Kp', '플레이트 수평이동(1순위) 비례 게인입니다.',
                              0, 3, 0.01, lambda: c.trans_kp, lambda v: setattr(c, 'trans_kp', v))
        self._labeled_slider(pid, 2, '수평이동 Ki', '수평이동 적분 게인입니다. 정상상태 오차를 없애지만 과하면 진동합니다.',
                              0, 0.5, 0.005, lambda: c.trans_ki, lambda v: setattr(c, 'trans_ki', v))
        self._labeled_slider(pid, 4, '수평이동 Kd', '수평이동 미분 게인입니다. 과도응답을 감쇠시킵니다.',
                              0, 1, 0.01, lambda: c.trans_kd, lambda v: setattr(c, 'trans_kd', v))
        self._labeled_slider(pid, 6, '틸팅 Kp (보조, 2순위)', '잔여 오차 미세조정용 틸팅 비례 게인입니다. 작게 유지하세요.',
                              0, 0.5, 0.005, lambda: c.tilt_kp, lambda v: setattr(c, 'tilt_kp', v))
        self._labeled_slider(pid, 8, '틸팅 Ki', '틸팅 적분 게인입니다.',
                              0, 0.05, 0.0005, lambda: c.tilt_ki, lambda v: setattr(c, 'tilt_ki', v))
        self._labeled_slider(pid, 10, '틸팅 Kd', '틸팅 미분 게인입니다.',
                              0, 0.1, 0.001, lambda: c.tilt_kd, lambda v: setattr(c, 'tilt_kd', v))
        self._labeled_slider(pid, 12, '틸팅 보조 비중 (0~1)',
                              '틸팅 보정을 얼마나 반영할지의 비중입니다. 0에 가까울수록 수평이동 위주로만 동작합니다.',
                              0, 1, 0.01, lambda: c.tilt_weight, lambda v: setattr(c, 'tilt_weight', v))
        self._labeled_slider(pid, 14, '자동탐색 게인', '자동 최적 자세 탐색이 틸트 바이어스를 조정하는 속도입니다.',
                              0, 0.005, 0.0001, lambda: c.auto_search_gain, lambda v: setattr(c, 'auto_search_gain', v))
        self._labeled_slider(pid, 16, '자동탐색 최대 바이어스 (deg)', '자동 탐색이 추가할 수 있는 최대 틸트 바이어스입니다.',
                              0, 10, 0.5, lambda: c.max_auto_bias_deg, lambda v: setattr(c, 'max_auto_bias_deg', v))

        out = ttk.LabelFrame(parent, text='제어기 출력 / 오차 (읽기 전용)', padding=6)
        out.pack(side='top', fill='x')
        self.lbl_ctrl_out = ttk.Label(out, text='dx=0.0 dy=0.0 mm | d_rx=0.0 d_ry=0.0 deg | bias_rx=0.0 bias_ry=0.0 deg')
        self.lbl_ctrl_out.pack(anchor='w', padx=6, pady=2)
        self.lbl_ctrl_err = ttk.Label(out, text='중심 오차: 0.0 mm  |  RMSE: 0.0 mm  |  오버슈트: 0.0 mm  |  정착시간: - s')
        self.lbl_ctrl_err.pack(anchor='w', padx=6, pady=2)
        self.lbl_speed_limits = ttk.Label(
            out,
            text='속도 제한: TCP 1.0 mm/s · Tilt 0.20 deg/s · Joint/Home 5.0 deg/s',
        )
        self.lbl_speed_limits.pack(anchor='w', padx=6, pady=2)

        btns = ttk.Frame(parent, padding=(0, 8))
        btns.pack(side='top', fill='x')
        ttk.Button(btns, text='홈 복귀', command=self.node.cmd_home).pack(side='left', padx=4)
        tk.Button(btns, text='E-STOP', bg=BMW['red'], fg=BMW['white'],
                  activebackground='#b51d13', activeforeground=BMW['white'],
                  relief='flat', bd=0, font=(self.font_family, 9, 'bold'),
                  command=self._on_estop).pack(side='left', padx=4, ipadx=10, ipady=7)

    def _on_apply_joints(self):
        try:
            vals = [float(v.get()) for v in self._joint_vars]
        except ValueError:
            messagebox.showerror('입력 오류', '조인트 값은 숫자여야 합니다.')
            return
        self.node.cmd_jog_joints(vals)

    def _on_set_home(self):
        if messagebox.askyesno('홈 포지션 저장', '현재 로봇 자세를 새로운 홈 포지션(비상/복귀 기준 자세)으로 저장하시겠습니까?'):
            self.node.cmd_set_home_from_current()

    def _build_tab_safety(self, parent):
        s = self.node.safety
        ws = ttk.LabelFrame(parent, text='워크스페이스 / 틸트 / 조인트 리미트 (연결 시점 자세 기준 오프셋)', padding=6)
        ws.pack(side='top', fill='x', pady=(0, 8))
        specs = [
            ('X min (mm)', 'ws_x_min_mm'), ('X max (mm)', 'ws_x_max_mm'),
            ('Y min (mm)', 'ws_y_min_mm'), ('Y max (mm)', 'ws_y_max_mm'),
            ('Z min (mm)', 'ws_z_min_mm'), ('Z max (mm)', 'ws_z_max_mm'),
            ('최대 틸트 (deg)', 'max_tilt_deg'), ('조인트 여유 (deg)', 'joint_margin_deg'),
        ]
        for i, (label, attr) in enumerate(specs):
            self._labeled_entry(ws, i // 4, (i % 4) * 2, label,
                                 (lambda a=attr: getattr(s, a)), (lambda v, a=attr: setattr(s, a, v)))

        det = ttk.LabelFrame(parent, text='공 이탈 / 통신 감지', padding=6)
        det.pack(side='top', fill='x', pady=(0, 8))
        self._labeled_slider(det, 0, '공 없음 판정 |Fx| 임계값 (N)',
                              '이 값보다 플레이트 법선방향 힘 |Fx|가 작은 상태가 지속되면 공이 없다고 판정합니다.',
                              0.02, 2.0, 0.01, lambda: s.no_ball_fx_n, lambda v: setattr(s, 'no_ball_fx_n', v))
        self._labeled_slider(det, 2, '공 이탈 판정 지속시간 (s)',
                              '이 시간 이상 공이 감지되지 않으면 이탈로 판정하고 밸런싱을 중단합니다.',
                              0.1, 5.0, 0.1, lambda: s.departure_hold_s, lambda v: setattr(s, 'departure_hold_s', v))
        self._labeled_slider(det, 4, '통신 타임아웃 (s)',
                              '이 시간 이상 정상 응답이 없으면 통신 두절로 판단해 즉시 정지합니다.',
                              0.2, 5.0, 0.1, lambda: s.comm_timeout_s, lambda v: setattr(s, 'comm_timeout_s', v))

        self._build_motion_speed_panel(parent)

        sim_row = ttk.Frame(parent, padding=(0, 4))
        sim_row.pack(side='top', fill='x')
        ttk.Checkbutton(sim_row, text='드라이런(시뮬레이션) 모드', variable=self.var_dry,
                         command=self._on_toggle_dry_run).pack(side='left', padx=4)

        log_frame = ttk.LabelFrame(parent, text='로깅', padding=6)
        log_frame.pack(side='top', fill='x', pady=(8, 8))
        ttk.Entry(log_frame, textvariable=self._log_path_var, width=50).pack(side='left', padx=4)
        ttk.Button(log_frame, text='찾아보기', command=self._on_browse_log).pack(side='left', padx=4)
        ttk.Button(log_frame, text='로깅 시작', command=self._on_start_logging).pack(side='left', padx=4)
        ttk.Button(log_frame, text='로깅 정지', command=self.node.cmd_stop_logging).pack(side='left', padx=4)
        self.lbl_logging = ttk.Label(log_frame, text='정지됨')
        self.lbl_logging.pack(side='left', padx=8)

        chart_frame = ttk.LabelFrame(parent, text='중심 오차 추이 (실시간)', padding=6)
        chart_frame.pack(side='top', fill='x', pady=(0, 8))
        self.canvas_chart = tk.Canvas(chart_frame, width=720, height=150, bg=BMW['surface_soft'],
                                       highlightthickness=1, highlightbackground=BMW['hairline'])
        self.canvas_chart.pack(fill='x')

        alert_frame = ttk.LabelFrame(parent, text='이벤트 / 경고 로그', padding=6)
        alert_frame.pack(side='top', fill='both', expand=True)
        scroll = ttk.Scrollbar(alert_frame)
        scroll.pack(side='right', fill='y')
        self.list_alerts = tk.Listbox(
            alert_frame, height=8, yscrollcommand=scroll.set,
            bg=BMW['surface_soft'], fg=BMW['body'], selectbackground=BMW['blue'],
            selectforeground=BMW['white'], highlightbackground=BMW['hairline'],
            highlightcolor=BMW['blue'], relief='flat', bd=0,
            font=(self.font_family, 9),
        )
        self.list_alerts.pack(side='left', fill='both', expand=True)
        scroll.config(command=self.list_alerts.yview)
        self._last_alert_count = 0

    def _on_browse_log(self):
        path = filedialog.asksaveasfilename(defaultextension='.csv', initialfile='session.csv',
                                             filetypes=[('CSV', '*.csv')])
        if path:
            self._log_path_var.set(path)

    def _on_start_logging(self):
        self.node.cmd_start_logging(self._log_path_var.get())

    def _draw_error_chart(self, canvas: tk.Canvas):
        canvas.delete('all')
        w, h = int(canvas['width']), int(canvas['height'])
        pad = 20
        canvas.create_rectangle(pad, 5, w - 5, h - pad, outline=BMW['hairline'])
        if len(self._error_history) < 2:
            return
        vals = list(self._error_history)
        max_v = max(max(vals), 10.0)
        n = len(vals)
        step = (w - pad - 10) / max(n - 1, 1)
        pts = []
        for i, v in enumerate(vals):
            x = pad + i * step
            y = (h - pad) - (v / max_v) * (h - pad - 10)
            pts.extend([x, y])
        if len(pts) >= 4:
            canvas.create_line(*pts, fill=BMW['blue'], width=2)
        canvas.create_text(pad + 4, 12, text=f'{max_v:.0f}mm', anchor='w',
                           font=(self.font_family, 7), fill=BMW['muted'])

    def _build_tab_presets(self, parent):
        left = ttk.Frame(parent)
        left.pack(side='left', fill='both', expand=True, padx=(0, 8))
        ttk.Label(left, text='저장된 프리셋').pack(anchor='w')
        self.list_presets = tk.Listbox(
            left, height=16, bg=BMW['surface_soft'], fg=BMW['body'],
            selectbackground=BMW['blue'], selectforeground=BMW['white'],
            highlightbackground=BMW['hairline'], highlightcolor=BMW['blue'],
            relief='flat', bd=0, font=(self.font_family, 10),
        )
        self.list_presets.pack(fill='both', expand=True)
        self._refresh_preset_list()

        right = ttk.Frame(parent)
        right.pack(side='left', fill='y')
        ttk.Label(right, text='프리셋 이름').pack(anchor='w', pady=(0, 2))
        self.var_preset_name = tk.StringVar(value='default')
        ttk.Entry(right, textvariable=self.var_preset_name, width=24).pack(anchor='w')
        ttk.Button(right, text='현재 설정을 프리셋으로 저장', command=self._on_save_preset).pack(fill='x', pady=(8, 2))
        ttk.Button(right, text='선택 프리셋 불러오기', command=self._on_load_preset).pack(fill='x', pady=2)
        ttk.Button(right, text='선택 프리셋 삭제', command=self._on_delete_preset).pack(fill='x', pady=2)
        ttk.Label(right, text='필터 계수, Hz, PID 게인, 안전 리미트,\n물리 파라미터, 홈 포지션이 모두 하나의\n'
                               '프리셋 파일(JSON)로 저장/복원됩니다.',
                  foreground=BMW['muted'], justify='left').pack(anchor='w', pady=(12, 0))

    def _refresh_preset_list(self):
        self.list_presets.delete(0, tk.END)
        for name in self.node.list_presets():
            self.list_presets.insert(tk.END, name)

    def _selected_preset(self):
        sel = self.list_presets.curselection()
        return self.list_presets.get(sel[0]) if sel else None

    def _on_save_preset(self):
        name = self.var_preset_name.get().strip()
        if not name:
            return
        self.node.cmd_save_preset(name)
        self._refresh_preset_list()

    def _on_load_preset(self):
        name = self._selected_preset()
        if name:
            self.node.cmd_load_preset(name)

    def _on_delete_preset(self):
        name = self._selected_preset()
        if name:
            self.node.cmd_delete_preset(name)
            self._refresh_preset_list()

    # ---------------------------------------------------------------- refresh

    def _refresh(self):
        snap = self.node.shared.snapshot()

        if snap['comm_error']:
            self.lbl_conn.config(text='COMM ERROR', bg=BMW['red'])
        elif snap['dry_run']:
            self.lbl_conn.config(text='DRY-RUN', bg=BMW['blue'])
        elif snap['connected']:
            self.lbl_conn.config(text='CONNECTED', bg=BMW['success'])
        else:
            self.lbl_conn.config(text='DISCONNECTED', bg=BMW['muted'])

        conf = snap['confidence']
        self.lbl_confidence.config(
            text=CONF_LABELS.get(conf, '신뢰도: -'),
            bg=CONF_COLORS.get(conf, BMW['muted']),
        )
        self.lbl_ball.config(text=f"공: {'있음' if snap['ball_present'] else '없음'}")
        self.lbl_comm.config(text=f"통신: {'오류' if snap['comm_error'] else '정상'}")
        self.lbl_limit.config(text=f"리미트: {'도달' if snap['limit_hit'] else '정상'}")
        self.lbl_busy.config(text=(f"이동 중 (robot_state={snap['robot_state_code']})"
                                    if snap['busy_discrete_move'] else ''))

        color = CONF_COLORS.get(conf, BMW['muted'])
        radius = self.node.physical.plate_radius_mm
        self._draw_plate(self.canvas_raw, snap['raw_x_mm'], snap['raw_y_mm'], radius, color)
        self._draw_plate(self.canvas_filt, snap['filt_x_mm'], snap['filt_y_mm'], radius, color)
        self.lbl_raw_xy.config(text=f"y={snap['raw_x_mm']:.1f} mm  z={snap['raw_y_mm']:.1f} mm")
        w = snap['wrench_plate']
        self.lbl_raw_wrench.config(text=f"Fx={snap['fx_n']:.2f} N  My={w[4]:.3f} Nm  Mz={w[5]:.3f} Nm")
        self.lbl_filt_xy.config(text=f"y={snap['filt_x_mm']:.1f} mm  z={snap['filt_y_mm']:.1f} mm")
        self.lbl_filt_err.config(text=f"중심오차={snap['error_mm']:.1f} mm")

        jp = snap['joint_pose']
        self.lbl_joint_current.config(text='  '.join(f'{v:.1f}' for v in jp))
        dx, dy = snap['ctrl_translation_mm']
        drx, dry = snap['ctrl_tilt_deg']
        brx, bry = snap['auto_bias_deg']
        self.lbl_ctrl_out.config(text=f'dx={dx:.1f} dy={dy:.1f} mm | d_rx={drx:.2f} d_ry={dry:.2f} deg | '
                                       f'bias_rx={brx:.2f} bias_ry={bry:.2f} deg')
        settle = snap['settling_s']
        settle_txt = f'{settle:.1f}' if settle is not None else '-'
        self.lbl_ctrl_err.config(text=f"중심 오차: {snap['error_mm']:.1f} mm  |  RMSE: {snap['rmse_mm']:.1f} mm  |  "
                                       f"오버슈트: {snap['overshoot_mm']:.1f} mm  |  정착시간: {settle_txt} s")
        s = self.node.safety
        self.lbl_speed_limits.config(
            text=(f'속도 제한: TCP {s.max_lin_vel_mms:.1f} mm/s · '
                  f'Tilt {s.max_tilt_vel_dps:.2f} deg/s · '
                  f'Joint/Home {s.joint_move_vel_dps:.1f} deg/s')
        )

        self._error_history.append(snap['error_mm'])
        self._draw_error_chart(self.canvas_chart)
        self.lbl_logging.config(text=f'기록 중: {self.node.logging_path}' if self.node.logging_active else '정지됨')

        alerts = snap['alerts']
        if len(alerts) != self._last_alert_count:
            self.list_alerts.delete(0, tk.END)
            for ts, level, msg in list(alerts)[-80:]:
                tstr = datetime.fromtimestamp(ts).strftime('%H:%M:%S')
                self.list_alerts.insert(tk.END, f'[{tstr}] {level.upper()}: {msg}')
            self.list_alerts.yview_moveto(1.0)
            self._last_alert_count = len(alerts)

        for widget, get_fn in self._sync_widgets:
            try:
                cur = get_fn()
                if isinstance(widget, tk.Scale):
                    if abs(float(widget.get()) - float(cur)) > 1e-9:
                        widget.set(cur)
                elif isinstance(widget, ttk.Entry):
                    if widget.focus_get() is not widget:
                        txt = f'{cur:.3g}'
                        if widget.get() != txt:
                            widget.delete(0, tk.END)
                            widget.insert(0, txt)
            except Exception:
                pass

        self.title('BALL BALANCE CONTROL / E-STOP ACTIVE' if snap['estop_active']
                   else 'BALL BALANCE CONTROL / DOOSAN M0609 + RG2')
        self.after(int(1000 / GUI_REFRESH_HZ), self._refresh)

    def _on_close(self):
        self.node.cmd_stop_motion()
        self.destroy()


# --------------------------------------------------------------------------------
# Simplified operator HMI
# --------------------------------------------------------------------------------

class SimpleHMI(tk.Tk):
    """Compact one-screen HMI based on docs/hmi-features.md."""

    def __init__(self, node: BallBalanceNode):
        super().__init__()
        self.node = node
        self.path_positions = []
        self._last_alert_count = -1

        self.title('M0609 OPERATOR HMI')
        self.geometry('1180x760')
        self.minsize(980, 680)
        self.configure(bg=BMW['canvas'])
        self.protocol('WM_DELETE_WINDOW', self._on_close)

        self._configure_style()
        self._build_header()
        self._build_body()
        self._build_log()
        self.after(100, self._refresh)

    def _configure_style(self):
        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure(
            '.',
            background=BMW['canvas'],
            foreground=BMW['body'],
            font=('DejaVu Sans', 10),
        )
        style.configure('TFrame', background=BMW['canvas'])
        style.configure(
            'TLabelframe',
            background=BMW['surface'],
            foreground=BMW['white'],
            bordercolor=BMW['hairline'],
        )
        style.configure(
            'TLabelframe.Label',
            background=BMW['surface'],
            foreground=BMW['white'],
            font=('DejaVu Sans', 10, 'bold'),
        )
        style.configure(
            'TLabel',
            background=BMW['surface'],
            foreground=BMW['body'],
        )
        style.configure(
            'TButton',
            background=BMW['elevated'],
            foreground=BMW['white'],
            padding=(10, 7),
        )
        style.map('TButton', background=[('active', BMW['hairline'])])
        style.configure(
            'TCheckbutton',
            background=BMW['surface'],
            foreground=BMW['body'],
        )
        style.configure(
            'Horizontal.TScale',
            background=BMW['surface'],
            troughcolor=BMW['surface_soft'],
        )

    def _build_header(self):
        header = tk.Frame(self, bg=BMW['canvas'], padx=18, pady=12)
        header.pack(fill='x')

        title = tk.Frame(header, bg=BMW['canvas'])
        title.pack(side='left')
        tk.Label(
            title,
            text='M0609 OPERATOR HMI',
            bg=BMW['canvas'],
            fg=BMW['white'],
            font=('DejaVu Sans', 20, 'bold'),
        ).pack(anchor='w')
        tk.Label(
            title,
            text='DOOSAN M0609 + RG2  |  TCP POSITION / CONTACT MONITOR',
            bg=BMW['canvas'],
            fg=BMW['muted'],
            font=('DejaVu Sans', 9),
        ).pack(anchor='w')

        self.btn_estop = tk.Button(
            header,
            text='EMERGENCY\nSTOP',
            command=self.node.cmd_estop,
            bg=BMW['red'],
            fg=BMW['white'],
            activebackground='#a91d14',
            activeforeground=BMW['white'],
            relief='flat',
            font=('DejaVu Sans', 12, 'bold'),
            width=14,
            height=2,
        )
        self.btn_estop.pack(side='right')

        self.lbl_main_status = tk.Label(
            header,
            text='INITIALIZING',
            bg=BMW['muted'],
            fg=BMW['white'],
            font=('DejaVu Sans', 12, 'bold'),
            width=16,
            height=2,
        )
        self.lbl_main_status.pack(side='right', padx=10)

    def _panel(self, parent, title):
        panel = ttk.LabelFrame(parent, text=title, padding=10)
        panel.pack(fill='both', expand=True, pady=(0, 8))
        return panel

    def _build_body(self):
        body = tk.Frame(self, bg=BMW['canvas'], padx=18)
        body.pack(fill='both', expand=True)
        for column in range(3):
            body.grid_columnconfigure(column, weight=1, uniform='column')
        body.grid_rowconfigure(0, weight=1)

        left = tk.Frame(body, bg=BMW['canvas'])
        middle = tk.Frame(body, bg=BMW['canvas'])
        right = tk.Frame(body, bg=BMW['canvas'])
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 5))
        middle.grid(row=0, column=1, sticky='nsew', padx=5)
        right.grid(row=0, column=2, sticky='nsew', padx=(5, 0))

        self._build_state_panel(left)
        self._build_communication_panel(left)
        self._build_control_panel(middle)
        self._build_position_panel(right)
        self._build_sensor_panel(right)

    def _build_state_panel(self, parent):
        panel = self._panel(parent, '1. ROBOT STATE')
        self.lbl_mode = ttk.Label(panel, text='Mode: -')
        self.lbl_mode.pack(anchor='w', pady=2)
        self.lbl_robot_state = ttk.Label(panel, text='Robot state: -')
        self.lbl_robot_state.pack(anchor='w', pady=2)
        self.lbl_tcp_pose = ttk.Label(
            panel,
            text='TCP: -',
            justify='left',
            font=('DejaVu Sans Mono', 9),
        )
        self.lbl_tcp_pose.pack(anchor='w', pady=(8, 2))
        self.lbl_joint_pose = ttk.Label(
            panel,
            text='Joint: -',
            justify='left',
            font=('DejaVu Sans Mono', 8),
        )
        self.lbl_joint_pose.pack(anchor='w', pady=2)
        self.lbl_speed = ttk.Label(panel, text='Speed: -')
        self.lbl_speed.pack(anchor='w', pady=(8, 2))

    def _build_communication_panel(self, parent):
        panel = self._panel(parent, '2. COMMUNICATION')
        row = ttk.Frame(panel)
        row.pack(fill='x')
        ttk.Label(row, text='Namespace /').pack(side='left')
        self.var_namespace = tk.StringVar(value=self.node.ns)
        ttk.Entry(row, textvariable=self.var_namespace, width=10).pack(
            side='left',
            padx=4,
        )
        ttk.Button(
            row,
            text='Reconnect',
            command=self._reconnect,
        ).pack(side='left')

        self.lbl_ros = ttk.Label(panel, text='ROS: -')
        self.lbl_ros.pack(anchor='w', pady=(10, 2))
        self.lbl_pose_age = ttk.Label(panel, text='Pose data age: -')
        self.lbl_pose_age.pack(anchor='w', pady=2)

    def _build_control_panel(self, parent):
        panel = self._panel(parent, '3. ROBOT CONTROL')

        command_row = tk.Frame(panel, bg=BMW['surface'])
        command_row.pack(fill='x', pady=(0, 10))
        tk.Button(
            command_row,
            text='START',
            command=self.node.cmd_start_motion,
            bg=BMW['success'],
            fg=BMW['white'],
            relief='flat',
            font=('DejaVu Sans', 11, 'bold'),
            width=9,
        ).pack(side='left', padx=(0, 4), ipady=7)
        tk.Button(
            command_row,
            text='STOP',
            command=self.node.cmd_stop_motion,
            bg=BMW['warning'],
            fg=BMW['canvas'],
            relief='flat',
            font=('DejaVu Sans', 11, 'bold'),
            width=9,
        ).pack(side='left', padx=4, ipady=7)
        ttk.Button(
            command_row,
            text='E-STOP 해제',
            command=self.node.cmd_clear_estop,
        ).pack(side='right')

        self.lbl_control_mode = tk.Label(
            panel,
            text='RELATIVE moveL  |  DR_BASE  |  ACTIVE TCP',
            bg=BMW['blue'],
            fg=BMW['white'],
            font=('DejaVu Sans', 9, 'bold'),
            pady=5,
        )
        self.lbl_control_mode.pack(fill='x', pady=(0, 8))

        ttk.Label(panel, text='XYZ 이동 간격 (mm)').pack(anchor='w')
        self.var_step = tk.DoubleVar(value=1.0)
        ttk.Scale(
            panel,
            from_=1.0,
            to=20.0,
            variable=self.var_step,
            orient='horizontal',
        ).pack(fill='x')
        self.lbl_step = ttk.Label(panel, text='Step: 1.0 mm')
        self.lbl_step.pack(anchor='e')

        jog = tk.Frame(panel, bg=BMW['surface'])
        jog.pack(pady=8)
        buttons = (
            ('X-', 0, 0, 0, -1),
            ('X+', 0, 2, 0, 1),
            ('Y+', 0, 1, 1, 1),
            ('Y-', 2, 1, 1, -1),
            ('Z+', 1, 0, 2, 1),
            ('Z-', 1, 2, 2, -1),
        )
        for text, row, column, axis, direction in buttons:
            ttk.Button(
                jog,
                text=text,
                command=lambda a=axis, d=direction: self._jog(a, d),
                width=6,
            ).grid(row=row, column=column, padx=3, pady=3)
        ttk.Label(
            jog,
            text='moveL',
            anchor='center',
            width=7,
        ).grid(row=1, column=1, padx=3, pady=3)

        # --- TCP 회전 (Rx/Ry/Rz 상대 moveL) ---
        ttk.Label(panel, text='TCP 회전 간격 (deg)').pack(anchor='w', pady=(8, 0))
        self.var_rot_step = tk.DoubleVar(value=5.0)
        ttk.Scale(
            panel,
            from_=1.0,
            to=45.0,
            variable=self.var_rot_step,
            orient='horizontal',
        ).pack(fill='x')
        self.lbl_rot_step = ttk.Label(panel, text='Rot: 5.0 deg')
        self.lbl_rot_step.pack(anchor='e')

        rot = tk.Frame(panel, bg=BMW['surface'])
        rot.pack(pady=8)
        rot_buttons = (
            ('Rx-', 0, 0, 3, -1),
            ('Rx+', 0, 2, 3, 1),
            ('Ry+', 0, 1, 4, 1),
            ('Ry-', 2, 1, 4, -1),
            ('Rz+', 1, 0, 5, 1),
            ('Rz-', 1, 2, 5, -1),
        )
        for text, row, column, axis, direction in rot_buttons:
            ttk.Button(
                rot,
                text=text,
                command=lambda a=axis, d=direction: self._jog_rot(a, d),
                width=6,
            ).grid(row=row, column=column, padx=3, pady=3)
        ttk.Label(
            rot,
            text='회전',
            anchor='center',
            width=7,
        ).grid(row=1, column=1, padx=3, pady=3)

        # --- 그리퍼 회전 (J6 조인트 상대 jog) ---
        ttk.Label(panel, text='그리퍼 회전 J6 간격 (deg)').pack(
            anchor='w',
            pady=(8, 0),
        )
        self.var_j6_step = tk.DoubleVar(value=10.0)
        ttk.Scale(
            panel,
            from_=1.0,
            to=90.0,
            variable=self.var_j6_step,
            orient='horizontal',
        ).pack(fill='x')
        self.lbl_j6_step = ttk.Label(panel, text='J6: 10.0 deg')
        self.lbl_j6_step.pack(anchor='e')
        j6row = tk.Frame(panel, bg=BMW['surface'])
        j6row.pack(pady=6)
        ttk.Button(
            j6row,
            text='J6 -',
            command=lambda: self._jog_joint6(-1),
            width=10,
        ).pack(side='left', padx=4)
        ttk.Button(
            j6row,
            text='J6 +',
            command=lambda: self._jog_joint6(1),
            width=10,
        ).pack(side='left', padx=4)

        # --- 그리퍼 열기 / 닫기 (OnRobot RG2) ---
        grip = tk.Frame(panel, bg=BMW['surface'])
        grip.pack(fill='x', pady=(8, 0))
        ttk.Button(
            grip,
            text='그리퍼 열기',
            command=lambda: self.node.cmd_gripper('o'),
        ).pack(side='left', fill='x', expand=True, padx=(0, 2), ipady=4)
        ttk.Button(
            grip,
            text='그리퍼 닫기',
            command=lambda: self.node.cmd_gripper('c'),
        ).pack(side='left', fill='x', expand=True, padx=(2, 0), ipady=4)

        ttk.Label(panel, text='TCP 선속도 제한 (mm/s)').pack(
            anchor='w',
            pady=(8, 0),
        )
        self.var_linear_speed = tk.DoubleVar(
            value=self.node.safety.max_lin_vel_mms
        )
        ttk.Scale(
            panel,
            from_=1.0,
            to=300.0,
            variable=self.var_linear_speed,
            orient='horizontal',
            command=self._set_linear_speed,
        ).pack(fill='x')
        self.lbl_manual_offset = ttk.Label(
            panel,
            text='버튼 1회 = 선택한 BASE축으로 상대 직선이동',
        )
        self.lbl_manual_offset.pack(anchor='w', pady=(8, 0))

    def _build_position_panel(self, parent):
        panel = self._panel(parent, '4. TCP PATH POINTS')
        ttk.Label(
            panel,
            text='현재 활성 TCP의 DR_BASE 절대 자세를 순서대로 저장합니다.',
            wraplength=310,
        ).pack(anchor='w', pady=(0, 8))
        ttk.Button(
            panel,
            text='현재 위치를 다음 경로점으로 추가',
            command=self._capture_position,
        ).pack(fill='x', pady=2)

        list_frame = ttk.Frame(panel)
        list_frame.pack(fill='both', expand=True, pady=6)
        self.list_path_positions = tk.Listbox(
            list_frame,
            height=7,
            bg=BMW['surface_soft'],
            fg=BMW['body'],
            selectbackground=BMW['blue'],
            borderwidth=0,
            font=('DejaVu Sans Mono', 8),
        )
        path_scroll = ttk.Scrollbar(
            list_frame,
            orient='vertical',
            command=self.list_path_positions.yview,
        )
        self.list_path_positions.configure(yscrollcommand=path_scroll.set)
        self.list_path_positions.pack(side='left', fill='both', expand=True)
        path_scroll.pack(side='right', fill='y')

        edit_row = ttk.Frame(panel)
        edit_row.pack(fill='x', pady=2)
        ttk.Button(
            edit_row,
            text='마지막 좌표 삭제',
            command=self._remove_last_position,
        ).pack(side='left', fill='x', expand=True, padx=(0, 2))
        ttk.Button(
            edit_row,
            text='전체 삭제',
            command=self._clear_positions,
        ).pack(side='left', fill='x', expand=True, padx=(2, 0))
        ttk.Button(
            panel,
            text='전체 경로 CSV 클립보드 복사',
            command=self._copy_positions,
        ).pack(fill='x', pady=2)

    def _build_sensor_panel(self, parent):
        panel = self._panel(parent, '5. SENSOR / SAFETY')
        self.lbl_force = ttk.Label(
            panel,
            text='Force: -',
            font=('DejaVu Sans Mono', 9),
        )
        self.lbl_force.pack(anchor='w', pady=2)
        self.lbl_contact = tk.Label(
            panel,
            text='CONTACT: NORMAL',
            bg=BMW['success'],
            fg=BMW['white'],
            font=('DejaVu Sans', 10, 'bold'),
            pady=5,
        )
        self.lbl_contact.pack(fill='x', pady=6)
        self.lbl_safety = ttk.Label(
            panel,
            text='Safety: -',
            justify='left',
        )
        self.lbl_safety.pack(anchor='w', pady=2)

    def _build_log(self):
        frame = ttk.LabelFrame(self, text='6. EVENT LOG', padding=8)
        frame.pack(fill='both', padx=18, pady=(0, 14))
        tools = ttk.Frame(frame)
        tools.pack(fill='x', pady=(0, 4))
        ttk.Label(tools, text='Filter').pack(side='left')
        self.var_log_filter = tk.StringVar(value='ALL')
        ttk.Combobox(
            tools,
            textvariable=self.var_log_filter,
            values=('ALL', 'INFO', 'WARN', 'ERROR'),
            state='readonly',
            width=8,
        ).pack(side='left', padx=5)
        self.list_log = tk.Listbox(
            frame,
            height=7,
            bg=BMW['surface_soft'],
            fg=BMW['body'],
            selectbackground=BMW['blue'],
            borderwidth=0,
            font=('DejaVu Sans Mono', 9),
        )
        self.list_log.pack(fill='both', expand=True)

    def _reconnect(self):
        self.node.request_namespace_change(
            self.var_namespace.get().strip()
        )

    def _set_linear_speed(self, value):
        speed = float(value)
        self.node.safety.max_lin_vel_mms = speed
        self.node.safety.max_lin_acc_mms2 = min(
            400.0,
            max(100.0, speed * 2.0),
        )

    def _jog(self, axis, direction):
        step = float(self.var_step.get()) * direction
        self.node.cmd_jog_task(axis, step)

    def _jog_rot(self, axis, direction):
        step = float(self.var_rot_step.get()) * direction
        self.node.cmd_jog_task(axis, step)

    def _jog_joint6(self, direction):
        step = float(self.var_j6_step.get()) * direction
        self.node.cmd_jog_joint6(step)

    @staticmethod
    def _pose_text(pose):
        return '[' + ', '.join(f'{float(value):.4f}' for value in pose) + ']'

    def _capture_position(self):
        snap = self.node.shared.snapshot()
        if snap['dry_run']:
            self.node.shared.add_alert(
                'warn',
                '경로점 기록 실패: DRY-RUN 모드입니다',
            )
            return
        if not snap['connected'] or snap['comm_error']:
            self.node.shared.add_alert(
                'error',
                '경로점 기록 실패: 로봇 통신을 확인하세요',
            )
            return
        pose = [float(value) for value in snap['task_pose']]
        if not all(math.isfinite(value) for value in pose):
            self.node.shared.add_alert(
                'error',
                '경로점 기록 실패: 좌표가 유효하지 않습니다',
            )
            return
        self.path_positions.append(pose)
        index = len(self.path_positions)
        self.list_path_positions.insert(
            tk.END,
            f'P{index:02d}  ' + ', '.join(f'{value:.4f}' for value in pose),
        )
        self.list_path_positions.yview_moveto(1.0)
        self.node.shared.add_alert(
            'info',
            f'Path point P{index:02d} captured (TCP / DR_BASE): '
            f'{self._pose_text(pose)}',
        )

    def _remove_last_position(self):
        if not self.path_positions:
            self.node.shared.add_alert('warn', '삭제할 경로점이 없습니다')
            return
        removed_index = len(self.path_positions)
        self.path_positions.pop()
        self.list_path_positions.delete(tk.END)
        self.node.shared.add_alert(
            'info',
            f'Path point P{removed_index:02d} removed',
        )

    def _clear_positions(self):
        count = len(self.path_positions)
        self.path_positions.clear()
        self.list_path_positions.delete(0, tk.END)
        self.node.shared.add_alert(
            'info',
            f'Cleared {count} saved path point(s)',
        )

    def _copy_positions(self):
        if not self.path_positions:
            self.node.shared.add_alert('warn', '복사할 경로점이 없습니다')
            return
        lines = [
            '기준 좌표계 = DR_BASE',
            'TCP = 현재 활성 TCP',
            'X,Y,Z,A,B,C',
        ]
        lines.extend(
            ','.join(f'{value:.4f}' for value in pose)
            for pose in self.path_positions
        )
        self.clipboard_clear()
        self.clipboard_append('\n'.join(lines))
        self.node.shared.add_alert(
            'info',
            f'{len(self.path_positions)}개 경로점을 CSV로 복사했습니다',
        )

    def _refresh(self):
        snap = self.node.shared.snapshot()
        now = time.time()

        if snap['estop_active']:
            status, color = 'E-STOP', BMW['red']
        elif snap['comm_error']:
            status, color = 'COMM ERROR', BMW['red']
        elif snap['dry_run']:
            status, color = 'DRY-RUN', BMW['blue']
        elif self.node.motion_enabled:
            status, color = 'RUNNING', BMW['success']
        elif snap['connected']:
            status, color = 'IDLE', BMW['muted']
        else:
            status, color = 'DISCONNECTED', BMW['red']
        self.lbl_main_status.configure(text=status, bg=color)

        mode = 'SIMULATION' if snap['dry_run'] else (
            'MANUAL JOG' if self.node.motion_enabled else 'IDLE'
        )
        self.lbl_mode.configure(text=f'Mode: {mode}')
        self.lbl_robot_state.configure(
            text=f"Robot state code: {snap['robot_state_code']}"
        )

        pose = snap['task_pose']
        self.lbl_tcp_pose.configure(
            text=(
                f'X {pose[0]:9.3f}  Y {pose[1]:9.3f}\n'
                f'Z {pose[2]:9.3f} mm\n'
                f'A {pose[3]:9.3f}  B {pose[4]:9.3f}  '
                f'C {pose[5]:9.3f} deg'
            )
        )
        joints = snap['joint_pose']
        self.lbl_joint_pose.configure(
            text='Joint: ' + ' '.join(f'{value:6.1f}' for value in joints)
        )
        self.lbl_speed.configure(
            text=(
                f'Speed limit: linear '
                f'{self.node.safety.max_lin_vel_mms:.1f} mm/s, '
                f'angular {self.node.safety.max_tilt_vel_dps:.2f} deg/s'
            )
        )

        ros_text = 'CONNECTED' if snap['connected'] else 'DISCONNECTED'
        self.lbl_ros.configure(text=f'ROS: {ros_text}')
        pose_age_ms = max(0.0, now - self.node._last_posx_ok_t) * 1000.0
        self.lbl_pose_age.configure(
            text=f'Pose data age: {pose_age_ms:.0f} ms'
        )

        self.lbl_step.configure(
            text=f'Step: {float(self.var_step.get()):.1f} mm'
        )
        self.lbl_rot_step.configure(
            text=f'Rot: {float(self.var_rot_step.get()):.1f} deg'
        )
        self.lbl_j6_step.configure(
            text=f'J6: {float(self.var_j6_step.get()):.1f} deg'
        )
        self.lbl_manual_offset.configure(
            text=(
                '버튼 1회 = 선택한 BASE축으로 '
                f'{float(self.var_step.get()):.1f} mm 상대 moveL'
            )
        )

        wrench = snap['wrench_plate']
        self.lbl_force.configure(
            text=(
                f'F = [{wrench[0]:+.2f}, {wrench[1]:+.2f}, '
                f'{wrench[2]:+.2f}] N\n'
                f'M = [{wrench[3]:+.3f}, {wrench[4]:+.3f}, '
                f'{wrench[5]:+.3f}] Nm'
            )
        )
        contact = max(abs(float(value)) for value in wrench[:3]) >= 10.0
        self.lbl_contact.configure(
            text='CONTACT: DETECTED' if contact else 'CONTACT: NORMAL',
            bg=BMW['red'] if contact else BMW['success'],
        )
        self.lbl_safety.configure(
            text=(
                f"E-STOP: {'ACTIVE' if snap['estop_active'] else 'CLEAR'}\n"
                f"Communication: {'FAULT' if snap['comm_error'] else 'OK'}\n"
                f"Workspace limit: {'HIT' if snap['limit_hit'] else 'OK'}"
            )
        )

        alerts = snap['alerts']
        selected_filter = self.var_log_filter.get().lower()
        if (
            len(alerts) != self._last_alert_count
            or getattr(self, '_last_filter', None) != selected_filter
        ):
            self.list_log.delete(0, tk.END)
            for timestamp, level, message in list(alerts)[-100:]:
                if selected_filter != 'all' and level != selected_filter:
                    continue
                stamp = datetime.fromtimestamp(timestamp).strftime('%H:%M:%S')
                self.list_log.insert(
                    tk.END,
                    f'[{stamp}] {level.upper():5s}  {message}',
                )
            self.list_log.yview_moveto(1.0)
            self._last_alert_count = len(alerts)
            self._last_filter = selected_filter

        self.after(100, self._refresh)

    def _on_close(self):
        self.node.cmd_stop_motion()
        self.destroy()


# --------------------------------------------------------------------------------
# Entry point. Per the project's required architecture: rclpy spins on a background
# thread while Tkinter owns the main thread's mainloop().
# --------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    os.makedirs(LOG_DIR_DEFAULT, exist_ok=True)
    node = BallBalanceNode()

    # All of the node's timers/service-response callbacks stay on the node's
    # default (mutually-exclusive) callback group, so rclpy.spin() here serializes
    # them on this one background thread -- no two ever run concurrently, which is
    # what lets the control loop read SharedState/config fields without a lock
    # (see SharedState's docstring-comment) and keeps a slow discrete-move response
    # from racing the fast control tick. The Tkinter mainloop() owns the main thread.
    def _spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=_spin_node, daemon=True)
    spin_thread.start()

    app = SimpleHMI(node)
    try:
        app.mainloop()
    finally:
        node.cmd_stop_motion()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
