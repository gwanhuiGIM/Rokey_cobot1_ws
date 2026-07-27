#!/usr/bin/env python3
"""Infer a cylindrical object's center and radius by compliant radial probing.

The scanned object is assumed to be a CYLINDER standing on the table (its
cross-section at the scan height z_mm is a circle). That assumption lets a
small number of contact points determine the whole shape: a circle has three
degrees of freedom (center x, center y, radius), so three contacts define it
and every extra contact improves the least-squares fit.

The probing motion is the one proven in move_until_contact.py: move toward
the object under task compliance control (soft in X/Y so the tool yields,
stiff in Z to hold height) and stop the instant check_force_condition
reports sustained external force.

Per probe angle around the approximate center (center_xy):
  1. Rigidly reposition to an "outer" point on a circle of radius
     probe_radius_mm around center_xy, at the fixed scan height z_mm.
  2. Enable task compliance control.
  3. Move inward toward center_xy (compliant) and stop on debounced contact
     (Base X or Y force), or give up at probe_min_radius_mm.
  4. Release compliance and rigidly retreat back to the outer point.

After the sweep the contact points are fitted with a circle (Kasa algebraic
least-squares fit). The reported cylinder radius subtracts
tool_contact_radius_mm, because the recorded TCP position stops one tool
radius short of the true surface. Results (contact points + fitted circle)
are logged and saved to a timestamped JSON file.

Assumptions and tuning:
  - center_xy must be INSIDE the cylinder's footprint (any rough guess works;
    the fit recovers the true center) and probe_radius_mm must clear the
    cylinder at every angle, since the tool travels between angles along that
    outer circle.
  - At least 3 contacts are required for a fit; probes that reach
    probe_min_radius_mm without contact are recorded as misses.

This is a contact-search example, not a safety-rated protective function.
"""

import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
import DR_init
from dsr_msgs2.srv import MoveStop


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_inference_shape"

# Fixed tool orientation used for every probe (rx, ry, rz), taken from the
# taught poses in move_until_contact.py. Retune for the active Tool/TCP.
DEFAULT_TOOL_ORIENTATION_DEG = [122.3513, -179.1610, 87.2026]
# Rough guess of the cylinder's center and the scan height in DR_BASE; must
# be retaught per object/scene. The guess only needs to be inside the
# cylinder — the circle fit recovers the true center.
DEFAULT_CENTER_XY = [500.0, 80.0]
DEFAULT_Z_MM = 65.44

# A circle is determined by 3 points; fewer contacts cannot be fitted.
MIN_CONTACTS_FOR_FIT = 3

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def wait_until_idle(check_motion, idle_state, timeout_sec):
    """Return True when the controller reports that path motion is idle."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if check_motion() == idle_state:
            return True
        time.sleep(0.02)
    return False


def request_stop(node, client, stop_mode):
    """Call the controller's MoveStop service and return its success flag."""
    if not client.wait_for_service(timeout_sec=2.0):
        raise RuntimeError(f"/{ROBOT_ID}/motion/move_stop service is unavailable")

    request = MoveStop.Request()
    request.stop_mode = stop_mode
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if not future.done() or future.result() is None:
        raise RuntimeError("MoveStop service timed out")
    if not future.result().success:
        raise RuntimeError("MoveStop service rejected the stop request")
    return True


def build_angles(start_deg, end_deg, step_deg):
    """Return the sweep angles, half-open on the end (no duplicate wrap)."""
    span = end_deg - start_deg
    count = max(1, round(span / step_deg))
    return [start_deg + index * step_deg for index in range(count)]


def fit_circle(points_xy):
    """Kasa algebraic least-squares circle fit.

    Minimizes sum((x^2 + y^2 + D*x + E*y + F)^2) over D, E, F by solving the
    3x3 normal equations, then converts to center/radius. Returns
    (center_x, center_y, radius, rms_residual_mm). Raises ValueError when the
    points are degenerate (fewer than 3, or collinear).
    """
    n = len(points_xy)
    if n < MIN_CONTACTS_FOR_FIT:
        raise ValueError(f"circle fit needs at least {MIN_CONTACTS_FOR_FIT} points, got {n}")

    # Normal equations A^T A [D E F]^T = A^T b with rows [x, y, 1], b = -(x^2+y^2).
    sxx = sxy = syy = sx = sy = 0.0
    sxz = syz = sz = 0.0
    for x, y in points_xy:
        z = -(x * x + y * y)
        sxx += x * x
        sxy += x * y
        syy += y * y
        sx += x
        sy += y
        sxz += x * z
        syz += y * z
        sz += z

    # Solve the symmetric 3x3 system via Cramer's rule.
    a11, a12, a13 = sxx, sxy, sx
    a21, a22, a23 = sxy, syy, sy
    a31, a32, a33 = sx, sy, float(n)
    det = (
        a11 * (a22 * a33 - a23 * a32)
        - a12 * (a21 * a33 - a23 * a31)
        + a13 * (a21 * a32 - a22 * a31)
    )
    if abs(det) < 1e-9:
        raise ValueError("contact points are collinear; cannot fit a circle")

    det_d = (
        sxz * (a22 * a33 - a23 * a32)
        - a12 * (syz * a33 - a23 * sz)
        + a13 * (syz * a32 - a22 * sz)
    )
    det_e = (
        a11 * (syz * a33 - a23 * sz)
        - sxz * (a21 * a33 - a23 * a31)
        + a13 * (a21 * sz - syz * a31)
    )
    det_f = (
        a11 * (a22 * sz - syz * a32)
        - a12 * (a21 * sz - syz * a31)
        + sxz * (a21 * a32 - a22 * a31)
    )
    d = det_d / det
    e = det_e / det
    f = det_f / det

    center_x = -d / 2.0
    center_y = -e / 2.0
    radius_sq = center_x * center_x + center_y * center_y - f
    if radius_sq <= 0.0:
        raise ValueError("circle fit produced a non-positive radius")
    radius = math.sqrt(radius_sq)

    rms = math.sqrt(
        sum(
            (math.dist((x, y), (center_x, center_y)) - radius) ** 2
            for x, y in points_xy
        ) / n
    )
    return center_x, center_y, radius, rms


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    # Running this node must not move a real robot unless explicitly armed.
    node.declare_parameter("arm", False)
    node.declare_parameter("center_xy", DEFAULT_CENTER_XY)
    node.declare_parameter("z_mm", DEFAULT_Z_MM)
    node.declare_parameter("min_z_mm", 30.0)
    node.declare_parameter("tool_orientation_deg", DEFAULT_TOOL_ORIENTATION_DEG)
    node.declare_parameter("probe_radius_mm", 120.0)
    node.declare_parameter("probe_min_radius_mm", 0.0)
    # The TCP stops one effective tool radius short of the cylinder surface;
    # this offset is subtracted from the fitted radius. Measure it for the
    # actual finger/probe geometry (0 keeps the raw TCP-circle radius).
    node.declare_parameter("tool_contact_radius_mm", 0.0)
    # A cylinder needs only a handful of probes: 60 deg spacing gives 6
    # contacts, twice the minimum needed for a stable circle fit.
    node.declare_parameter("angle_start_deg", 0.0)
    node.declare_parameter("angle_end_deg", 360.0)
    node.declare_parameter("angle_step_deg", 60.0)
    node.declare_parameter("move_to_first_outer", False)
    node.declare_parameter("start_position_tolerance_mm", 5.0)
    node.declare_parameter("start_orientation_tolerance_deg", 3.0)
    node.declare_parameter("probe_speed_mm_s", 8.0)
    node.declare_parameter("probe_acc_mm_s2", 15.0)
    node.declare_parameter("reposition_speed_mm_s", 20.0)
    node.declare_parameter("reposition_acc_mm_s2", 40.0)
    node.declare_parameter("contact_force_n", 10.0)
    node.declare_parameter("contact_debounce_count", 3)
    node.declare_parameter("poll_period_s", 0.02)
    node.declare_parameter("settle_time_s", 0.5)
    node.declare_parameter(
        "stiffness",
        # Soft in X/Y so the tool yields on contact from any probe angle;
        # stiff in Z to hold the scan height.
        [500.0, 500.0, 3000.0, 200.0, 200.0, 200.0],
    )
    node.declare_parameter("save_results", True)
    node.declare_parameter(
        "output_dir", str(Path.home() / "shape_inference_logs")
    )

    armed = bool(node.get_parameter("arm").value)
    center_xy = [float(v) for v in node.get_parameter("center_xy").value]
    z_mm = float(node.get_parameter("z_mm").value)
    min_z = float(node.get_parameter("min_z_mm").value)
    tool_orientation = [
        float(v) for v in node.get_parameter("tool_orientation_deg").value
    ]
    probe_radius = float(node.get_parameter("probe_radius_mm").value)
    probe_min_radius = float(node.get_parameter("probe_min_radius_mm").value)
    tool_contact_radius = float(
        node.get_parameter("tool_contact_radius_mm").value
    )
    angle_start = float(node.get_parameter("angle_start_deg").value)
    angle_end = float(node.get_parameter("angle_end_deg").value)
    angle_step = float(node.get_parameter("angle_step_deg").value)
    move_to_first_outer = bool(
        node.get_parameter("move_to_first_outer").value
    )
    position_tolerance = float(
        node.get_parameter("start_position_tolerance_mm").value
    )
    orientation_tolerance = float(
        node.get_parameter("start_orientation_tolerance_deg").value
    )
    probe_speed = float(node.get_parameter("probe_speed_mm_s").value)
    probe_acceleration = float(node.get_parameter("probe_acc_mm_s2").value)
    reposition_speed = float(node.get_parameter("reposition_speed_mm_s").value)
    reposition_acceleration = float(
        node.get_parameter("reposition_acc_mm_s2").value
    )
    contact_force = float(node.get_parameter("contact_force_n").value)
    contact_debounce_count = int(
        node.get_parameter("contact_debounce_count").value
    )
    poll_period = float(node.get_parameter("poll_period_s").value)
    settle_time = float(node.get_parameter("settle_time_s").value)
    stiffness = [float(v) for v in node.get_parameter("stiffness").value]
    save_results = bool(node.get_parameter("save_results").value)
    output_dir = Path(str(node.get_parameter("output_dir").value))

    # -- Parameter validation -------------------------------------------------
    if len(center_xy) != 2:
        logger.error("center_xy must contain [x, y]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if len(tool_orientation) != 3:
        logger.error("tool_orientation_deg must contain [rx, ry, rz]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if z_mm < min_z:
        logger.error(
            f"z_mm={z_mm:.2f} mm is below the configured safety floor "
            f"min_z_mm={min_z:.2f} mm; refusing to move."
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if probe_min_radius < 0.0 or probe_radius <= probe_min_radius:
        logger.error(
            "probe_radius_mm must be greater than probe_min_radius_mm >= 0"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if probe_radius > 500.0:
        logger.error("probe_radius_mm must be at most 500")
        node.destroy_node()
        rclpy.shutdown()
        return
    if tool_contact_radius < 0.0 or tool_contact_radius > 100.0:
        logger.error("tool_contact_radius_mm must be in the range [0, 100]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if angle_step <= 0.0 or angle_step > 120.0:
        # More than 120 deg between probes cannot yield the 3 spread-out
        # contacts a full-circle fit needs.
        logger.error("angle_step_deg must be in the range (0, 120]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if angle_end <= angle_start or (angle_end - angle_start) > 360.0:
        logger.error(
            "angle_end_deg must be greater than angle_start_deg by at most "
            "360 degrees"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    angles = build_angles(angle_start, angle_end, angle_step)
    if len(angles) < MIN_CONTACTS_FOR_FIT:
        logger.error(
            f"The angle sweep yields only {len(angles)} probe(s); a circle "
            f"fit needs at least {MIN_CONTACTS_FOR_FIT} contacts. Reduce "
            "angle_step_deg or widen the angle span."
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if len(angles) > 60:
        logger.error(
            f"angle_step_deg={angle_step:.2f} over the requested span yields "
            f"{len(angles)} probes; a cylinder does not need more than 60 — "
            "increase angle_step_deg"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if probe_speed <= 0.0 or probe_speed > 20.0:
        logger.error("probe_speed_mm_s must be in the range (0, 20]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if probe_acceleration <= 0.0 or probe_acceleration > 50.0:
        logger.error("probe_acc_mm_s2 must be in the range (0, 50]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if reposition_speed <= 0.0 or reposition_speed > 50.0:
        logger.error("reposition_speed_mm_s must be in the range (0, 50]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if reposition_acceleration <= 0.0 or reposition_acceleration > 100.0:
        logger.error("reposition_acc_mm_s2 must be in the range (0, 100]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if contact_force < 5.0 or contact_force > 30.0:
        logger.error("contact_force_n must be in the range [5, 30]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if contact_debounce_count < 1 or contact_debounce_count > 10:
        logger.error("contact_debounce_count must be in the range [1, 10]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if poll_period < 0.01 or poll_period > 0.1:
        logger.error("poll_period_s must be in the range [0.01, 0.1]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if settle_time < 0.0 or settle_time > 3.0:
        logger.error("settle_time_s must be in the range [0, 3]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if len(stiffness) != 6:
        logger.error("stiffness must contain six values")
        node.destroy_node()
        rclpy.shutdown()
        return

    if not armed:
        logger.warning(
            "Robot is NOT armed; no motion was sent. After checking Tool/TCP, "
            "center_xy, z_mm, and the workspace, rerun with "
            "--ros-args -p arm:=true"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        from DSR_ROBOT2 import (
            DR_AXIS_X,
            DR_AXIS_Y,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_QSTOP,
            DR_STATE_IDLE,
            amovel,
            check_force_condition,
            check_motion,
            get_current_posx,
            get_tcp,
            get_tool,
            get_tool_force,
            movel,
            release_compliance_ctrl,
            set_ref_coord,
            task_compliance_ctrl,
        )
        from DR_common2 import posx
    except ImportError as exc:
        logger.error(f"Failed to import Doosan API: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    stop_client = node.create_client(MoveStop, "motion/move_stop")
    force_axes = ((DR_AXIS_X, "X"), (DR_AXIS_Y, "Y"))
    state = {"motion_active": False, "compliance_active": False}

    def probe_inward(outer_pose, inner_pose, label):
        """amovel from outer_pose toward inner_pose, stop on contact.

        Returns ("contact" | "miss", final_pose). Leaves the robot stopped
        and idle. Raises on rejection/timeout.
        """
        segment_distance = math.dist(outer_pose[:3], inner_pose[:3])
        result = amovel(
            posx(inner_pose),
            vel=[probe_speed, 5.0],
            acc=[probe_acceleration, 10.0],
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        if result != 0:
            raise RuntimeError(f"{label}: amovel was rejected")
        state["motion_active"] = True
        motion_was_observed = False
        consecutive_hits = 0
        outcome = None
        move_start = time.monotonic()
        deadline = move_start + segment_distance / probe_speed + 5.0
        last_diag = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            in_settle = (time.monotonic() - move_start) < settle_time
            hit_axes = [
                axis_name
                for axis_const, axis_name in force_axes
                if check_force_condition(
                    axis_const, min=contact_force, ref=DR_BASE
                ) == 0
            ]
            if hit_axes and not in_settle:
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            now = time.monotonic()
            if now - last_diag >= 0.5:
                last_diag = now
                logger.info(
                    f"[{label}] {'settling' if in_settle else 'probing'}, "
                    f"hits={hit_axes}"
                )

            if consecutive_hits >= contact_debounce_count:
                request_stop(node, stop_client, DR_QSTOP)
                force = get_tool_force(ref=DR_BASE)
                logger.info(
                    f"[{label}] CONTACT on Base axis {','.join(hit_axes)} "
                    f"(force={force}, {consecutive_hits} consecutive hits)"
                )
                outcome = "contact"
                break

            motion_state = check_motion()
            if motion_state != DR_STATE_IDLE:
                motion_was_observed = True
            elif motion_was_observed:
                outcome = "miss"
                break
            time.sleep(poll_period)

        if outcome is None:
            logger.warning(
                f"[{label}] monitoring timed out before the inner limit; "
                "stopping"
            )
            request_stop(node, stop_client, DR_QSTOP)
            outcome = "miss"

        if not wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
            raise RuntimeError(f"{label}: controller did not report idle after stop")
        state["motion_active"] = False

        pose_raw, _ = get_current_posx(ref=DR_BASE)
        return outcome, [float(v) for v in pose_raw[:6]]

    contact_points = []
    misses = 0

    try:
        logger.info(
            f"Active Tool='{get_tool()}', TCP='{get_tcp()}', "
            f"cylinder center guess={center_xy}, z={z_mm:.2f} mm, "
            f"probe_radius={probe_radius:.1f} mm, {len(angles)} angles"
        )

        first_angle_rad = math.radians(angles[0])
        first_outer_pose = [
            center_xy[0] + probe_radius * math.cos(first_angle_rad),
            center_xy[1] + probe_radius * math.sin(first_angle_rad),
            z_mm,
            *tool_orientation,
        ]
        start_pose_raw, _ = get_current_posx(ref=DR_BASE)
        start_pose = [float(v) for v in start_pose_raw[:6]]
        start_position_error = math.dist(start_pose[:3], first_outer_pose[:3])
        start_orientation_error = max(
            abs((current - target + 180.0) % 360.0 - 180.0)
            for current, target in zip(start_pose[3:], first_outer_pose[3:])
        )
        if (
            start_position_error > position_tolerance
            or start_orientation_error > orientation_tolerance
        ):
            if not move_to_first_outer:
                raise RuntimeError(
                    "Current TCP is not at the first outer probe point: "
                    f"position error={start_position_error:.2f} mm, "
                    f"orientation error={start_orientation_error:.2f} deg. "
                    "Move there safely first (verify the straight line from "
                    "here is collision-free) or set "
                    "move_to_first_outer:=true."
                )
            logger.warning(
                "move_to_first_outer=true: moving to the first outer probe "
                "point without contact monitoring. Verify this leg is "
                "collision-free first."
            )
            result = movel(
                posx(first_outer_pose),
                vel=[reposition_speed, 5.0],
                acc=[reposition_acceleration, 10.0],
                ref=DR_BASE,
                mod=DR_MV_MOD_ABS,
            )
            if result != 0:
                raise RuntimeError(
                    "Move to the first outer probe point was rejected"
                )

        logger.warning(
            f"ARMED: {len(angles)}-angle cylinder probe around {center_xy}, "
            f"radius {probe_min_radius:.1f}-{probe_radius:.1f} mm, "
            f"probe speed {probe_speed:.1f} mm/s, contact threshold "
            f"{contact_force:.1f} N"
        )

        for index, angle_deg in enumerate(angles, start=1):
            angle_rad = math.radians(angle_deg)
            dir_out = (math.cos(angle_rad), math.sin(angle_rad))
            outer_pose = [
                center_xy[0] + probe_radius * dir_out[0],
                center_xy[1] + probe_radius * dir_out[1],
                z_mm,
                *tool_orientation,
            ]
            inner_pose = [
                center_xy[0] + probe_min_radius * dir_out[0],
                center_xy[1] + probe_min_radius * dir_out[1],
                z_mm,
                *tool_orientation,
            ]
            label = f"probe {index}/{len(angles)} (angle={angle_deg:.1f} deg)"

            result = movel(
                posx(outer_pose),
                vel=[reposition_speed, 5.0],
                acc=[reposition_acceleration, 10.0],
                ref=DR_BASE,
                mod=DR_MV_MOD_ABS,
            )
            if result != 0:
                raise RuntimeError(f"{label}: reposition to outer point was rejected")

            set_ref_coord(DR_BASE)
            if task_compliance_ctrl(stx=stiffness, time=0.5) != 0:
                raise RuntimeError(f"{label}: failed to enable task compliance control")
            state["compliance_active"] = True
            time.sleep(0.6)  # let the controller finish the mode transition

            outcome, stop_pose = probe_inward(outer_pose, inner_pose, label)

            release_compliance_ctrl()
            state["compliance_active"] = False
            time.sleep(0.3)

            if outcome == "contact":
                contact_points.append({
                    "index": index,
                    "angle_deg": angle_deg,
                    "pose": stop_pose,
                    "timestamp": datetime.now().isoformat(),
                })
                logger.info(f"{label}: recorded contact point {stop_pose}")
            else:
                misses += 1
                logger.warning(f"{label}: no contact within the scan radius")

            # Rigid retreat back to the outer point, clear of the cylinder,
            # before moving on to the next angle.
            result = movel(
                posx(outer_pose),
                vel=[reposition_speed, 5.0],
                acc=[reposition_acceleration, 10.0],
                ref=DR_BASE,
                mod=DR_MV_MOD_ABS,
            )
            if result != 0:
                raise RuntimeError(f"{label}: retreat to outer point was rejected")

        hits = len(contact_points)
        logger.info(
            f"Scan complete: {hits} contact point(s), {misses} miss(es) "
            f"out of {len(angles)} angle(s)"
        )

        fit = None
        if hits < MIN_CONTACTS_FOR_FIT:
            logger.error(
                f"Only {hits} contact(s) were recorded; a circle fit needs "
                f"at least {MIN_CONTACTS_FOR_FIT}. Check center_xy, "
                "probe_radius_mm, z_mm, and the contact threshold."
            )
        else:
            try:
                fitted_cx, fitted_cy, fitted_r, rms = fit_circle(
                    [point["pose"][:2] for point in contact_points]
                )
            except ValueError as exc:
                logger.error(f"Circle fit failed: {exc}")
            else:
                # The TCP circle sits one effective tool radius outside the
                # real surface.
                object_radius = fitted_r - tool_contact_radius
                fit = {
                    "center_xy": [fitted_cx, fitted_cy],
                    "tcp_circle_radius_mm": fitted_r,
                    "tool_contact_radius_mm": tool_contact_radius,
                    "object_radius_mm": object_radius,
                    "object_diameter_mm": 2.0 * object_radius,
                    "rms_residual_mm": rms,
                    "num_points": hits,
                }
                center_shift = math.dist((fitted_cx, fitted_cy), center_xy)
                logger.info(
                    "CYLINDER FIT: center=("
                    f"{fitted_cx:.2f}, {fitted_cy:.2f}) mm in DR_BASE "
                    f"(shifted {center_shift:.2f} mm from the initial guess), "
                    f"radius={object_radius:.2f} mm "
                    f"(diameter={2.0 * object_radius:.2f} mm), "
                    f"fit RMS={rms:.3f} mm over {hits} points"
                )
                if object_radius <= 0.0:
                    logger.warning(
                        "object_radius_mm is not positive after subtracting "
                        f"tool_contact_radius_mm={tool_contact_radius:.1f}; "
                        "check that value against the real tool geometry"
                    )
                if rms > 2.0:
                    logger.warning(
                        f"Fit RMS {rms:.2f} mm is high for a cylinder; the "
                        "object may not be cylindrical, may have moved during "
                        "probing, or some contacts may be false triggers"
                    )

        if save_results and hits > 0:
            output_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = output_dir / f"inference_shape_{timestamp}.json"
            output_path.write_text(json.dumps({
                "created_at": datetime.now().isoformat(),
                "robot_id": ROBOT_ID,
                "robot_model": ROBOT_MODEL,
                "shape_model": "cylinder",
                "center_guess_xy": center_xy,
                "z_mm": z_mm,
                "probe_radius_mm": probe_radius,
                "probe_min_radius_mm": probe_min_radius,
                "contact_force_n": contact_force,
                "angles_deg": angles,
                "hits": hits,
                "misses": misses,
                "contact_points": contact_points,
                "cylinder_fit": fit,
            }, indent=2))
            logger.info(f"Saved scan results to {output_path}")

    except KeyboardInterrupt:
        logger.warning("Interrupted by user; stopping active motion")
        if state["motion_active"]:
            request_stop(node, stop_client, DR_QSTOP)
            if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                state["motion_active"] = False
    except Exception as exc:
        logger.error(f"Shape inference scan aborted: {exc}")
        if state["motion_active"]:
            try:
                request_stop(node, stop_client, DR_QSTOP)
                if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                    state["motion_active"] = False
            except Exception as stop_exc:
                logger.error(f"Automatic stop request failed: {stop_exc}")
    finally:
        if state["compliance_active"]:
            if state["motion_active"]:
                logger.error(
                    "Motion state is not confirmed idle; compliance control "
                    "was intentionally left enabled. Use the teach pendant "
                    "to stop/recover the robot."
                )
            else:
                try:
                    release_compliance_ctrl()
                except Exception as exc:
                    logger.error(f"Failed to release compliance control: {exc}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
