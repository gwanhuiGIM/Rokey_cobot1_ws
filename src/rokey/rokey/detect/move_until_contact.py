#!/usr/bin/env python3
"""Move from pose A to pose B in DR_BASE and stop on contact.

This is a contact-search example, not a safety-rated protective function.
Both poses are absolute TCP poses in the DR_BASE frame. The travel from A to
B is almost purely along Base +X (dx~145.5mm vs dy~0.02mm, dz~0.004mm for the
default poses), so contact is monitored on DR_AXIS_X.
"""

import math
import time

import rclpy
import DR_init
from dsr_msgs2.srv import MoveStop


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_move_until_contact"

DEFAULT_POSE_A = [404.4691, -58.3859, 65.4405, 122.3513, -179.1610, 87.2026]
DEFAULT_POSE_B = [549.9774, -58.3665, 65.4449, 122.4150, -179.1603, 87.2636]

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
        raise RuntimeError("/dsr01/motion/move_stop service is unavailable")

    request = MoveStop.Request()
    request.stop_mode = stop_mode
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

    if not future.done() or future.result() is None:
        raise RuntimeError("MoveStop service timed out")
    if not future.result().success:
        raise RuntimeError("MoveStop service rejected the stop request")
    return True


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    # Running this node must not move a real robot unless explicitly armed.
    node.declare_parameter("arm", False)
    node.declare_parameter("pose_a", DEFAULT_POSE_A)
    node.declare_parameter("pose_b", DEFAULT_POSE_B)
    node.declare_parameter("speed_mm_s", 10.0)
    node.declare_parameter("acc_mm_s2", 20.0)
    node.declare_parameter("contact_force_n", 10.0)
    node.declare_parameter("poll_period_s", 0.02)
    node.declare_parameter("min_z_mm", 100.0)
    node.declare_parameter("contact_debounce_count", 3)
    node.declare_parameter(
        "stiffness",
        [500.0, 3000.0, 3000.0, 200.0, 200.0, 200.0],
    )

    armed = node.get_parameter("arm").value
    pose_a = [float(v) for v in node.get_parameter("pose_a").value]
    pose_b = [float(v) for v in node.get_parameter("pose_b").value]
    speed = float(node.get_parameter("speed_mm_s").value)
    acceleration = float(node.get_parameter("acc_mm_s2").value)
    contact_force = float(node.get_parameter("contact_force_n").value)
    poll_period = float(node.get_parameter("poll_period_s").value)
    min_z = float(node.get_parameter("min_z_mm").value)
    contact_debounce_count = int(
        node.get_parameter("contact_debounce_count").value
    )
    stiffness = [
        float(value) for value in node.get_parameter("stiffness").value
    ]

    if len(pose_a) != 6 or len(pose_b) != 6:
        logger.error("pose_a and pose_b must each contain six values")
        node.destroy_node()
        rclpy.shutdown()
        return
    if pose_a[2] < min_z or pose_b[2] < min_z:
        logger.error(
            f"pose_a Z={pose_a[2]:.2f} mm or pose_b Z={pose_b[2]:.2f} mm is "
            f"below the configured safety floor min_z_mm={min_z:.2f} mm; "
            "refusing to move. Measure the actual clearance above the table "
            "in DR_BASE before lowering min_z_mm."
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if speed <= 0.0 or speed > 50.0:
        logger.error("speed_mm_s must be in the range (0, 50]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if acceleration <= 0.0 or acceleration > 100.0:
        logger.error("acc_mm_s2 must be in the range (0, 100]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if contact_force < 5.0 or contact_force > 30.0:
        logger.error("contact_force_n must be in the range [5, 30]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if poll_period < 0.01 or poll_period > 0.1:
        logger.error("poll_period_s must be in the range [0.01, 0.1]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if len(stiffness) != 6:
        logger.error("stiffness must contain six values")
        node.destroy_node()
        rclpy.shutdown()
        return
    if contact_debounce_count < 1 or contact_debounce_count > 10:
        logger.error("contact_debounce_count must be in the range [1, 10]")
        node.destroy_node()
        rclpy.shutdown()
        return

    if not armed:
        logger.warning(
            "Robot is NOT armed; no motion was sent. After checking Tool/TCP, "
            "workspace, and E-stop, rerun with --ros-args -p arm:=true"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        from DSR_ROBOT2 import (
            DR_AXIS_X,
            DR_BASE,
            DR_QSTOP,
            DR_STATE_IDLE,
            amovel,
            check_force_condition,
            check_motion,
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
    compliance_active = False
    motion_active = False
    contact_detected = False
    motion_was_observed = False

    distance = math.dist(pose_a[:3], pose_b[:3])

    try:
        logger.info(
            f"Active Tool='{get_tool()}', TCP='{get_tcp()}', "
            f"A={pose_a}, B={pose_b}, travel={distance:.2f} mm"
        )

        # Rigid, non-compliant approach to the start pose A; no contact expected here.
        if movel(posx(pose_a), vel=speed, acc=acceleration, ref=DR_BASE) != 0:
            raise RuntimeError("Failed to reach pose A")

        # A pre-existing load at A must not be mistaken for a new contact.
        initial_force = get_tool_force(ref=DR_BASE)
        initial_fx = abs(float(initial_force[0]))
        logger.info(f"At A: initial |Fx|={initial_fx:.2f} N")
        if initial_fx >= contact_force:
            raise RuntimeError(
                "Initial force at A already exceeds the contact threshold. "
                "Remove contact and verify the configured Tool/TCP payload."
            )

        set_ref_coord(DR_BASE)
        if task_compliance_ctrl(stx=stiffness, time=0.5) != 0:
            raise RuntimeError("Failed to enable task compliance control")
        compliance_active = True

        # Give the real controller time to complete its mode transition.
        time.sleep(0.6)

        logger.warning(
            f"ARMED: A -> B, {speed:.1f} mm/s, contact threshold {contact_force:.1f} N"
        )
        result = amovel(posx(pose_b), vel=speed, acc=acceleration, ref=DR_BASE)
        if result != 0:
            raise RuntimeError("The asynchronous move command was rejected")
        motion_active = True

        deadline = time.monotonic() + distance / speed + 5.0
        consecutive_hits = 0
        while rclpy.ok() and time.monotonic() < deadline:
            # DSR_ROBOT2's ROS wrapper returns 0 when the condition is true
            # and -1 when false, unlike the DRL manual's True/False wording.
            condition = check_force_condition(
                DR_AXIS_X,
                min=contact_force,
                ref=DR_BASE,
            )
            if condition == 0:
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            if consecutive_hits >= contact_debounce_count:
                contact_detected = True
                force = get_tool_force(ref=DR_BASE)
                logger.warning(
                    f"CONTACT: Fx={force[0]:.2f} N "
                    f"({consecutive_hits} consecutive hits). Sending quick stop."
                )
                request_stop(node, stop_client, DR_QSTOP)
                break

            motion_state = check_motion()
            if motion_state != DR_STATE_IDLE:
                motion_was_observed = True
            elif motion_was_observed:
                motion_active = False
                break
            time.sleep(poll_period)

        if motion_active:
            if not contact_detected:
                logger.error("Monitoring timed out; stopping the robot")
                request_stop(node, stop_client, DR_QSTOP)
            if not wait_until_idle(
                check_motion,
                DR_STATE_IDLE,
                timeout_sec=2.0,
            ):
                raise RuntimeError(
                    "The controller did not report an idle state after stop"
                )
            motion_active = False

        if contact_detected:
            logger.info("Stopped on contact before reaching B")
        else:
            logger.warning("Reached B without detecting contact")

    except KeyboardInterrupt:
        logger.warning("Interrupted by user; stopping active motion")
        if motion_active:
            request_stop(node, stop_client, DR_QSTOP)
            if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                motion_active = False
    except Exception as exc:
        logger.error(f"Move-until-contact aborted: {exc}")
        if motion_active:
            try:
                request_stop(node, stop_client, DR_QSTOP)
                if wait_until_idle(
                    check_motion,
                    DR_STATE_IDLE,
                    timeout_sec=2.0,
                ):
                    motion_active = False
            except Exception as stop_exc:
                logger.error(f"Automatic stop request failed: {stop_exc}")
    finally:
        if compliance_active:
            if motion_active:
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
