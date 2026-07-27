#!/usr/bin/env python3
"""Move the configured TCP from pose A to pose B and stop on contact.

This is a contact-search example, not a safety-rated protective function.
Both poses are absolute TCP poses in the Base coordinate system.

Besides the single A-to-B search, this node can also sweep a multi-waypoint
path (see PATH_WAYPOINTS / the `path` parameter) via `path_mode`:
  - "test":    blind moveL dry run with contact detection OFF, to verify the
               path is collision-free before ever arming contact detection.
  - "contact": amovel through the waypoints in order, monitoring Base-frame
               force change on every axis during each segment. The whole
               sweep aborts immediately (does not continue to the next
               waypoint) the first time contact is detected.
  - "off":     the original single A-to-B contact search.
"""

import math
import time

import rclpy
import DR_init
from dsr_msgs2.srv import (
    CheckMotion,
    GetCurrentPosx,
    GetCurrentToolFlangePosx,
    GetCurrentTcp,
    GetCurrentTool,
    GetToolForce,
    MoveLine,
    MoveStop,
    SetCurrentTcp,
)


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_contact_stop"
TCP_NAME = "GripperDA_v1"

# Taught TCP poses: absolute pose in DR_BASE, using the controller's active TCP.
POSITION_A = [404.4691, -58.3859, 65.4405, 122.3513, -179.1610, 87.2026]
POSITION_B = [549.9774, -58.3665, 65.4449, 122.4150, -179.1603, 87.2636]
# Taught cup-shelf corners in DR_BASE, reversed (last captured point first) so
# the sweep traces a ㄹ-shaped (boustrophedon) path across the shelf.
PATH_WAYPOINTS = [
    [426.8600, 236.4322, 50.5734, 118.6593, -179.1288, 83.6433],
    [587.0817, 236.5865, 50.6877, 120.1958, -179.1272, 85.1806],
    [587.1510, 76.3926, 50.6925, 120.1473, -179.1257, 85.1367],
    [427.1051, 76.4458, 50.6712, 119.8656, -179.1272, 84.8540],
    [426.9945, -83.5724, 50.6119, 119.2391, -179.1323, 84.2300],
    [607.1321, -83.5495, 50.7010, 120.0698, -179.1222, 85.0555],
]

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


def call_service(node, client, request, service_name, timeout_sec=3.0):
    """Call a ROS service with a finite response timeout."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"{service_name} service timed out")
    return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    # Running this node must not move a real robot unless explicitly armed.
    node.declare_parameter("arm", False)
    node.declare_parameter("tcp_name", TCP_NAME)
    node.declare_parameter("path_mode", "test")
    node.declare_parameter(
        "path",
        [value for waypoint in PATH_WAYPOINTS for value in waypoint],
    )
    node.declare_parameter("path_min_z_mm", 30.0)
    node.declare_parameter("path_speed_mm_s", 20.0)
    node.declare_parameter("path_acc_mm_s2", 40.0)
    node.declare_parameter("move_to_path_start", False)
    node.declare_parameter("contact_retreat_mm", 20.0)
    node.declare_parameter("contact_advance_mm", 50.0)
    node.declare_parameter("position_a", POSITION_A)
    node.declare_parameter("position_b", POSITION_B)
    node.declare_parameter("move_to_a", False)
    node.declare_parameter("motion_speed_mm_s", 5.0)
    node.declare_parameter("motion_acc_mm_s2", 10.0)
    node.declare_parameter("contact_force_n", 10.0)
    node.declare_parameter("poll_period_s", 0.02)
    node.declare_parameter("a_position_tolerance_mm", 2.0)
    node.declare_parameter("a_orientation_tolerance_deg", 2.0)

    armed = node.get_parameter("arm").value
    tcp_name = str(node.get_parameter("tcp_name").value)
    path_mode = str(node.get_parameter("path_mode").value)
    flat_path = [float(value) for value in node.get_parameter("path").value]
    path = [
        flat_path[index:index + 6]
        for index in range(0, len(flat_path), 6)
    ]
    path_min_z = float(node.get_parameter("path_min_z_mm").value)
    path_speed = float(node.get_parameter("path_speed_mm_s").value)
    path_acceleration = float(node.get_parameter("path_acc_mm_s2").value)
    move_to_path_start = bool(node.get_parameter("move_to_path_start").value)
    contact_retreat_mm = float(node.get_parameter("contact_retreat_mm").value)
    contact_advance_mm = float(node.get_parameter("contact_advance_mm").value)
    position_a = [
        float(value) for value in node.get_parameter("position_a").value
    ]
    position_b = [
        float(value) for value in node.get_parameter("position_b").value
    ]
    move_to_a = bool(node.get_parameter("move_to_a").value)
    speed = float(node.get_parameter("motion_speed_mm_s").value)
    acceleration = float(node.get_parameter("motion_acc_mm_s2").value)
    contact_force = float(node.get_parameter("contact_force_n").value)
    poll_period = float(node.get_parameter("poll_period_s").value)
    position_tolerance = float(
        node.get_parameter("a_position_tolerance_mm").value
    )
    orientation_tolerance = float(
        node.get_parameter("a_orientation_tolerance_deg").value
    )
    if len(position_a) != 6 or len(position_b) != 6:
        logger.error(
            "position_a and position_b must each contain "
            "[x, y, z, rx, ry, rz]"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if len(flat_path) < 6 or len(flat_path) % 6 != 0:
        logger.error("path must contain one or more flattened 6D TCP poses")
        node.destroy_node()
        rclpy.shutdown()
        return
    if path_mode not in ("off", "test", "contact"):
        logger.error(
            f"path_mode must be one of 'off', 'test', 'contact'; got "
            f"'{path_mode}'"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if path_mode in ("test", "contact") and len(path) < 2:
        logger.error(
            f"path_mode='{path_mode}' needs at least two waypoints; "
            f"got {len(path)}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if contact_retreat_mm <= 0.0 or contact_retreat_mm > 100.0:
        logger.error("contact_retreat_mm must be in the range (0, 100]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if contact_advance_mm <= 0.0 or contact_advance_mm > 200.0:
        logger.error("contact_advance_mm must be in the range (0, 200]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if path_min_z < 0.0:
        logger.error("path_min_z_mm must be non-negative")
        node.destroy_node()
        rclpy.shutdown()
        return
    low_path_points = [
        (index + 1, waypoint[2])
        for index, waypoint in enumerate(path)
        if waypoint[2] < path_min_z
    ]
    if path_mode in ("test", "contact") and low_path_points:
        logger.error(
            f"Path contains points below path_min_z_mm={path_min_z:.2f}: "
            f"{low_path_points}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    if path_speed <= 0.0 or path_speed > 50.0:
        logger.error("path_speed_mm_s must be in the range (0, 50]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if path_acceleration <= 0.0 or path_acceleration > 100.0:
        logger.error("path_acc_mm_s2 must be in the range (0, 100]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if speed <= 0.0 or speed > 10.0:
        logger.error("motion_speed_mm_s must be in the range (0, 10]")
        node.destroy_node()
        rclpy.shutdown()
        return
    if acceleration <= 0.0 or acceleration > 50.0:
        logger.error("motion_acc_mm_s2 must be in the range (0, 50]")
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
    translation_distance = math.dist(position_a[:3], position_b[:3])
    if translation_distance <= 0.0 or translation_distance > 300.0:
        logger.error(
            "The A-to-B translation distance must be in the range "
            f"(0, 300] mm; received {translation_distance:.2f} mm"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    if not armed:
        logger.warning(
            "Robot is NOT armed; no motion was sent. After checking Tool/TCP, "
            "poses A/B, workspace, and E-stop, rerun with "
            "--ros-args -p arm:=true"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    # DSR_ROBOT2 creates all of its service clients during import and calls them
    # immediately without wait_for_service().  On Fast DDS, a request made before
    # endpoint discovery can wait forever.  Discover every service used by this
    # program first so the wrapper cannot enter that race.
    required_services = (
        (GetCurrentTool, "tool/get_current_tool"),
        (GetCurrentTcp, "tcp/get_current_tcp"),
        (SetCurrentTcp, "tcp/set_current_tcp"),
        (GetCurrentPosx, "aux_control/get_current_posx"),
        (
            GetCurrentToolFlangePosx,
            "aux_control/get_current_tool_flange_posx",
        ),
        (GetToolForce, "aux_control/get_tool_force"),
        (MoveLine, "motion/move_line"),
        (MoveStop, "motion/move_stop"),
        (CheckMotion, "motion/check_motion"),
    )
    service_clients = {}
    for service_type, service_name in required_services:
        client = node.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            logger.error(
                f"Required service '/{ROBOT_ID}/{service_name}' is unavailable"
            )
            node.destroy_node()
            rclpy.shutdown()
            return
        service_clients[service_name] = client
    logger.info("All required robot services are ready")

    tcp_response = call_service(
        node,
        service_clients["tcp/get_current_tcp"],
        GetCurrentTcp.Request(),
        "tcp/get_current_tcp",
    )
    if not tcp_response.success:
        logger.error("Controller rejected the active TCP query; no motion sent")
        node.destroy_node()
        rclpy.shutdown()
        return

    if tcp_response.info == tcp_name:
        logger.info(f"TCP preset '{tcp_name}' is already active")
    else:
        set_tcp_request = SetCurrentTcp.Request()
        set_tcp_request.name = tcp_name
        set_tcp_response = call_service(
            node,
            service_clients["tcp/set_current_tcp"],
            set_tcp_request,
            "tcp/set_current_tcp",
        )
        if not set_tcp_response.success:
            logger.error(
                f"Controller rejected TCP preset '{tcp_name}'; no motion sent"
            )
            node.destroy_node()
            rclpy.shutdown()
            return
        tcp_response = call_service(
            node,
            service_clients["tcp/get_current_tcp"],
            GetCurrentTcp.Request(),
            "tcp/get_current_tcp",
        )
        if tcp_response.info and tcp_response.info != tcp_name:
            logger.error(
                f"Requested TCP '{tcp_name}' but controller reports "
                f"'{tcp_response.info}'; no motion sent"
            )
            node.destroy_node()
            rclpy.shutdown()
            return

    tool_response = call_service(
        node,
        service_clients["tool/get_current_tool"],
        GetCurrentTool.Request(),
        "tool/get_current_tool",
    )
    if not tool_response.success or not tcp_response.success:
        logger.error("Controller rejected the active Tool/TCP query")
        node.destroy_node()
        rclpy.shutdown()
        return
    active_tool = tool_response.info or "<no active Tool preset>"
    active_tcp = tcp_response.info or tcp_name
    if not tool_response.info:
        logger.warning(
            "No Tool payload preset is active. Compliance control is disabled; "
            "contact will be detected from force change relative to position A."
        )
    if not tcp_response.info:
        logger.warning(
            "The TCP query returned an empty name. Continuing only because "
            "set_current_tcp explicitly succeeded and the TCP offset is checked "
            "against the flange below."
        )

    tcp_pose_request = GetCurrentPosx.Request()
    tcp_pose_request.ref = 0
    tcp_pose_response = call_service(
        node,
        service_clients["aux_control/get_current_posx"],
        tcp_pose_request,
        "aux_control/get_current_posx",
    )
    flange_pose_request = GetCurrentToolFlangePosx.Request()
    flange_pose_request.ref = 0
    flange_pose_response = call_service(
        node,
        service_clients["aux_control/get_current_tool_flange_posx"],
        flange_pose_request,
        "aux_control/get_current_tool_flange_posx",
    )
    if (
        not tcp_pose_response.success
        or not flange_pose_response.success
        or not tcp_pose_response.task_pos_info
    ):
        logger.error("Could not verify the applied TCP offset; no motion sent")
        node.destroy_node()
        rclpy.shutdown()
        return
    tcp_pose = list(tcp_pose_response.task_pos_info[0].data)[:6]
    flange_pose = list(flange_pose_response.pos)[:6]
    tcp_translation_offset = math.dist(tcp_pose[:3], flange_pose[:3])
    tcp_orientation_offset = max(
        abs((float(tcp) - float(flange) + 180.0) % 360.0 - 180.0)
        for tcp, flange in zip(tcp_pose[3:], flange_pose[3:])
    )
    if tcp_translation_offset < 0.1 and tcp_orientation_offset < 0.1:
        logger.error(
            f"TCP preset '{tcp_name}' has not been applied: current TCP pose "
            "is identical to the flange pose. No motion sent."
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    logger.info(
        f"TCP offset verified against flange: translation="
        f"{tcp_translation_offset:.2f} mm, orientation component="
        f"{tcp_orientation_offset:.2f} deg"
    )

    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_QSTOP,
            DR_STATE_IDLE,
            ON,
            OFF,
            amovel,
            check_motion,
            get_current_posx,
            get_tool_force,
            movel,
            set_digital_output,
        )
        from DR_common2 import posx
    except ImportError as exc:
        logger.error(f"Failed to import Doosan API: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    stop_client = service_clients["motion/move_stop"]
    motion_active = False
    contact_detected = False
    motion_was_observed = False

    try:
        logger.info("Reading the active Tool/TCP and current robot state")
        # A/B were taught with the Tool/TCP presets activated above.
        start_pose, _ = get_current_posx(ref=DR_BASE)

        if path_mode == "test":
            logger.warning(
                "PATH TEST ARMED: contact detection is OFF; "
                f"executing {len(path)} absolute moveL waypoints in DR_BASE "
                f"at {path_speed:.1f} mm/s"
            )
            for index, waypoint in enumerate(path, start=1):
                logger.info(
                    f"moveL waypoint {index}/{len(path)}: {waypoint}"
                )
                result = movel(
                    posx(waypoint),
                    vel=[path_speed, 5.0],
                    acc=[path_acceleration, 10.0],
                    ref=DR_BASE,
                    mod=DR_MV_MOD_ABS,
                )
                if result != 0:
                    raise RuntimeError(
                        f"moveL waypoint {index}/{len(path)} was rejected"
                    )
                logger.info(f"Reached waypoint {index}/{len(path)}")
            logger.info("PATH TEST COMPLETE: all moveL waypoints reached")
            return

        if path_mode == "contact":
            logger.warning(
                f"PATH CONTACT ARMED: sweeping {len(path)} waypoints in "
                f"DR_BASE at {path_speed:.1f} mm/s; per-axis force-change "
                f"threshold={contact_force:.1f} N; the sweep stops "
                "immediately on the first contact"
            )
            path_start = path[0]
            start_position_error = math.dist(
                [float(value) for value in start_pose[:3]], path_start[:3]
            )
            start_orientation_error = max(
                abs((float(current) - target + 180.0) % 360.0 - 180.0)
                for current, target in zip(start_pose[3:], path_start[3:])
            )
            if move_to_path_start:
                logger.warning(
                    "move_to_path_start=true: moving to the first path "
                    "waypoint without contact monitoring. Verify this leg "
                    "is collision-free first, e.g. with path_mode:=test."
                )
                result = movel(
                    posx(path_start),
                    vel=[path_speed, 5.0],
                    acc=[path_acceleration, 10.0],
                    ref=DR_BASE,
                    mod=DR_MV_MOD_ABS,
                )
                if result != 0:
                    raise RuntimeError(
                        "Move to the first path waypoint was rejected"
                    )
            elif (
                start_position_error > position_tolerance
                or start_orientation_error > orientation_tolerance
            ):
                raise RuntimeError(
                    "Current TCP is not at the first path waypoint: "
                    f"position error={start_position_error:.2f} mm, "
                    f"orientation component error="
                    f"{start_orientation_error:.2f} deg. Move there safely "
                    "first or explicitly set move_to_path_start:=true."
                )

            pose_at_start, _ = get_current_posx(ref=DR_BASE)
            # Average the unloaded force at the path start. Contact is
            # detected from force change relative to this baseline, not
            # absolute force, so an uncompensated Tool/payload bias does not
            # trigger a false stop. The first waypoint must be free of
            # contact.
            force_samples = []
            for _ in range(10):
                sample = get_tool_force(ref=DR_BASE)
                if sample == -1 or len(sample) < 3:
                    raise RuntimeError(
                        "Failed to read the Tool force at the path start"
                    )
                force_samples.append([float(value) for value in sample[:3]])
                time.sleep(0.02)
            baseline_force_xyz = [
                sum(sample[axis] for sample in force_samples)
                / len(force_samples)
                for axis in range(3)
            ]
            logger.info(
                f"Path start reached={list(pose_at_start)}, "
                f"unloaded Base force baseline={baseline_force_xyz} N"
            )

            reached_index = 0
            for index in range(1, len(path)):
                segment_start = path[index - 1]
                target = path[index]
                segment_distance = math.dist(
                    segment_start[:3], target[:3]
                )
                logger.info(
                    f"Segment {index}/{len(path) - 1}: -> waypoint "
                    f"{index + 1}/{len(path)} {target}"
                )
                result = amovel(
                    posx(target),
                    vel=[path_speed, 5.0],
                    acc=[path_acceleration, 10.0],
                    ref=DR_BASE,
                    mod=DR_MV_MOD_ABS,
                )
                if result != 0:
                    raise RuntimeError(
                        f"Segment {index}/{len(path) - 1} amovel was "
                        "rejected"
                    )
                motion_active = True
                motion_was_observed = False
                deadline = (
                    time.monotonic()
                    + segment_distance / path_speed
                    + 5.0
                )
                while rclpy.ok() and time.monotonic() < deadline:
                    force = get_tool_force(ref=DR_BASE)
                    if force == -1 or len(force) < 3:
                        raise RuntimeError(
                            "Tool force monitoring failed during motion"
                        )
                    force_xyz = [float(value) for value in force[:3]]
                    force_delta = [
                        force_xyz[axis] - baseline_force_xyz[axis]
                        for axis in range(3)
                    ]
                    contact_axes = [
                        axis_name
                        for axis_name, delta in zip(
                            ("X", "Y", "Z"), force_delta
                        )
                        if abs(delta) >= contact_force
                    ]

                    if contact_axes:
                        contact_detected = True
                        request_stop(node, stop_client, DR_QSTOP)
                        logger.warning(
                            f"CONTACT on Base axis "
                            f"{','.join(contact_axes)} during segment "
                            f"{index}/{len(path) - 1}: force={force_xyz} N, "
                            f"delta={force_delta} N. Quick stop sent."
                        )
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
                        logger.error(
                            f"Segment {index}/{len(path) - 1} monitoring "
                            "timed out; stopping the robot"
                        )
                        request_stop(node, stop_client, DR_QSTOP)
                    if not wait_until_idle(
                        check_motion,
                        DR_STATE_IDLE,
                        timeout_sec=2.0,
                    ):
                        raise RuntimeError(
                            "The controller did not report an idle state "
                            "after stop"
                        )
                    motion_active = False

                if contact_detected:
                    break
                reached_index = index

            if not contact_detected:
                final_pose, _ = get_current_posx(ref=DR_BASE)
                logger.warning(
                    f"Completed all {len(path)} path waypoints without "
                    f"detecting contact; final TCP pose={list(final_pose)}"
                )
                return

            # segment_start/target still hold the in-progress segment from
            # the amovel call that was interrupted by contact.
            contact_pose_raw, _ = get_current_posx(ref=DR_BASE)
            contact_pose = [float(value) for value in contact_pose_raw[:6]]
            logger.info(
                "Stopped on contact after reaching waypoint "
                f"{reached_index + 1}/{len(path)}; "
                f"saved contact position={contact_pose}"
            )

            segment_length = math.dist(segment_start[:3], target[:3])
            if segment_length <= 0.0:
                raise RuntimeError(
                    "Cannot compute a travel direction: the contact "
                    "segment has zero length"
                )
            travel_unit = [
                (target[axis] - segment_start[axis]) / segment_length
                for axis in range(3)
            ]

            retreat_pose = list(contact_pose)
            for axis in range(3):
                retreat_pose[axis] -= travel_unit[axis] * contact_retreat_mm
            logger.info(
                f"Retreating {contact_retreat_mm:.1f} mm toward the "
                f"previous waypoint: {retreat_pose}"
            )
            result = movel(
                posx(retreat_pose),
                vel=[path_speed, 5.0],
                acc=[path_acceleration, 10.0],
                ref=DR_BASE,
                mod=DR_MV_MOD_ABS,
            )
            if result != 0:
                raise RuntimeError(
                    "Retreat move after contact was rejected"
                )

            logger.info("Opening the gripper")
            set_digital_output(1, OFF)
            set_digital_output(2, ON)
            time.sleep(0.5)

            advance_pose = list(retreat_pose)
            for axis in range(3):
                advance_pose[axis] += travel_unit[axis] * contact_advance_mm
            logger.info(
                f"Advancing {contact_advance_mm:.1f} mm along the "
                f"direction of travel: {advance_pose}"
            )
            result = movel(
                posx(advance_pose),
                vel=[path_speed, 5.0],
                acc=[path_acceleration, 10.0],
                ref=DR_BASE,
                mod=DR_MV_MOD_ABS,
            )
            if result != 0:
                raise RuntimeError(
                    "Advance move after contact was rejected"
                )

            final_pose, _ = get_current_posx(ref=DR_BASE)
            logger.info(
                "Post-contact recovery complete: retreated "
                f"{contact_retreat_mm:.1f} mm, opened gripper, advanced "
                f"{contact_advance_mm:.1f} mm; final TCP pose="
                f"{list(final_pose)}"
            )
            return

        initial_force = get_tool_force(ref=DR_BASE)
        if initial_force == -1 or len(initial_force) < 3:
            raise RuntimeError("Failed to read the initial Tool force")
        initial_force_xyz = [float(value) for value in initial_force[:3]]

        logger.info(
            f"Active Tool='{active_tool}', TCP='{active_tcp}', "
            f"current TCP={list(start_pose)}"
        )
        logger.warning(
            f"ARMED: TCP A={position_a} -> B={position_b}, "
            f"distance={translation_distance:.1f} mm, speed={speed:.1f} mm/s, "
            f"per-axis force-change threshold={contact_force:.1f} N"
        )

        logger.info(
            f"Initial Base force={initial_force_xyz} N. Absolute force is not "
            "used for contact because the active Tool payload has a static bias."
        )

        position_error = math.dist(
            [float(value) for value in start_pose[:3]],
            position_a[:3],
        )
        orientation_error = max(
            abs((float(current) - target + 180.0) % 360.0 - 180.0)
            for current, target in zip(start_pose[3:], position_a[3:])
        )

        if move_to_a:
            logger.warning(
                "move_to_a=true: moving to A without contact monitoring. "
                "The path to A must already be verified collision-free."
            )
            result = movel(
                posx(position_a),
                vel=[speed, 5.0],
                acc=[acceleration, 10.0],
                ref=DR_BASE,
                mod=DR_MV_MOD_ABS,
            )
            if result != 0:
                raise RuntimeError("Move to position A was rejected")
        elif (
            position_error > position_tolerance
            or orientation_error > orientation_tolerance
        ):
            raise RuntimeError(
                "Current TCP is not at position A: "
                f"position error={position_error:.2f} mm, "
                f"orientation component error={orientation_error:.2f} deg. "
                "Move to A safely first or explicitly set move_to_a:=true."
            )

        pose_at_a, _ = get_current_posx(ref=DR_BASE)
        # Average the unloaded force at A. Contact is detected from force change,
        # not absolute force, so an uncompensated Tool/payload bias does not
        # trigger a false stop. Position A must be free of contact.
        force_samples = []
        for _ in range(10):
            sample = get_tool_force(ref=DR_BASE)
            if sample == -1 or len(sample) < 3:
                raise RuntimeError("Failed to read the Tool force at position A")
            force_samples.append([float(value) for value in sample[:3]])
            time.sleep(0.02)
        force_at_a_xyz = [
            sum(sample[axis] for sample in force_samples) / len(force_samples)
            for axis in range(3)
        ]
        logger.info(
            f"TCP ready at A={list(pose_at_a)}, "
            f"unloaded Base force baseline={force_at_a_xyz} N"
        )

        result = amovel(
            posx(position_b),
            vel=[speed, 5.0],
            acc=[acceleration, 10.0],
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        if result != 0:
            raise RuntimeError("The asynchronous A-to-B command was rejected")
        motion_active = True

        deadline = time.monotonic() + translation_distance / speed + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            force = get_tool_force(ref=DR_BASE)
            if force == -1 or len(force) < 3:
                raise RuntimeError("Tool force monitoring failed during motion")
            force_xyz = [float(value) for value in force[:3]]
            force_delta = [
                force_xyz[axis] - force_at_a_xyz[axis]
                for axis in range(3)
            ]
            contact_axes = [
                axis_name
                for axis_name, delta in zip(("X", "Y", "Z"), force_delta)
                if abs(delta) >= contact_force
            ]

            if contact_axes:
                contact_detected = True
                request_stop(node, stop_client, DR_QSTOP)
                logger.warning(
                    f"CONTACT on Base axis {','.join(contact_axes)}: "
                    f"force={force_xyz} N, delta={force_delta} N. "
                    "Quick stop sent."
                )
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

        final_pose, _ = get_current_posx(ref=DR_BASE)
        if contact_detected:
            logger.info(
                f"Stopped before B; final TCP pose={list(final_pose)}"
            )
        else:
            final_position_error = math.dist(
                [float(value) for value in final_pose[:3]],
                position_b[:3],
            )
            if final_position_error <= position_tolerance:
                logger.info(
                    "Reached B without contact; "
                    f"final TCP pose={list(final_pose)}"
                )
            else:
                logger.error(
                    "Motion ended without contact but TCP did not reach B: "
                    f"position error={final_position_error:.2f} mm, "
                    f"final TCP pose={list(final_pose)}"
                )

    except KeyboardInterrupt:
        logger.warning("Interrupted by user; stopping active motion")
        if motion_active:
            request_stop(node, stop_client, DR_QSTOP)
            if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                motion_active = False
    except Exception as exc:
        logger.error(f"Contact search aborted: {exc}")
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
