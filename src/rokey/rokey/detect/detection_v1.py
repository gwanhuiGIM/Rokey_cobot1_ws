#!/usr/bin/env python3
"""Sweep a ㄹ-shaped (boustrophedon) path and act on the first contact.

This integrates two earlier examples:
  - move_until_contact.py : move A -> B and stop when a force threshold is hit,
                            using task compliance control + check_force_condition.
  - contact_stop.py       : sweep a multi-waypoint path.

detection_v1 merges them into a single behaviour:

  1. Sweep PATH_WAYPOINTS in order (taught as a ㄹ-shaped zig-zag across the
     shelf) using amovel, one segment at a time.
  2. Detection is ON for EVERY move, including the leg that reaches the first
     waypoint. During motion the robot is under task compliance control (so it
     yields on contact) and contact is detected with the Doosan-native
     check_force_condition on Base X/Y/Z: if the external force on any axis
     stays above contact_force_n for contact_debounce_count consecutive polls,
     that is a contact. The live tool force is logged periodically so you can
     see what values are actually being read.
  3. On the first contact the whole sweep stops immediately and the recovery
     sequence runs:
       a. save the contact position,
       b. retreat contact_retreat_mm (default 20 mm = 2 cm) back toward the
          previous waypoint,
       c. open the gripper,
       d. advance contact_advance_mm (default 50 mm = 5 cm) along the original
          direction of travel.
     Closing the gripper is intentionally left for a later step.

This is a contact-search example, not a safety-rated protective function.

Set detect_enabled:=false for a blind dry-run of the path (moveL, no force
monitoring, no compliance) to confirm it is collision-free before arming.
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
# The end effector is an OnRobot RG2, driven over a ROS service (not the
# control-box digital outputs), matching plate_move.py's GripperClient.
from onrobot_rg_msgs.srv import SetCommand


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_detection_v1"
TCP_NAME = "GripperDA_v1"
GRIPPER_SERVICE = "/onrobot/sendCommand"  # onrobot_rg_control, "o"=open "c"=close

# Taught cup-shelf corners in DR_BASE, ordered so the sweep traces a ㄹ-shaped
# (boustrophedon) path across the shelf. Index 0 is the first waypoint reached.
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
    node.declare_parameter("detect_enabled", True)
    # Isolated DO test: close (DO1=ON, DO2=OFF) 2 s, open (DO1=OFF, DO2=ON) 2 s,
    # then exit. Verifies the gripper wiring without any robot motion.
    node.declare_parameter("gripper_test", False)
    node.declare_parameter(
        "path",
        [value for waypoint in PATH_WAYPOINTS for value in waypoint],
    )
    node.declare_parameter("move_to_path_start", True)
    node.declare_parameter("path_min_z_mm", 30.0)
    node.declare_parameter("speed_mm_s", 20.0)
    node.declare_parameter("acc_mm_s2", 40.0)
    node.declare_parameter("contact_force_n", 8.0)
    node.declare_parameter("contact_debounce_count", 3)
    node.declare_parameter("poll_period_s", 0.02)
    # Suppress contact detection for this long at the start of every leg. Each
    # segment begins with an acceleration phase, and at a waypoint the previous
    # leg's deceleration plus the new leg's acceleration cause an inertial force
    # spike that would otherwise be a false contact. Cover that transient here.
    node.declare_parameter("settle_time_s", 0.8)
    node.declare_parameter("use_compliance", True)
    node.declare_parameter(
        "stiffness",
        # Soft in the horizontal plane (X, Y) so the tool yields on contact
        # from any sweep direction; stiff in Z to hold height.
        [500.0, 500.0, 3000.0, 200.0, 200.0, 200.0],
    )
    node.declare_parameter("contact_retreat_mm", 20.0)
    node.declare_parameter("contact_advance_mm", 50.0)
    node.declare_parameter("start_position_tolerance_mm", 2.0)
    node.declare_parameter("start_orientation_tolerance_deg", 2.0)

    armed = bool(node.get_parameter("arm").value)
    tcp_name = str(node.get_parameter("tcp_name").value)
    detect_enabled = bool(node.get_parameter("detect_enabled").value)
    gripper_test = bool(node.get_parameter("gripper_test").value)
    flat_path = [float(value) for value in node.get_parameter("path").value]
    path = [
        flat_path[index:index + 6]
        for index in range(0, len(flat_path), 6)
    ]
    move_to_path_start = bool(node.get_parameter("move_to_path_start").value)
    path_min_z = float(node.get_parameter("path_min_z_mm").value)
    speed = float(node.get_parameter("speed_mm_s").value)
    acceleration = float(node.get_parameter("acc_mm_s2").value)
    contact_force = float(node.get_parameter("contact_force_n").value)
    contact_debounce_count = int(
        node.get_parameter("contact_debounce_count").value
    )
    poll_period = float(node.get_parameter("poll_period_s").value)
    settle_time = float(node.get_parameter("settle_time_s").value)
    use_compliance = bool(node.get_parameter("use_compliance").value)
    stiffness = [float(v) for v in node.get_parameter("stiffness").value]
    contact_retreat_mm = float(node.get_parameter("contact_retreat_mm").value)
    contact_advance_mm = float(node.get_parameter("contact_advance_mm").value)
    position_tolerance = float(
        node.get_parameter("start_position_tolerance_mm").value
    )
    orientation_tolerance = float(
        node.get_parameter("start_orientation_tolerance_deg").value
    )

    # -- Parameter validation -------------------------------------------------
    if len(flat_path) < 12 or len(flat_path) % 6 != 0:
        logger.error("path must contain at least two flattened 6D TCP poses")
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
    if low_path_points:
        logger.error(
            f"Path contains points below path_min_z_mm={path_min_z:.2f}: "
            f"{low_path_points}"
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
    if contact_force < 3.0 or contact_force > 30.0:
        logger.error("contact_force_n must be in the range [3, 30]")
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

    if not armed:
        logger.warning(
            "Robot is NOT armed; no motion was sent. After checking Tool/TCP, "
            "waypoints, workspace, and E-stop, rerun with "
            "--ros-args -p arm:=true"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    # DSR_ROBOT2 creates all of its service clients during import and calls them
    # immediately without wait_for_service(). On Fast DDS, a request made before
    # endpoint discovery can wait forever. Discover every service used by this
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

    # The OnRobot RG2 gripper is a separate node (onrobot_rg_control on the real
    # robot, or gripper_virtual_node in the emulator) at an absolute service
    # path outside the /dsr01 namespace.
    gripper_client = node.create_client(SetCommand, GRIPPER_SERVICE)
    if not gripper_client.wait_for_service(timeout_sec=5.0):
        logger.error(
            f"Gripper service '{GRIPPER_SERVICE}' is unavailable. Start "
            "onrobot_rg_control (real) or gripper_virtual_node (emulator)."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    def send_gripper(command, description):
        """Call the OnRobot service; 'c'=close, 'o'=open."""
        logger.info(f"Gripper: {description} (command '{command}')")
        request = SetCommand.Request()
        request.command = command
        result = call_service(
            node, gripper_client, request, GRIPPER_SERVICE
        )
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            raise RuntimeError(f"Gripper command '{command}' failed: {message}")

    # -- Isolated gripper test (no robot motion, no TCP setup) ----------------
    if gripper_test:
        try:
            logger.warning("GRIPPER TEST: CLOSE command, hold 2 s")
            send_gripper("c", "close")
            time.sleep(2.0)
            logger.warning("GRIPPER TEST: OPEN command, hold 2 s")
            send_gripper("o", "open")
            time.sleep(2.0)
            logger.info(
                "GRIPPER TEST complete. If the gripper closed then opened, the "
                "service wiring is correct."
            )
        except Exception as exc:
            logger.error(f"GRIPPER TEST failed: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # -- Activate and verify the taught TCP -----------------------------------
    # The waypoints were taught with a specific TCP active; if the controller
    # has no active TCP the same numbers resolve against the flange and moves
    # get rejected ("Failed to reach pose ..."). Set and verify it here.
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
    if not tool_response.success:
        logger.error("Controller rejected the active Tool query")
        node.destroy_node()
        rclpy.shutdown()
        return
    active_tool = tool_response.info or "<no active Tool preset>"
    active_tcp = tcp_response.info or tcp_name
    if not tool_response.info:
        logger.warning(
            "No Tool payload preset is active. Without a correct payload the "
            "external-force estimate is biased, which makes contact detection "
            "unreliable. Activate the Tool preset used when teaching."
        )

    # Confirm the TCP offset actually took effect by comparing the TCP pose to
    # the raw flange pose. Identical poses mean the TCP was not applied.
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
        f"Active Tool='{active_tool}', TCP='{active_tcp}'; TCP offset vs "
        f"flange: translation={tcp_translation_offset:.2f} mm, orientation "
        f"component={tcp_orientation_offset:.2f} deg"
    )

    try:
        from DSR_ROBOT2 import (
            DR_AXIS_X,
            DR_AXIS_Y,
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_QSTOP,
            DR_STATE_IDLE,
            ON,
            OFF,
            amovel,
            check_force_condition,
            check_motion,
            get_current_posx,
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

    stop_client = service_clients["motion/move_stop"]
    force_axes = (
        (DR_AXIS_X, "X"),
        (DR_AXIS_Y, "Y"),
        (DR_AXIS_Z, "Z"),
    )
    # Mutable flags shared with the monitored_move() closure below.
    state = {"motion_active": False, "compliance_active": False}

    def read_tool_force():
        """Return [Fx, Fy, Fz] in DR_BASE, or None if unavailable."""
        force = get_tool_force(ref=DR_BASE)
        if force == -1 or len(force) < 3:
            return None
        return [float(value) for value in force[:3]]

    def open_gripper():
        send_gripper("o", "open")
        time.sleep(0.5)

    def close_gripper():
        send_gripper("c", "close")
        time.sleep(0.5)

    def monitored_move(target, from_pose, label):
        """amovel to target while watching for contact.

        Returns "contact" or "reached". Raises on rejection/timeout. Leaves the
        robot stopped and idle in every non-exception path.
        """
        segment_distance = math.dist(from_pose[:3], target[:3])
        result = amovel(
            posx(target),
            vel=[speed, 5.0],
            acc=[acceleration, 10.0],
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        if result != 0:
            raise RuntimeError(f"{label}: amovel was rejected")
        state["motion_active"] = True
        motion_was_observed = False
        consecutive_hits = 0
        peak_force = [0.0, 0.0, 0.0]
        last_diag = 0.0
        outcome = None
        move_start = time.monotonic()
        deadline = move_start + segment_distance / speed + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            # The start of every leg has an acceleration / direction-change
            # inertial transient that would false-trigger. Suppress detection
            # until the tool has settled into steady-state motion.
            in_settle = (time.monotonic() - move_start) < settle_time

            # Doosan-native external-force check. The ROS wrapper returns 0
            # when the condition (|force| >= min) is true, -1 when false.
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

            # Periodic diagnostics so a "no detection" run is explainable:
            # show the live force and the running peak per axis.
            now = time.monotonic()
            if now - last_diag >= 0.5:
                last_diag = now
                live = read_tool_force()
                if live is not None:
                    peak_force = [
                        max(peak_force[i], abs(live[i])) for i in range(3)
                    ]
                    phase = "settling" if in_settle else "monitoring"
                    logger.info(
                        f"[{label}] ({phase}) live |F|="
                        f"{[round(v, 2) for v in live]} N, "
                        f"peak={[round(v, 2) for v in peak_force]} N, "
                        f"threshold={contact_force:.1f} N, hits={hit_axes}"
                    )

            if consecutive_hits >= contact_debounce_count:
                request_stop(node, stop_client, DR_QSTOP)
                live = read_tool_force()
                logger.warning(
                    f"[{label}] CONTACT on Base axis {','.join(hit_axes)} "
                    f"(force={live} N, {consecutive_hits} consecutive hits). "
                    "Quick stop sent."
                )
                outcome = "contact"
                break

            motion_state = check_motion()
            if motion_state != DR_STATE_IDLE:
                motion_was_observed = True
            elif motion_was_observed:
                outcome = "reached"
                break
            time.sleep(poll_period)

        if outcome is None:
            logger.error(f"[{label}] monitoring timed out; stopping the robot")
            request_stop(node, stop_client, DR_QSTOP)

        if not wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
            raise RuntimeError(
                f"{label}: controller did not report idle after stop"
            )
        state["motion_active"] = False

        if outcome is None:
            raise RuntimeError(f"{label}: monitoring timed out")
        return outcome

    try:
        start_pose_raw, _ = get_current_posx(ref=DR_BASE)
        start_pose = [float(value) for value in start_pose_raw[:6]]

        # -- Blind dry-run: verify the path is collision-free, detection OFF ---
        if not detect_enabled:
            logger.warning(
                "DRY-RUN ARMED: detection is OFF; executing "
                f"{len(path)} absolute moveL waypoints in DR_BASE at "
                f"{speed:.1f} mm/s"
            )
            for index, waypoint in enumerate(path, start=1):
                logger.info(f"moveL waypoint {index}/{len(path)}: {waypoint}")
                result = movel(
                    posx(waypoint),
                    vel=[speed, 5.0],
                    acc=[acceleration, 10.0],
                    ref=DR_BASE,
                    mod=DR_MV_MOD_ABS,
                )
                if result != 0:
                    raise RuntimeError(
                        f"moveL waypoint {index}/{len(path)} was rejected"
                    )
                logger.info(f"Reached waypoint {index}/{len(path)}")
            logger.info("DRY-RUN COMPLETE: all moveL waypoints reached")
            return

        # -- Enable task compliance control for the whole traverse ------------
        # Compliance makes the tool yield on contact, which keeps the external
        # force bounded and makes check_force_condition fire reliably. It is
        # released before the precise recovery moves.
        if use_compliance:
            set_ref_coord(DR_BASE)
            if task_compliance_ctrl(stx=stiffness, time=0.5) != 0:
                raise RuntimeError("Failed to enable task compliance control")
            state["compliance_active"] = True
            time.sleep(0.6)  # let the controller finish the mode transition
            logger.info(f"Task compliance control ON, stiffness={stiffness}")
        else:
            logger.warning(
                "use_compliance=false: the robot stays stiff. Detection may be "
                "less reliable on light objects."
            )

        logger.warning(
            f"DETECTION ARMED: sweeping {len(path)} waypoints (ㄹ path) at "
            f"{speed:.1f} mm/s; per-axis force threshold={contact_force:.1f} N, "
            f"debounce={contact_debounce_count}, settle={settle_time:.2f} s per "
            "leg. Detection is ON for every move (including the leg to the first "
            "waypoint); the sweep stops immediately on the first contact."
        )

        contact_detected = False
        contact_from = None
        contact_target = None

        # -- Move to the first waypoint (monitored) --------------------------
        path_start = path[0]
        start_position_error = math.dist(start_pose[:3], path_start[:3])
        start_orientation_error = max(
            abs((current - target + 180.0) % 360.0 - 180.0)
            for current, target in zip(start_pose[3:], path_start[3:])
        )
        already_at_start = (
            start_position_error <= position_tolerance
            and start_orientation_error <= orientation_tolerance
        )
        if not already_at_start:
            if not move_to_path_start:
                raise RuntimeError(
                    "Current TCP is not at the first waypoint: "
                    f"position error={start_position_error:.2f} mm, "
                    f"orientation error={start_orientation_error:.2f} deg. "
                    "Move there safely first or set move_to_path_start:=true."
                )
            label = "leg 0 -> waypoint 1"
            logger.info(f"{label}: {path_start}")
            outcome = monitored_move(path_start, start_pose, label)
            if outcome == "contact":
                contact_detected = True
                contact_from = start_pose
                contact_target = path_start

        # -- At the start point: close the gripper if it is open -------------
        # Done here (after arriving) rather than before moving, so the gripper
        # only changes state once the robot is at the sweep's start point.
        if not contact_detected:
            logger.info("Reached the start point; closing the gripper")
            close_gripper()

        # -- Sweep the remaining waypoints; stop at the first contact --------
        if not contact_detected:
            for index in range(1, len(path)):
                from_pose = path[index - 1]
                target = path[index]
                label = f"segment -> waypoint {index + 1}/{len(path)}"
                logger.info(f"{label}: {target}")
                outcome = monitored_move(target, from_pose, label)
                if outcome == "contact":
                    contact_detected = True
                    contact_from = from_pose
                    contact_target = target
                    break

        if not contact_detected:
            final_pose, _ = get_current_posx(ref=DR_BASE)
            logger.warning(
                f"Completed the whole path without detecting contact; "
                f"final TCP pose={list(final_pose)}"
            )
            return

        # -- Recovery sequence after contact ---------------------------------
        # 1. Save the contact position.
        contact_pose_raw, _ = get_current_posx(ref=DR_BASE)
        contact_pose = [float(value) for value in contact_pose_raw[:6]]
        logger.info(f"Saved contact position={contact_pose}")

        # Release compliance so the recovery moves are position-accurate.
        if state["compliance_active"]:
            try:
                release_compliance_ctrl()
            finally:
                state["compliance_active"] = False
            time.sleep(0.3)

        # Direction of travel for the interrupted leg, used to retreat backward
        # and later advance forward.
        segment_length = math.dist(contact_from[:3], contact_target[:3])
        if segment_length <= 0.0:
            raise RuntimeError(
                "Cannot compute a travel direction: the contact segment has "
                "zero length"
            )
        travel_unit = [
            (contact_target[axis] - contact_from[axis]) / segment_length
            for axis in range(3)
        ]

        # 2. Retreat toward the previous waypoint (against travel direction).
        retreat_pose = list(contact_pose)
        for axis in range(3):
            retreat_pose[axis] -= travel_unit[axis] * contact_retreat_mm
        logger.info(
            f"Retreating {contact_retreat_mm:.1f} mm toward the previous "
            f"waypoint: {retreat_pose}"
        )
        if movel(
            posx(retreat_pose),
            vel=[speed, 5.0],
            acc=[acceleration, 10.0],
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        ) != 0:
            raise RuntimeError("Retreat move after contact was rejected")

        # 3. Open the gripper (closing is left for a later step).
        logger.info("Opening the gripper")
        open_gripper()

        # 4. Advance along the original direction of travel.
        advance_pose = list(retreat_pose)
        for axis in range(3):
            advance_pose[axis] += travel_unit[axis] * contact_advance_mm
        logger.info(
            f"Advancing {contact_advance_mm:.1f} mm along the direction of "
            f"travel: {advance_pose}"
        )
        if movel(
            posx(advance_pose),
            vel=[speed, 5.0],
            acc=[acceleration, 10.0],
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        ) != 0:
            raise RuntimeError("Advance move after contact was rejected")

        final_pose, _ = get_current_posx(ref=DR_BASE)
        logger.info(
            "Post-contact recovery complete: retreated "
            f"{contact_retreat_mm:.1f} mm, opened gripper, advanced "
            f"{contact_advance_mm:.1f} mm; final TCP pose={list(final_pose)}"
        )

    except KeyboardInterrupt:
        logger.warning("Interrupted by user; stopping active motion")
        if state["motion_active"]:
            request_stop(node, stop_client, DR_QSTOP)
            if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                state["motion_active"] = False
    except Exception as exc:
        logger.error(f"Detection sweep aborted: {exc}")
        if state["motion_active"]:
            try:
                request_stop(node, stop_client, DR_QSTOP)
                if wait_until_idle(
                    check_motion, DR_STATE_IDLE, timeout_sec=2.0
                ):
                    state["motion_active"] = False
            except Exception as stop_exc:
                logger.error(f"Automatic stop request failed: {stop_exc}")
    finally:
        if state["compliance_active"]:
            if state["motion_active"]:
                logger.error(
                    "Motion state is not confirmed idle; compliance control "
                    "was left enabled. Use the teach pendant to recover."
                )
            else:
                try:
                    release_compliance_ctrl()
                except Exception as exc:
                    logger.error(
                        f"Failed to release compliance control: {exc}"
                    )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
