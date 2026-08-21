#!/usr/bin/env python3
# =============================================================
# probe_grip_v4.py — 단일 X접촉 + 반경 오프셋으로 물체 중심좌표 산출 (좌표 탐지 전용)
# -------------------------------------------------------------
# 실행: ros2 run cup_detect probe_grip_v4 --ros-args -p arm:=true
# 노드: m0609_probe_grip_v4 | 좌표계: DR_BASE | 포즈 6D=[x,y,z(mm),A,B,C(ZYZ deg)]
#
# v3와의 차이: v3는 X축 양쪽면(+X/-X)을 두 번 찍어 중앙을 평균으로 구했다.
# v4는 각 축 1회씩만 접촉하고, 알려진/가정된 object_radius_mm으로 중심을
# 추정한다 — 접촉 횟수는 늘지만(X 1회 + Y 1회) 매 축마다 반대편 재접촉이
# 필요 없다. 대신 반지름 가정이 실제 물체와 다르면 그 오차만큼 중심이 어긋난다.
#
# 흐름: movej 안전자세 → (옵션) 그리퍼 닫기 → 감지 준비위치 →
#   +X 직진 접촉 탐지(force[0]=Fx) → x_center = contact_x + object_radius_mm →
#   +Z 회피 → x_center, +Y 오프셋 위치로 이동 → -Z 감지 높이 복귀 →
#   -Y 직진 접촉 탐지(force[1]=Fy) → y_center = contact_y - object_radius_mm →
#   +Z 회피 상승(안전 마무리) → 계산된 중심 XY로 이동(안전 높이 유지, 하강 없음 —
#   육안 검증용, 물체는 사용자가 직접 치움) → (x_center, y_center, dz) 반환.
#
# 주의(실기 안전): arm:=true 없이는 모션 없음. detect_enabled:=false 면
#   힘 탐지 없이 sim_probe_mm 고정 전진(드라이런).
# 주의(미검증): get_tool_force(ref=DR_BASE)는 base 고정축 기준이라 -Y 접근이어도
#   Z가 아니라 force[1](Fy)을 읽어야 한다(DSR_ROBOT2.py 소스로 확인, 추측 아님).
#   다만 -Y 접촉 시 Fy 부호(contact_force_dir_y)는 X축과 달리 실기 검증이 안 된
#   값이다 — # UNVERIFIED 표시된 기본값은 첫 실기 테스트에서 반드시 확인할 것.
# =============================================================
"""단일 X/+Y 접촉 + 반경 오프셋으로 물체 중심좌표를 구하는 탐지 전용 노드.

probe_grip_v3의 실기 기계장치(컴플라이언스 접촉 탐지, TCP, 서비스 discovery)를
재사용하되, X축은 1회 접촉 후 반경 오프셋으로 중심을 추정하고, 같은 방식으로
Y축도 1회 접촉해 중심을 구한다. 파지/이송은 하지 않는다.
"""

import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
import DR_init
from dsr_msgs2.srv import (
    CheckMotion,
    GetCurrentPosx,
    GetCurrentTcp,
    GetCurrentTool,
    GetToolForce,
    MoveLine,
    MoveStop,
    SetCurrentTcp,
    SetCurrentTool,
)
from onrobot_rg_msgs.srv import SetCommand


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_probe_grip_v4"
TOOL_NAME = "Tool Weight123"
TCP_NAME = "GripperDA_v1"
GRIPPER_SERVICE = "/onrobot/sendCommand"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

AXIS_X = 0
AXIS_Y = 1


@dataclass
class ProbeGripContext:
    """run_once()에 넘기는 실행 컨텍스트(모듈 통합용).

    오케스트레이터는 자기 노드/파라미터를 채워 run_once(ctx)를 부르면 된다.
    CLI(main)는 declare_parameter 값으로 이 컨텍스트를 채운다.
    """
    node: object
    detection_ready_xyz: list
    probe_orientation: list
    approach_joint: list = field(default_factory=lambda: [0.0, 0.0, 90.0, 90.0, -90.0, 0.0])
    speed_mm_s: float = 40.0
    acc_mm_s2: float = 40.0
    joint_speed_deg_s: float = 30.0
    joint_acc_deg_s2: float = 30.0
    detect_enabled: bool = True
    sim_probe_mm: float = 150.0
    probe_depth_mm: float = 200.0
    contact_force_n_x: float = 8
    contact_force_dir_x: float = -1.0  # 실기검증됨(v3): +X 압박 시 Fx는 음수
    contact_force_n_y: float = 3.7  # UNVERIFIED: Y축 실측 없음, X와 동일값으로 임시 설정
    contact_force_dir_y: float = -1.0  # UNVERIFIED: -Y 접촉 시 Fy 부호 실기 미검증
    poll_period_s: float = 0.02
    settle_time_s: float = 2.0
    stiffness: list = field(
        default_factory=lambda: [200.0, 200.0, 3000.0, 200.0, 200.0, 200.0]
    )  # X,Y를 약하게: 두 축 모두 접촉을 부드럽게 감지
    retreat_z_mm: float = 150.0  # 접촉 후 다음 이동 전 회피 상승량
    object_radius_mm: float = 55.0  # 컵 실측 반지름 45mm + 그리퍼 바깥 접촉분 여유 5mm
    y_probe_start_offset_mm: float = 150.0  # Y탐지 시작위치를 dy에서 +Y로 띄우는 여유(물체보다 커야 함)
    close_gripper_on_start: bool = True
    gripper_channel: str = "service"
    open_do: list = field(default_factory=lambda: [1, 1])
    close_do: list = field(default_factory=lambda: [1, 0])
    tool_name: str = TOOL_NAME
    tcp_name: str = TCP_NAME


def wait_until_idle(check_motion, idle_state, timeout_sec):
    """컨트롤러가 경로 모션 idle을 보고할 때 True."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if check_motion() == idle_state:
            return True
        time.sleep(0.02)
    return False


def call_service(node, client, request, service_name, timeout_sec=3.0):
    """유한 타임아웃으로 ROS 서비스를 호출."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done() or future.result() is None:
        raise RuntimeError(f"{service_name} service timed out")
    return future.result()


def run_once(ctx: ProbeGripContext) -> dict:
    """물체의 X 근접면 1회 + Y 근접면 1회를 접촉 탐지해 반경 오프셋으로 중심좌표를 구한다.

    반환 키: found, reason, contact_pose_x, contact_pose_y,
             object_center_xyz([x,y,z]), object_diameter_mm.
    예외는 호출자에게 전파(모션 중이면 이미 정지 시도됨).
    """
    node = ctx.node
    logger = node.get_logger()
    result = {
        "found": False, "reason": "",
        "contact_pose_x": None, "contact_pose_y": None,
        "object_center_xyz": None, "object_diameter_mm": None,
    }

    # -- 서비스 사전 discovery (DSR wrapper의 Fast DDS race 회피) --------------
    required_services = (
        (GetCurrentTool, "tool/get_current_tool"),
        (GetCurrentTcp, "tcp/get_current_tcp"),
        (SetCurrentTcp, "tcp/set_current_tcp"),
        (SetCurrentTool, "tool/set_current_tool"),
        (GetCurrentPosx, "aux_control/get_current_posx"),
        (GetToolForce, "aux_control/get_tool_force"),
        (MoveLine, "motion/move_line"),
        (MoveStop, "motion/move_stop"),
        (CheckMotion, "motion/check_motion"),
    )
    service_clients = {}
    for service_type, service_name in required_services:
        client = node.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"Required service '/{ROBOT_ID}/{service_name}' unavailable")
        service_clients[service_name] = client
    logger.info("All required robot services are ready")

    gripper_client = None
    if ctx.gripper_channel == "service":
        gripper_client = node.create_client(SetCommand, GRIPPER_SERVICE)
        if not gripper_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f"Gripper service '{GRIPPER_SERVICE}' unavailable. Start "
                "onrobot_rg_control (real) or gripper emulator."
            )

    # -- TCP 활성화·검증 -----------------------------------------------------
    tcp_response = call_service(
        node, service_clients["tcp/get_current_tcp"],
        GetCurrentTcp.Request(), "tcp/get_current_tcp",
    )
    if not tcp_response.success:
        raise RuntimeError("Controller rejected the active TCP query")
    if tcp_response.info != ctx.tcp_name:
        set_req = SetCurrentTcp.Request()
        set_req.name = ctx.tcp_name
        set_res = call_service(
            node, service_clients["tcp/set_current_tcp"], set_req,
            "tcp/set_current_tcp",
        )
        if not set_res.success:
            raise RuntimeError(f"Controller rejected TCP preset '{ctx.tcp_name}'")
    logger.info(f"TCP preset '{ctx.tcp_name}' active")

    # -- Tool 무게 프리셋 활성화·검증 -----------------------------------------
    tool_response = call_service(
        node, service_clients["tool/get_current_tool"],
        GetCurrentTool.Request(), "tool/get_current_tool",
    )
    if not tool_response.success:
        raise RuntimeError("Controller rejected the active Tool query")
    if tool_response.info != ctx.tool_name:
        set_req = SetCurrentTool.Request()
        set_req.name = ctx.tool_name
        set_res = call_service(
            node, service_clients["tool/set_current_tool"], set_req,
            "tool/set_current_tool",
        )
        if not set_res.success:
            raise RuntimeError(f"Controller rejected Tool preset '{ctx.tool_name}'")
    logger.info(f"Tool preset '{ctx.tool_name}' active")

    from DSR_ROBOT2 import (
        DR_BASE,
        DR_MV_MOD_ABS,
        DR_QSTOP,
        DR_STATE_IDLE,
        amovel,
        check_motion,
        get_current_posx,
        get_tool_force,
        movej,
        movel,
        release_compliance_ctrl,
        set_digital_output,
        set_ref_coord,
        task_compliance_ctrl,
    )
    from DR_common2 import posj, posx

    stop_client = service_clients["motion/move_stop"]
    fsm = {"motion_active": False, "compliance_active": False}

    speed, acceleration = ctx.speed_mm_s, ctx.acc_mm_s2

    def request_stop(stop_mode):
        if not stop_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("move_stop service unavailable")
        req = MoveStop.Request(); req.stop_mode = stop_mode
        res = call_service(node, stop_client, req, "motion/move_stop", timeout_sec=2.0)
        if not res.success:
            raise RuntimeError("MoveStop rejected")

    def move_line(pose, label):
        logger.info(f"[move_line] {label}: {pose}")
        if movel(posx(pose), vel=[speed, 50.0], acc=[acceleration, 10.0],
                 ref=DR_BASE, mod=DR_MV_MOD_ABS) != 0:
            raise RuntimeError(f"{label}: movel rejected")

    def move_joint(joint_deg, label):
        logger.info(f"[move_joint] {label}: {joint_deg}")
        if movej(posj(joint_deg), vel=ctx.joint_speed_deg_s, acc=ctx.joint_acc_deg_s2) != 0:
            raise RuntimeError(f"{label}: movej rejected")

    def send_gripper(command, description):
        logger.info(f"Gripper: {description} (command '{command}')")
        req = SetCommand.Request(); req.command = command
        res = call_service(node, gripper_client, req, GRIPPER_SERVICE)
        if res is None or not res.success:
            msg = res.message if res is not None else "no response"
            raise RuntimeError(f"Gripper command '{command}' failed: {msg}")

    def actuate_gripper(action):
        """open/close를 선택 채널로 구동. 두 채널을 한 실행에서 섞지 말 것."""
        if ctx.gripper_channel == "digital_output":
            do1, do2 = ctx.open_do if action == "open" else ctx.close_do
            logger.info(f"Gripper[DO]: {action} (DO1={do1}, DO2={do2})")
            set_digital_output(1, do1)
            set_digital_output(2, do2)
        else:
            send_gripper("o" if action == "open" else "c", action)

    def read_tool_force(axis):
        """DR_BASE 고정축 기준 외력 벡터의 axis 성분(0=Fx,1=Fy,2=Fz)을 읽는다.

        get_tool_force(ref=DR_BASE)는 툴 방향과 무관하게 base 고정좌표를 쓴다
        (DSR_ROBOT2.py 소스로 확인) — 그래서 -Y 프로브도 force[2]가 아니라
        force[1]을 읽어야 한다.
        """
        force = get_tool_force(ref=DR_BASE)
        if force == -1 or len(force) <= axis:
            return None
        return float(force[axis])

    def probe_axis(from_pose, axis, direction, depth_mm, label, force_dir, threshold):
        """from_pose에서 Base axis(0=X,1=Y) 방향(direction=+1.0/-1.0)으로 depth_mm 전진.

        접촉 시 급정지. force_dir: 이 축에서 접근 방향과 같은 부호로 눌렸을 때
        힘 신호가 읽히는 부호(축마다 실기로 검증해야 함).
        반환: "contact" | "reached_depth". 컴플라이언스는 호출 전 진입돼 있어야 함.
        """
        target = list(from_pose)
        target[axis] += direction * depth_mm
        if amovel(posx(target), vel=[speed, 50.0], acc=[acceleration, 10.0],
                  ref=DR_BASE, mod=DR_MV_MOD_ABS) != 0:
            raise RuntimeError(f"{label}: amovel(probe) rejected")
        fsm["motion_active"] = True

        effective_dir = force_dir * direction
        motion_seen, peak_f, last_diag, outcome = False, 0.0, 0.0, None
        move_start = time.monotonic()
        deadline = move_start + depth_mm / speed + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            in_settle = (time.monotonic() - move_start) < ctx.settle_time_s
            f = read_tool_force(axis)
            hit = (f is not None and effective_dir * f >= threshold)
            now = time.monotonic()
            if now - last_diag >= 0.1:
                last_diag = now
                if f is not None:
                    peak_f = max(peak_f, abs(f))
                    phase = "settling" if in_settle else "monitoring"
                    logger.info(f"[{label}] ({phase}) F[{axis}]={f:.2f} N, peak={peak_f:.2f}, "
                                f"dir={effective_dir:+.0f} thr={threshold:.1f}")
            if hit:
                request_stop(DR_QSTOP)
                logger.warning(f"[{label}] CONTACT (F[{axis}]={f:.2f} N, "
                               f"dir={effective_dir:+.0f}). Stop.")
                outcome = "contact"
                break
            motion_state = check_motion()
            if motion_state != DR_STATE_IDLE:
                motion_seen = True
            elif motion_seen:
                outcome = "reached_depth"
                break
            time.sleep(ctx.poll_period_s)

        if outcome is None:
            logger.error(f"[{label}] monitoring timed out; stopping")
            request_stop(DR_QSTOP)
        if not wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
            raise RuntimeError(f"{label}: controller not idle after stop")
        fsm["motion_active"] = False
        if outcome is None:
            raise RuntimeError(f"{label}: monitoring timed out")
        return outcome

    def enable_compliance():
        set_ref_coord(DR_BASE)
        if task_compliance_ctrl(stx=ctx.stiffness, time=0.5) != 0:
            raise RuntimeError("task_compliance_ctrl failed")
        fsm["compliance_active"] = True
        time.sleep(0.6)

    def disable_compliance():
        release_compliance_ctrl()
        fsm["compliance_active"] = False
        time.sleep(0.3)

    def probe_axis_side(from_pose, axis, direction, label, force_dir, threshold):
        """한 축 접촉 탐지 1회(컴플라이언스 on/off 포함). (outcome, contact_pose)."""
        if ctx.detect_enabled:
            enable_compliance()
            try:
                outcome = probe_axis(
                    from_pose, axis, direction, ctx.probe_depth_mm, label,
                    force_dir, threshold)
            finally:
                disable_compliance()
        else:
            sim_pose = list(from_pose)
            sim_pose[axis] += direction * ctx.sim_probe_mm
            move_line(sim_pose, f"{label} (sim fixed advance)")
            outcome = "contact"
        raw, _ = get_current_posx(ref=DR_BASE)
        return outcome, [float(v) for v in raw[:6]]

    # ===================== 실행 시퀀스 =====================
    try:
        dx, dy, dz = ctx.detection_ready_xyz
        orient = list(ctx.probe_orientation)
        ready_pose = [dx, dy, dz] + orient

        # 1. 안전 관절자세.
        move_joint(ctx.approach_joint, "movej safe posture")

        # 2. 시작 시 그리퍼 닫기(닫힌 강체로 탐지 오차 축소).
        if ctx.close_gripper_on_start:
            actuate_gripper("close")
            time.sleep(0.5)

        # 3. 감지 준비위치.
        move_line(ready_pose, "move to detection-ready")
        time.sleep(1)

        # 4. X축: +X 직진 접촉 탐지(근접 X 에지, force[0]=Fx).
        outcome_x, contact_x = probe_axis_side(
            ready_pose, AXIS_X, +1.0, "probe +X (near edge)",
            force_dir=ctx.contact_force_dir_x, threshold=ctx.contact_force_n_x)
        if outcome_x != "contact":
            result["reason"] = "no X contact (probe depth exhausted)"
            logger.warning(result["reason"])
            return result
        result["contact_pose_x"] = contact_x
        logger.info(f"X contact at pose={contact_x}")

        # UNVERIFIED: 반대편 재접촉 대신 가정 반지름으로 중심 추정.
        # 실제 반지름이 object_radius_mm과 다르면 그 오차만큼 x_center가 어긋난다.
        x_center = contact_x[0] + ctx.object_radius_mm
        logger.info(f"x_center = {contact_x[0]:.2f} + radius {ctx.object_radius_mm:.1f} "
                    f"= {x_center:.2f}")

        # 5. +Z 회피 → x_center 고정, dy보다 +Y로 띄운 Y탐지 시작위치로 이동 → -Z 복귀.
        retreat_pose = list(contact_x); retreat_pose[2] += ctx.retreat_z_mm
        move_line(retreat_pose, "retreat +Z after X contact")
        y_ready_pose = [x_center, dy + ctx.y_probe_start_offset_mm,
                        retreat_pose[2]] + orient
        move_line(y_ready_pose, "move to Y-probe ready (x fixed, +Y offset)")

        y_descend_pose = list(y_ready_pose); y_descend_pose[2] = dz
        move_line(y_descend_pose, "descend to detection height for Y probe")
        time.sleep(1)

        # 6. Y축: -Y 직진 접촉 탐지(근접 Y 에지, force[1]=Fy).
        outcome_y, contact_y = probe_axis_side(
            y_descend_pose, AXIS_Y, -1.0, "probe -Y (near edge)",
            force_dir=ctx.contact_force_dir_y, threshold=ctx.contact_force_n_y)
        if outcome_y != "contact":
            result["reason"] = "no Y contact (probe depth exhausted)"
            logger.warning(result["reason"])
            return result
        result["contact_pose_y"] = contact_y
        logger.info(f"Y contact at pose={contact_y}")

        y_center = contact_y[1] - ctx.object_radius_mm
        logger.info(f"y_center = {contact_y[1]:.2f} - radius {ctx.object_radius_mm:.1f} "
                    f"= {y_center:.2f}")

        # 7. 마무리 회피 상승(물체에 닿은 채로 끝내지 않도록).
        final_retreat = list(contact_y); final_retreat[2] += ctx.retreat_z_mm
        move_line(final_retreat, "retreat +Z after Y contact")

        # 8. 검증용: 계산된 중심 XY로 안전 높이 유지한 채 이동(하강 없음).
        #    물체는 여전히 그 자리에 있으므로 육안으로 정렬 확인 후 사용자가 직접 치운다.
        verify_pose = [x_center, y_center, final_retreat[2]] + orient
        move_line(verify_pose, "move to detected object center (verify, no descent)")

        result["found"] = True
        result["reason"] = "ok"
        result["object_center_xyz"] = [x_center, y_center, dz]
        result["object_diameter_mm"] = ctx.object_radius_mm * 2.0
        logger.info(f"DONE object_center_xyz={result['object_center_xyz']} "
                    f"(assumed diameter={result['object_diameter_mm']:.1f}mm)")
        return result

    except BaseException as exc:  # 정지 후 재전파(실기 안전)
        logger.error(f"probe_grip aborted: {exc}")
        if fsm["motion_active"]:
            try:
                request_stop(DR_QSTOP)
                wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0)
                fsm["motion_active"] = False
            except Exception as stop_exc:
                logger.error(f"stop failed: {stop_exc}")
        raise
    finally:
        if fsm["compliance_active"] and not fsm["motion_active"]:
            try:
                release_compliance_ctrl()
            except Exception as exc:
                logger.error(f"release_compliance_ctrl failed: {exc}")


def _ctx_from_params(node) -> Optional[ProbeGripContext]:
    """CLI: declare_parameter 값으로 컨텍스트를 채운다. 미장전/검증실패면 None."""
    logger = node.get_logger()
    p = node.declare_parameter
    p("arm", False)
    p("tool_name", TOOL_NAME)
    p("tcp_name", TCP_NAME)
    p("detect_enabled", True)
    p("speed_mm_s", 40.0)
    p("acc_mm_s2", 40.0)
    p("approach_joint", [0.0, 0.0, 90.0, 90.0, -90.0, 0.0])
    p("joint_speed_deg_s", 30.0)
    p("joint_acc_deg_s2", 30.0)
    p("probe_orientation", [-90.0, 90.0, 90.0])
    p("detection_ready_xyz", [300.0, -600.0, 110.0])
    p("close_gripper_on_start", True)
    p("sim_probe_mm", 150.0)
    p("probe_depth_mm", 200.0)
    p("contact_force_n_x", 7.5)
    p("contact_force_dir_x", -1.0)
    p("contact_force_n_y", 3.7)
    p("contact_force_dir_y", -1.0)
    p("poll_period_s", 0.02)
    p("settle_time_s", 2.0)
    p("stiffness", [200.0, 200.0, 3000.0, 200.0, 200.0, 200.0])
    p("retreat_z_mm", 150.0)
    p("object_radius_mm", 55.0)
    p("y_probe_start_offset_mm", 150.0)
    p("gripper_channel", "service")
    p("open_do", [1, 1])
    p("close_do", [1, 0])

    g = lambda n: node.get_parameter(n).value  # noqa: E731
    if not bool(g("arm")):
        logger.error("Robot is NOT armed; no motion. Rerun with -p arm:=true")
        return None
    contact_force_x = float(g("contact_force_n_x"))
    if contact_force_x < 3.0 or contact_force_x > 30.0:
        logger.error("contact_force_n_x must be in [3, 30]")
        return None
    contact_force_y = float(g("contact_force_n_y"))
    if contact_force_y < 1.0 or contact_force_y > 30.0:
        logger.error("contact_force_n_y must be in [1, 30]")
        return None
    contact_dir_x = float(g("contact_force_dir_x"))
    if contact_dir_x not in (-1.0, 1.0):
        logger.error("contact_force_dir_x must be -1.0 or +1.0")
        return None
    contact_dir_y = float(g("contact_force_dir_y"))
    if contact_dir_y not in (-1.0, 1.0):
        logger.error("contact_force_dir_y must be -1.0 or +1.0")
        return None
    if float(g("object_radius_mm")) <= 0.0:
        logger.error("object_radius_mm must be > 0")
        return None
    if str(g("gripper_channel")) not in ("service", "digital_output"):
        logger.error("gripper_channel must be 'service' or 'digital_output'")
        return None
    if len(list(g("probe_orientation"))) != 3 or len(list(g("detection_ready_xyz"))) != 3:
        logger.error("probe_orientation / detection_ready_xyz must have 3 values")
        return None

    return ProbeGripContext(
        node=node,
        detection_ready_xyz=[float(v) for v in g("detection_ready_xyz")],
        probe_orientation=[float(v) for v in g("probe_orientation")],
        approach_joint=[float(v) for v in g("approach_joint")],
        speed_mm_s=float(g("speed_mm_s")), acc_mm_s2=float(g("acc_mm_s2")),
        joint_speed_deg_s=float(g("joint_speed_deg_s")),
        joint_acc_deg_s2=float(g("joint_acc_deg_s2")),
        detect_enabled=bool(g("detect_enabled")),
        sim_probe_mm=float(g("sim_probe_mm")),
        probe_depth_mm=float(g("probe_depth_mm")),
        contact_force_n_x=contact_force_x, contact_force_dir_x=contact_dir_x,
        contact_force_n_y=contact_force_y, contact_force_dir_y=contact_dir_y,
        poll_period_s=float(g("poll_period_s")), settle_time_s=float(g("settle_time_s")),
        stiffness=[float(v) for v in g("stiffness")],
        retreat_z_mm=float(g("retreat_z_mm")),
        object_radius_mm=float(g("object_radius_mm")),
        y_probe_start_offset_mm=float(g("y_probe_start_offset_mm")),
        close_gripper_on_start=bool(g("close_gripper_on_start")),
        gripper_channel=str(g("gripper_channel")),
        open_do=[int(v) for v in g("open_do")], close_do=[int(v) for v in g("close_do")],
        tool_name=str(g("tool_name")),
        tcp_name=str(g("tcp_name")),
    )


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        ctx = _ctx_from_params(node)
        if ctx is None:
            return
        try:
            from DSR_ROBOT2 import DR_BASE  # noqa: F401  (import 가능 여부 조기 확인)
        except ImportError as exc:
            node.get_logger().error(f"Failed to import Doosan API: {exc}")
            return
        run_once(ctx)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
