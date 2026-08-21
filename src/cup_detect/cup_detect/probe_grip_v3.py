#!/usr/bin/env python3
# =============================================================
# probe_grip_v3.py — X-직진 양측 접촉 탐지로 물체 중심좌표 산출 (좌표 탐지 전용)
# -------------------------------------------------------------
# 실행: ros2 run cup_detect probe_grip_v3 --ros-args -p arm:=true
# 노드: m0609_probe_grip_v3 | 좌표계: DR_BASE | 포즈 6D=[x,y,z(mm),A,B,C(ZYZ deg)]
#
# v2와의 차이: v2는 +X 접촉 1회 후 바로 측면 파지(grasp)까지 수행했다.
# v3는 파지를 하지 않는다 — **좌표만 찾아서 반환**하고, 파지/이송 등 후속 행동은
# 이 결과를 받는 별도 오케스트레이션 코드가 담당한다(통합 지점).
#
# 흐름: movej 안전자세 → (옵션) 그리퍼 닫기 → 감지 준비위치 →
#   +X 직진 접촉 탐지(물체 A측면, y/z는 감지 평면값으로 고정) →
#   접촉 후 +Z 회피 상승 → +X 오프셋 이동(물체를 넘어감) → -Z로 감지 높이 복귀 →
#   -X 직진 접촉 탐지(물체 B측면, 반대편에서 되짚어 옴) →
#   접촉 후 +Z 회피 상승(안전 마무리) →
#   두 접촉 X값의 중앙 = 물체 중심좌표로 저장, 반환.
# 주의(실기 안전): arm:=true 없이는 모션 없음. detect_enabled:=false 면
#   힘 탐지 없이 sim_probe_x_mm 고정 전진/후진(드라이런).
# =============================================================
"""X-직진으로 물체 양 옆면을 접촉 탐지해 중심 X좌표를 구하는 탐지 전용 노드.

probe_grip_v2의 실기 기계장치(컴플라이언스 접촉 탐지, TCP, 서비스 discovery)를
재사용하되, 접촉 후 곧장 파지하던 v2와 달리 물체 반대편까지 돌아가 두 번째
접촉점을 찍고 그 중앙을 물체 위치로 반환한다. 파지/이송은 하지 않는다.
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
NODE_NAME = "m0609_probe_grip_v3"
TOOL_NAME = "Tool Weight123"
TCP_NAME = "GripperDA_v1"
GRIPPER_SERVICE = "/onrobot/sendCommand"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


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
    sim_probe_x_mm: float = 150.0
    probe_x_depth_mm: float = 200.0
    contact_force_n: float = 8.0
    contact_force_n_neg: float = 3.0  # -X(되짚어 접근) 실측 반력이 +X보다 작아 별도 threshold
    contact_force_dir: float = -1.0  # +X 접근 반력은 -X → -1. (-X 접근 시엔 부호 자동 반전)
    poll_period_s: float = 0.02
    settle_time_s: float = 2
    stiffness: list = field(
        default_factory=lambda: [200.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]
    )  # X를 약하게: 접촉을 부드럽게, 부호감지는 그대로 유지
    retreat_z_mm: float = 150.0  # 접촉 후 물체를 넘어가기 전 회피 상승량
    side_offset_x_mm: float = 180.0  # 회피 상승 후 +X로 넘어가는 거리(물체 폭보다 커야 함)
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
    """물체 1개의 양 옆면을 접촉 탐지해 중심좌표를 구해 반환(파지는 하지 않음).

    반환 키: found, reason, contact_pose_1, contact_pose_2,
             object_center_xyz([x,y,z]), object_width_mm.
    예외는 호출자에게 전파(모션 중이면 이미 정지 시도됨).
    """
    node = ctx.node
    logger = node.get_logger()
    result = {
        "found": False, "reason": "",
        "contact_pose_1": None, "contact_pose_2": None,
        "object_center_xyz": None, "object_width_mm": None,
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
    # 미보정 시 get_tool_force가 그리퍼 자중을 외력으로 오독 → 방향성 힘 편향/
    # 오검출(false contact) 원인이 되므로 TCP와 동일하게 능동 설정+검증한다.
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

    def read_tool_force_x():
        force = get_tool_force(ref=DR_BASE)
        if force == -1 or len(force) < 1:
            return None
        return float(force[0])

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

    def probe_x(from_pose, direction, depth_mm, label, threshold=None):
        """from_pose에서 Base X방향(direction=+1.0/-1.0)으로 depth_mm 전진, 접촉 시 급정지.

        direction=+1.0: 원래 v2와 동일(+X 접근, contact_force_dir 그대로).
        direction=-1.0: 반대편에서 되짚어 오는 접근 — 반력 부호가 뒤집히므로
          effective contact_force_dir = ctx.contact_force_dir * direction 로 자동 반전.
        threshold: 미지정 시 ctx.contact_force_n(+X 기준). -X는 실측 반력이 작아
          run_once에서 ctx.contact_force_n_neg를 넘겨준다.
        반환: "contact" | "reached_depth". 컴플라이언스는 호출 전 진입돼 있어야 함.
        """
        threshold = ctx.contact_force_n if threshold is None else threshold
        target = list(from_pose)
        target[0] += direction * depth_mm
        if amovel(posx(target), vel=[speed, 50.0], acc=[acceleration, 10.0],
                  ref=DR_BASE, mod=DR_MV_MOD_ABS) != 0:
            raise RuntimeError(f"{label}: amovel(probe) rejected")
        fsm["motion_active"] = True

        effective_dir = ctx.contact_force_dir * direction
        motion_seen, peak_fx, last_diag, outcome = False, 0.0, 0.0, None
        move_start = time.monotonic()
        deadline = move_start + depth_mm / speed + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            in_settle = (time.monotonic() - move_start) < ctx.settle_time_s
            fx = read_tool_force_x()
            x_hit = (fx is not None
                     and effective_dir * fx >= threshold)
            now = time.monotonic()
            if now - last_diag >= 0.5:
                last_diag = now
                if fx is not None:
                    peak_fx = max(peak_fx, abs(fx))
                    phase = "settling" if in_settle else "monitoring"
                    logger.info(f"[{label}] ({phase}) Fx={fx:.2f} N, peak={peak_fx:.2f}, "
                                f"dir={effective_dir:+.0f} thr={threshold:.1f}")
            if x_hit and not in_settle:
                request_stop(DR_QSTOP)
                logger.warning(f"[{label}] X CONTACT (Fx={fx:.2f} N, "
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

    def probe_side(from_pose, direction, label, threshold=None):
        """한쪽 옆면 접촉 탐지 1회(컴플라이언스 on/off 포함). (outcome, contact_pose)."""
        if ctx.detect_enabled:
            enable_compliance()
            try:
                outcome = probe_x(
                    from_pose, direction, ctx.probe_x_depth_mm, label, threshold=threshold)
            finally:
                disable_compliance()
        else:
            sim_pose = list(from_pose)
            sim_pose[0] += direction * ctx.sim_probe_x_mm
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

        # 4. A측면: +X 직진 접촉 탐지.
        outcome_a, contact_a = probe_side(ready_pose, +1.0, "probe side A (+X)")
        if outcome_a != "contact":
            result["reason"] = "no X contact on side A (probe depth exhausted)"
            logger.warning(result["reason"])
            return result
        result["contact_pose_1"] = contact_a
        cx_a = contact_a[0]
        logger.info(f"side A contact at pose={contact_a}")

        # 5. 접촉 후 +Z 회피 상승 → +X 오프셋으로 물체를 넘어감 → -Z로 감지 높이 복귀.
        retreat_pose = list(contact_a); retreat_pose[2] += ctx.retreat_z_mm
        move_line(retreat_pose, "retreat +Z clear of object")
        offset_pose = list(retreat_pose); offset_pose[0] += ctx.side_offset_x_mm
        move_line(offset_pose, "move +X offset past object")
        descend_pose = list(offset_pose); descend_pose[2] = dz
        move_line(descend_pose, "descend to detection height on far side")

        # 6. B측면: 반대편에서 -X로 되짚어 접촉 탐지.
        time.sleep(1)
        outcome_b, contact_b = probe_side(
            descend_pose, -1.0, "probe side B (-X)", threshold=ctx.contact_force_n_neg)
        if outcome_b != "contact":
            result["reason"] = "no X contact on side B (probe depth exhausted)"
            logger.warning(result["reason"])
            return result
        result["contact_pose_2"] = contact_b
        cx_b = contact_b[0]
        logger.info(f"side B contact at pose={contact_b}")

        # 7. 마무리 회피 상승(물체에 닿은 채로 끝내지 않도록).
        final_retreat = list(contact_b); final_retreat[2] += ctx.retreat_z_mm
        move_line(final_retreat, "retreat +Z after side B contact")

        # 8. 중앙 X = 두 접촉 X의 평균. Y/Z는 감지 평면 지령값(dy,dz)으로 고정
        #    (컴플라이언스 드리프트로 오염된 contact값 대신 감지 시작 좌표를 쓴다).
        center_x = (cx_a + cx_b) / 2.0
        result["found"] = True
        result["reason"] = "ok"
        result["object_center_xyz"] = [center_x, dy, dz]
        result["object_width_mm"] = abs(cx_b - cx_a)
        logger.info(f"DONE object_center_xyz={result['object_center_xyz']} "
                    f"width={result['object_width_mm']:.1f}mm")
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
    p("sim_probe_x_mm", 150.0)
    p("probe_x_depth_mm", 200.0)
    p("contact_force_n", 8.0)
    p("contact_force_n_neg", 3.0)
    p("contact_force_dir", -1.0)
    p("poll_period_s", 0.02)
    p("settle_time_s", 2)
    p("stiffness", [200.0, 3000.0, 3000.0, 200.0, 200.0, 200.0])
    p("retreat_z_mm", 150.0)
    p("side_offset_x_mm", 180.0)
    p("gripper_channel", "service")
    p("open_do", [1, 1])
    p("close_do", [1, 0])

    g = lambda n: node.get_parameter(n).value  # noqa: E731
    if not bool(g("arm")):
        logger.error("Robot is NOT armed; no motion. Rerun with -p arm:=true")
        return None
    contact_force = float(g("contact_force_n"))
    if contact_force < 3.0 or contact_force > 30.0:
        logger.error("contact_force_n must be in [3, 30]")
        return None
    contact_force_neg = float(g("contact_force_n_neg"))
    if contact_force_neg < 1.0 or contact_force_neg > 30.0:
        logger.error("contact_force_n_neg must be in [1, 30]")
        return None
    contact_dir = float(g("contact_force_dir"))
    if contact_dir not in (-1.0, 1.0):
        logger.error("contact_force_dir must be -1.0 or +1.0")
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
        sim_probe_x_mm=float(g("sim_probe_x_mm")),
        probe_x_depth_mm=float(g("probe_x_depth_mm")),
        contact_force_n=contact_force, contact_force_n_neg=contact_force_neg,
        contact_force_dir=contact_dir,
        poll_period_s=float(g("poll_period_s")), settle_time_s=float(g("settle_time_s")),
        stiffness=[float(v) for v in g("stiffness")],
        retreat_z_mm=float(g("retreat_z_mm")),
        side_offset_x_mm=float(g("side_offset_x_mm")),
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
