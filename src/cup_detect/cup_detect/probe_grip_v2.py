#!/usr/bin/env python3
# =============================================================
# probe_grip_v2.py — X-직진 접촉 탐지 + 측면 파지 (실전 최종본)
# -------------------------------------------------------------
# 실행: ros2 run rokey probe_grip_v2 --ros-args -p arm:=true
#       (또는: python3 rokey/detect/probe_grip_v2.py --ros-args -p arm:=true)
# 노드: m0609_probe_grip_v2 | 좌표계: DR_BASE | 포즈 6D=[x,y,z(mm),A,B,C(ZYZ deg)]
#
# ┌──────────────── 모듈 통합 인터페이스 (전달 필요요소) ────────────────┐
# │ [사전조건]                                                            │
# │  · 브링업: m0609_rg2_bringup bringup.launch.py (mode:=real host:=IP)  │
# │  · TCP 프리셋 tcp_name('GripperDA_v1') 교시·등록                       │
# │  · 그리퍼 노드 onrobot_rg_control:                                     │
# │      서비스 /onrobot/sendCommand, 토픽 /onrobot/grip_detected,         │
# │      /onrobot_joint_states  (verify_grip·service채널에 필요)           │
# │ [입력 — 이 물체 1개를 파지하기 위해 호출자가 정해줘야 하는 값]          │
# │  · detection_ready_xyz [x,y,z]  : +X 직진 탐지를 시작할 준비 위치      │
# │  · grasp_orientation  [A,B,C]   : 감지·파지 공통 손목 자세(ZYZ)        │
# │  · object_radius_mm             : 접촉면→파지중심 X 후퇴량             │
# │  · grasp_y_standoff_mm / grasp_y_offset_mm : +Y 안전진입/최종 Y        │
# │  · contact_force_n, detect_enabled, gripper_channel, tcp_name         │
# │ [출력 — 호출자에게 돌려주는 결과]  run_once()의 반환 dict:             │
# │  · {"found": bool, "grip_ok": bool, "reason": str,                    │
# │     "contact_pose": [6], "grasp_pose": [6], "final_pose": [6]}         │
# │ [공개 진입점]                                                          │
# │  · main(): 노드 생성→run_once() 1회→종료 (CLI 실행용)                  │
# │  · run_once(ctx) -> dict : 오케스트레이터가 자기 노드/모션핸들로 호출   │
# │      (ctx = ProbeGripContext; 아래 dataclass 참조)                    │
# └──────────────────────────────────────────────────────────────────────┘
#
# ┌──────────── 통합 방법 (별도 오케스트레이션 계층 불필요) ─────────────┐
# │ 이 노드는 자기 완결형: 서비스 discovery·TCP·그리퍼·모션·컴플라이언스를 │
# │ 스스로 처리하고 run_once()가 결과 dict를 돌려준다. 두 방법 중 하나로:  │
# │  (A) 독립 실행 — ros2 run rokey probe_grip_v2 --ros-args -p arm:=true  │
# │      상위 시스템은 이 프로세스를 픽마다 실행/대기하면 끝(스텝 단위 조립).│
# │  (B) 인프로세스 호출 —                                                 │
# │      from rokey.detect.probe_grip_v2 import run_once, ProbeGripContext │
# │      ctx = ProbeGripContext(node=my_node,                             │
# │              detection_ready_xyz=[..], grasp_orientation=[..])         │
# │      res = run_once(ctx)   # 블로킹, 결과 dict 반환                    │
# │                                                                        │
# │ [다른 노드와의 충돌 주의]                                              │
# │  · 모션·컴플라이언스 명령을 내는 DSR 노드는 시스템에 '하나'만. 둘이     │
# │    동시에 movel/compliance를 내면 컨트롤러 모드가 싸운다(위험) — 금지.  │
# │  · DSR_ROBOT2는 전역 노드(DR_init.__dsr__node)를 공유하므로 (B)는       │
# │    '한 프로세스에 DSR 모션 노드 1개' 원칙을 지킬 것.                    │
# │  · 외부 모니터링 노드는 '구독 전용'(joint_states/error/tool_force 등)   │
# │    이고 별도 프로세스면 충돌 없음 — 권장 패턴. 읽기 서비스 호출도 OK,   │
# │    단 모션 중 get_tool_force 등을 고빈도로 쏘면 서비스 부하만 유의.     │
# └──────────────────────────────────────────────────────────────────────┘
#
# 흐름: movej 안전자세 → 그리퍼 닫기 → 감지 준비위치 → +X 직진 접촉 탐지 →
#   X로 물체반경 후퇴 → 그리퍼 열기 → +Y standoff → -Y 진입(Z유지) →
#   그리퍼 닫기(그립) → 파지 판정 → +Z 상승 → 목표(place_pose) 이송.
# 주의(실기 안전): arm:=true 없이는 모션 없음. 실기는 사람 감독·저속·클리어 공간에서.
#   detect_enabled:=false 면 힘 탐지 없이 sim_probe_x_mm 고정 전진(드라이런).
# =============================================================
"""X-직진 접촉 탐지 후 측면 파지하는 실전 노드.

probe_grip_v1(위→아래 Z탐지, 3점 순회)의 실기 기계장치(컴플라이언스 접촉 탐지,
그리퍼 개폐, 파지 판정, TCP)를 재사용하되, 탐지 축을 Base +X 로, 파지 기하를
'X 물체반경 후퇴 + Y 안전진입'으로 바꾼 버전.
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
)
from onrobot_rg_msgs.srv import SetCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_probe_grip_v2"
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
    grasp_orientation: list
    approach_joint: list = field(default_factory=lambda: [0.0, 0.0, 90.0, 90.0, -90.0, 0.0])
    speed_mm_s: float = 40.0
    acc_mm_s2: float = 40.0
    joint_speed_deg_s: float = 30.0
    joint_acc_deg_s2: float = 30.0
    detect_enabled: bool = True
    sim_probe_x_mm: float = 150.0
    probe_x_depth_mm: float = 200.0
    contact_force_n: float = 8.0
    contact_force_dir: float = -1.0  # +X 접근 반력은 -X → -1. 부호 뒤집으려면 +1.
    poll_period_s: float = 0.02
    settle_time_s: float = 0.6
    stiffness: list = field(
        default_factory=lambda: [200.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]
    )  # X를 약하게(500→200): 접촉을 부드럽게, 부호감지는 그대로 유지
    object_radius_mm: float = 55.0
    grasp_y_standoff_mm: float = 100.0
    grasp_y_offset_mm: float = 0.0
    grasp_z_above_center_mm: float = 0.0
    retreat_z_mm: float = 100.0
    place_via_pose: list = field(
        default_factory=lambda: [779.0, -352.0, 400.0, 148.0, -90.0, -90.0]
    )  # 상승 후 place 전 경유(접근) 위치 (DR_BASE ABS, ZYZ)
    place_pose: list = field(
        default_factory=lambda: [740.0, -600.0, 115.0, 125.0, -90.0, -90.0]
    )  # 파지·상승 후 이송할 목표(적재) 위치 (DR_BASE ABS, ZYZ)
    close_gripper_on_start: bool = True
    verify_grip: bool = True
    grip_settle_s: float = 1.5
    pos_closed_empty: float = 0.7587
    pos_margin: float = 0.02
    grip_verify_timeout_s: float = 3.0
    gripper_channel: str = "service"
    open_do: list = field(default_factory=lambda: [1, 1])
    close_do: list = field(default_factory=lambda: [1, 0])
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
    """물체 1개에 대한 탐지→파지→상승 1회 실행. 결과 dict 반환.

    반환 키: found, grip_ok, reason, contact_pose, grasp_pose, final_pose.
    예외는 호출자에게 전파(모션 중이면 이미 정지 시도됨).
    """
    node = ctx.node
    logger = node.get_logger()
    result = {
        "found": False, "grip_ok": False, "reason": "",
        "contact_pose": None, "grasp_pose": None, "final_pose": None,
    }

    # -- 서비스 사전 discovery (DSR wrapper의 Fast DDS race 회피) --------------
    required_services = (
        (GetCurrentTool, "tool/get_current_tool"),
        (GetCurrentTcp, "tcp/get_current_tcp"),
        (SetCurrentTcp, "tcp/set_current_tcp"),
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

    tool_response = call_service(
        node, service_clients["tool/get_current_tool"],
        GetCurrentTool.Request(), "tool/get_current_tool",
    )
    if not tool_response.info:
        logger.warning(
            "No Tool payload preset active — external-force estimate is biased, "
            "making X contact detection unreliable. Activate the taught Tool."
        )

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

    def verify_grip():
        """close 직후 파지 판정. (grip 비트 > effort > position 갭). (held, reason)."""
        latest = {"grip": None, "eff": None, "pos": None}

        def on_bit(msg):
            latest["grip"] = msg.data

        def on_js(msg):
            if msg.position:
                latest["pos"] = msg.position[0]
            if msg.effort:
                latest["eff"] = msg.effort[0]

        sub_bit = node.create_subscription(Bool, "/onrobot/grip_detected", on_bit, 3)
        sub_js = node.create_subscription(JointState, "/onrobot_joint_states", on_js, 3)
        try:
            deadline = time.monotonic() + ctx.grip_verify_timeout_s
            time.sleep(0.5)
            while latest["grip"] is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_subscription(sub_bit)
            node.destroy_subscription(sub_js)

        if latest["grip"] is not None:
            return bool(latest["grip"]), f"hardware grip bit={latest['grip']}"
        if latest["eff"] is not None:
            return latest["eff"] != 0.0, f"effort={latest['eff']:.2f} (nonzero=grip)"
        if latest["pos"] is not None:
            held = latest["pos"] < (ctx.pos_closed_empty - ctx.pos_margin)
            return held, (f"position={latest['pos']:.4f} vs empty="
                          f"{ctx.pos_closed_empty:.4f} (margin {ctx.pos_margin})")
        return False, "no gripper state (onrobot_rg_control running?)"

    def probe_forward_x(from_pose, label):
        """from_pose에서 Base +X로 probe_x_depth 전진, X 반력 접촉 시 급정지.

        반환: "contact" | "reached_depth". 컴플라이언스는 호출 전 진입돼 있어야 함.
        """
        target = list(from_pose)
        target[0] += ctx.probe_x_depth_mm  # Base +X 전진
        if amovel(posx(target), vel=[speed, 50.0], acc=[acceleration, 10.0],
                  ref=DR_BASE, mod=DR_MV_MOD_ABS) != 0:
            raise RuntimeError(f"{label}: amovel(probe) rejected")
        fsm["motion_active"] = True

        motion_seen, peak_fx, last_diag, outcome = False, 0.0, 0.0, None
        move_start = time.monotonic()
        deadline = move_start + ctx.probe_x_depth_mm / speed + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            in_settle = (time.monotonic() - move_start) < ctx.settle_time_s
            # 접촉 판정: get_tool_force(로그와 같은 부호)를 직접 읽어, 지정 방향의
            #   반력이 임계 이상일 때만. +X 접근 시 물체 반력은 -X 이므로
            #   기본 contact_force_dir=-1 (dir*fx = +6 >= thr). 매 루프 읽는다.
            fx = read_tool_force_x()
            x_hit = (fx is not None
                     and ctx.contact_force_dir * fx >= ctx.contact_force_n)
            now = time.monotonic()
            if now - last_diag >= 0.5:
                last_diag = now
                if fx is not None:
                    peak_fx = max(peak_fx, abs(fx))
                    phase = "settling" if in_settle else "monitoring"
                    logger.info(f"[{label}] ({phase}) Fx={fx:.2f} N, peak={peak_fx:.2f}, "
                                f"dir={ctx.contact_force_dir:+.0f} thr={ctx.contact_force_n:.1f}")
            if x_hit and not in_settle:
                request_stop(DR_QSTOP)
                logger.warning(f"[{label}] X CONTACT (Fx={fx:.2f} N, "
                               f"dir={ctx.contact_force_dir:+.0f}). Stop.")
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

    # ===================== 실행 시퀀스 =====================
    try:
        dx, dy, dz = ctx.detection_ready_xyz
        orient = list(ctx.grasp_orientation)

        # 1. 안전 관절자세.
        move_joint(ctx.approach_joint, "movej safe posture")

        # 2. 시작 시 그리퍼 닫기(닫힌 강체로 탐지 오차 축소).
        if ctx.close_gripper_on_start:
            actuate_gripper("close")
            time.sleep(0.5)

        # 3. 감지 준비위치.
        move_line([dx, dy, dz] + orient, "move to detection-ready")
        time.sleep(1)
        # 4. +X 접촉 탐지 (detect_enabled=false 면 고정 전진 드라이런).
        if ctx.detect_enabled:
            set_ref_coord(DR_BASE)
            if task_compliance_ctrl(stx=ctx.stiffness, time=0.5) != 0:
                raise RuntimeError("task_compliance_ctrl failed")
            fsm["compliance_active"] = True
            time.sleep(0.6)
            logger.info("compliance ON (X soft), probing +X")
            outcome = probe_forward_x([dx, dy, dz] + orient, "probe +X")
            release_compliance_ctrl()
            fsm["compliance_active"] = False
            time.sleep(0.3)
            if outcome != "contact":
                logger.warning("Reached probe depth without X contact; no object")
                return result
        else:
            move_line([dx + ctx.sim_probe_x_mm, dy, dz] + orient,
                      "probe +X (sim fixed advance)")

        contact_raw, _ = get_current_posx(ref=DR_BASE)
        contact_pose = [float(v) for v in contact_raw[:6]]
        result["found"] = True
        result["contact_pose"] = contact_pose
        cx = contact_pose[0]
        logger.info(f"X contact at pose={contact_pose} (Y/Z drift vs command "
                    f"dy={dy} dz={dz}: dY={contact_pose[1]-dy:.1f} dZ={contact_pose[2]-dz:.1f})")

        # 5. 파지 기하: 접촉에서 취하는 건 X뿐(물체반경 후퇴). Y/Z는 컴플라이언스
        #    드리프트로 오염된 contact값 대신 '탐지 평면' 지령값(dy,dz)을 쓴다 —
        #    수평 probe는 고정 높이 dz·고정 Y=dy에서 물체의 X만 재는 것이 의도.
        grasp_x = cx + ctx.object_radius_mm
        grasp_z = dz + ctx.grasp_z_above_center_mm
        out_pose = [cx, dy + ctx.grasp_y_standoff_mm, grasp_z] + orient
        entry_pose = [grasp_x, dy + ctx.grasp_y_standoff_mm, grasp_z] + orient
        grasp_pose = [grasp_x, dy + ctx.grasp_y_offset_mm, grasp_z] + orient
        result["grasp_pose"] = grasp_pose

        # 6. 그리퍼 열기(물체를 받도록) → +Y standoff → -Y 진입.

        time.sleep(0.5)
        move_line(out_pose, "safe out ")
        move_line(entry_pose, "safe entry at +Y standoff (Z held)")
        actuate_gripper("open")
        move_line(grasp_pose, "approach grip at +Y offset (enter -Y, Z held)")

        # 7. 그리퍼 닫기(그립). 물리적으로 다 닫히는 데 시간이 걸리므로 판정 전 정착 대기.
        actuate_gripper("close")
        time.sleep(ctx.grip_settle_s)

        # 8. 파지 판정.
        if ctx.verify_grip:
            grip_ok, reason = verify_grip()
            result["grip_ok"], result["reason"] = grip_ok, reason
            (logger.info if grip_ok else logger.warning)(f"grip verify: {reason}")
        else:
            time.sleep(0.5)
            result["grip_ok"], result["reason"] = True, "verification disabled"

        # 9. +Z 상승.
        lift_raw, _ = get_current_posx(ref=DR_BASE)
        lift_pose = [float(v) for v in lift_raw[:6]]
        lift_pose[2] += ctx.retreat_z_mm
        move_line(lift_pose, "lift +Z after grip")

        # 10. 상승 후 경유점 → 목표(적재) 위치로 이송.
        move_line(list(ctx.place_via_pose), "move to place via-waypoint")
        move_line(list(ctx.place_pose), "move to place target")
        time.sleep(0.5)
        actuate_gripper("open")
        time.sleep(0.5)

        final_raw, _ = get_current_posx(ref=DR_BASE)
        result["final_pose"] = [float(v) for v in final_raw[:6]]
        logger.info(f"DONE grip_ok={result['grip_ok']} final={result['final_pose']}")
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
    p("tcp_name", TCP_NAME)
    p("detect_enabled", True)
    p("speed_mm_s", 40.0)
    p("acc_mm_s2", 40.0)
    p("approach_joint", [0.0, 0.0, 90.0, 90.0, -90.0, 0.0])
    p("joint_speed_deg_s", 30.0)
    p("joint_acc_deg_s2", 30.0)
    p("grasp_orientation", [-90.0, 90.0, 90.0])
    p("detection_ready_xyz", [300.0, -600.0, 110.0])
    p("close_gripper_on_start", True)
    p("sim_probe_x_mm", 150.0)
    p("probe_x_depth_mm", 200.0)
    p("contact_force_n", 8.0)
    p("contact_force_dir", -1.0)
    p("poll_period_s", 0.02)
    p("settle_time_s", 0.6)
    p("stiffness", [200.0, 3000.0, 3000.0, 200.0, 200.0, 200.0])
    p("object_radius_mm", 55.0)
    p("grasp_y_standoff_mm", 100.0)
    p("grasp_y_offset_mm", 0.0)
    p("grasp_z_above_center_mm", 0.0)
    p("retreat_z_mm", 100.0)
    p("place_via_pose", [779.0, -352.0, 400.0, 148.0, -90.0, -90.0])
    p("place_pose", [740.0, -600.0, 165.0, 115.0, -90.0, -90.0])
    p("verify_grip", True)
    p("grip_settle_s", 1.5)
    p("pos_closed_empty", 0.7587)
    p("pos_margin", 0.02)
    p("grip_verify_timeout_s", 3.0)
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
    contact_dir = float(g("contact_force_dir"))
    if contact_dir not in (-1.0, 1.0):
        logger.error("contact_force_dir must be -1.0 or +1.0")
        return None
    if str(g("gripper_channel")) not in ("service", "digital_output"):
        logger.error("gripper_channel must be 'service' or 'digital_output'")
        return None
    if len(list(g("grasp_orientation"))) != 3 or len(list(g("detection_ready_xyz"))) != 3:
        logger.error("grasp_orientation / detection_ready_xyz must have 3 values")
        return None
    if len(list(g("place_pose"))) != 6 or len(list(g("place_via_pose"))) != 6:
        logger.error("place_pose / place_via_pose must have 6 values [x,y,z,A,B,C]")
        return None

    return ProbeGripContext(
        node=node,
        detection_ready_xyz=[float(v) for v in g("detection_ready_xyz")],
        grasp_orientation=[float(v) for v in g("grasp_orientation")],
        approach_joint=[float(v) for v in g("approach_joint")],
        speed_mm_s=float(g("speed_mm_s")), acc_mm_s2=float(g("acc_mm_s2")),
        joint_speed_deg_s=float(g("joint_speed_deg_s")),
        joint_acc_deg_s2=float(g("joint_acc_deg_s2")),
        detect_enabled=bool(g("detect_enabled")),
        sim_probe_x_mm=float(g("sim_probe_x_mm")),
        probe_x_depth_mm=float(g("probe_x_depth_mm")),
        contact_force_n=contact_force, contact_force_dir=contact_dir,
        poll_period_s=float(g("poll_period_s")), settle_time_s=float(g("settle_time_s")),
        stiffness=[float(v) for v in g("stiffness")],
        object_radius_mm=float(g("object_radius_mm")),
        grasp_y_standoff_mm=float(g("grasp_y_standoff_mm")),
        grasp_y_offset_mm=float(g("grasp_y_offset_mm")),
        grasp_z_above_center_mm=float(g("grasp_z_above_center_mm")),
        retreat_z_mm=float(g("retreat_z_mm")),
        place_via_pose=[float(v) for v in g("place_via_pose")],
        place_pose=[float(v) for v in g("place_pose")],
        close_gripper_on_start=bool(g("close_gripper_on_start")),
        verify_grip=bool(g("verify_grip")),
        grip_settle_s=float(g("grip_settle_s")),
        pos_closed_empty=float(g("pos_closed_empty")), pos_margin=float(g("pos_margin")),
        grip_verify_timeout_s=float(g("grip_verify_timeout_s")),
        gripper_channel=str(g("gripper_channel")),
        open_do=[int(v) for v in g("open_do")], close_do=[int(v) for v in g("close_do")],
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
