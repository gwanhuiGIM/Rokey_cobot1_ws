#!/usr/bin/env python3
# =============================================================
# probe_grip_v1.py — 3개 점을 순서대로 Z하강 탐색, Z반력이 있는 첫 물체를 파지
# -------------------------------------------------------------
# 실행: ros2 run rokey probe_grip_v1 --ros-args -p arm:=true
#       (또는: python3 rokey/detect/probe_grip_v1.py --ros-args -p arm:=true)
# 노드: m0609_probe_grip_v1 | 좌표계: DR_BASE
#   서비스(구독): tcp/{get,set}_current_tcp, tool/get_current_tool,
#     aux_control/{get_current_posx, get_current_tool_flange_posx, get_tool_force},
#     motion/{move_line, move_stop, check_motion}, /onrobot/sendCommand(그리퍼)
# 의존: rclpy, doosan-robot2(DSR_ROBOT2), onrobot_rg_msgs
# 입력: PROBE_POINTS(각 물체 위 접근 포즈 3개, DR_BASE 6D) — 교시 필요
# 주의: arm:=true 없이는 모션 없음. 하강은 컴플라이언스(Z 무름)로 진행하며
#   Z 외력 임계 초과 시 급정지. detection_v1.py 뼈대 재사용.
# =============================================================
"""3개 점을 순서대로 위에서 아래로 프로빙하여 첫 물체를 파지한다.

동작 (첫 물체 발견 시 종료):
  각 접근 포즈(물체 위)에 대해 순서대로:
    1. 접근 포즈로 강체 이동(위에서 대기), 그리퍼 열기.
    2. task compliance control 진입(Z 무름, XY 뻣뻣) 후 Base -Z로 하강.
    3. 하강 중 check_force_condition(DR_AXIS_Z)로 Z 반력 감시.
       - Z 반력이 contact_force_n 이상이면 즉시 접촉=물체 (단일 샘플, 디바운스
         없음 — 연속 대기 중 물체를 계속 누르는 것을 피하기 위함).
       - probe_depth_mm까지 무접촉이면 그 점엔 물체 없음 → 위로 복귀, 다음 점.
    4. 첫 접촉 시: 컴플라이언스 해제 → 물체 위로 상승 → 안전 관절자세로 movej(그리퍼
       수평 재배향) → grasp_pose 바로 위(+Z standoff)로 접근 → -Z 수직 하강으로
       grasp_pose 진입 → 닫기(측면 파지) → retreat_z_mm 상승 후 종료.

detect_enabled:=false 이면 감지/컴플라이언스 없이 접근 포즈만 순회하는
드라이런(경로 충돌 확인용). 이것은 접촉탐색 예제이며 안전등급 보호기능이 아니다.
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
# 그리퍼는 제어박스 DO가 아니라 onrobot_rg_control ROS 서비스로 구동
# (detection_v1.py / plate_move.py와 동일 방식). "o"=open, "c"=close.
from onrobot_rg_msgs.srv import SetCommand
# 파지 성공 판정용 모니터링 토픽 (onrobot_rg_control 노드가 발행).
# GRIP_DETECTION_README 참조: RG2는 실측 힘이 없어 grip 비트/effort/position로 판정.
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_probe_grip_v1"
TCP_NAME = "GripperDA_v1"
GRIPPER_SERVICE = "/onrobot/sendCommand"

# 각 물체 "위"의 접근 포즈 3개 (DR_BASE, 6D TCP 포즈; ZYZ 오일러 A,B,C[deg]).
# 물체보다 충분히 높은 안전 높이에서 교시할 것 — 여기서 Base -Z로 하강 탐색한다.
# TODO(확정 필요): 실제 교시값으로 교체. 아래는 자리표시자(placeholder)이며 실행 금지.
PROBE_POINTS = [
    [250.0, -300.0, 200.0, 90.0, 180.0, 90.0],   # TODO(확정 필요)
    [300.0, -300.0, 200.0, 90.0, 180.0, 90.0],     # TODO(확정 필요)
    [350.0, -300.0, 200.0, 90.0, 180.0, 90.0],  # TODO(확정 필요)
]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def wait_until_idle(check_motion, idle_state, timeout_sec):
    """컨트롤러가 경로 모션 idle을 보고할 때 True."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if check_motion() == idle_state:
            return True
        time.sleep(0.02)
    return False


def request_stop(node, client, stop_mode):
    """MoveStop 서비스를 호출하고 success 플래그를 반환."""
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
    """유한 타임아웃으로 ROS 서비스를 호출."""
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

    # -- 파라미터 (매직넘버 금지: 모두 노출) ---------------------------------
    node.declare_parameter("arm", False)             # 미장전 시 모션 없음
    node.declare_parameter("tcp_name", TCP_NAME)
    node.declare_parameter("detect_enabled", True)
    node.declare_parameter(
        "points", [v for point in PROBE_POINTS for v in point]
    )
    node.declare_parameter("speed_mm_s", 40.0)       # mm/s
    node.declare_parameter("acc_mm_s2", 40.0)        # mm/s^2
    node.declare_parameter("probe_depth_mm", 150.0)  # 점당 최대 하강 탐색 깊이
    node.declare_parameter("contact_force_n", 5.0)   # Z 반력 임계 (N)
    node.declare_parameter("poll_period_s", 0.02)
    # 각 하강 시작의 가속 관성 스파이크를 오탐하지 않도록 초기 감지 억제 시간.
    node.declare_parameter("settle_time_s", 0.6)
    node.declare_parameter(
        "stiffness",
        # 하강 프로브: Z는 무르게(접촉 시 툴이 물러 외력 발산 방지),
        # XY/회전은 뻣뻣하게(수평 위치·자세 유지). detection_v1과 축이 반대.
        [3000.0, 3000.0, 500.0, 200.0, 200.0, 200.0],
    )
    # 측면 수평 파지. 그리퍼 수평 자세는 orientation(ZYZ) 지정 대신 안전 관절자세로
    #   movej해 확정한다(특이점/스윕 없이 자세 도달, 자세값 실측 교시 불필요).
    #   object_height_mm/grasp_center_frac: 물체 높이와 중앙 비율(중앙 높이 계산용).
    node.declare_parameter("object_height_mm", 140.0)  # 파지 대상 높이 (컵=140)
    node.declare_parameter("grasp_center_frac", 0.5)   # 0.5=중앙
    # approach_joint: 수평 파지 진입 안전 관절자세(deg, J1..J6). 이 자세가 곧 그리퍼
    #   수평 방향을 정한다 — 실측 교시값. 자세 자체가 특이점 없이 도달 가능해야 한다.
    node.declare_parameter("approach_joint", [0.0, 30.0, 120.0, 90.0, -90.0, 45.0])
    node.declare_parameter("joint_speed_deg_s", 30.0)  # movej 속도(보수적: 큰 재배향)
    node.declare_parameter("joint_acc_deg_s2", 30.0)
    # 파지각 orientation: movej 뒤 movel로 감지위치에 접근할 때 쓰는 손목 자세
    #   (ZYZ A,B,C[deg]). 실측 확정값. 이 자세로 +Z 오프셋 pre-grasp → -Z 수직 진입.
    node.declare_parameter("grasp_orientation", [-90.0, 90.0, 90.0])  # 실측값
    # pre-grasp: 바로 그립은 위험 → grasp_pose 바로 위(+Z)로 grasp_z_standoff만큼
    #   물러난 곳에 먼저 이동한 뒤 -Z(수직 하강)로 grasp_pose까지 진입한다.
    #   수평 회전된 그리퍼가 위에서 내려오며 물체를 감싼다(+Y 수평 진입 대신).
    node.declare_parameter("grasp_z_standoff_mm", 80.0)     # +Z pre-grasp 오프셋(튜닝)
    # 그리퍼 수평 시 부품-바닥 충돌 방지: 물체 중앙보다 위에서 파지.
    node.declare_parameter("grasp_z_above_center_mm", 30.0)  # 물체 중앙 +Z
    node.declare_parameter("retreat_z_mm", 100.0)     # 파지 후 상승 높이
    # 파지 성공 판정 (GRIP_DETECTION_README 참조). 우선순위: 하드웨어 grip 비트
    #   > effort(gSTA 비0=stall) > position 갭. RG2는 실측 힘이 없어 width(=position)
    #   차분으로 판정한다.
    node.declare_parameter("verify_grip", True)
    # pos_closed_empty: 빈손으로 "c" 했을 때의 관절값(rad). 개체·"c" 목표폭 설정별로
    #   다르므로 실측 교정 필요 (빈손 close 후 ros2 topic echo /onrobot_joint_states).
    node.declare_parameter("pos_closed_empty", 0.7587)  # rad (README 3장 실측값)
    node.declare_parameter("pos_margin", 0.02)          # rad
    node.declare_parameter("grip_verify_timeout_s", 2.0)
    # 그리퍼 구동 채널 (GRIP_DETECTION_README 1장). 주변 파이프라인이 제어박스
    #   DO로 그리퍼를 굴리면 여기도 'digital_output'으로 맞춰 채널 충돌을 피한다.
    #   'service'=onrobot Modbus "o"/"c"(단독 테스트), 'digital_output'=DO1/DO2 조합.
    node.declare_parameter("gripper_channel", "service")
    # DO 조합 → 컴퓨트박스 프리셋 폭 (박스 설정 의존; 커피시스템 기준 매핑).
    #   [1,1]=104mm(handle, 85mm 컵을 감싸도록 예비개방), [1,0]=5mm(grip_close).
    #   과압이 문제면 저력 프리셋을 만든 DO 조합으로 close_do를 바꿔라.
    node.declare_parameter("open_do", [1, 1])
    node.declare_parameter("close_do", [1, 0])

    armed = bool(node.get_parameter("arm").value)
    tcp_name = str(node.get_parameter("tcp_name").value)
    detect_enabled = bool(node.get_parameter("detect_enabled").value)
    flat_points = [float(v) for v in node.get_parameter("points").value]
    points = [flat_points[i:i + 6] for i in range(0, len(flat_points), 6)]
    speed = float(node.get_parameter("speed_mm_s").value)
    acceleration = float(node.get_parameter("acc_mm_s2").value)
    probe_depth = float(node.get_parameter("probe_depth_mm").value)
    contact_force = float(node.get_parameter("contact_force_n").value)
    poll_period = float(node.get_parameter("poll_period_s").value)
    settle_time = float(node.get_parameter("settle_time_s").value)
    stiffness = [float(v) for v in node.get_parameter("stiffness").value]
    object_height = float(node.get_parameter("object_height_mm").value)
    grasp_center_frac = float(node.get_parameter("grasp_center_frac").value)
    approach_joint = [float(v) for v in node.get_parameter("approach_joint").value]
    joint_speed = float(node.get_parameter("joint_speed_deg_s").value)
    joint_acc = float(node.get_parameter("joint_acc_deg_s2").value)
    grasp_orientation = [
        float(v) for v in node.get_parameter("grasp_orientation").value
    ]
    grasp_z_standoff = float(node.get_parameter("grasp_z_standoff_mm").value)
    grasp_z_above_center = float(
        node.get_parameter("grasp_z_above_center_mm").value
    )
    retreat_z = float(node.get_parameter("retreat_z_mm").value)
    verify_grip_enabled = bool(node.get_parameter("verify_grip").value)
    pos_closed_empty = float(node.get_parameter("pos_closed_empty").value)
    pos_margin = float(node.get_parameter("pos_margin").value)
    grip_verify_timeout = float(node.get_parameter("grip_verify_timeout_s").value)
    gripper_channel = str(node.get_parameter("gripper_channel").value)
    open_do = [int(v) for v in node.get_parameter("open_do").value]
    close_do = [int(v) for v in node.get_parameter("close_do").value]

    # -- 파라미터 검증 --------------------------------------------------------
    def bail(message):
        logger.error(message)
        node.destroy_node()
        rclpy.shutdown()

    if len(flat_points) < 6 or len(flat_points) % 6 != 0:
        return bail("points must contain at least one flattened 6D TCP pose")
    if speed <= 0.0 or speed > 50.0:
        return bail("speed_mm_s must be in the range (0, 50]")
    if acceleration <= 0.0 or acceleration > 100.0:
        return bail("acc_mm_s2 must be in the range (0, 100]")
    if probe_depth <= 0.0 or probe_depth > 300.0:
        return bail("probe_depth_mm must be in the range (0, 300]")
    if contact_force < 3.0 or contact_force > 30.0:
        return bail("contact_force_n must be in the range [3, 30]")
    if poll_period < 0.01 or poll_period > 0.1:
        return bail("poll_period_s must be in the range [0.01, 0.1]")
    if settle_time < 0.0 or settle_time > 3.0:
        return bail("settle_time_s must be in the range [0, 3]")
    if len(stiffness) != 6:
        return bail("stiffness must contain six values")
    if object_height <= 0.0 or object_height > 400.0:
        return bail("object_height_mm must be in the range (0, 400]")
    if grasp_center_frac <= 0.0 or grasp_center_frac > 1.0:
        return bail("grasp_center_frac must be in the range (0, 1]")
    if len(approach_joint) != 6:
        return bail("approach_joint must contain six joint angles (deg)")
    if joint_speed <= 0.0 or joint_speed > 90.0:
        return bail("joint_speed_deg_s must be in the range (0, 90]")
    if joint_acc <= 0.0 or joint_acc > 180.0:
        return bail("joint_acc_deg_s2 must be in the range (0, 180]")
    if len(grasp_orientation) != 3:
        return bail("grasp_orientation must contain three ZYZ euler values")
    if grasp_z_standoff <= 0.0 or grasp_z_standoff > 300.0:
        return bail("grasp_z_standoff_mm must be in the range (0, 300]")
    if abs(grasp_z_above_center) > 200.0:
        return bail("grasp_z_above_center_mm magnitude must be <= 200")
    if retreat_z <= 0.0 or retreat_z > 300.0:
        return bail("retreat_z_mm must be in the range (0, 300]")
    if pos_closed_empty <= 0.0 or pos_closed_empty > 2.0:
        return bail("pos_closed_empty must be in the range (0, 2]")
    if pos_margin <= 0.0 or pos_margin > 0.2:
        return bail("pos_margin must be in the range (0, 0.2]")
    if grip_verify_timeout <= 0.0 or grip_verify_timeout > 5.0:
        return bail("grip_verify_timeout_s must be in the range (0, 5]")
    if gripper_channel not in ("service", "digital_output"):
        return bail("gripper_channel must be 'service' or 'digital_output'")
    for do_name, do_pair in (("open_do", open_do), ("close_do", close_do)):
        if len(do_pair) != 2 or any(v not in (0, 1) for v in do_pair):
            return bail(f"{do_name} must be two values, each 0 or 1")

    if not armed:
        return bail(
            "Robot is NOT armed; no motion was sent. After checking Tool/TCP, "
            "points, workspace, and E-stop, rerun with --ros-args -p arm:=true"
        )

    # -- 서비스 사전 discovery (DSR_ROBOT2 wrapper의 Fast DDS race 회피) -------
    required_services = (
        (GetCurrentTool, "tool/get_current_tool"),
        (GetCurrentTcp, "tcp/get_current_tcp"),
        (SetCurrentTcp, "tcp/set_current_tcp"),
        (GetCurrentPosx, "aux_control/get_current_posx"),
        (GetCurrentToolFlangePosx, "aux_control/get_current_tool_flange_posx"),
        (GetToolForce, "aux_control/get_tool_force"),
        (MoveLine, "motion/move_line"),
        (MoveStop, "motion/move_stop"),
        (CheckMotion, "motion/check_motion"),
    )
    service_clients = {}
    for service_type, service_name in required_services:
        client = node.create_client(service_type, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            return bail(f"Required service '/{ROBOT_ID}/{service_name}' is unavailable")
        service_clients[service_name] = client
    logger.info("All required robot services are ready")

    # 'service' 채널만 sendCommand가 필요. 'digital_output' 채널은 DO로 구동하므로
    # sendCommand는 없어도 되지만, verify_grip은 onrobot_rg_control의 모니터링
    # 토픽을 쓰므로 그 노드 자체는 어느 채널이든 떠 있어야 한다.
    gripper_client = None
    if gripper_channel == "service":
        gripper_client = node.create_client(SetCommand, GRIPPER_SERVICE)
        if not gripper_client.wait_for_service(timeout_sec=5.0):
            return bail(
                f"Gripper service '{GRIPPER_SERVICE}' is unavailable. Start "
                "onrobot_rg_control (real) or gripper_virtual_node (emulator)."
            )

    def send_gripper(command, description):
        logger.info(f"Gripper: {description} (command '{command}')")
        request = SetCommand.Request()
        request.command = command
        result = call_service(node, gripper_client, request, GRIPPER_SERVICE)
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            raise RuntimeError(f"Gripper command '{command}' failed: {message}")

    def actuate_gripper(action):
        """open/close를 선택된 채널로 구동. action: "open" | "close".

        'digital_output'이면 제어박스 DO1/DO2 조합(주변 파이프라인과 동일 채널)을
        set한다 — DO 값은 컴퓨트박스 프리셋(폭·힘)을 고르며, 실제 폭은 박스 설정에
        달렸다. 두 채널을 한 실행에서 섞지 말 것(박스가 command/DI 배타 모드일 수 있음).
        set_digital_output은 DSR_ROBOT2 import 이후에만 호출된다(Phase 1).
        """
        if gripper_channel == "digital_output":
            do1, do2 = open_do if action == "open" else close_do
            logger.info(f"Gripper[DO]: {action} (DO1={do1}, DO2={do2})")
            set_digital_output(1, do1)
            set_digital_output(2, do2)
        else:
            send_gripper("o" if action == "open" else "c", action)

    def verify_grip():
        """close 직후 파지 성공 판정. (grip 비트 > effort > position 갭 순)

        반환: (held: bool, reason: str). GRIP_DETECTION_README 4장 참조.
        RG2는 실측 힘이 없어 물체를 물면 position이 빈손보다 덜 닫힌다(width 갭).
        하드웨어 grip 비트 토픽이 없으면 effort/position으로 자동 폴백한다.
        """
        latest = {"grip": None, "eff": None, "pos": None}

        def on_bit(msg):
            latest["grip"] = msg.data

        def on_js(msg):
            if msg.position:
                latest["pos"] = msg.position[0]
            if msg.effort:
                latest["eff"] = msg.effort[0]

        sub_bit = node.create_subscription(Bool, "/onrobot/grip_detected", on_bit, 3)
        sub_js = node.create_subscription(
            JointState, "/onrobot_joint_states", on_js, 3
        )
        try:
            deadline = time.monotonic() + grip_verify_timeout
            time.sleep(0.5)  # 손가락이 물체에 안착할 시간
            while latest["grip"] is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
        finally:
            node.destroy_subscription(sub_bit)
            node.destroy_subscription(sub_js)

        if latest["grip"] is not None:  # 1순위: 하드웨어 grip 비트
            return bool(latest["grip"]), f"hardware grip bit={latest['grip']}"
        if latest["eff"] is not None:  # 2순위: effort(gSTA 비0=stall/grip)
            return latest["eff"] != 0.0, f"effort={latest['eff']:.2f} (nonzero=grip)"
        if latest["pos"] is not None:  # 3순위: position 갭
            held = latest["pos"] < (pos_closed_empty - pos_margin)
            return held, (
                f"position={latest['pos']:.4f} rad vs empty="
                f"{pos_closed_empty:.4f} (margin {pos_margin})"
            )
        return False, "no gripper state received (is onrobot_rg_control running?)"

    # -- TCP 활성화·검증 (detection_v1과 동일: 교시 시 TCP가 반영돼야 좌표 일치) -
    tcp_response = call_service(
        node, service_clients["tcp/get_current_tcp"],
        GetCurrentTcp.Request(), "tcp/get_current_tcp",
    )
    if not tcp_response.success:
        return bail("Controller rejected the active TCP query; no motion sent")
    if tcp_response.info == tcp_name:
        logger.info(f"TCP preset '{tcp_name}' is already active")
    else:
        set_req = SetCurrentTcp.Request()
        set_req.name = tcp_name
        set_res = call_service(
            node, service_clients["tcp/set_current_tcp"], set_req,
            "tcp/set_current_tcp",
        )
        if not set_res.success:
            return bail(f"Controller rejected TCP preset '{tcp_name}'; no motion sent")
        tcp_response = call_service(
            node, service_clients["tcp/get_current_tcp"],
            GetCurrentTcp.Request(), "tcp/get_current_tcp",
        )
        if tcp_response.info and tcp_response.info != tcp_name:
            return bail(
                f"Requested TCP '{tcp_name}' but controller reports "
                f"'{tcp_response.info}'; no motion sent"
            )

    tool_response = call_service(
        node, service_clients["tool/get_current_tool"],
        GetCurrentTool.Request(), "tool/get_current_tool",
    )
    if not tool_response.success:
        return bail("Controller rejected the active Tool query")
    if not tool_response.info:
        logger.warning(
            "No Tool payload preset is active. Without a correct payload the "
            "external-force estimate is biased, which makes contact detection "
            "unreliable. Activate the Tool preset used when teaching."
        )
    active_tool = tool_response.info or "<no active Tool preset>"
    active_tcp = tcp_response.info or tcp_name

    # TCP 오프셋이 실제 적용됐는지(플랜지 포즈와 동일하지 않은지) 확인.
    tcp_pose_req = GetCurrentPosx.Request(); tcp_pose_req.ref = 0
    tcp_pose_res = call_service(
        node, service_clients["aux_control/get_current_posx"], tcp_pose_req,
        "aux_control/get_current_posx",
    )
    flange_req = GetCurrentToolFlangePosx.Request(); flange_req.ref = 0
    flange_res = call_service(
        node, service_clients["aux_control/get_current_tool_flange_posx"],
        flange_req, "aux_control/get_current_tool_flange_posx",
    )
    if (not tcp_pose_res.success or not flange_res.success
            or not tcp_pose_res.task_pos_info):
        return bail("Could not verify the applied TCP offset; no motion sent")
    tcp_pose = list(tcp_pose_res.task_pos_info[0].data)[:6]
    flange_pose = list(flange_res.pos)[:6]
    translation_offset = math.dist(tcp_pose[:3], flange_pose[:3])
    orientation_offset = max(
        abs((float(t) - float(f) + 180.0) % 360.0 - 180.0)
        for t, f in zip(tcp_pose[3:], flange_pose[3:])
    )
    if translation_offset < 0.1 and orientation_offset < 0.1:
        return bail(
            f"TCP preset '{tcp_name}' has not been applied: current TCP pose "
            "is identical to the flange pose. No motion sent."
        )
    logger.info(
        f"Active Tool='{active_tool}', TCP='{active_tcp}'; TCP offset vs flange:"
        f" translation={translation_offset:.2f} mm, orientation="
        f"{orientation_offset:.2f} deg"
    )

    try:
        from DSR_ROBOT2 import (
            DR_AXIS_Z,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_QSTOP,
            DR_STATE_IDLE,
            amovel,
            check_force_condition,
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
    except ImportError as exc:
        return bail(f"Failed to import Doosan API: {exc}")

    stop_client = service_clients["motion/move_stop"]
    # motion_active/compliance_active: 예외·finally 정리에서 공유하는 플래그.
    fsm = {"motion_active": False, "compliance_active": False}

    def read_tool_force_z():
        """DR_BASE 기준 Fz(N), 없으면 None."""
        force = get_tool_force(ref=DR_BASE)
        if force == -1 or len(force) < 3:
            return None
        return float(force[2])

    def move_abs(pose, label):
        """강체 movel 절대이동. 거부 시 예외."""
        if movel(
            posx(pose), vel=[speed, 50.0], acc=[acceleration, 10.0],
            ref=DR_BASE, mod=DR_MV_MOD_ABS,
        ) != 0:
            raise RuntimeError(f"{label}: movel was rejected")

    def move_joint(joint_deg, label):
        """관절 절대이동(movej). 거부 시 예외.

        # UNVERIFIED: movej가 성공 시 0을 반환한다고 가정(movel과 동일 규약).
        # Phase 1 movel이 0 반환을 확인했으므로 같은 래퍼에서 일관될 것으로 추정.
        """
        if movej(posj(joint_deg), vel=joint_speed, acc=joint_acc) != 0:
            raise RuntimeError(f"{label}: movej was rejected")

    def descend_until_z_contact(from_pose, label):
        """from_pose에서 Base -Z로 probe_depth 하강, Z 반력 접촉 시 급정지.

        반환: "contact" 또는 "reached_depth". 어느 경로든 로봇은 정지·idle.
        컴플라이언스는 호출 전 진입돼 있어야 한다.
        """
        target = list(from_pose)
        target[2] -= probe_depth  # Base -Z 하강
        if amovel(
            posx(target), vel=[speed, 50.0], acc=[acceleration, 10.0],
            ref=DR_BASE, mod=DR_MV_MOD_ABS,
        ) != 0:
            raise RuntimeError(f"{label}: amovel(descend) was rejected")
        fsm["motion_active"] = True

        motion_seen = False
        peak_fz = 0.0
        last_diag = 0.0
        outcome = None
        move_start = time.monotonic()
        deadline = move_start + probe_depth / speed + 5.0
        while rclpy.ok() and time.monotonic() < deadline:
            # settle: 하강 시작의 가속 관성 스파이크는 물체 접촉이 아니므로 무시.
            in_settle = (time.monotonic() - move_start) < settle_time
            # 래퍼: 조건(|Fz| >= min) 참이면 0, 거짓이면 -1.
            z_hit = check_force_condition(
                DR_AXIS_Z, min=contact_force, ref=DR_BASE
            ) == 0

            now = time.monotonic()
            if now - last_diag >= 0.5:
                last_diag = now
                fz = read_tool_force_z()
                if fz is not None:
                    peak_fz = max(peak_fz, abs(fz))
                    phase = "settling" if in_settle else "monitoring"
                    logger.info(
                        f"[{label}] ({phase}) live Fz={fz:.2f} N, "
                        f"peak={peak_fz:.2f} N, threshold={contact_force:.1f} N"
                    )

            # 디바운스 없음: 첫 감지에서 즉시 급정지(물체를 계속 누르지 않도록).
            if z_hit and not in_settle:
                request_stop(node, stop_client, DR_QSTOP)
                fz = read_tool_force_z()
                logger.warning(f"[{label}] Z CONTACT (Fz={fz} N). Quick stop sent.")
                outcome = "contact"
                break

            motion_state = check_motion()
            if motion_state != DR_STATE_IDLE:
                motion_seen = True
            elif motion_seen:
                outcome = "reached_depth"
                break
            time.sleep(poll_period)

        if outcome is None:
            logger.error(f"[{label}] monitoring timed out; stopping the robot")
            request_stop(node, stop_client, DR_QSTOP)
        if not wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
            raise RuntimeError(f"{label}: controller did not report idle after stop")
        fsm["motion_active"] = False
        if outcome is None:
            raise RuntimeError(f"{label}: monitoring timed out")
        return outcome

    try:
        # -- 드라이런: 감지/컴플라이언스 없이 접근 포즈만 순회 (충돌 확인) -----
        if not detect_enabled:
            logger.warning(
                f"DRY-RUN ARMED: detection OFF; visiting {len(points)} approach "
                f"poses at {speed:.1f} mm/s"
            )
            for index, point in enumerate(points, start=1):
                logger.info(f"approach pose {index}/{len(points)}: {point}")
                move_abs(point, f"approach {index}")
            logger.info("DRY-RUN COMPLETE")
            return

        logger.warning(
            f"PROBE-GRIP ARMED: probing {len(points)} points top-down; "
            f"Z threshold={contact_force:.1f} N (single-sample, no debounce), "
            f"probe_depth={probe_depth:.1f} mm. Grips the FIRST object found."
        )

        contact_point_index = None

        # Phase 1: 각 점 순서대로 하강 탐색, 첫 접촉에서 종료 ------------------
        for index, point in enumerate(points, start=1):
            label = f"point {index}/{len(points)}"
            logger.info(f"{label}: approach {point}")
            move_abs(point, f"approach {index}")

            # 하강 전 그리퍼 예비개방(물체를 손가락 사이로 감싸며 내려가도록).
            # open_do 폭이 물체 폭보다 넓어야 벽을 감싼다(85mm 컵→104mm 개방 등).
            actuate_gripper("open")
            time.sleep(0.5)

            set_ref_coord(DR_BASE)
            if task_compliance_ctrl(stx=stiffness, time=0.5) != 0:
                raise RuntimeError("Failed to enable task compliance control")
            fsm["compliance_active"] = True
            time.sleep(0.6)  # 모드 전환 안정화
            logger.info(f"{label}: compliance ON, descending")

            outcome = descend_until_z_contact(point, label)

            if outcome == "contact":
                contact_point_index = index
                break  # 첫 물체 발견 → Phase 2로

            # 무접촉: 컴플라이언스 해제 후 접근 높이로 복귀, 다음 점.
            release_compliance_ctrl()
            fsm["compliance_active"] = False
            time.sleep(0.3)
            logger.info(f"{label}: no object (reached probe depth); retreating")
            move_abs(point, f"retreat {index}")

        if contact_point_index is None:
            logger.warning("Probed all points without any Z contact; no object found")
            return

        # Phase 2: 측면 수평 파지 (안전 관절자세 movej → +X standoff → 진입축 insert)
        # 탐지 접촉 포즈에서 물체 중앙 XY(cx,cy)와 윗면 높이(cz_top)를 취한다.
        contact_pose_raw, _ = get_current_posx(ref=DR_BASE)
        contact_pose = [float(v) for v in contact_pose_raw[:6]]
        cx, cy, cz_top = contact_pose[0], contact_pose[1], contact_pose[2]
        logger.info(f"Object at point {contact_point_index}; contact pose={contact_pose}")

        # 컴플라이언스 해제 후 물체 위로 상승(관절 재배향 전 간섭 회피).
        if fsm["compliance_active"]:
            release_compliance_ctrl()
            fsm["compliance_active"] = False
            time.sleep(0.3)
        clear_pose = list(contact_pose)
        clear_pose[2] += retreat_z
        move_abs(clear_pose, "retreat above object")

        # ① 안전 관절자세로 movej → 2단계 파지 첫 위치(그리퍼 수평 재배향, 특이점 회피).
        actuate_gripper("open")
        time.sleep(0.5)
        move_joint([0.0, 30.0, 90.0, 90.0, -90.0, 0], "(1) movej horizontal first posture")
        logger.warning(f"SIDE-GRASP: movej to approach_joint={approach_joint} (deg)")
        move_joint(approach_joint, "(1) movej horizontal first posture")

        # 파지 높이 = 물체 중앙(cz_top - object_height*frac) + grasp_z_above_center.
        #   그리퍼 수평 시 손가락/부품이 바닥에 닿지 않도록 중앙보다 위에서 문다.
        object_center_z = cz_top - object_height * grasp_center_frac
        grasp_z = object_center_z + grasp_z_above_center

        # ② pre-grasp: 바로 그립은 위험 → grasp_pose 바로 위(+Z standoff)로 먼저 이동.
        #    손목 자세는 실측 파지각 grasp_orientation(ZYZ)로 고정.
        pre_grasp_pose = [cx, cy, grasp_z + grasp_z_standoff] + grasp_orientation
        logger.warning(
            f"SIDE-GRASP: pre-grasp={pre_grasp_pose} (+Z {grasp_z_standoff:.1f}mm), "
            f"grasp_z={grasp_z:.1f} (center {object_center_z:.1f} "
            f"+{grasp_z_above_center:.1f}), orient={grasp_orientation}"
        )
        move_abs(pre_grasp_pose, "(2) pre-grasp at +Z standoff")

        # ③ 감지 그립 위치로 -Z 수직 하강(자세 유지) → 물체를 손가락 사이에.
        grasp_pose = [cx, cy, grasp_z] + grasp_orientation
        logger.warning(f"SIDE-GRASP: grip pose={grasp_pose} (descend -Z)")
        move_abs(grasp_pose, "(3) descend -Z to detected grip position")

        logger.info("Closing the gripper (grip)")
        actuate_gripper("close")

        # 파지 성공 판정 (README 4장). 실패해도 빈손 상승은 무해하므로 중단하지 않고
        # 경고만 남긴다 — 시퀀스 중단/재시도 정책은 상위에서 결정.
        # ponytail: 실패 시 abort 대신 warn. 실제 픽 신뢰가 필요하면 정책 추가.
        grip_ok = True
        grip_reason = "verification disabled"
        if verify_grip_enabled:
            grip_ok, grip_reason = verify_grip()
            if grip_ok:
                logger.info(f"Grip verified: {grip_reason}")
            else:
                logger.warning(
                    f"GRIP NOT DETECTED ({grip_reason}). Object may have been "
                    "missed, dropped, or is too light for the RG2 grip bit. "
                    "Lifting anyway (empty lift is harmless); do not rely on this pick."
                )
        else:
            time.sleep(0.5)  # verify를 끄면 안착 대기만.

        # 파지 후 상승(진입 후 실제 포즈 기준 수직 상승).
        grip_pose_raw, _ = get_current_posx(ref=DR_BASE)
        lift_pose = [float(v) for v in grip_pose_raw[:6]]
        lift_pose[2] += retreat_z
        logger.info(f"Lifting {retreat_z:.1f} mm after grip: {lift_pose}")
        move_abs(lift_pose, "lift after grip")

        final_pose, _ = get_current_posx(ref=DR_BASE)
        logger.info(
            f"Grip complete at point {contact_point_index} (grip_ok={grip_ok}: "
            f"{grip_reason}); final TCP pose={list(final_pose)}"
        )

    except KeyboardInterrupt:
        logger.warning("Interrupted by user; stopping active motion")
        if fsm["motion_active"]:
            request_stop(node, stop_client, DR_QSTOP)
            if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                fsm["motion_active"] = False
    except Exception as exc:
        logger.error(f"Probe-grip aborted: {exc}")
        if fsm["motion_active"]:
            try:
                request_stop(node, stop_client, DR_QSTOP)
                if wait_until_idle(check_motion, DR_STATE_IDLE, timeout_sec=2.0):
                    fsm["motion_active"] = False
            except Exception as stop_exc:
                logger.error(f"Automatic stop request failed: {stop_exc}")
    finally:
        if fsm["compliance_active"]:
            if fsm["motion_active"]:
                logger.error(
                    "Motion state is not confirmed idle; compliance control was "
                    "left enabled. Use the teach pendant to recover."
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
