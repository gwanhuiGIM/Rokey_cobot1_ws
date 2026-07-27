#!/usr/bin/env python3
# =============================================================
# probe_grip_sim_v3.py — v2와 같은 모션을, "스텝 상태머신 + 버튼 재시도"로 실행
# -------------------------------------------------------------
# 실행: ros2 run rokey probe_grip_sim_v3 --ros-args -p arm:=true
# 노드: m0609_probe_grip_sim_v3 | 좌표계: DR_BASE
# 목적: v2는 예외가 나면 그 자리에서 죽는다. v3는 스텝 단위로 쪼개서
#   실패한 스텝에서 멈추고, 사용자가 DI15를 한 번 누르면 그 스텝만 다시 한다.
#   모션 자체(자세·좌표·순서)는 v2와 동일하다.
# 흐름: movej 안전자세 → 그리퍼 닫기 → 감지 준비위치 → +X 직진(감지 흉내) →
#   X로 물체반경 오프셋 → +Y standoff에서 -Y 안전진입(Z 유지) → (그립) → +Z 상승.
# 주의: "안전 시뮬"이 아니다. movej/movel을 연결된 로봇에 그대로 보낸다 →
#   반드시 virtual/시뮬 브링업(mode:=virtual)에서만 실행. arm:=true 없이는 모션 없음.
# -------------------------------------------------------------
# 재시도 설계 (여기가 v2와 다른 전부)
# - 스텝 = (이름, 실행함수, 재시도 전 복귀동작). 상태머신 정의는 steps 리스트뿐.
# - 복귀동작이 있는 스텝은 "실패한 자리에서 그대로 다시 쏘면 위험한" 스텝이다:
#     * 준비위치 이동 실패 -> 관절 안전자세로 되돌린 뒤 재시도
#     * 파지 진입 실패     -> +Y standoff로 물러난 뒤 재진입
#   나머지(직선 전진/수직 상승)는 중간에 멈춰도 같은 직선의 연장이라 그대로 재실행.
# - 실패 판정은 bringup 터미널 로그가 아니라 movel/movej 반환값이다.
#   (movel_cb 로그는 "명령을 보냈다"는 뜻이지 "도착했다"가 아니다.)
# =============================================================

import rclpy
import DR_init
from dsr_msgs2.srv import MoveLine
from onrobot_rg_msgs.srv import SetCommand

from rokey.monitor_pjt.step_runner import RetryButton, run_steps


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_probe_grip_sim_v3"
GRIPPER_SERVICE = "/onrobot/sendCommand"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# (파라미터, 기본값, 허용범위) — v2의 if-검증 나열을 표 하나로 접었다.
LIMITS = (
    ("speed_mm_s", 40.0, (0.1, 50.0)),
    ("acc_mm_s2", 40.0, (0.1, 100.0)),
    ("joint_speed_deg_s", 30.0, (0.1, 90.0)),
    ("joint_acc_deg_s2", 30.0, (0.1, 180.0)),
    ("sim_probe_x_mm", 150.0, (0.1, 400.0)),
    ("object_radius_mm", 45.0, (0.0, 200.0)),
    ("grasp_y_standoff_mm", 300.0, (0.1, 400.0)),
    ("grasp_y_offset_mm", 50.0, (-300.0, 300.0)),
    ("grasp_z_above_center_mm", 0.0, (-200.0, 200.0)),
    ("retreat_z_mm", 100.0, (0.1, 300.0)),
)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    node.declare_parameter("arm", False)              # 미장전 시 모션 없음
    node.declare_parameter("retry_di", 15)            # 재시도 버튼 DI 번호
    node.declare_parameter("approach_joint", [0.0, 0.0, 90.0, 90.0, -90.0, 0.0])
    node.declare_parameter("grasp_orientation", [-90.0, 90.0, 90.0])
    node.declare_parameter("detection_ready_xyz", [250.0, -300.0, 150.0])
    node.declare_parameter("close_gripper_on_start", True)
    for name, default, _range in LIMITS:
        node.declare_parameter(name, default)

    def bail(message):
        logger.error(message)
        node.destroy_node()
        rclpy.shutdown()

    value = {}
    for name, _default, (low, high) in LIMITS:
        value[name] = float(node.get_parameter(name).value)
        if not low <= value[name] <= high:
            return bail(f"{name} must be within [{low}, {high}]")

    approach_joint = [float(v) for v in node.get_parameter("approach_joint").value]
    grasp_orientation = [float(v) for v in node.get_parameter("grasp_orientation").value]
    detection_ready_xyz = [float(v) for v in node.get_parameter("detection_ready_xyz").value]
    if len(approach_joint) != 6:
        return bail("approach_joint must contain six joint angles (deg)")
    if len(grasp_orientation) != 3:
        return bail("grasp_orientation must contain three euler values")
    if len(detection_ready_xyz) != 3:
        return bail("detection_ready_xyz must contain three values (x, y, z)")
    if not bool(node.get_parameter("arm").value):
        return bail(
            "Robot is NOT armed; no motion was sent. Connect a VIRTUAL bringup "
            "(mode:=virtual), then rerun with --ros-args -p arm:=true")

    speed = value["speed_mm_s"]
    acceleration = value["acc_mm_s2"]

    move_client = node.create_client(MoveLine, "motion/move_line")
    if not move_client.wait_for_service(timeout_sec=5.0):
        logger.warning("motion/move_line not ready after 5s; trying anyway")

    gripper_client = node.create_client(SetCommand, GRIPPER_SERVICE)
    gripper_ready = gripper_client.wait_for_service(timeout_sec=3.0)
    if not gripper_ready:
        logger.warning(f"{GRIPPER_SERVICE} unavailable; start-close will be skipped")

    try:
        from DSR_ROBOT2 import DR_BASE, DR_MV_MOD_ABS, movej, movel
        from DR_common2 import posj, posx
    except ImportError as exc:
        return bail(f"Failed to import Doosan API: {exc}")

    def move_line(pose, label):
        """직선(movel) 절대이동. 거부되면 예외 -> 러너가 재시도 대기로 넘어간다."""
        logger.info(f"[move_line] {label}: {pose}")
        if movel(posx(pose), vel=[speed, 50.0], acc=[acceleration, 10.0],
                 ref=DR_BASE, mod=DR_MV_MOD_ABS) != 0:
            raise RuntimeError(f"{label}: movel was rejected")

    def move_joint(joint_deg, label):
        logger.info(f"[move_joint] {label}: {joint_deg}")
        if movej(posj(joint_deg), vel=value["joint_speed_deg_s"],
                 acc=value["joint_acc_deg_s2"]) != 0:
            raise RuntimeError(f"{label}: movej was rejected")

    def close_gripper():
        """시작 시 그리퍼 닫기. v2와 달리 응답 없으면 예외 -> 재시도 대상이 된다."""
        if not (bool(node.get_parameter("close_gripper_on_start").value) and gripper_ready):
            logger.info("start-close skipped (disabled or gripper unavailable)")
            return
        future = gripper_client.call_async(SetCommand.Request(command="c"))
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if not future.done():
            raise RuntimeError("gripper close timed out (3s)")
        logger.info("gripper closed at start")

    # -- 좌표 계산 (v2와 동일) ------------------------------------------------
    dx, dy, dz = detection_ready_xyz
    ready_pose = [dx, dy, dz] + grasp_orientation
    contact_x = dx + value["sim_probe_x_mm"]
    contact_pose = [contact_x, dy, dz] + grasp_orientation
    grasp_x = contact_x + value["object_radius_mm"]
    grasp_z = dz + value["grasp_z_above_center_mm"]
    entry_pose = [grasp_x, dy + value["grasp_y_standoff_mm"], grasp_z] + grasp_orientation
    grasp_pose = [grasp_x, dy + value["grasp_y_offset_mm"], grasp_z] + grasp_orientation
    lift_pose = list(grasp_pose)
    lift_pose[2] += value["retreat_z_mm"]

    def to_safe():
        move_joint(approach_joint, "복귀: 안전 관절자세")

    # -- 상태머신: 이 리스트가 공정의 유일한 정의 ------------------------------
    steps = [
        ("safe_posture", lambda: move_joint(approach_joint, "movej safe posture")),
        ("close_gripper", close_gripper),
        ("detection_ready", lambda: move_line(ready_pose, "move to detection-ready"), to_safe),
        ("probe_x", lambda: move_line(contact_pose, "probe +X to pseudo-contact")),
        ("entry_standoff", lambda: move_line(entry_pose, "safe entry at +Y standoff")),
        ("grasp_approach", lambda: move_line(grasp_pose, "approach grip (enter -Y)"),
         lambda: move_line(entry_pose, "복귀: +Y standoff로 후퇴")),
        ("grip_close", lambda: logger.info("SIM: gripper grip-close would happen here")),
        ("lift", lambda: move_line(lift_pose, "lift +Z after grip")),
    ]

    retry = RetryButton(node, di=int(node.get_parameter("retry_di").value))
    logger.warning(
        f"SIM v3 (motion only): {len(steps)} steps, 실패 시 DI{retry.number} 누르면 재시도")

    try:
        if run_steps(node, steps, retry):
            logger.info("SIM v3 COMPLETE (motion only)")
        else:
            logger.warning("SIM v3 ABORTED (사용자 중단 또는 복귀 실패)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
