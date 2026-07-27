#!/usr/bin/env python3
# =============================================================
# probe_grip_sim_v2.py — X-직진 탐지 방식 "모션만" 보는 시뮬 전용 버전
# -------------------------------------------------------------
# 실행: ros2 run rokey probe_grip_sim_v2 --ros-args -p arm:=true
#       (또는: python3 rokey/detect/probe_grip_sim_v2.py --ros-args -p arm:=true)
# 노드: m0609_probe_grip_sim_v2 | 좌표계: DR_BASE
# 목적: probe_grip_sim(위→아래 Z탐지) 대신, 감지 준비위치에서 base +X로 직진하며
#   물체를 감지하는 전략. 힘 감지/컴플라이언스는 없고 sim_probe_x_mm 고정 전진으로
#   감지를 흉내낸다. 시작 시 그리퍼만 닫는다(툴 끝 오차 축소). 궤적 확인용.
# 흐름: movej 안전자세 → 그리퍼 닫기 → 감지 준비위치 → +X 직진(감지 흉내) →
#   X로 물체반경 오프셋 → +Y standoff에서 -Y 안전진입(Z 유지) → (그립) → +Z 상승.
# 주의: "안전 시뮬"이 아니다. movej/movel을 연결된 로봇에 그대로 보낸다 →
#   반드시 virtual/시뮬 브링업(mode:=virtual)에서만 실행. arm:=true 없이는 모션 없음.
# =============================================================
"""안전자세 → 감지 준비위치 → base +X 직진 탐지 → 측면 파지 모션만 재생한다.

동작:
  1. 특이점 회피 안전 관절자세 approach_joint로 movej. 시작 시 그리퍼 닫기(오차 축소).
  2. 감지 준비위치 detection_ready_xyz(파지와 같은 grasp_orientation)로 이동.
  3. base +X로 sim_probe_x_mm 직진(감지 흉내 → pseudo-contact).
  4. X로 object_radius_mm 오프셋(파지 깊이) → grasp X 확정.
  5. +Y grasp_y_standoff 안전진입 후 -Y로 grasp_y_offset 지점까지 수평 진입(Z 유지).
     그 뒤 (그립) → retreat_z_mm 만큼 +Z 상승.
"""

import rclpy
import DR_init
from dsr_msgs2.srv import MoveLine
from onrobot_rg_msgs.srv import SetCommand


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_probe_grip_sim_v2"
GRIPPER_SERVICE = "/onrobot/sendCommand"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    # -- 파라미터 (모션 관련만; 그리퍼·감지 파라미터는 없음) -------------------
    node.declare_parameter("arm", False)             # 미장전 시 모션 없음
    node.declare_parameter("speed_mm_s", 40.0)       # mm/s
    node.declare_parameter("acc_mm_s2", 40.0)        # mm/s^2
    # approach_joint: 특이점 회피용 안전 시작 관절자세(deg, J1..J6).
    node.declare_parameter("approach_joint", [0.0, 0.0, 90.0, 90.0, -90.0, 0.0])
    node.declare_parameter("joint_speed_deg_s", 30.0)
    node.declare_parameter("joint_acc_deg_s2", 30.0)
    # 파지·감지 공통 손목 자세(ZYZ A,B,C[deg]). 감지 준비위치와 파지가 같은 자세를 쓴다.
    node.declare_parameter("grasp_orientation", [-90.0, 90.0, 90.0])
    # 감지 준비위치(base x,y,z[mm]). 여기로 이동 후 +X로 직진하며 물체를 감지한다.
    node.declare_parameter("detection_ready_xyz", [250.0, -300.0, 150.0])
    # 시작 시 그리퍼를 닫아 툴 끝 오차를 줄인다(닫힌 강체로 감지·진입).
    node.declare_parameter("close_gripper_on_start", True)
    # 감지 대신 base +X로 이만큼 직진한 지점을 접촉으로 본다(실기: X 힘 감지로 대체).
    node.declare_parameter("sim_probe_x_mm", 150.0)
    # 파지 깊이: 접촉면에서 X로 물체반경만큼 후퇴(-X)해 grasp X를 잡는다(부호는 실기 튜닝).
    node.declare_parameter("object_radius_mm", 45.0)
    # 안전 진입: grasp 접근위치의 +Y로 grasp_y_standoff 물러난 곳(Z=grasp 높이 유지)에서
    #   -Y로 grasp_y_offset 지점까지 수평 진입. standoff > offset 이어야 -Y 진입.
    node.declare_parameter("grasp_y_standoff_mm", 300.0)  # +Y 안전진입 오프셋(standoff)
    node.declare_parameter("grasp_y_offset_mm", 50.0)     # +Y grasp 접근 오프셋
    node.declare_parameter("grasp_z_above_center_mm", 0.0)  # 접촉 높이 대비 +Z(바닥충돌 방지)
    node.declare_parameter("retreat_z_mm", 100.0)         # 파지 후 +Z 상승

    armed = bool(node.get_parameter("arm").value)
    speed = float(node.get_parameter("speed_mm_s").value)
    acceleration = float(node.get_parameter("acc_mm_s2").value)
    approach_joint = [float(v) for v in node.get_parameter("approach_joint").value]
    joint_speed = float(node.get_parameter("joint_speed_deg_s").value)
    joint_acc = float(node.get_parameter("joint_acc_deg_s2").value)
    grasp_orientation = [
        float(v) for v in node.get_parameter("grasp_orientation").value
    ]
    detection_ready_xyz = [
        float(v) for v in node.get_parameter("detection_ready_xyz").value
    ]
    close_gripper_on_start = bool(
        node.get_parameter("close_gripper_on_start").value
    )
    sim_probe_x = float(node.get_parameter("sim_probe_x_mm").value)
    object_radius = float(node.get_parameter("object_radius_mm").value)
    grasp_y_standoff = float(node.get_parameter("grasp_y_standoff_mm").value)
    grasp_y_offset = float(node.get_parameter("grasp_y_offset_mm").value)
    grasp_z_above_center = float(
        node.get_parameter("grasp_z_above_center_mm").value
    )
    retreat_z = float(node.get_parameter("retreat_z_mm").value)

    # -- 파라미터 검증 --------------------------------------------------------
    def bail(message):
        logger.error(message)
        node.destroy_node()
        rclpy.shutdown()

    if speed <= 0.0 or speed > 50.0:
        return bail("speed_mm_s must be in the range (0, 50]")
    if acceleration <= 0.0 or acceleration > 100.0:
        return bail("acc_mm_s2 must be in the range (0, 100]")
    if len(approach_joint) != 6:
        return bail("approach_joint must contain six joint angles (deg)")
    if len(grasp_orientation) != 3:
        return bail("grasp_orientation must contain three ZYZ euler values")
    if len(detection_ready_xyz) != 3:
        return bail("detection_ready_xyz must contain three values (x, y, z)")
    if joint_speed <= 0.0 or joint_speed > 90.0:
        return bail("joint_speed_deg_s must be in the range (0, 90]")
    if joint_acc <= 0.0 or joint_acc > 180.0:
        return bail("joint_acc_deg_s2 must be in the range (0, 180]")
    if sim_probe_x <= 0.0 or sim_probe_x > 400.0:
        return bail("sim_probe_x_mm must be in the range (0, 400]")
    if object_radius < 0.0 or object_radius > 200.0:
        return bail("object_radius_mm must be in the range [0, 200]")
    if grasp_y_standoff <= 0.0 or grasp_y_standoff > 400.0:
        return bail("grasp_y_standoff_mm must be in the range (0, 400]")
    if abs(grasp_y_offset) > 300.0:
        return bail("grasp_y_offset_mm magnitude must be <= 300")
    if abs(grasp_z_above_center) > 200.0:
        return bail("grasp_z_above_center_mm magnitude must be <= 200")
    if retreat_z <= 0.0 or retreat_z > 300.0:
        return bail("retreat_z_mm must be in the range (0, 300]")

    if not armed:
        return bail(
            "Robot is NOT armed; no motion was sent. Connect a VIRTUAL bringup "
            "(mode:=virtual), then rerun with --ros-args -p arm:=true"
        )

    # 모션 서비스 readiness만 확인(Fast DDS race 회피). 없으면 경고 후 진행.
    move_client = node.create_client(MoveLine, "motion/move_line")
    if not move_client.wait_for_service(timeout_sec=5.0):
        logger.warning("motion/move_line not ready after 5s; trying anyway")
    else:
        logger.info("motion service ready")

    # 그리퍼 서비스(선택): 시작 시 닫기용. 없으면 경고 후 계속(시뮬엔 없을 수 있음).
    gripper_client = node.create_client(SetCommand, GRIPPER_SERVICE)
    gripper_ready = gripper_client.wait_for_service(timeout_sec=3.0)
    if not gripper_ready:
        logger.warning(f"{GRIPPER_SERVICE} unavailable; start-close will be skipped")

    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_MV_MOD_ABS,
            movej,
            movel,
        )
        from DR_common2 import posj, posx
    except ImportError as exc:
        return bail(f"Failed to import Doosan API: {exc}")

    def move_line(pose, label):
        """직선(movel) 절대이동. DSR movel을 base·ABS 고정으로 감쌈. 거부 시 예외."""
        logger.info(f"[move_line] {label}: {pose}")
        if movel(
            posx(pose), vel=[speed, 50.0], acc=[acceleration, 10.0],
            ref=DR_BASE, mod=DR_MV_MOD_ABS,
        ) != 0:
            raise RuntimeError(f"{label}: movel was rejected")

    def move_joint(joint_deg, label):
        """관절 절대이동(movej). 거부 시 예외."""
        logger.info(f"[move_joint] {label}: {joint_deg}")
        if movej(posj(joint_deg), vel=joint_speed, acc=joint_acc) != 0:
            raise RuntimeError(f"{label}: movej was rejected")

    def close_gripper():
        """시작 시 그리퍼 닫기(닫힌 강체로 툴 끝 오차 축소). 서비스 없으면 건너뜀."""
        if not (close_gripper_on_start and gripper_ready):
            logger.info("start-close skipped (disabled or gripper unavailable)")
            return
        req = SetCommand.Request()
        req.command = "c"
        future = gripper_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        logger.info("gripper closed at start")

    try:
        logger.warning(
            f"SIM v2 (motion only): movej safe -> detection-ready {detection_ready_xyz}"
            f" -> probe +X {sim_probe_x:.1f}mm -> X offset by radius "
            f"{object_radius:.1f} -> +Y standoff / -Y entry -> +Z lift."
        )

        # 1. 특이점 회피 안전 관절자세로 movej.
        move_joint(approach_joint, "movej safe posture")

        # 2. 시작 시 그리퍼 닫기(닫힌 강체로 오차 축소).
        close_gripper()

        # 3. 감지 준비위치로 이동(파지와 같은 grasp_orientation).
        dx, dy, dz = detection_ready_xyz
        ready_pose = [dx, dy, dz] + grasp_orientation
        move_line(ready_pose, "move to detection-ready")

        # 4. base +X로 직진(감지 흉내) → pseudo-contact. 자세 유지.
        contact_x = dx + sim_probe_x
        contact_pose = [contact_x, dy, dz] + grasp_orientation
        move_line(contact_pose, "probe +X to pseudo-contact")

        # 5. 파지 기하:
        #    X = 접촉면에서 물체반경만큼 후퇴(-X)한 파지 깊이(부호는 실기 튜닝 노브).
        #    Y = 물체 Y(=dy); +Y standoff 에서 -Y 로 grasp_y_offset 지점까지 진입.
        #    Z = 접촉 높이 + grasp_z_above_center(수평 그리퍼-바닥 충돌 방지).
        grasp_x = contact_x + object_radius
        grasp_z = dz + grasp_z_above_center
        entry_pose = [grasp_x, dy + grasp_y_standoff, grasp_z] + grasp_orientation
        grasp_pose = [grasp_x, dy + grasp_y_offset, grasp_z] + grasp_orientation
        move_line(entry_pose, "safe entry at +Y standoff (Z held)")
        move_line(grasp_pose, "approach grip at +Y offset (enter -Y, Z held)")

        # 6. (파지 close 자리 — 시뮬에서는 생략)
        logger.info("SIM: gripper grip-close would happen here (skipped)")

        # 7. +Z 상승.
        lift_pose = list(grasp_pose)
        lift_pose[2] += retreat_z
        move_line(lift_pose, "lift +Z after grip")

        logger.info("SIM v2 COMPLETE (motion only)")

    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
    except Exception as exc:
        logger.error(f"Sim aborted: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
