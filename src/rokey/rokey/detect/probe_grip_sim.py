#!/usr/bin/env python3
# =============================================================
# probe_grip_sim.py — probe_grip_v1의 "모션만" 보는 시뮬 전용 버전
# -------------------------------------------------------------
# 실행: ros2 run rokey probe_grip_sim --ros-args -p arm:=true
#       (또는: python3 rokey/detect/probe_grip_sim.py --ros-args -p arm:=true)
# 노드: m0609_probe_grip_sim | 좌표계: DR_BASE
# 목적: 그리퍼 구동 / Z반력 감지 / task compliance 를 전부 제거하고,
#   감지 대신 sim_descend_mm 고정 하강으로 "물체 발견"을 흉내낸 뒤 Phase 2
#   파지 모션(movej → +Y standoff → -Y 수평진입(Z유지) → 상승)만 실행. 궤적 확인용.
# 주의: 이것은 "안전 시뮬"이 아니다. movej/movel을 연결된 로봇에 그대로 보낸다 →
#   반드시 virtual/시뮬 브링업(mode:=virtual)에 연결한 상태에서만 실행할 것.
#   arm:=true 없이는 모션 없음.
# =============================================================
"""probe_grip_v1에서 그리퍼·감지·컴플라이언스를 빼고 로봇 동작만 재생한다.

동작 (sim_target_point 한 점에 대해서만):
  1. 접근 포즈로 이동(위에서 대기).
  2. sim_descend_mm 만큼 Base -Z로 하강(감지 없음; 물체 발견을 흉내).
  3. 그 포즈를 pseudo-contact로 삼아 Phase 2 파지 모션:
     안전 관절자세로 movej → grasp 접근위치의 +Y standoff(Z 유지) → -Y 수평 진입으로
     grasp 접근위치(+Y offset) 도달 → retreat_z_mm 상승.
"""

import time

import rclpy
import DR_init
from dsr_msgs2.srv import MoveLine


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_probe_grip_sim"

# 접근 포즈 (DR_BASE, 6D; ZYZ 오일러 A,B,C[deg]). probe_grip_v1과 동일한 자리표시자.
PROBE_POINTS = [[250.0, -300.0, 200.0, 90.0, 180.0, 90.0],   # TODO(확정 필요)
    [300.0, -300.0, 200.0, 90.0, 180.0, 90.0],     # TODO(확정 필요)
    [350.0, -300.0, 200.0, 90.0, 180.0, 90.0],  # TODO(확정 필요)
]
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    logger = node.get_logger()

    # -- 파라미터 (probe_grip_v1의 모션 관련만; 그리퍼·감지 파라미터는 없음) -----
    node.declare_parameter("arm", False)             # 미장전 시 모션 없음
    node.declare_parameter(
        "points", [v for point in PROBE_POINTS for v in point]
    )
    node.declare_parameter("sim_target_point", 1)    # 몇 번째 점을 파지 시연할지(1-base)
    node.declare_parameter("speed_mm_s", 40.0)       # mm/s
    node.declare_parameter("acc_mm_s2", 40.0)        # mm/s^2
    # 감지 대신 고정 하강으로 "물체 발견"을 흉내. 이 깊이만큼 내려간 지점을 접촉으로 본다.
    node.declare_parameter("sim_descend_mm", 100.0)
    node.declare_parameter("object_height_mm", 140.0)
    node.declare_parameter("grasp_center_frac", 0.25)
    node.declare_parameter("approach_joint", [0.0, 30.0, 120.0, 90.0, -90.0, 45.0])
    node.declare_parameter("joint_speed_deg_s", 30.0)
    node.declare_parameter("joint_acc_deg_s2", 30.0)
    node.declare_parameter("grasp_orientation", [-90.0, 90.0, 90.0])
    # 안전진입: grasp 접근위치의 +Y로 grasp_y_standoff만큼 물러난 곳(Z는 grasp 높이 유지).
    #   거기서 -Y로 수평 진입(Z 유지)해 grasp 접근위치에 도달한다.
    node.declare_parameter("grasp_y_standoff_mm", 150.0)  # +Y 안전진입 오프셋(standoff)
    # grasp 접근위치: 물체 중앙 XY가 아니라 +Y로 grasp_y_offset만큼 떨어진 곳에서 파지.
    node.declare_parameter("grasp_y_offset_mm", 50.0)     # +Y grasp 접근 오프셋
    node.declare_parameter("grasp_z_above_center_mm", 30.0)
    node.declare_parameter("retreat_z_mm", 100.0)

    armed = bool(node.get_parameter("arm").value)
    flat_points = [float(v) for v in node.get_parameter("points").value]
    points = [flat_points[i:i + 6] for i in range(0, len(flat_points), 6)]
    sim_target = int(node.get_parameter("sim_target_point").value)
    speed = float(node.get_parameter("speed_mm_s").value)
    acceleration = float(node.get_parameter("acc_mm_s2").value)
    sim_descend = float(node.get_parameter("sim_descend_mm").value)
    object_height = float(node.get_parameter("object_height_mm").value)
    grasp_center_frac = float(node.get_parameter("grasp_center_frac").value)
    approach_joint = [float(v) for v in node.get_parameter("approach_joint").value]
    joint_speed = float(node.get_parameter("joint_speed_deg_s").value)
    joint_acc = float(node.get_parameter("joint_acc_deg_s2").value)
    grasp_orientation = [
        float(v) for v in node.get_parameter("grasp_orientation").value
    ]
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

    if len(flat_points) < 6 or len(flat_points) % 6 != 0:
        return bail("points must contain at least one flattened 6D TCP pose")
    if sim_target < 1 or sim_target > len(points):
        return bail(f"sim_target_point must be in the range [1, {len(points)}]")
    if speed <= 0.0 or speed > 50.0:
        return bail("speed_mm_s must be in the range (0, 50]")
    if acceleration <= 0.0 or acceleration > 100.0:
        return bail("acc_mm_s2 must be in the range (0, 100]")
    if sim_descend <= 0.0 or sim_descend > 300.0:
        return bail("sim_descend_mm must be in the range (0, 300]")
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

    # 모션 서비스 readiness만 확인(Fast DDS race 회피). TCP 프리셋 설정/검증은
    # 시뮬에선 불필요(가상 브링업엔 교시 TCP가 없어 오히려 막힘) → 전부 제거.
    move_client = node.create_client(MoveLine, "motion/move_line")
    if not move_client.wait_for_service(timeout_sec=5.0):
        logger.warning("motion/move_line not ready after 5s; trying anyway")
    else:
        logger.info("motion service ready")

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

    try:
        logger.warning(
            f"SIM (motion only): target point {sim_target}/{len(points)}, "
            f"descend {sim_descend:.1f} mm to pseudo-contact, then side-grasp motion. "
            "No gripper, no force detection, no compliance."
        )

        point = points[sim_target - 1]

        # 1. 접근 포즈로 이동.
        move_line(point, f"approach point {sim_target}")

        # 2. 감지 대신 sim_descend 고정 하강 → pseudo-contact.
        contact = list(point)
        contact[2] -= sim_descend
        move_line(contact, "descend (sim pseudo-contact)")

        # 3. pseudo-contact 포즈 = 방금 하강한 목표(시뮬이라 실제와 동일; 되읽기 불필요).
        contact_pose = list(contact)
        cx, cy, cz_top = contact_pose[0], contact_pose[1], contact_pose[2]
        logger.info(f"pseudo-contact pose={contact_pose}")

        # 물체 위로 상승(재배향 스윕 전 물체와의 간섭 회피).
        clear_pose = list(contact_pose)
        clear_pose[2] += retreat_z
        move_line(clear_pose, "retreat above object")

        # 안전 관절자세로 movej → 수평 재배향.
        move_joint(approach_joint, "movej horizontal posture")

        # 파지 높이 = 물체 중앙 + grasp_z_above_center (수평 그리퍼-바닥 충돌 방지).
        object_center_z = cz_top - object_height * grasp_center_frac
        grasp_z = object_center_z + grasp_z_above_center

        # 안전진입: grasp 접근위치의 +Y로 grasp_y_standoff 물러난 곳(Z=grasp 높이 유지).
        entry_pose = [cx, cy + grasp_y_standoff, grasp_z] + grasp_orientation
        move_line(entry_pose, "safe entry at +Y standoff (Z held)")

        # grasp 접근위치로 -Y 수평 진입(Z 유지). 물체 중앙이 아니라 +Y grasp_y_offset 지점.
        grasp_pose = [cx, cy + grasp_y_offset, grasp_z] + grasp_orientation
        move_line(grasp_pose, "approach grip at +Y offset (enter -Y, Z held)")

        # (그리퍼 close 자리 — 시뮬에서는 생략)
        logger.info("SIM: gripper close would happen here (skipped)")

        # 상승.
        lift_pose = list(grasp_pose)
        lift_pose[2] += retreat_z
        move_line(lift_pose, "lift after grip")

        logger.info("SIM COMPLETE (motion only)")

    except KeyboardInterrupt:
        logger.warning("Interrupted by user")
    except Exception as exc:
        logger.error(f"Sim aborted: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
