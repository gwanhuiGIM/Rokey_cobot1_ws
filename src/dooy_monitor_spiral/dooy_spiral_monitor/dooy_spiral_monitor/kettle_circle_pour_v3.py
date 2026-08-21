#!/usr/bin/env python3
"""
핸드드립 커피 - 주전자 잡기 + 나선 붓기 + 내려놓기 (m0609, 올인원 v3)

kettle_circle_pour_v1.py 의 "손잡이 원호 접근 / 그리퍼 제어 / TCP·Tool 전환"과
kettle_circle_pour_v2.py 의 "티칭 포즈 기반 기울임 + 나선 붓기"를 하나로 합친
버전이다. kettle_circle_pour_v3_sub.drl(펜던트 Sub Program)과 동일한 로직을
ROS2에서 그대로 개발/테스트할 수 있도록 만들었다.

동작 순서
    1. 진입 시 활성 TCP/Tool 저장
    2. 이 작업 전용 TCP/Tool(TOOL_NAME/TCP_NAME)로 전환
    3. 안전 조인트 포즈(KETTLE_APPROACH_J)로 moveJ -> 그리퍼 열기 ->
       손잡이 파지 포즈(KETTLE_GRIP_POS)로 moveL -> 그리퍼 닫기 -> 들어올리기
    4. 붓기 대기 조인트 포즈(AFTER_POUR_J)로 moveJ -> 붓기 조인트 포즈
       (POUR_CENTER_J)로 천천히 moveJ(기울이기) -> 나선/moveC 붓기 ->
       AFTER_POUR_J로 천천히 복귀
    5. KETTLE_APPROACH_J로 moveJ -> 내려놓을 위치로 moveL -> 그리퍼 열기 ->
       KETTLE_APPROACH_J로 다시 moveJ (안전 위치 후퇴)
    6. 진입 시 저장해둔 TCP/Tool로 복귀 (오류가 나도 finally 에서 반드시 복귀)

KETTLE_APPROACH_J(potGripJoint)/KETTLE_GRIP_POS(potGrip)/AFTER_POUR_J
(afterPoring)/POUR_CENTER_J(pooringPoint)는 전부 재티칭된 최신 값이다.
POUR_CENTER_J는 조인트 포즈라 실제 붓기에 쓰는 Cartesian 중심(주둥이 오프셋
계산, moveC 원)은 그 자세에 도착한 뒤 get_current_posx()로 실시간으로 읽어
사용한다. TOOL_NAME/TCP_NAME 은 v1에서 그대로 가져온 값이라 실제 등록된
이름과 다르면 교체해야 한다.

붓기 방식은 소스의 POUR_METHOD 상수(기본 "movec")로 정하거나, 실행 시
--movec 플래그로 그 자리에서 movec 방식으로 덮어쓸 수 있다:
    ros2 run rokey kettle_circle_pour_v3            # POUR_METHOD 그대로
    ros2 run rokey kettle_circle_pour_v3 --movec     # movec 강제
(.drl 은 펜던트 Play 버튼으로 실행되어 실행 인자가 없으므로, 그쪽은
POUR_METHOD 상수를 직접 고쳐야 한다.)
"""

import argparse
import math
import sys

import numpy as np
import rclpy
import DR_init


# ROS2 네임스페이스와 DSR 제어 대상에 사용하는 로봇 ID.
ROBOT_ID = "dsr01"
# DSR_ROBOT2가 로드할 두산 로봇 모델명.
ROBOT_MODEL = "m0609"

# ===== 로봇/그리퍼 설정 (v1) =====
# 제어기에 등록된 주전자 작업용 Tool(중량/무게중심) 설정 이름.
TOOL_NAME = "Tool Weight_1"
# 제어기에 등록된 그리퍼 TCP(툴 끝점) 설정 이름.
TCP_NAME = "GripperDA_v1"

# ============================================================
# ===== 좌표/위치 파라미터 =====
# ============================================================

# --- 주전자 잡기/내려놓기 (재티칭) ---
# potGripJoint: 주전자 잡기 전 + 내려놓기 전(후퇴 지점 겸용) 안전 조인트
# 포즈 [J1..J6] deg.
KETTLE_APPROACH_J = [-35.300, 47.640, 100.500, 122.62, 66.81, -238.08]
# potGrip: 주전자 손잡이를 실제로 잡는(그리퍼 닫는) Cartesian 포즈
# [X, Y, Z(mm), A, B, C(deg)], ref=DR_BASE.
KETTLE_GRIP_POS = [822.640, -190.910, 72.710, 13.73, 95.72, -88.70]
# 파지 직후 확보하는 수직 안전 높이(mm).
KETTLE_LIFT_MM = 120.0
# 다 붓고 내려놓을 포즈. KETTLE_GRIP_POS와 X/Y/자세는 같고 Z만 +20mm 높게
# (잡을 때는 potGrip이 맞지만 내려놓을 때는 그대로 놓으면 낮아서 보정).
KETTLE_PLACE_POS = [822.640, -190.910, 92.710, 13.73, 95.72, -88.70]

# --- 붓기 조인트 경유점 (재티칭) ---
# afterPoring: 잡고 나서 붓기 전에 들르고, 붓기 끝나고 내려놓기 전에도 다시
# 들르는 대기 조인트 포즈 [J1..J6] deg.
AFTER_POUR_J = [-31.930, 50.320, 26.430, 77.87, 102.74, -171.32]
# pooringPoint: moveC(붓기)가 시작되는 기울인 조인트 포즈 [J1..J6] deg.
POUR_CENTER_J = [10.490, 41.710, 29.500, 14.37, 94.72, -178.78]

# --- 붓기 궤적 형상 ---
# 나선/moveC 원 반지름(mm).
SPIRAL_RADIUS_MM = 40.0
# spiral: 바깥쪽 확장 구간과 안쪽 복귀 구간에서 각각 도는 회전 수.
# movec: 원을 반복할 바퀴(랩) 수.
SPIRAL_REVS = 3
# movesx 최대 100점 이하: 바깥 48점 + 안쪽 48점 = 총 96점
# spiral 한 바퀴를 근사하는 경유점 수. 높이면 부드러워지지만 총 점 수가 증가.
SPIRAL_STEPS_PER_REV = 13
# spiral Z 오실레이션의 최저점 대비 최대 상승 높이(mm).
SPIRAL_Z_LIFT_MM = 20.0
# spiral 나선 1회전당 Z 오실레이션 주기 수(현재 값은 3회전에 1주기).
Z_OSCILLATIONS_PER_REV = 1.0 / 3.0
# POUR_CENTER_J 자세에서 TCP→주전자 주둥이의 로컬 [X,Y,Z] 오프셋(mm). spiral
# 방식에서만 쓰인다(movec는 자세 고정이라 불필요).
SPOUT_OFFSET_MM = [0.0, 60.0, 0.0]
# 나선 전체 진행 동안 붓기 자세의 C(Rz)에 누적할 추가 기울기(deg).
SPIRAL_EXTRA_TILT_DEG = -30.0
# 진행률 0→1에 비례해 누적하는 베이스 X축 궤적 보정량(mm).
SPIRAL_X_DRIFT_MM = 0.0
# 나선이 진행되며 중심에서 벗어나는 걸 상쇄하는 보정값(motion_progress에
# 비례해 누적). 현재 SPOUT_OFFSET_MM 부정확으로 인한 중심 이탈 문제 조사 중.
SPIRAL_Y_DRIFT_MM = 0.0

# ============================================================
# ===== 속도/가속도 파라미터 =====
# ============================================================

# --- 잡기/내려놓기 직선(moveL) ---
# 직선 이동의 병진 속도(mm/s). moveL 및 set_velx의 첫 번째 값.
VEL_X_TRANS = 20.0
# 직선 이동의 회전 속도(deg/s). set_velx의 두 번째 값.
VEL_X_ROT = 30.0
# 직선 이동의 병진 가속도(mm/s^2).
ACC_X_TRANS = 100.0
# 직선 이동의 회전 가속도(deg/s^2).
ACC_X_ROT = 80.0

# --- 조인트(moveJ) 경유 이동 ---
# potGripJoint/afterPoring 등 일반 경유점 이동에 사용.
VEL_J = 30.0
ACC_J = 60.0
# afterPoring <-> pooringPoint 사이 기울임 이동(moveJ) 속도 - 물 안 튀게 저속.
TILT_VEL_J = 8.0
TILT_ACC_J = 20.0

# --- 나선/moveC 붓기 궤적 ---
# movesx/moveC 붓기 궤적의 병진 속도(mm/s).
SPIRAL_VEL_MM_S = 30.0
# 붓기 접근 및 나선/원 궤적의 병진 가속도(mm/s^2).
MOVE_ACC_MM_S2 = 200.0

# ===== 붓기 방식 선택 =====
# 붓기 궤적 방식: "spiral"(나선, 점진적 기울임/Z 오실레이션 - 현재 중심
# 이탈 문제로 보류 중) | "movec"(자세 고정, moveC로 원을 SPIRAL_REVS바퀴
# 반복 - 현재 기본값)
POUR_METHOD = "movec"


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def _rz(deg):
    rad = math.radians(deg)
    cos_value, sin_value = math.cos(rad), math.sin(rad)
    return np.array([
        [cos_value, -sin_value, 0.0],
        [sin_value, cos_value, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _ry(deg):
    rad = math.radians(deg)
    cos_value, sin_value = math.cos(rad), math.sin(rad)
    return np.array([
        [cos_value, 0.0, sin_value],
        [0.0, 1.0, 0.0],
        [-sin_value, 0.0, cos_value],
    ])


def zyz_to_matrix(a, b, c):
    """Doosan posx의 ZYZ 오일러 자세를 회전 행렬로 변환한다."""
    return _rz(a) @ _ry(b) @ _rz(c)


def main(args=None):
    rclpy.init(args=args)

    cli_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parser = argparse.ArgumentParser(
        description="핸드드립 주전자 잡기 + 나선/moveC 붓기 + 내려놓기"
    )
    parser.add_argument(
        "--movec", action="store_true",
        help="붓기를 나선(spiral) 대신 moveC 원 반복 방식으로 실행 (기본: movec)",
    )
    parsed_args, _ = parser.parse_known_args(cli_args)
    pour_method = "movec" if parsed_args.movec else POUR_METHOD

    node = rclpy.create_node("kettle_circle_pour_v3", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            DR_AVOID,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MVS_VEL_CONST,
            ON,
            OFF,
            get_current_posx,
            get_tcp,
            get_tool,
            movec,
            movej,
            movel,
            movesx,
            set_accj,
            set_accx,
            set_digital_output,
            set_singularity_handling,
            set_tcp,
            set_tool,
            set_velj,
            set_velx,
            wait,
        )
        from DR_common2 import posj, posx
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # ===== 그리퍼 (F_3 모듈과 동일한 digital output 방식, v1) =====
    def gripper_open():
        # 완전 개방(1,2)=(ON,ON). 주전자 잡기 전 안전 위치(potGripJoint,
        # 손잡이와 거리가 있는 자세)에서만 사용 - 손잡이를 완전히 감쌀 만큼
        # 넓게 벌려야 잡힌다.
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(0.5)

    def gripper_open_narrow():
        # (1,2)=(OFF,ON) 완전 개방은 손잡이 근처에서 걸리는 문제가 있어
        # 도입했던, 주전자 잡기에 맞게 미리 조절해둔 폭 프리셋. 내려놓기
        # 단계에서 그대로 사용한다.
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        wait(0.5)

    def gripper_close():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    # ===== 잡기/내려놓기 모션 헬퍼 =====
    def move_linear_abs(pose):
        movel(posx(list(pose)), vel=VEL_X_TRANS, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    def move_linear_rel(delta):
        """현재 자세 기준 상대 이동(위치만 변경, 평행 유지)."""
        movel(posx(delta), vel=VEL_X_TRANS, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

    def move_joint(joint_pose):
        movej(posj(list(joint_pose)), vel=VEL_J, acc=ACC_J)

    def move_joint_tilt(joint_pose):
        """붓기 자세로 천천히 기울이거나 되돌릴 때 사용(물 안 튀게 저속)."""
        movej(posj(list(joint_pose)), vel=TILT_VEL_J, acc=TILT_ACC_J)

    # ===== 붓기 모션 헬퍼 (v2, pour_center_pose는 실행 중 채워짐) =====
    def spiral_point(pour_rotation, spout_offset, pour_spout_center,
                      pour_center_pose, radius, angle_deg, z_lift,
                      motion_progress):
        """주둥이 궤적을 유지하면서 tool rz 방향 기울임을 점차 증가시킨다."""
        angle_rad = math.radians(angle_deg)
        spout_target = pour_spout_center + np.array([
            radius * math.cos(angle_rad)
            + SPIRAL_X_DRIFT_MM * motion_progress,
            radius * math.sin(angle_rad)
            + SPIRAL_Y_DRIFT_MM * motion_progress,
            z_lift,
        ])

        extra_tilt = SPIRAL_EXTRA_TILT_DEG * motion_progress
        tilted_rotation = pour_rotation @ _rz(extra_tilt)
        tcp_target = spout_target - tilted_rotation @ spout_offset

        return posx([
            tcp_target[0],
            tcp_target[1],
            tcp_target[2],
            pour_center_pose[3],
            pour_center_pose[4],
            pour_center_pose[5] + extra_tilt,
        ])

    def z_oscillation(angle_deg):
        """나선 한 바퀴마다 0 ↔ Z 최대 높이를 지정 횟수만큼 왕복한다."""
        phase = math.radians(angle_deg * Z_OSCILLATIONS_PER_REV)
        return SPIRAL_Z_LIFT_MM * 0.5 * (1.0 - math.cos(phase))

    def draw_spiral(pour_center_pose):
        """같은 회전 방향을 유지하며 바깥으로 갔다가 안쪽으로 복귀한다."""
        pour_rotation = zyz_to_matrix(*pour_center_pose[3:6])
        spout_offset = np.array(SPOUT_OFFSET_MM, dtype=float)
        pour_spout_center = (
            np.array(pour_center_pose[:3], dtype=float)
            + pour_rotation @ spout_offset
        )

        point_count = max(1, int(SPIRAL_REVS * SPIRAL_STEPS_PER_REV))
        total_point_count = point_count * 2
        path = []

        # 중심 -> 바깥: 0 ~ SPIRAL_REVS 회전
        for index in range(1, point_count + 1):
            fraction = index / point_count
            angle = 360.0 * SPIRAL_REVS * fraction
            path.append(
                spiral_point(
                    pour_rotation, spout_offset, pour_spout_center,
                    pour_center_pose,
                    SPIRAL_RADIUS_MM * fraction,
                    angle,
                    z_oscillation(angle),
                    index / total_point_count,
                )
            )

        # 바깥 -> 중심: 회전 방향을 뒤집지 않고 계속 회전
        for index in range(1, point_count + 1):
            fraction = index / point_count
            angle = 360.0 * SPIRAL_REVS * (1.0 + fraction)
            path.append(
                spiral_point(
                    pour_rotation, spout_offset, pour_spout_center,
                    pour_center_pose,
                    SPIRAL_RADIUS_MM * (1.0 - fraction),
                    angle,
                    z_oscillation(angle),
                    (point_count + index) / total_point_count,
                )
            )

        result = movesx(
            path,
            vel=SPIRAL_VEL_MM_S,
            acc=MOVE_ACC_MM_S2,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            vel_opt=DR_MVS_VEL_CONST,
        )
        if result != 0:
            raise RuntimeError("나선 movesx 실행 실패")

    def circle_point(center, radius, angle_deg):
        angle_rad = math.radians(angle_deg)
        return posx([
            center[0] + radius * math.cos(angle_rad),
            center[1] + radius * math.sin(angle_rad),
            center[2],
            center[3],
            center[4],
            center[5],
        ])

    def draw_circles_movec(pour_center_pose):
        """pour_center_pose(붓기 조인트 포즈에서 읽은 현재 Cartesian 자세)를
        중심으로 자세 고정한 채 moveC로 원을 SPIRAL_REVS 바퀴 반복한다.
        자세가 고정이라 주둥이 오프셋도 고정 벡터가 되어, 이 원의 반지름이
        그대로 주둥이가 그리는 원의 반지름이 된다(SPOUT_OFFSET_MM 불필요)."""
        center = pour_center_pose
        radius = SPIRAL_RADIUS_MM
        via = circle_point(center, radius, 120.0)
        end = circle_point(center, radius, 240.0)
        movel(circle_point(center, radius, 0.0),
              vel=SPIRAL_VEL_MM_S, acc=MOVE_ACC_MM_S2,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)
        for _ in range(SPIRAL_REVS):
            # angle=[360, 0]: pos1/pos2로 원의 평면/반지름만 잡고 360도 풀 원을
            # 한 번의 movec 호출로 그린다.
            movec(via, end, vel=SPIRAL_VEL_MM_S, acc=MOVE_ACC_MM_S2,
                  ref=DR_BASE, mod=DR_MV_MOD_ABS, angle=[360, 0])
        movel(posx(list(center)), vel=SPIRAL_VEL_MM_S, acc=MOVE_ACC_MM_S2,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    # =========================================================================
    # 실행: 진입 TCP/Tool 저장 -> 작업용으로 전환 -> 잡기/붓기/내려놓기 -> 복귀
    # =========================================================================
    original_tcp = get_tcp()
    original_tool = get_tool()
    node.get_logger().info(
        f"진입 시 TCP/Tool 저장: tcp={original_tcp}, tool={original_tool}"
    )

    try:
        set_tool(TOOL_NAME)
        set_tcp(TCP_NAME)
        set_singularity_handling(DR_AVOID)
        set_velj(VEL_J)
        set_accj(ACC_J)
        set_velx(VEL_X_TRANS, VEL_X_ROT)
        set_accx(ACC_X_TRANS, ACC_X_ROT)

        node.get_logger().info("1) 손잡이 접근 위치로 이동 후 그리퍼 열기")
        move_joint(KETTLE_APPROACH_J)
        gripper_open()

        node.get_logger().info("1) 손잡이 최종 위치로 이동 후 잡기")
        move_linear_abs(KETTLE_GRIP_POS)
        gripper_close()
        move_linear_rel([0.0, 0.0, KETTLE_LIFT_MM, 0.0, 0.0, 0.0])

        node.get_logger().info("2) 붓기 대기 위치로 이동")
        move_joint(AFTER_POUR_J)

        node.get_logger().info("3) 붓기 자세로 천천히 기울이기")
        move_joint_tilt(POUR_CENTER_J)
        pour_center_pose = list(get_current_posx(DR_BASE)[0])

        node.get_logger().info(f"4) 붓기 시작 (method={pour_method})")
        if pour_method == "movec":
            node.get_logger().info(
                f"   movec: radius={SPIRAL_RADIUS_MM} mm, "
                f"laps={SPIRAL_REVS}, speed={SPIRAL_VEL_MM_S} mm/s"
            )
            draw_circles_movec(pour_center_pose)
        else:
            node.get_logger().info(
                f"   spiral: radius={SPIRAL_RADIUS_MM} mm, "
                f"Z lift={SPIRAL_Z_LIFT_MM} mm, "
                f"extra tilt={SPIRAL_EXTRA_TILT_DEG} deg, "
                f"X drift={SPIRAL_X_DRIFT_MM:+.1f} mm, "
                f"Y drift={SPIRAL_Y_DRIFT_MM:+.1f} mm, "
                f"speed={SPIRAL_VEL_MM_S} mm/s, "
                f"out/in={SPIRAL_REVS} rev"
            )
            draw_spiral(pour_center_pose)

        node.get_logger().info("5) 붓기 완료, 대기 위치로 복귀")
        move_joint_tilt(AFTER_POUR_J)

        node.get_logger().info("6) 내려놓을 위치로 접근")
        move_joint(KETTLE_APPROACH_J)
        place = KETTLE_PLACE_POS if KETTLE_PLACE_POS is not None else KETTLE_GRIP_POS
        move_linear_abs(place)
        gripper_open_narrow()

        node.get_logger().info("6) 안전 위치로 후퇴")
        move_joint(KETTLE_APPROACH_J)

        node.get_logger().info("전체 동작 완료")
    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")
    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")
    finally:
        set_tcp(original_tcp)
        set_tool(original_tool)
        node.get_logger().info(
            f"TCP/Tool 복귀 완료: tcp={original_tcp}, tool={original_tool}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
