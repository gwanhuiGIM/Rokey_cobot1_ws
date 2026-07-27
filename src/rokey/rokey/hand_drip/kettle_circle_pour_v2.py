#!/usr/bin/env python3
"""지정한 DR_BASE 포즈에서 주전자를 기울이고 나선 궤적을 실행한다.

TCP와 Tool은 로봇에 현재 활성화된 설정을 그대로 사용한다.
"""

import math

import numpy as np
import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 주전자를 세운 상태의 접근 포즈, ref=DR_BASE, TCP=현재 활성 TCP
LEVEL_START_POS = [
    375.0383,
    70.9099,
    232.0346,
    87.8106,
    101.1609,
    90.9382,
]

# 실제 물이 나오는 자세에서 주둥이 끝이 필터 중심에 맞는 티칭 포즈
POUR_CENTER_POS = [
    391.1458,
    71.9224,
    232.0133,
    87.8128,
    101.1582,
    111.0334,
]

TILT_VEL_TRANS_MM_S = 20.0
TILT_VEL_ROT_DEG_S = 6.0
TILT_ACC_TRANS_MM_S2 = 80.0
TILT_ACC_ROT_DEG_S2 = 20.0

# 현재 활성 TCP 기준 주전자 주둥이 오프셋과 나선 중 추가 기울임
SPOUT_OFFSET_MM = [40.0, 170.0, 0.0]
SPIRAL_EXTRA_TILT_DEG = 30.0
# 나선 시작부터 끝까지 DR_BASE X 방향으로 누적 이동하는 거리
SPIRAL_X_DRIFT_MM = +150.0

# 나선 설정
SPIRAL_RADIUS_MM = 40.0
SPIRAL_REVS = 3
# movesx 최대 100점 이하: 바깥 48점 + 안쪽 48점 = 총 96점
SPIRAL_STEPS_PER_REV = 16
# 나선 1회전마다 기준 Z ↔ base +Z 높이 운동을 반복한다.
# 0.0으로 설정하면 기존처럼 높이 변화 없이 수평 나선만 실행한다.
SPIRAL_Z_LIFT_MM = 70.0
Z_OSCILLATIONS_PER_REV = 1.0 / 3.0

# 나선 회전 속도
SPIRAL_VEL_MM_S = 30.0
APPROACH_VEL_MM_S = 20.0
MOVE_ACC_MM_S2 = 200.0
MOVE_ROT_VEL_DEG_S = 20.0
MOVE_ROT_ACC_DEG_S2 = 80.0


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

    node = rclpy.create_node("kettle_circle_pour_v2", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MVS_VEL_CONST,
            get_tcp,
            movel,
            movesx,
            set_accx,
            set_velx,
        )
        from DR_common2 import posx
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    pour_rotation = zyz_to_matrix(*POUR_CENTER_POS[3:6])
    spout_offset = np.array(SPOUT_OFFSET_MM, dtype=float)
    pour_spout_center = (
        np.array(POUR_CENTER_POS[:3], dtype=float)
        + pour_rotation @ spout_offset
    )

    def move_tilt(pose):
        movel(
            posx(list(pose)),
            vel=[TILT_VEL_TRANS_MM_S, TILT_VEL_ROT_DEG_S],
            acc=[TILT_ACC_TRANS_MM_S2, TILT_ACC_ROT_DEG_S2],
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

    def spiral_point(radius, angle_deg, z_lift, motion_progress):
        """주둥이 궤적을 유지하면서 tool rz 방향 기울임을 점차 증가시킨다."""
        angle_rad = math.radians(angle_deg)
        spout_target = pour_spout_center + np.array([
            radius * math.cos(angle_rad)
            + SPIRAL_X_DRIFT_MM * motion_progress,
            radius * math.sin(angle_rad),
            z_lift,
        ])

        extra_tilt = SPIRAL_EXTRA_TILT_DEG * motion_progress
        tilted_rotation = pour_rotation @ _rz(extra_tilt)
        tcp_target = spout_target - tilted_rotation @ spout_offset

        return posx([
            tcp_target[0],
            tcp_target[1],
            tcp_target[2],
            POUR_CENTER_POS[3],
            POUR_CENTER_POS[4],
            POUR_CENTER_POS[5] + extra_tilt,
        ])

    def z_oscillation(angle_deg):
        """나선 한 바퀴마다 0 ↔ Z 최대 높이를 지정 횟수만큼 왕복한다."""
        phase = math.radians(angle_deg * Z_OSCILLATIONS_PER_REV)
        return SPIRAL_Z_LIFT_MM * 0.5 * (1.0 - math.cos(phase))

    def draw_spiral():
        """같은 회전 방향을 유지하며 바깥으로 갔다가 안쪽으로 복귀한다."""
        point_count = max(1, int(SPIRAL_REVS * SPIRAL_STEPS_PER_REV))
        total_point_count = point_count * 2
        path = []

        # 중심 -> 바깥: 0 ~ SPIRAL_REVS 회전
        for index in range(1, point_count + 1):
            fraction = index / point_count
            angle = 360.0 * SPIRAL_REVS * fraction
            path.append(
                spiral_point(
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
                    SPIRAL_RADIUS_MM * (1.0 - fraction),
                    angle,
                    z_oscillation(angle),
                    (point_count + index) / total_point_count,
                )
            )

        # 점별 movel 대신 전체 경로를 한 번에 보내 컨트롤러에서 스플라인 보간.
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

    try:
        # 좌표를 캡처할 때 사용한 현재 활성 TCP를 그대로 사용한다.
        active_tcp = get_tcp()
        node.get_logger().info(f"현재 활성 TCP 유지: {active_tcp}")

        node.get_logger().info("DR_BASE 기준 시작 포즈로 이동")
        set_velx(APPROACH_VEL_MM_S, MOVE_ROT_VEL_DEG_S)
        set_accx(MOVE_ACC_MM_S2, MOVE_ROT_ACC_DEG_S2)

        movel(
            posx(LEVEL_START_POS),
            vel=APPROACH_VEL_MM_S,
            acc=MOVE_ACC_MM_S2,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        # 계산 기울임 대신 실제 물이 나오는 티칭 포즈를 사용한다.
        node.get_logger().info("티칭된 물 붓기 자세로 천천히 기울이기")
        move_tilt(POUR_CENTER_POS)

        node.get_logger().info(
            f"나선 시작: radius={SPIRAL_RADIUS_MM} mm, "
            f"Z lift={SPIRAL_Z_LIFT_MM} mm, "
            f"Z cycles/rev={Z_OSCILLATIONS_PER_REV}, "
            f"extra tilt={SPIRAL_EXTRA_TILT_DEG} deg, "
            f"X drift={SPIRAL_X_DRIFT_MM:+.1f} mm, "
            f"speed={SPIRAL_VEL_MM_S} mm/s, "
            f"out/in={SPIRAL_REVS} rev"
        )
        draw_spiral()

        node.get_logger().info("나선 완료, 주전자를 다시 세우기")
        move_tilt(LEVEL_START_POS)
        node.get_logger().info("전체 동작 완료")
    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")
    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
