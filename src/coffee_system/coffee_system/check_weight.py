#!/usr/bin/env python3

import rclpy
import DR_init


# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# TOOL_NAME = "Tool Weight_gripper"
TOOL_NAME = "Tool Weight_1"
TCP_NAME = "GripperDA_v1"

VEL_J = 60.0
ACC_J = 100.0

VEL_X_TRANS = 250.0
VEL_X_ROT = 80.625

ACC_X_TRANS = 1000.0
ACC_X_ROT = 322.5


# =============================================================================
# 힘 측정 설정
# =============================================================================
# 힘 측정(모니터링) 주기 (초)
FORCE_SAMPLE_INTERVAL_SEC = 0.2


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "m0609_check_weight",
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_robot_mode,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            movej,
            wait,
            # get_tool_force,
            # DR_TOOL,
            reset_workpiece_weight,
            get_workpiece_weight,
            DR_MV_RA_DUPLICATE,
            ROBOT_MODE_AUTONOMOUS,
        )

        from DR_common2 import posj

    except ImportError as error:
        node.get_logger().error(
            f"두산 로봇 모듈 import 실패: {error}"
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        return

    # =========================================================================
    # 측정 자세
    # 플레이트를 이미 파지한 상태에서 joint3만 90도, 나머지 축은 0도
    # =========================================================================
    measure_pose = posj([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])

    # =========================================================================
    # 모션 함수
    # =========================================================================
    def move_joint(target):
        movej(
            target,
            vel=VEL_J,
            acc=ACC_J,
            radius=0.0,
            ra=DR_MV_RA_DUPLICATE,
        )

    try:
        # =====================================================================
        # 초기 설정
        # =====================================================================
        node.get_logger().info("플레이트 무게 모니터링 시작")

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        set_tool(TOOL_NAME)
        set_tcp(TCP_NAME)

        set_velj(VEL_J)
        set_accj(ACC_J)

        set_velx(
            VEL_X_TRANS,
            VEL_X_ROT,
        )

        set_accx(
            ACC_X_TRANS,
            ACC_X_ROT,
        )

        # =====================================================================
        # 측정 자세로 이동
        # 플레이트를 파지한 상태 그대로 이동 (그리퍼 동작 없음)
        # =====================================================================
        move_joint(measure_pose)

        node.get_logger().info("측정 자세 도달.")

        # =====================================================================
        # 워크피스 무게 측정 알고리즘 초기화 (현재 상태 = 빈 플레이트를 0으로 영점)
        # =====================================================================
        reset_workpiece_weight()

        node.get_logger().info(
            "영점 완료. 실시간 무게 출력 시작 (Ctrl+C로 종료)"
        )

        # =====================================================================
        # 실시간 모니터링
        # =====================================================================
        while rclpy.ok():
            # force = get_tool_force(ref=DR_TOOL)
            #
            # if not force or len(force) < 6:
            #     raise RuntimeError(
            #         "get_tool_force 응답이 올바르지 않습니다."
            #     )
            #
            # fx, fy, fz, mx, my, mz = force
            #
            # node.get_logger().info(
            #     f"Fx={fx:.2f}N, Fy={fy:.2f}N, Fz={fz:.2f}N, "
            #     f"Mx={mx:.2f}Nm, My={my:.2f}Nm, Mz={mz:.2f}Nm"
            # )

            weight_kg = get_workpiece_weight()

            node.get_logger().info(
                f"측정 무게={weight_kg:.3f}kg ({weight_kg * 1000:.1f}g)"
            )

            wait(FORCE_SAMPLE_INTERVAL_SEC)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 프로그램 정지")

    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()