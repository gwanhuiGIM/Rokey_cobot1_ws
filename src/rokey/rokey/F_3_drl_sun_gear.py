#!/usr/bin/env python3

import time as pytime

import rclpy
import DR_init


# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight_1"
TCP_NAME = "GripperDA_v1"

CYCLE = 1

VEL_J = 60.0 
ACC_J = 100.0

VEL_X_TRANS = 250.0
VEL_X_ROT = 80.625

ACC_X_TRANS = 1000.0
ACC_X_ROT = 322.5

LIFT_DISTANCE_MM = 150.0


# =============================================================================
# T0 기어 삽입 설정
# =============================================================================
INSERT_FORCE_N = 25.0

# BASE 기준 TCP Z가 35 mm 이하가 되면 삽입 완료
INSERT_SUCCESS_Z_MAX_MM = 30.0

# 회전 진폭
WIGGLE_ANGLE_DEG = 10.0

# 한 번 왕복하는 시간
WIGGLE_PERIOD_SEC = 1.5

# 조건 만족 전까지 충분히 오래 회전하도록 설정
# 100 × 1.5초 = 최대 150초의 비동기 periodic 명령
# 실제로는 조건 만족 시 stop_robot()으로 즉시 정지
WIGGLE_REPEAT = 100

# 회전 시작 직후 잘못 완료 판정되는 것을 막기 위한 최소 시간
MIN_WIGGLE_TIME_SEC = 1.5

# 삽입 실패 시 무한 대기 방지
INSERT_TIMEOUT_SEC = 20.0

# DSR_ROBOT2 조건 함수 반환값
DSR_CONDITION_TRUE = 0
DSR_CONDITION_FALSE = -1


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "m0609_gear",
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = node

    compliance_enabled = False
    force_enabled = False
    periodic_motion_started = False

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_singularity_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            set_ref_coord,
            movej,
            movel,
            amove_periodic,
            set_digital_output,
            wait,
            task_compliance_ctrl,
            set_stiffnessx,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            check_position_condition,
            ON,
            OFF,
            DR_AVOID,
            DR_BASE,
            DR_TOOL,
            DR_AXIS_Z,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MV_RA_DUPLICATE,
            DR_FC_MOD_REL,
            DR_QSTOP,
        )

        from DR_common2 import posj, posx
        from dsr_msgs2.srv import MoveStop

    except ImportError as error:
        node.get_logger().error(
            f"두산 로봇 모듈 import 실패: {error}"
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        return

    # =========================================================================
    # 작업 좌표
    # =========================================================================
    system_t1 = posx([
        525.32,
        -156.54,
        23.33,
        126.12,
        179.56,
        126.68,
    ])

    system_t3 = posx([
        512.11,
        -260.58,
        22.08,
        91.75,
        179.83,
        92.35,
    ])

    system_t2 = posx([
        428.71,
        -197.66,
        23.31,
        68.75,
        179.45,
        69.58,
    ])

    system_t0 = posx([
        488.61,
        -204.43,
        22.33,
        112.80,
        179.54,
        113.58,
    ])

    system_t3_dst = posx([
        393.28,
        -101.28,
        22.55,
        115.49,
        179.53,
        116.68,
    ])

    system_t2_dst = posx([
        289.61,
        -87.20,
        22.52,
        76.68,
        179.68,
        78.21,
    ])

    system_t1_dst = posx([
        354.22,
        -4.61,
        22.80,
        131.46,
        179.78,
        133.10,
    ])

    system_t0_dst = posx([
        345.90,
        -64.66,
        55.51,
        103.22,
        179.55,
        105.41,
    ])

    # =========================================================================
    # 조인트 경유점
    # =========================================================================
    q_t3_approach = posj([
        -25.31,
        25.28,
        68.30,
        -1.98,
        79.07,
        -9.51,
    ])

    q_t3_place_approach = posj([
        -10.08,
        3.25,
        95.71,
        -4.23,
        78.75,
        -9.51,
    ])

    q_t3_exit = posj([
        -13.51,
        8.20,
        93.67,
        -2.40,
        78.66,
        -14.21,
    ])

    q_t1_approach = posj([
        -16.24,
        26.94,
        66.66,
        -1.22,
        85.91,
        -14.21,
    ])

    q_t1_place_approach = posj([
        -3.07,
        0.98,
        101.29,
        1.20,
        78.34,
        -14.21,
    ])

    q_t2_approach = posj([
        -25.24,
        17.03,
        83.23,
        1.10,
        80.81,
        -36.35,
    ])

    q_t2_place_approach = posj([
        -18.67,
        -6.18,
        108.24,
        1.15,
        78.97,
        -29.85,
    ])

    q_t2_exit = posj([
        -17.34,
        -7.85,
        108.01,
        0.34,
        79.73,
        -16.27,
    ])

    q_t0_approach = posj([
        -23.18,
        24.37,
        70.91,
        0.58,
        84.70,
        -22.04,
    ])

    q_t0_place_approach = posj([
        -11.67,
        -0.90,
        101.69,
        0.58,
        79.15,
        -10.60,
    ])

    # =========================================================================
    # 비동기 모션 정지 서비스
    # =========================================================================
    stop_client = node.create_client(
        MoveStop,
        "motion/move_stop",
    )

    def stop_robot(stop_mode=DR_QSTOP):
        if not stop_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(
                f"/{ROBOT_ID}/motion/move_stop 서비스를 찾을 수 없습니다."
            )

        request = MoveStop.Request()
        request.stop_mode = int(stop_mode)

        future = stop_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        result = future.result()

        if result is None:
            raise RuntimeError("MoveStop 서비스 응답이 없습니다.")

        if not result.success:
            raise RuntimeError("MoveStop 서비스 호출 실패")

    # =========================================================================
    # 그리퍼
    # =========================================================================
    def gripper_off():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def gripper_on():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

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

    def move_linear_absolute(target):
        movel(
            target,
            vel=VEL_X_TRANS,
            acc=ACC_X_TRANS,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            ra=DR_MV_RA_DUPLICATE,
        )

    def lift_gear():
        movel(
            posx([
                0.0,
                0.0,
                LIFT_DISTANCE_MM,
                0.0,
                0.0,
                0.0,
            ]),
            vel=VEL_X_TRANS,
            acc=ACC_X_TRANS,
            radius=0.0,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
            ra=DR_MV_RA_DUPLICATE,
        )

    try:
        # =====================================================================
        # 초기 설정
        # =====================================================================
        node.get_logger().info("M0609 기어 조립 시작")

        set_tool(TOOL_NAME)
        set_tcp(TCP_NAME)

        set_singularity_handling(DR_AVOID)

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
        # 전체 사이클
        # =====================================================================
        for cycle_index in range(CYCLE):
            node.get_logger().info(
                f"Cycle {cycle_index + 1}/{CYCLE}"
            )

            # =============================================================
            # T3
            # =============================================================
            node.get_logger().info("T3 기어 이송 시작")

            gripper_off()

            move_joint(q_t3_approach)
            move_linear_absolute(system_t3)

            gripper_on()
            lift_gear()

            move_joint(q_t3_place_approach)
            move_linear_absolute(system_t3_dst)

            gripper_off()

            node.get_logger().info("T3 끝")

            # =============================================================
            # T1
            # =============================================================
            node.get_logger().info("T1 기어 이송 시작")

            move_joint(q_t3_exit)
            move_joint(q_t1_approach)

            move_linear_absolute(system_t1)

            gripper_on()
            lift_gear()

            move_joint(q_t1_place_approach)
            move_linear_absolute(system_t1_dst)

            gripper_off()

            node.get_logger().info("T1 끝")

            # =============================================================
            # T2
            # =============================================================
            node.get_logger().info("T2 기어 이송 시작")

            move_joint(q_t1_place_approach)
            move_joint(q_t2_approach)

            move_linear_absolute(system_t2)

            gripper_on()
            lift_gear()

            move_joint(q_t2_place_approach)
            move_linear_absolute(system_t2_dst)

            gripper_off()

            node.get_logger().info("T2 끝")

            # =============================================================
            # T0 픽업
            # =============================================================
            node.get_logger().info("T0 기어 이송 시작")

            move_joint(q_t2_exit)
            move_joint(q_t0_approach)

            move_linear_absolute(system_t0)

            gripper_on()
            lift_gear()

            move_joint(q_t0_place_approach)

            # =============================================================
            # T0 삽입 시작 위치
            # =============================================================
            node.get_logger().info(
                "T0 기어 삽입 시작 위치로 이동"
            )

            move_linear_absolute(system_t0_dst)

            # =============================================================
            # T0 힘 제어 + 반복 회전
            # =============================================================
            node.get_logger().info(
                "T0 힘 제어 및 반복 회전 시작"
            )

            set_ref_coord(DR_TOOL)

            task_compliance_ctrl(
                stx=[
                    3000.0,
                    3000.0,
                    3000.0,
                    200.0,
                    200.0,
                    200.0,
                ],
                time=0.0,
            )
            compliance_enabled = True

            wait(0.5)

            set_stiffnessx(
                [
                    3000.0,
                    3000.0,
                    3000.0,
                    200.0,
                    200.0,
                    200.0,
                ],
                time=0.0,
            )

            # Tool Z축 방향으로 힘을 주어 아래로 삽입
            set_desired_force(
                fd=[
                    0.0,
                    0.0,
                    INSERT_FORCE_N,
                    0.0,
                    0.0,
                    0.0,
                ],
                dir=[
                    0,
                    0,
                    1,
                    0,
                    0,
                    0,
                ],
                time=0.0,
                mod=DR_FC_MOD_REL,
            )
            force_enabled = True

            # Tool C축 기준 ±10도 반복 회전
            amove_periodic(
                amp=[
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    WIGGLE_ANGLE_DEG,
                ],
                period=[
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    WIGGLE_PERIOD_SEC,
                ],
                atime=0.0,
                repeat=WIGGLE_REPEAT,
                ref=DR_TOOL,
            )
            periodic_motion_started = True

            wiggle_start_time = pytime.monotonic()
            last_log_time = 0.0

            # 비동기 모션이 실제 시작할 시간을 확보
            wait(0.1)

            while rclpy.ok():
                elapsed_time = (
                    pytime.monotonic() - wiggle_start_time
                )

                position_condition_result = (
                    check_position_condition(
                        axis=DR_AXIS_Z,
                        max=INSERT_SUCCESS_Z_MAX_MM,
                        ref=DR_BASE,
                        mod=DR_MV_MOD_ABS,
                    )
                )

                # 0.5초마다 상태 출력
                if elapsed_time - last_log_time >= 0.5:
                    node.get_logger().info(
                        "T0 삽입 중: "
                        f"elapsed={elapsed_time:.1f}s, "
                        f"condition_ret={position_condition_result}"
                    )
                    last_log_time = elapsed_time

                # ---------------------------------------------------------
                # 중요:
                # check_position_condition 반환값
                #
                # 조건 만족   = 0
                # 조건 불만족 = -1
                # ---------------------------------------------------------
                insertion_complete = (
                    position_condition_result
                    == DSR_CONDITION_TRUE
                )

                minimum_wiggle_completed = (
                    elapsed_time >= MIN_WIGGLE_TIME_SEC
                )

                if (
                    insertion_complete
                    and minimum_wiggle_completed
                ):
                    node.get_logger().info(
                        "T0 삽입 완료: "
                        "BASE Z <= 35 mm"
                    )

                    # 반복 회전 정지
                    stop_robot(DR_QSTOP)
                    periodic_motion_started = False

                    # 힘 제어 해제
                    release_force(time=0.0)
                    force_enabled = False

                    release_compliance_ctrl()
                    compliance_enabled = False

                    # 삽입이 완료된 이후에만 그리퍼를 연다.
                    gripper_off()

                    # 위로 후퇴
                    lift_gear()

                    node.get_logger().info(
                        "T0 기어 조립 완료"
                    )
                    break

                if elapsed_time >= INSERT_TIMEOUT_SEC:
                    node.get_logger().error(
                        "T0 삽입 시간 초과. "
                        "기어가 끝까지 삽입되지 않았습니다."
                    )

                    # 회전 정지
                    stop_robot(DR_QSTOP)
                    periodic_motion_started = False

                    # 힘 제어 해제
                    release_force(time=0.0)
                    force_enabled = False

                    release_compliance_ctrl()
                    compliance_enabled = False

                    # 실패 시에는 기어를 떨어뜨리지 않도록
                    # 그리퍼를 열지 않는다.
                    raise RuntimeError(
                        "T0 기어 삽입 실패: "
                        "그리퍼를 닫은 상태로 정지합니다."
                    )

                wait(0.02)

    except KeyboardInterrupt:
        node.get_logger().info(
            "사용자 요청으로 프로그램 정지"
        )

        if periodic_motion_started:
            try:
                stop_robot(DR_QSTOP)
            except Exception as stop_error:
                node.get_logger().error(
                    f"비동기 모션 정지 실패: {stop_error}"
                )

    except Exception as error:
        node.get_logger().error(
            f"Robot Error: {error}"
        )

        if periodic_motion_started:
            try:
                stop_robot(DR_QSTOP)
            except Exception as stop_error:
                node.get_logger().error(
                    f"비동기 모션 정지 실패: {stop_error}"
                )

    finally:
        if force_enabled:
            try:
                release_force(time=0.0)
            except Exception as error:
                node.get_logger().error(
                    f"힘 제어 해제 실패: {error}"
                )

        if compliance_enabled:
            try:
                release_compliance_ctrl()
            except Exception as error:
                node.get_logger().error(
                    f"순응 제어 해제 실패: {error}"
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()