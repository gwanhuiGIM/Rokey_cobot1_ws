#!/usr/bin/env python3
"""
핸드드립 커피 - 주둥이 고정 피벗 붓기 (m0609, v4)

v3와 달리 잡기/내려놓기 시퀀스가 없다. 주전자는 이미 그리퍼에 잡혀 바닥과
평행한 상태이고, 주둥이는 필터 중앙 위에 있다고 가정한다. 이 프로그램은
붓기(pouring)만 수행한다.

v3 spiral의 중심 이탈 버그를 겪으며 확립한 원리를 전면 적용했다:
"기울이기"를 조인트 티칭 포즈로 만들지 않고, 주둥이 끝을 피벗으로 고정한
회전으로 계산한다(control_GUI의 rotate_pose_about_tool_offset과 동일 수학).
그래서 기울이는 동안 주둥이가 움직이지 않고, 물이 떨어지는 위치가 궤적
계산과 정확히 일치한다.

동작 순서
    1. 진입 시 활성 TCP/Tool 저장 -> 작업용 TCP/Tool로 전환
    2. 현재 자세(수평)에서 주둥이 위치를 계산해 앵커로 저장
       (= 필터 중앙. 이후 모든 목표의 기준점)
    3. 주둥이를 고정한 채 POUR_TILT_DEG(65도)만큼 천천히 기울이기 -> 물 시작
    4. 주둥이를 반지름 CIRCLE_RADIUS_MM(40mm) 원의 시작점으로 이동
    5. 주둥이가 필터 중앙 기준 원을 CIRCLE_REVS바퀴 도는 동안, 추가로
       EXTRA_TILT_DEG(40도)를 진행률에 비례해 더 기울인다. 매 경유점마다
       "주둥이가 원 위의 그 점에 있도록" TCP를 역산하므로, 기울임이 커져도
       주둥이 궤적은 정확히 반지름 40mm 원을 유지한다 (movesx 사용 -
       movec는 궤적 중 자세를 점진 변경할 수 없어 쓸 수 없다).
       --updown이면 이 원 궤적에 Z축 상하 운동(UPDOWN_RANGE_MM, 기본
       UPDOWN_REVS_PER_CYCLE(2)바퀴에 한 번 왕복)도 함께 더해진다
    6. 주둥이를 필터 중앙으로 복귀 (최대 기울임 유지)
    7. 필터와 부딪히지 않도록, 기울인 자세 그대로 BASE Z로
       POST_POUR_LIFT_MM(200mm) 상승
    8. 상승한 높이에서 원래 수평 자세로 천천히 복귀 -> 물 정지
    9. 진입 시 저장해둔 TCP/Tool로 복귀 (finally, 에러 나도 항상 실행)

⚠️ 실기 첫 테스트 전 확인 사항
    - SPOUT_OFFSET_MM: 이 값이 부정확하면 기울일수록 주둥이가 드리프트한다
      (v3 spiral 버그와 같은 원인). control_GUI의 "검증 TCP 오프셋"에 값을
      넣고 회전 버튼으로 주둥이가 제자리에 머무는 값을 실측 보정할 것.
    - TILT_SIGN: 기울임 방향이 반대면(주둥이가 위로 들리면) 부호를 뒤집을 것.
    - 첫 실행은 물 없이 저속으로 궤적만 확인 권장.

실행:
    ros2 run rokey kettle_circle_pour_v4
    ros2 run rokey kettle_circle_pour_v4 --viz     # 실시간 TCP/필터 경로 패널
    ros2 run rokey kettle_circle_pour_v4 --updown  # 붓기 원에 Z 상하 운동 추가
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
import DR_init


# ROS2 네임스페이스와 DSR 제어 대상에 사용하는 로봇 ID.
ROBOT_ID = "dsr01"
# DSR_ROBOT2가 로드할 두산 로봇 모델명.
ROBOT_MODEL = "m0609"

# ===== 로봇/그리퍼 설정 =====
# 제어기에 등록된 주전자 작업용 Tool(중량/무게중심) 설정 이름.
TOOL_NAME = "Tool Weight_1"
# 제어기에 등록된 그리퍼 TCP(툴 끝점) 설정 이름.
# TCP_NAME = "GripperDA_v1"
TCP_NAME = "pot"

# ============================================================
# ===== 기하 파라미터 =====
# ============================================================

# 활성 TCP -> 주전자 주둥이 끝의 TOOL 좌표 [X,Y,Z] 오프셋(mm).
# ⚠️ 이 프로그램 전체가 이 값의 정확도에 의존한다. control_GUI의
# "검증 TCP 오프셋" + 회전 버튼으로 실측 보정한 값을 넣을 것.
SPOUT_OFFSET_MM = [0.0, 00.0, 10.0]

# SPOUT_OFFSET_MM = [60.0, 50.0, 10.0]

# SPOUT_OFFSET_MM = [00.0, 150.0, 10.0]

# 수평 상태에서 물을 붓기 시작하는 1차 기울임 각도(deg).
POUR_TILT_DEG = 50.0
# 원을 도는 동안 진행률(0->1)에 비례해 추가하는 2차 기울임 각도(deg).
EXTRA_TILT_DEG = 40.0
# 기울임 회전 방향. 실기에서 주둥이가 내려가는(물이 나오는) 방향이 되도록
# 부호를 정한다. 반대로 움직이면 -1.0 <-> +1.0으로 뒤집을 것.
TILT_SIGN = -1.0

# 주둥이가 그리는 원의 반지름(mm).
CIRCLE_RADIUS_MM = 25.0
# 원을 도는 바퀴 수.
CIRCLE_REVS = 3
# 원 한 바퀴를 근사하는 movesx 경유점 수.
# 총 경유점 = CIRCLE_REVS * STEPS_PER_REV 는 movesx 한계(100점) 이하일 것.
STEPS_PER_REV = 24

# 붓기 종료 후 원래(수평) 자세로 복귀하기 전, 필터에 부딪히지 않도록 먼저
# 현재 자세를 유지한 채 BASE Z로 들어올리는 높이(mm).
POST_POUR_LIFT_MM = 20.0

# --updown일 때만 붓기 원 궤적에 추가하는 Z축 상하 운동 범위(최저~최고, mm).
UPDOWN_RANGE_MM = 30.0
# 상하 운동 1왕복에 도는 원 바퀴 수 (2바퀴에 위아래 한 번).
UPDOWN_REVS_PER_CYCLE = 2.0

# ============================================================
# ===== 속도/가속도 파라미터 =====
# ============================================================

# 1차 기울임(POUR_TILT_DEG, 물 붓기 시작 전) 속도 - 아직 물이 안 나오므로
# TILT_VEL보다 조금 더 빠르게.
POUR_TILT_VEL = [30.0, 12.0]
POUR_TILT_ACC = [150.0, 30.0]

# 필터 회피 상승 + 수평 복귀(물 정지) 속도 - 물 안 튀게 저속.
# movel vel/acc의 [병진 mm/s|mm/s^2, 회전 deg/s|deg/s^2] 쌍.
TILT_VEL = [20.0, 8.0]
TILT_ACC = [100.0, 20.0]

# 원 궤적(movesx) 및 중앙<->원 시작점 이동 속도.
CIRCLE_VEL = [30.0, 20.0]
CIRCLE_ACC = [200.0, 60.0]

# --viz 패널에 표시할 커피 필터 상단 반지름(mm).
VIZ_FILTER_RADIUS_MM = 60.0
# 별도 시각화 프로세스가 ROS 서비스에 연결하고 첫 포즈를 받을 준비 시간(s).
VIZ_STARTUP_DELAY_S = 1.0
# 비동기 모션이 끝날 때까지 기다리는 단계별 최대 시간(s).
MOTION_TIMEOUT_S = 180.0
# check_motion 상태 확인 주기(s). 시각화 포즈 서비스와 컨트롤러를 공유하므로
# 지나치게 짧게 잡지 않는다.
MOTION_POLL_PERIOD_S = 0.20


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
        description="핸드드립 주둥이 고정 피벗 붓기"
    )
    parser.add_argument(
        "--viz",
        action="store_true",
        help="필터와 TCP/주둥이 실시간 이동경로 패널을 함께 실행",
    )
    parser.add_argument(
        "--updown",
        action="store_true",
        help=(
            f"붓기 원 궤적에 Z축 상하 운동 추가 "
            f"(범위 {UPDOWN_RANGE_MM:.0f}mm, {UPDOWN_REVS_PER_CYCLE:.0f}바퀴에 1왕복)"
        ),
    )
    parsed_args, _ = parser.parse_known_args(cli_args)

    node = rclpy.create_node("kettle_circle_pour_v4", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    viz_process = None

    try:
        from DSR_ROBOT2 import (
            DR_AVOID,
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MVS_VEL_CONST,
            DR_STATE_IDLE,
            amovel,
            amovesx,
            check_motion,
            get_current_posx,
            get_tcp,
            get_tool,
            set_singularity_handling,
            set_tcp,
            set_tool,
        )
        from DR_common2 import posx
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    spout_offset = np.array(SPOUT_OFFSET_MM, dtype=float)

    # ===== 비동기 모션 + 완료 대기 =====
    def wait_motion_complete(label):
        """컨트롤러를 막지 않고 check_motion으로 비동기 모션 완료를 기다린다.

        동기 movel/movesx는 dsr_controller2 서비스 콜백을 모션 종료까지
        점유하여 path_viz의 get_current_posx가 응답하지 못한다. 비동기 명령은
        즉시 반환하므로 모션 중에도 실시간 TCP 포즈를 읽을 수 있다.
        """
        started_at = time.monotonic()
        motion_seen = False
        consecutive_idle = 0

        while rclpy.ok():
            elapsed = time.monotonic() - started_at
            if elapsed > MOTION_TIMEOUT_S:
                raise TimeoutError(
                    f"{label}: 모션 완료 대기 {MOTION_TIMEOUT_S:.0f}s 초과"
                )

            state = check_motion()
            if state < 0:
                raise RuntimeError(f"{label}: check_motion 실패({state})")

            if state == DR_STATE_IDLE:
                consecutive_idle += 1
                # 일반 모션은 BUSY 상태를 본 뒤 연속 IDLE이면 완료다. 아주 짧은
                # 모션은 첫 폴링 전에 끝날 수 있어 0.5초 후 연속 IDLE도 허용한다.
                if (
                    (motion_seen and consecutive_idle >= 2)
                    or (elapsed >= 0.5 and consecutive_idle >= 3)
                ):
                    return
            else:
                motion_seen = True
                consecutive_idle = 0

            time.sleep(MOTION_POLL_PERIOD_S)

        raise RuntimeError(f"{label}: ROS2 종료로 모션 대기 중단")

    def run_amovel(pose, vel, acc, label):
        """비동기 moveL을 시작하고 로봇이 IDLE로 돌아올 때까지 기다린다."""
        result = amovel(
            pose,
            vel=vel,
            acc=acc,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        if result != 0:
            raise RuntimeError(f"{label}: amovel 실행 실패")
        wait_motion_complete(label)

    # ===== 주둥이 피벗 계산 헬퍼 =====
    def tilted_pose(home_pose, extra_c_deg, spout_target):
        """home_pose 자세에 C(툴 로컬 Z) 회전 extra_c_deg를 더하고, 주둥이가
        spout_target(BASE 좌표)에 오도록 TCP 위치를 역산한 posx를 만든다.

        ZYZ 오일러(R = Rz(A) Ry(B) Rz(C))에서 C에 델타를 더하는 것은 정확히
        "현재 자세의 툴 로컬 Z축 기준 회전"(R @ Rz(delta))이다. v3 spiral과
        control_GUI 검증에서 확인한 기울임 축이며, 리그가 바뀌어 축이 다르면
        TILT_SIGN 또는 이 함수를 수정할 것.
        """
        a, b, c = home_pose[3], home_pose[4], home_pose[5]
        new_c = c + extra_c_deg
        rotation = zyz_to_matrix(a, b, new_c)
        tcp = np.asarray(spout_target, dtype=float) - rotation @ spout_offset
        return posx([tcp[0], tcp[1], tcp[2], a, b, new_c])

    def circle_spout_target(spout_center, angle_deg, updown_enabled=False):
        """필터 중앙(spout_center) 기준, BASE 수평면 위 원 위의 주둥이 목표점.

        updown_enabled면 UPDOWN_REVS_PER_CYCLE바퀴마다 한 번, 0 ~
        UPDOWN_RANGE_MM 사이를 코사인으로 왕복하는 Z축 상하 운동을 더한다.
        """
        angle_rad = math.radians(angle_deg)
        z_offset = 0.0
        if updown_enabled:
            cycle_phase = math.radians(angle_deg / UPDOWN_REVS_PER_CYCLE)
            z_offset = UPDOWN_RANGE_MM * 0.5 * (1.0 - math.cos(cycle_phase))
        return spout_center + np.array([
            CIRCLE_RADIUS_MM * math.cos(angle_rad),
            CIRCLE_RADIUS_MM * math.sin(angle_rad),
            z_offset,
        ])

    def move_tilt(pose, vel=TILT_VEL, acc=TILT_ACC):
        """주둥이 피벗 기울이기/복귀 - 기본은 저속, 1차 기울임만 조금 빠르게."""
        run_amovel(pose, vel=vel, acc=acc, label="주둥이 피벗 기울임")

    def move_circle_seg(pose):
        """중앙 <-> 원 시작점 등 짧은 직선 구간 - 붓기 속도."""
        run_amovel(
            pose,
            vel=CIRCLE_VEL,
            acc=CIRCLE_ACC,
            label="주둥이 원 경유 이동",
        )

    def draw_pour_circle(home_pose, spout_center, updown_enabled=False):
        """주둥이가 원을 돌면서 진행률에 비례해 EXTRA_TILT_DEG를 추가 기울임.

        매 경유점에서 주둥이 목표(원 위의 점)와 그 시점의 기울임 각도로 TCP를
        역산하므로, 기울임이 진행돼도 주둥이는 반지름 CIRCLE_RADIUS_MM 원을
        정확히 따라간다. updown_enabled면 그 목표점에 Z축 상하 운동도 더해진다
        (circle_spout_target 참고).
        """
        total_points = CIRCLE_REVS * STEPS_PER_REV
        if total_points > 100:
            raise ValueError(
                f"movesx 경유점 {total_points}개 > 100 한계 - "
                "CIRCLE_REVS/STEPS_PER_REV를 줄일 것"
            )
        path = []
        for index in range(1, total_points + 1):
            progress = index / total_points
            angle = 360.0 * CIRCLE_REVS * progress
            extra_c = TILT_SIGN * (
                POUR_TILT_DEG + EXTRA_TILT_DEG * progress
            )
            path.append(
                tilted_pose(
                    home_pose, extra_c,
                    circle_spout_target(spout_center, angle, updown_enabled),
                )
            )
        result = amovesx(
            path,
            vel=CIRCLE_VEL,
            acc=CIRCLE_ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
            vel_opt=DR_MVS_VEL_CONST,
        )
        if result != 0:
            raise RuntimeError("붓기 원 amovesx 실행 실패")
        wait_motion_complete("붓기 원 amovesx")

    # =========================================================================
    # 실행
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

        node.get_logger().info("1) 현재 자세(수평)에서 주둥이 앵커 계산")
        home_pose = list(get_current_posx(DR_BASE)[0])
        home_rotation = zyz_to_matrix(*home_pose[3:6])
        spout_center = (
            np.array(home_pose[:3], dtype=float) + home_rotation @ spout_offset
        )
        node.get_logger().info(
            f"   주둥이 앵커(BASE) = "
            f"[{spout_center[0]:.1f}, {spout_center[1]:.1f}, "
            f"{spout_center[2]:.1f}] mm"
        )

        if parsed_args.viz:
            from .path_viz import launch_path_viz

            node.get_logger().info("--viz: 실시간 TCP/필터 경로 패널 시작")
            viz_process = launch_path_viz(
                namespace=ROBOT_ID,
                spout_offset_mm=SPOUT_OFFSET_MM,
                filter_center_mm=spout_center,
                filter_top_radius_mm=VIZ_FILTER_RADIUS_MM,
                pour_radius_mm=CIRCLE_RADIUS_MM,
            )
            time.sleep(VIZ_STARTUP_DELAY_S)
            if viz_process.poll() is not None:
                raise RuntimeError(
                    f"경로 시각화 패널 시작 실패(exit={viz_process.returncode})"
                )

        node.get_logger().info(
            f"2) 주둥이 고정한 채 {POUR_TILT_DEG:.0f}도 기울이기 (물 시작)"
        )
        move_tilt(
            tilted_pose(home_pose, TILT_SIGN * POUR_TILT_DEG, spout_center),
            vel=POUR_TILT_VEL, acc=POUR_TILT_ACC,
        )

        node.get_logger().info(
            f"3) 주둥이를 원 시작점으로 이동 (반지름 {CIRCLE_RADIUS_MM:.0f} mm)"
        )
        move_circle_seg(tilted_pose(
            home_pose, TILT_SIGN * POUR_TILT_DEG,
            circle_spout_target(spout_center, 0.0)))

        node.get_logger().info(
            f"4) 원 {CIRCLE_REVS}바퀴 + 추가 기울임 {EXTRA_TILT_DEG:.0f}도 "
            f"(주둥이 궤적 고정)"
            + (
                f" + 상하운동 {UPDOWN_RANGE_MM:.0f}mm/"
                f"{UPDOWN_REVS_PER_CYCLE:.0f}바퀴"
                if parsed_args.updown else ""
            )
        )
        draw_pour_circle(home_pose, spout_center, parsed_args.updown)

        node.get_logger().info("5) 주둥이를 필터 중앙으로 복귀")
        post_circle_pose = tilted_pose(
            home_pose, TILT_SIGN * (POUR_TILT_DEG + EXTRA_TILT_DEG),
            spout_center)
        move_circle_seg(post_circle_pose)

        node.get_logger().info(
            f"6) 필터 회피: 기울인 자세 그대로 Z+{POST_POUR_LIFT_MM:.0f}mm 상승"
        )
        lifted_pose = list(post_circle_pose)
        lifted_pose[2] += POST_POUR_LIFT_MM
        move_tilt(posx(lifted_pose))

        node.get_logger().info(
            "7) 상승한 높이에서 주둥이 위치 고정한 채 수평 자세로 복귀 (물 정지)"
        )
        lifted_spout_target = spout_center + np.array(
            [0.0, 0.0, POST_POUR_LIFT_MM])
        move_tilt(tilted_pose(home_pose, 0.0, lifted_spout_target))

        node.get_logger().info("붓기 완료")
    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")
    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")
    finally:
        if viz_process is not None:
            from .path_viz import stop_path_viz

            stop_path_viz(viz_process)
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
