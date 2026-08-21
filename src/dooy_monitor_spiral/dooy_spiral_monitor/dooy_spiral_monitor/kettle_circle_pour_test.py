#!/usr/bin/env python3
"""
가상 제어기 전용 핸드드립 실패 궤적 재현기.

실제 주전자, Tool/TCP 등록, 그리퍼 동작 없이 DRCF 가상 제어기의 TCP 자세와
가상의 주둥이 오프셋만 사용한다. 정상 붓기 프로그램이 아니라, 과거에
발생했던 아래 두 현상을 영상으로 남기기 위한 테스트 코드다.

1) ``--case movec``
   수평 대기 자세의 주둥이 끝을 고정점으로 잡고, C 자세를 기울이면서 회전한
   오프셋만큼 TCP를 반대로 이동한 목표를 IK로 구한다. 시작과 종료의 주둥이
   좌표는 같지만 async MoveJ의 조인트 보간은 중간 TCP/주둥이 경로를 구속하지
   않으므로 기울이는 동안 필터 중심 밖으로 이탈할 수 있다. 기울임 종료 후에는
   고정 자세로 필터 중심 기준 MoveC 원을 그린다.

2) ``--case spiral``
   기울인 시작 자세의 실제 주둥이 위치를 필터 중심으로 고정한 뒤 나선과
   점진적 기울임을 movesx로 동시에 수행한다. 궤적 계산에는 과거의 잘못된
   주둥이 오프셋(ASSUMED_SPOUT_OFFSET_MM)을, 모니터에는 검증된 오프셋
   (TRUE_SPOUT_OFFSET_MM)을 사용해 기울임에 비례하는 중심 이탈을 재현한다.

가상 제어기 노드 ``/dsr01/virtual_node``가 발견되고, 실기 전용 노드나
중복 컨트롤러가 없을 때만 모션을 허용하므로 실제 로봇 bringup에 실수로
실행해도 움직이지 않는다.

실행 예:
    # 터미널 1: DRCF 에뮬레이터 + RViz
    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual

    # 터미널 2: 각 실패 장면
    ros2 run rokey kettle_circle_pour_test --case movec --viz --keep-viz
    ros2 run rokey kettle_circle_pour_test --case spiral --viz --keep-viz
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import DR_init


# DSR 제어 대상의 ROS 네임스페이스.
ROBOT_ID = "dsr01"
# DSR_ROBOT2가 사용할 로봇 모델.
ROBOT_MODEL = "m0609"

# v3의 붓기 전 대기 조인트 [J1..J6, deg].
# movec 장면에서 기울이기 전 주둥이 위치를 기록하는 기준 자세다.
LEVEL_J = [-31.930, 50.320, 26.430, 77.87, 102.74, -171.32]
# v3의 기울인 붓기 조인트 [J1..J6, deg].
# MoveJ 기울임 결과와 spiral의 시작 자세로 사용한다.
TILTED_J = [10.490, 41.710, 29.500, 14.37, 94.72, -178.78]

# 모니터가 실제 주둥이라고 간주할 TCP->주둥이 TOOL 좌표 오프셋(mm).
# 현재 v4/control_GUI에서 검증 대상으로 사용한 값이다.
TRUE_SPOUT_OFFSET_MM = [0.0, 150.0, 10.0]
# 실패한 spiral 계산이 사용했다고 가정하는 잘못된 오프셋(mm).
# TRUE 값과 차이가 클수록 점진 기울임 시 중심 이탈이 뚜렷해진다.
ASSUMED_SPOUT_OFFSET_MM = [0.0, 60.0, 0.0]

# MoveC 원 및 spiral의 최대 반지름(mm).
PATH_RADIUS_MM = 30.0
# MoveC가 반복하는 원의 바퀴 수.
MOVEC_REVS = 3
# movec 실패 장면에서 주둥이 끝을 고정하고 C 자세에 더할 기울임(deg).
# 시작/종료 TCP는 피벗 보정하지만 MoveJ 중간 경로에는 Cartesian 구속이 없다.
MOVEC_TILT_DEG = -30.0
# spiral이 바깥으로 확장하고 다시 안으로 복귀할 때 각각 도는 바퀴 수.
SPIRAL_REVS = 3
# spiral 한 바퀴당 경유점 수. 왕복 총점은 2 * REVS * STEPS이며
# DSR movesx 한계인 100개 이하여야 한다.
SPIRAL_STEPS_PER_REV = 13
# spiral 진행 중 누적할 추가 C축 기울임(deg).
SPIRAL_EXTRA_TILT_DEG = -30.0
# spiral Z 오실레이션의 최저점 대비 최대 상승량(mm).
SPIRAL_Z_LIFT_MM = 20.0
# spiral 1회전당 Z 오실레이션 횟수.
Z_OSCILLATIONS_PER_REV = 1.0 / 3.0

# 일반 조인트 이동 속도/가속도(deg/s, deg/s^2).
JOINT_VEL = 25.0
JOINT_ACC = 50.0
# 실패 장면의 async MoveJ 기울임 속도/가속도(deg/s, deg/s^2).
TILT_JOINT_VEL = 8.0
TILT_JOINT_ACC = 20.0
# MoveL/MoveC/movesx 병진 및 회전 속도[mm/s, deg/s].
PATH_VEL = [25.0, 15.0]
# MoveL/MoveC/movesx 병진 및 회전 가속도[mm/s^2, deg/s^2].
PATH_ACC = [150.0, 40.0]

# path_viz에 그릴 필터 상단 반지름(mm).
VIZ_FILTER_RADIUS_MM = 60.0
# path_viz 프로세스가 시작하고 첫 포즈를 받을 준비 시간(s).
VIZ_STARTUP_DELAY_S = 1.0
# bringup 직후 virtual_node보다 늦게 생성되는 컨트롤러/모션 서비스를 기다리는
# 최대 시간(s). DRCF Docker 기동과 controller spawner에 보통 수 초가 걸린다.
VIRTUAL_READY_TIMEOUT_S = 20.0
# 비동기 모션 하나의 최대 완료 대기 시간(s).
MOTION_TIMEOUT_S = 180.0
# check_motion 상태 확인 주기(s).
MOTION_POLL_PERIOD_S = 0.20
# --keep-viz를 쓰지 않았을 때 모션 종료 후 패널 유지 시간(s).
DEFAULT_HOLD_S = 8.0


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def _rz(deg):
    """Z축 회전 행렬."""
    rad = math.radians(deg)
    cos_value, sin_value = math.cos(rad), math.sin(rad)
    return np.array([
        [cos_value, -sin_value, 0.0],
        [sin_value, cos_value, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _ry(deg):
    """Y축 회전 행렬."""
    rad = math.radians(deg)
    cos_value, sin_value = math.cos(rad), math.sin(rad)
    return np.array([
        [cos_value, 0.0, sin_value],
        [0.0, 1.0, 0.0],
        [-sin_value, 0.0, cos_value],
    ])


def zyz_to_matrix(a, b, c):
    """Doosan posx의 ZYZ 자세 [A,B,C]를 회전 행렬로 변환한다."""
    return _rz(a) @ _ry(b) @ _rz(c)


def spout_position(pose, tool_offset):
    """TCP posx와 TOOL 좌표 오프셋으로 BASE 좌표의 주둥이 끝을 계산한다."""
    rotation = zyz_to_matrix(*pose[3:6])
    return np.asarray(pose[:3], dtype=float) + rotation @ tool_offset


def virtual_environment_issue(node):
    """virtual 단독 환경인지 검사하고, 안전하지 않으면 원인을 반환한다."""
    # 동일 PC에 오래된 real bringup 또는 다른 붓기 모션 프로세스가 남아 있으면
    # DDS 그래프에서 중복 노드명이 하나처럼 보일 수 있다. /proc의 명령행도
    # 확인해 virtual 서비스와 실기 서비스가 섞이기 전에 차단한다.
    local_conflicts = set()
    current_pid = os.getpid()
    proc_root = "/proc"
    try:
        proc_names = os.listdir(proc_root)
    except OSError:
        proc_names = []
    for proc_name in proc_names:
        if not proc_name.isdigit() or int(proc_name) == current_pid:
            continue
        try:
            with open(
                os.path.join(proc_root, proc_name, "cmdline"),
                "rb",
            ) as command_file:
                command = command_file.read().replace(b"\0", b" ").decode(
                    errors="replace"
                )
        except (OSError, ValueError):
            continue

        if "bringup.launch.py" in command and "mode:=real" in command:
            local_conflicts.add("mode:=real bringup")
        for program_name in (
            "kettle_circle_pour_v2",
            "kettle_circle_pour_v3",
            "kettle_circle_pour_v4",
        ):
            if program_name in command:
                local_conflicts.add(program_name)

    if local_conflicts:
        return (
            "로컬에 충돌 가능한 실기/모션 프로세스가 실행 중입니다: "
            + ", ".join(sorted(local_conflicts))
            + ". 해당 터미널에서 Ctrl+C로 먼저 종료하세요."
        )

    # run_emulator는 ROS executable 이름이고, VirtualDRCF가 생성하는 현재
    # 노드명은 virtual_node다. 구버전 launch와의 호환을 위해 둘 다 허용한다.
    virtual_node_names = {
        f"/{ROBOT_ID}/virtual_node",
        f"/{ROBOT_ID}/run_emulator",
    }
    required_single_nodes = (
        f"/{ROBOT_ID}/controller_manager",
        f"/{ROBOT_ID}/dsr_controller2",
    )
    # 커스텀 bringup에서 아래 노드는 mode:=real일 때만 생성된다.
    real_only_nodes = {
        "/gripper_joint_state_publisher",
        "/OnRobotRGControllerServer",
    }
    required_services = {
        f"/{ROBOT_ID}/motion/check_motion",
        f"/{ROBOT_ID}/motion/move_joint",
        f"/{ROBOT_ID}/motion/move_circle",
        f"/{ROBOT_ID}/motion/move_spline_task",
        f"/{ROBOT_ID}/motion/ikin",
        f"/{ROBOT_ID}/aux_control/get_current_posj",
        f"/{ROBOT_ID}/aux_control/get_current_posx",
        f"/{ROBOT_ID}/aux_control/get_current_solution_space",
    }

    started_at = time.monotonic()
    last_graph_names = []
    last_service_names = set()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        last_graph_names = [
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in node.get_node_names_and_namespaces()
        ]
        last_service_names = {
            service_name
            for service_name, _ in node.get_service_names_and_types()
        }

        # 같은 /dsr01에 real/virtual bringup이 겹치면 동일 서비스가 중복되어
        # 명령 대상이 불명확하므로 준비 대기와 관계없이 즉시 차단한다.
        found_real_nodes = sorted(
            real_only_nodes.intersection(last_graph_names)
        )
        if found_real_nodes:
            return (
                "실기 bringup 노드가 함께 실행 중입니다: "
                + ", ".join(found_real_nodes)
            )

        duplicate_nodes = [
            f"{name}({last_graph_names.count(name)}개)"
            for name in required_single_nodes
            if last_graph_names.count(name) > 1
        ]
        if duplicate_nodes:
            return "컨트롤러 노드가 중복입니다: " + ", ".join(
                duplicate_nodes
            )

        virtual_ready = any(
            name in virtual_node_names for name in last_graph_names
        )
        controllers_ready = all(
            last_graph_names.count(name) == 1
            for name in required_single_nodes
        )
        services_ready = required_services.issubset(last_service_names)
        if virtual_ready and controllers_ready and services_ready:
            return None

        if time.monotonic() - started_at >= VIRTUAL_READY_TIMEOUT_S:
            break
        time.sleep(0.25)

    missing_nodes = [
        name
        for name in required_single_nodes
        if last_graph_names.count(name) == 0
    ]
    missing_services = sorted(required_services - last_service_names)
    if not any(name in virtual_node_names for name in last_graph_names):
        return (
            f"/{ROBOT_ID}/virtual_node가 없습니다. 먼저 mode:=virtual "
            "bringup을 시작하세요."
        )
    return (
        f"가상 bringup 준비를 {VIRTUAL_READY_TIMEOUT_S:.0f}초 기다렸지만 "
        f"완료되지 않았습니다. 누락 노드={missing_nodes}, "
        f"누락 서비스={missing_services}"
    )


def build_parser():
    """테스트 실행 인자를 만든다."""
    parser = argparse.ArgumentParser(
        description="가상 제어기 전용 핸드드립 실패 궤적 재현기"
    )
    parser.add_argument(
        "--case",
        choices=("movec", "spiral"),
        required=True,
        help="재현 장면: MoveJ+MoveC 중심 이탈 또는 spiral+tilt 중심 이탈",
    )
    parser.add_argument(
        "--viz",
        action="store_true",
        help="실시간 주둥이 경로 패널을 함께 실행",
    )
    parser.add_argument(
        "--keep-viz",
        action="store_true",
        help="모션 종료 후 Enter를 누를 때까지 경로 패널 유지",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=DEFAULT_HOLD_S,
        metavar="SEC",
        help=f"--keep-viz 미사용 시 패널 유지 시간(기본 {DEFAULT_HOLD_S:g}s)",
    )
    parser.add_argument(
        "--true-offset",
        nargs=3,
        type=float,
        default=TRUE_SPOUT_OFFSET_MM,
        metavar=("X", "Y", "Z"),
        help="모니터가 실제값으로 표시할 TCP->주둥이 오프셋(mm)",
    )
    parser.add_argument(
        "--assumed-offset",
        nargs=3,
        type=float,
        default=ASSUMED_SPOUT_OFFSET_MM,
        metavar=("X", "Y", "Z"),
        help="spiral 실패 계산에서 사용할 잘못 가정한 오프셋(mm)",
    )
    return parser


def main(args=None):
    """가상 제어기의 선택한 실패 장면을 실행한다."""
    rclpy.init(args=args)

    cli_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    parsed_args, _ = build_parser().parse_known_args(cli_args)

    node = rclpy.create_node("kettle_circle_pour_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    viz_process = None

    try:
        environment_issue = virtual_environment_issue(node)
        if environment_issue is not None:
            raise RuntimeError(
                f"{environment_issue} 실제 로봇 또는 중복 bringup에서는 "
                "모션을 실행하지 않습니다."
            )

        from DSR_ROBOT2 import (
            DR_BASE,
            DR_MV_MOD_ABS,
            DR_MVS_VEL_CONST,
            DR_STATE_IDLE,
            amovec,
            amovej,
            amovel,
            amovesx,
            check_motion,
            get_current_posj,
            get_current_posx,
            get_current_solution_space,
            ikin,
        )
        from DR_common2 import posj, posx

        true_offset = np.asarray(parsed_args.true_offset, dtype=float)
        assumed_offset = np.asarray(parsed_args.assumed_offset, dtype=float)

        def wait_motion_complete(label):
            """비동기 모션 중 서비스가 열리도록 상태를 폴링해 완료를 기다린다."""
            started_at = time.monotonic()
            motion_seen = False
            consecutive_idle = 0

            while rclpy.ok():
                elapsed = time.monotonic() - started_at
                if elapsed > MOTION_TIMEOUT_S:
                    raise TimeoutError(
                        f"{label}: 모션 대기 {MOTION_TIMEOUT_S:.0f}s 초과"
                    )

                state = check_motion()
                if state < 0:
                    raise RuntimeError(f"{label}: check_motion 실패({state})")

                if state == DR_STATE_IDLE:
                    consecutive_idle += 1
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

        def run_async(command, label):
            """비동기 DSR 명령의 접수 결과를 확인하고 완료까지 기다린다."""
            result = command()
            if result != 0:
                raise RuntimeError(f"{label}: 비동기 모션 실행 실패({result})")
            wait_motion_complete(label)

        def move_to_joint(joints, vel, acc, label):
            """절대 조인트 자세로 비동기 MoveJ한다."""
            run_async(
                lambda: amovej(
                    posj(joints),
                    vel=vel,
                    acc=acc,
                    mod=DR_MV_MOD_ABS,
                ),
                label,
            )

        def current_pose():
            """현재 DR_BASE TCP posx를 일반 list로 읽는다."""
            return list(get_current_posx(DR_BASE)[0])

        def launch_viz(filter_center):
            """검증된 오프셋을 쓰는 주둥이 경로 패널을 시작한다."""
            nonlocal viz_process
            if not parsed_args.viz:
                return

            from .path_viz import launch_path_viz

            viz_process = launch_path_viz(
                namespace=ROBOT_ID,
                spout_offset_mm=true_offset,
                filter_center_mm=filter_center,
                filter_top_radius_mm=VIZ_FILTER_RADIUS_MM,
                pour_radius_mm=PATH_RADIUS_MM,
            )
            time.sleep(VIZ_STARTUP_DELAY_S)
            if viz_process.poll() is not None:
                raise RuntimeError(
                    f"path_viz 시작 실패(exit={viz_process.returncode})"
                )

        def circle_pose(center_pose, angle_deg):
            """고정 자세로 TCP가 수평 원 위에 놓이는 MoveC 경유점을 만든다."""
            angle_rad = math.radians(angle_deg)
            pose = list(center_pose)
            pose[0] += PATH_RADIUS_MM * math.cos(angle_rad)
            pose[1] += PATH_RADIUS_MM * math.sin(angle_rad)
            return posx(pose)

        def run_movec_failure():
            """주둥이 고정 목표를 MoveJ한 뒤 필터 중심에서 MoveC 원을 그린다."""
            node.get_logger().info("준비: v3 수평 대기 조인트로 이동")
            move_to_joint(
                LEVEL_J, JOINT_VEL, JOINT_ACC, "수평 대기 자세 MoveJ"
            )
            level_pose = current_pose()
            intended_center = spout_position(level_pose, true_offset)
            launch_viz(intended_center)

            node.get_logger().info(
                f"장면 1: 주둥이 끝 고정, C{MOVEC_TILT_DEG:+.0f}도 목표를 "
                "IK로 변환해 async MoveJ - 중간 주둥이 경로는 구속되지 않음"
            )
            target_a, target_b = level_pose[3], level_pose[4]
            target_c = level_pose[5] + MOVEC_TILT_DEG
            target_rotation = zyz_to_matrix(
                target_a, target_b, target_c
            )
            target_tcp = intended_center - target_rotation @ true_offset
            tilt_target_pose = [
                target_tcp[0],
                target_tcp[1],
                target_tcp[2],
                target_a,
                target_b,
                target_c,
            ]
            solution_space = get_current_solution_space()
            if solution_space < 0:
                raise RuntimeError(
                    "현재 IK solution space를 읽지 못했습니다."
                )
            tilt_target_joints = ikin(
                posx(tilt_target_pose),
                solution_space,
                ref=DR_BASE,
            )
            if (
                np.isscalar(tilt_target_joints)
                and float(tilt_target_joints) == -1.0
            ):
                raise RuntimeError(
                    "같은 TCP XYZ의 기울임 자세에 대한 IK 해를 찾지 못했습니다."
                )
            tilt_target_joints = list(tilt_target_joints)
            current_joints = list(get_current_posj())
            # J6는 ±360도 범위에서 같은 자세를 여러 각도로 표현할 수 있다.
            # IK가 현재 -171도 근처 대신 +159도처럼 반환하면 MoveJ가 약 330도를
            # 돌아가므로, 현재 J6에 가장 가까운 동치각으로 정규화한다.
            while tilt_target_joints[5] - current_joints[5] > 180.0:
                tilt_target_joints[5] -= 360.0
            while tilt_target_joints[5] - current_joints[5] < -180.0:
                tilt_target_joints[5] += 360.0
            move_to_joint(
                tilt_target_joints,
                TILT_JOINT_VEL,
                TILT_JOINT_ACC,
                "기울임 async MoveJ",
            )
            tilted_pose = current_pose()
            displaced_center = spout_position(tilted_pose, true_offset)
            drift = displaced_center - intended_center
            node.get_logger().warn(
                "MoveJ 종료점의 주둥이 고정 오차: "
                f"dX={drift[0]:+.1f}, dY={drift[1]:+.1f}, "
                f"dZ={drift[2]:+.1f} mm"
            )

            node.get_logger().info(
                f"장면 1: 필터 중심 기준 MoveC {MOVEC_REVS}바퀴 시작"
            )
            run_async(
                lambda: amovel(
                    circle_pose(tilted_pose, 0.0),
                    vel=PATH_VEL,
                    acc=PATH_ACC,
                    ref=DR_BASE,
                    mod=DR_MV_MOD_ABS,
                ),
                "MoveC 시작점 MoveL",
            )
            for lap in range(1, MOVEC_REVS + 1):
                run_async(
                    lambda: amovec(
                        circle_pose(tilted_pose, 90.0),
                        circle_pose(tilted_pose, 180.0),
                        vel=PATH_VEL,
                        acc=PATH_ACC,
                        ref=DR_BASE,
                        mod=DR_MV_MOD_ABS,
                    ),
                    f"MoveC {lap}바퀴 전반",
                )
                run_async(
                    lambda: amovec(
                        circle_pose(tilted_pose, 270.0),
                        circle_pose(tilted_pose, 360.0),
                        vel=PATH_VEL,
                        acc=PATH_ACC,
                        ref=DR_BASE,
                        mod=DR_MV_MOD_ABS,
                    ),
                    f"MoveC {lap}바퀴 후반",
                )

        def run_spiral_failure():
            """잘못된 오프셋으로 점진 기울임을 보정한 spiral을 실행한다."""
            node.get_logger().info("준비: v3 기울인 붓기 조인트로 이동")
            move_to_joint(
                TILTED_J, JOINT_VEL, JOINT_ACC, "spiral 시작 자세 MoveJ"
            )
            home_pose = current_pose()
            intended_center = spout_position(home_pose, true_offset)
            launch_viz(intended_center)

            total_half_points = SPIRAL_REVS * SPIRAL_STEPS_PER_REV
            total_points = 2 * total_half_points
            if total_points > 100:
                raise ValueError(
                    f"spiral 경유점 {total_points}개 > movesx 100개 한계"
                )

            path = []
            base_a, base_b, base_c = home_pose[3:6]
            base_rotation = zyz_to_matrix(base_a, base_b, base_c)
            # 과거 알고리즘은 시작 중심도 같은 잘못된 오프셋으로 계산했다.
            # 이 중심을 써야 첫 경유점에서 TCP가 갑자기 점프하지 않고,
            # 추가 기울기가 누적되는 만큼만 실제 주둥이가 서서히 이탈한다.
            assumed_center = (
                np.asarray(home_pose[:3], dtype=float)
                + base_rotation @ assumed_offset
            )
            for index in range(1, total_points + 1):
                motion_progress = index / total_points
                half_progress = (
                    index / total_half_points
                    if index <= total_half_points
                    else (index - total_half_points) / total_half_points
                )
                radius = (
                    PATH_RADIUS_MM * half_progress
                    if index <= total_half_points
                    else PATH_RADIUS_MM * (1.0 - half_progress)
                )
                angle_deg = 360.0 * SPIRAL_REVS * half_progress
                angle_rad = math.radians(angle_deg)
                z_lift = SPIRAL_Z_LIFT_MM * 0.5 * (
                    1.0 - math.cos(
                        2.0
                        * math.pi
                        * Z_OSCILLATIONS_PER_REV
                        * SPIRAL_REVS
                        * half_progress
                    )
                )
                desired_spout = assumed_center + np.array([
                    radius * math.cos(angle_rad),
                    radius * math.sin(angle_rad),
                    z_lift,
                ])

                new_c = (
                    base_c + SPIRAL_EXTRA_TILT_DEG * motion_progress
                )
                command_rotation = zyz_to_matrix(base_a, base_b, new_c)
                # 실패 재현의 핵심: 실제 오프셋이 아니라 과거의 가정값으로
                # TCP 위치를 역산한다. 따라서 실제 주둥이는 tilt에 따라 돈다.
                command_tcp = (
                    desired_spout - command_rotation @ assumed_offset
                )
                path.append(posx([
                    command_tcp[0],
                    command_tcp[1],
                    command_tcp[2],
                    base_a,
                    base_b,
                    new_c,
                ]))

            node.get_logger().warn(
                "장면 2: 잘못 가정한 오프셋으로 spiral+tilt 시작 - "
                f"actual={true_offset.tolist()}, "
                f"assumed={assumed_offset.tolist()}"
            )
            run_async(
                lambda: amovesx(
                    path,
                    vel=PATH_VEL,
                    acc=PATH_ACC,
                    ref=DR_BASE,
                    mod=DR_MV_MOD_ABS,
                    vel_opt=DR_MVS_VEL_CONST,
                ),
                "실패 spiral amovesx",
            )

        if parsed_args.case == "movec":
            run_movec_failure()
        else:
            run_spiral_failure()

        node.get_logger().info(f"{parsed_args.case} 실패 장면 재현 완료")
        if viz_process is not None:
            if parsed_args.keep_viz:
                try:
                    input("경로 확인이 끝나면 Enter를 누르세요... ")
                except EOFError:
                    node.get_logger().info(
                        "표준 입력이 없어 --hold 시간만큼 패널을 유지합니다."
                    )
                    time.sleep(max(0.0, parsed_args.hold))
            else:
                time.sleep(max(0.0, parsed_args.hold))

    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 테스트 정지")
    except Exception as error:
        node.get_logger().error(f"테스트 중단: {error}")
    finally:
        if viz_process is not None:
            from .path_viz import stop_path_viz

            stop_path_viz(viz_process)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
