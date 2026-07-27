#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plate_move.py

M0609 + OnRobot RG2. A point에서 판(plate)을 잡아 그대로(자세 변화 없이)
B point로 옮기는 pick-and-carry 데모.

동작 순서
========
1. 그리퍼 열기
2. A point 위쪽(APPROACH)으로 이동 -> A point로 하강
3. 그리퍼 닫기 (판을 잡는다)
4. A point 위쪽으로 복귀 -> B point 위쪽으로 이동 -> B point로 하강
5. (이 스크립트는 여기서 멈춘다 — 판을 잡은 채로 끝. B에서 놓고 싶으면
   맨 끝에 그리퍼 열기 호출을 추가할 것.)

"그리퍼 기준 X축 평행 유지"를 만족시키는 방법
================================================
POINT_A와 POINT_B의 자세(rx, ry, rz)를 **완전히 동일하게** 둔 채 movel로만
이동한다. movel(task-space 직선보간)은 시작/목표 자세가 같으면 궤적 전체에서
자세를 그대로 유지한 채 위치만 보간하므로, A -> B 전 구간에서 그리퍼(툴)
좌표계의 X/Y/Z축이 전부 처음 방향 그대로 유지된다 — "이동 중 한 번도
기울어지지 않는다"는 뜻이다.

★ 어떤 joint가 "고정"되는가? — 정확히는 "고정되는 joint는 없다".
--------------------------------------------------------------
plate_balance_controller.py의 servoj 방식(J1~J4는 명령값을 아예 안 바꿔서
물리적으로 고정, J5/J6만 직접 회전)과 이 스크립트의 movel 방식은 근본적으로
다르다.

movel은 task-space 제어라서, "목표 자세(rx,ry,rz)를 유지하라"는 조건을
만족시키기 위해 역기구학(IK) 솔버가 필요하면 J1~J6 전부를 움직인다. 즉
"고정 대상"은 특정 joint 각도가 아니라 **툴 좌표계의 방향(자세) 그 자체**이고,
그 방향을 지키기 위해 오히려 손목 joint(J4, J5, J6)가 어깨/팔꿈치(J1~J3)의
움직임을 실시간으로 상쇄하도록 계속 같이 돈다.

다만 M0609는 손목 3축(J4, J5, J6)이 한 점에서 만나는 구형 손목(spherical
wrist) 구조라, A와 B가 같은 높이/같은 접근방향이면 실제로는 주로 J1(베이스
회전)과 J2/J3(어깨/팔꿈치, 팔을 뻗는 정도)만 크게 바뀌고 J4~J6은 그 변화를
상쇄하는 수준의 비교적 작은 보정만 하게 되는 경우가 많다 — 그러나 이는
"J4~J6이 고정된다"는 보장이 아니라 A/B의 기하학적 배치에 따라 달라지는
결과일 뿐이다. 진짜로 특정 joint를 물리적으로 고정하고 싶다면 movel이 아니라
plate_balance_controller.py처럼 movej/servoj 기반 joint-space 제어로
설계를 바꿔야 한다.

주의
====
- POINT_A / POINT_B 좌표는 가상 에뮬레이터(DRCF)에서 실제로 도달 확인한
  값이지만, 실제 로봇/치구 배치와는 무관하다. 실기 적용 전 반드시
  티칭/실측한 좌표로 교체할 것.
- DRCF 에뮬레이터의 기본 홈 자세(팔이 수직으로 완전히 편 상태, z=1035mm
  부근)는 movel 기준 특이점(singularity)에 매우 가깝다. 이 자세에서 바로
  movel을 쏘면 컨트롤러가 RC_ERROR_MTN_SINGULARITY로 명령을 거부하는데,
  ROS 서비스 응답은 그래도 success=True를 돌려줘서 겉으로는 성공한 것처럼
  보이지만 실제로는 로봇이 전혀 움직이지 않는다(2026-07-22 직접 재현 확인).
  그래서 이 스크립트는 시작하자마자 movej로 팔을 굽힌 안전 자세(HOME_J)로
  옮긴 뒤에만 movel을 시작한다. 실기에서도 로봇이 완전히 펴진 자세 근처에
  있다면 같은 문제가 날 수 있으니 주의할 것.
- movel의 success=True가 "실제로 그 자리에 도달했다"를 보장하지 않는다는
  걸 위에서 확인했기 때문에, 마지막에 get_current_posx()로 실제 도착 위치를
  POINT_B와 비교해 어긋나면 경고 로그를 남긴다.
- 그리퍼 제어는 이 워크스페이스의 onrobot_rg_control(실기) /
  gripper_virtual_node(에뮬레이터) 둘 다 구현하는 공통 서비스
  '/onrobot/sendCommand' (onrobot_rg_msgs/srv/SetCommand, command='o'|'c')를
  사용한다. 둘 중 하나가 떠 있지 않으면 서비스 대기에서 타임아웃하고
  안전하게 중단한다.
- Tool/TCP는 set_tool/set_tcp로 지정하지 않는다. EE가 RG2 하나뿐이라
  plate_monitor.py/plate_monitor_v2.py와 마찬가지로 컨트롤러(티치펜던트)에
  이미 등록/활성화된 Tool/TCP를 그대로 쓴다는 전제다. 시작 시 get_tool()/
  get_tcp()로 현재 활성 이름을 로그로 남기니, 실행 후 그 값이 RG2용이
  맞는지 확인할 것 — 아니라면 movel의 x,y,z가 실제 손끝 위치와 어긋난다.
- 처음 실행할 때는 저속(VEL/ACC를 낮게)으로, 사람이 비상정지에 손을 올린
  채로 확인할 것.
"""

import math

import rclpy
import DR_init

from onrobot_rg_msgs.srv import SetCommand


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "rokey_plate_move"

# task-space 이동 속도/가속도 (mm/s, mm/s^2)
TASK_VEL = 30.0
TASK_ACC = 30.0

# joint-space 이동 속도/가속도 (deg/s, deg/s^2) — 시작 시 HOME_J로 갈 때만 사용.
JOINT_VEL = 30.0
JOINT_ACC = 30.0

# A/B 위쪽으로 얼마나 띄워서 접근/후퇴할지 (mm). 충돌 방지용.
APPROACH_HEIGHT_MM = 50.0

# 도착 위치 검증 허용오차 (mm). movel이 success=True를 줘도 실제로는 특이점
# 등으로 안 움직였을 수 있어(위 docstring 참고), 마지막에 실측 대조한다.
POSITION_TOLERANCE_MM = 5.0

GRIPPER_SERVICE = "/onrobot/sendCommand"
GRIPPER_SERVICE_TIMEOUT_SEC = 5.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def hovered(point, height_mm):
    """point(x, y, z, rx, ry, rz)에서 z만 height_mm 올린 새 posx를 만든다.
    rx/ry/rz는 그대로 유지 -> X축 평행 유지 조건이 접근/후퇴 구간에도
    똑같이 적용된다."""
    from DR_common2 import posx

    return posx(
        [
            point[0],
            point[1],
            point[2] + height_mm,
            point[3],
            point[4],
            point[5],
        ]
    )


class GripperClient:
    def __init__(self, node) -> None:
        self.node = node
        self.client = node.create_client(SetCommand, GRIPPER_SERVICE)

        if not self.client.wait_for_service(
            timeout_sec=GRIPPER_SERVICE_TIMEOUT_SEC
        ):
            raise RuntimeError(
                f"그리퍼 서비스 '{GRIPPER_SERVICE}'를 찾을 수 없습니다. "
                "onrobot_rg_control(실기) 또는 gripper_virtual_node(에뮬레이터)"
                "가 떠 있는지 확인하십시오."
            )

    def _send(self, command: str) -> None:
        request = SetCommand.Request()
        request.command = command

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "응답 없음"
            raise RuntimeError(f"그리퍼 명령 '{command}' 실패: {message}")

    def open(self) -> None:
        self.node.get_logger().info("그리퍼 열기")
        self._send("o")

    def close(self) -> None:
        self.node.get_logger().info("그리퍼 닫기 (판 잡기)")
        self._send("c")


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME, namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import movel, movej, get_current_posx, get_tool, get_tcp, DR_BASE
        from DR_common2 import posx, posj
    except ImportError as error:
        node.get_logger().error(f"DSR_ROBOT2 import 실패: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # movel 특이점을 피하기 위해 시작 시 거쳐가는 안전한(팔이 굽은) 관절 자세.
    # move.py의 homej와 동일한 값 — 이 저장소에서 이미 검증된 관절 홈 자세다.
    HOME_J = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

    # A/B point: rx, ry, rz를 반드시 동일하게 유지할 것 (X축 평행 유지의 핵심).
    # 2026-07-22: 가상 에뮬레이터(DRCF)에서 HOME_J로 이동한 뒤 move_line으로
    # 실제 도달을 확인한 좌표. 실제 로봇/치구 배치에 맞는 값이 아니므로 실기
    # 적용 전 반드시 티칭/실측한 좌표로 교체할 것.
    POINT_A = posx([368.0, 156.25, 425.0, 45.0, 180.0, 45.0])
    POINT_B = posx([368.0, -143.75, 425.0, 45.0, 180.0, 45.0])

    assert POINT_A[3:] == POINT_B[3:], (
        "POINT_A와 POINT_B의 자세(rx, ry, rz)가 다릅니다. "
        "X축 평행 유지를 위해 두 점의 자세는 반드시 동일해야 합니다."
    )

    point_a_approach = hovered(POINT_A, APPROACH_HEIGHT_MM)
    point_b_approach = hovered(POINT_B, APPROACH_HEIGHT_MM)

    # EE가 RG2 하나뿐이라 컨트롤러(티치펜던트)에 이미 등록된 Tool/TCP가
    # 그대로 맞다고 보고 set_tool/set_tcp는 호출하지 않는다
    # (plate_monitor.py / plate_monitor_v2.py와 동일한 방침). 다만 실제로
    # 뭐가 활성 상태인지는 로그로 남겨서 눈으로 확인할 수 있게 한다.
    node.get_logger().info(
        f"현재 활성 Tool='{get_tool()}', TCP='{get_tcp()}' "
        "(다르면 티치펜던트에서 RG2용으로 다시 등록/활성화할 것)"
    )

    try:
        gripper = GripperClient(node)
    except RuntimeError as error:
        node.get_logger().error(str(error))
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        node.get_logger().info("특이점 회피용 홈 자세(HOME_J)로 이동")
        movej(HOME_J, vel=JOINT_VEL, acc=JOINT_ACC)

        gripper.open()

        node.get_logger().info("A point 위쪽으로 접근")
        movel(point_a_approach, vel=TASK_VEL, acc=TASK_ACC, ref=DR_BASE)

        node.get_logger().info("A point로 하강")
        movel(POINT_A, vel=TASK_VEL, acc=TASK_ACC, ref=DR_BASE)

        gripper.close()

        node.get_logger().info("A point 위쪽으로 후퇴")
        movel(point_a_approach, vel=TASK_VEL, acc=TASK_ACC, ref=DR_BASE)

        node.get_logger().info("B point 위쪽으로 이동")
        movel(point_b_approach, vel=TASK_VEL, acc=TASK_ACC, ref=DR_BASE)

        node.get_logger().info("B point로 하강")
        movel(POINT_B, vel=TASK_VEL, acc=TASK_ACC, ref=DR_BASE)

        # movel의 success=True가 실제 도달을 보장하지 않는다는 게 확인됐으므로
        # (docstring 참고, 특이점에서 조용히 무시될 수 있음) 실측으로 대조한다.
        actual_pose, _solution_space = get_current_posx(ref=DR_BASE)
        arrival_error_mm = math.dist(actual_pose[:3], POINT_B[:3])

        if arrival_error_mm > POSITION_TOLERANCE_MM:
            node.get_logger().warn(
                f"[경고] B point 도착 오차 {arrival_error_mm:.1f} mm > "
                f"허용치 {POSITION_TOLERANCE_MM} mm — 실제 posx="
                f"{list(actual_pose)} (특이점 등으로 movel이 무시됐을 수 있음)"
            )
            node.get_logger().info("완료 (도착 오차 있음, 위 경고 확인할 것)")
        else:
            node.get_logger().info(
                f"완료 (판을 잡은 채로 B point에 도착, 오차 "
                f"{arrival_error_mm:.1f} mm)"
            )

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
