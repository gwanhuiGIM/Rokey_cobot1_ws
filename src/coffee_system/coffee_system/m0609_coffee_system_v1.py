#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
m0609_coffee_system.py

M0609 + OnRobot RG2 협동로봇 자동 커피 추출 시스템
원본 Task Writer(DRL) 프로그램(m0609_coffe_system.drl, System.drvar)을
ROS2 단일 파일 노드로 변환한 것입니다.

동작 순서 (bean_drop -> grinder -> dripper_in, 1회 실행)
==========================================================
1. bean_drop   : 스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입
2. grinder     : 그라인더 손잡이를 잡고 컴플라이언스 제어로 원호를 그리며 분쇄
3. dripper_in  : 드립 병을 잡고 주기 운동(move_periodic)으로 커피를 추출

실행
====
    ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609
    ros2 run rokey m0609_coffee_system
    (또는 패키지에 등록하지 않고 바로:  python3 m0609_coffee_system.py)

주의
====
- 이 노드는 시작과 동시에 실제 로봇 모션을 수행합니다. 원본 좌표는 teach pendant에서
  교시된 값(System.drvar)을 그대로 사용하므로, 실행 전 작업 공간(그라인더, 드리퍼,
  병, 컵, 스푼 위치)이 교시 당시와 동일한지 반드시 확인하십시오.
- OnRobot RG2 그리퍼는 컨트롤러 Digital Output 1, 2번 접점 조합으로 제어됩니다.
  (그리퍼 컨트롤 박스가 해당 접점에 매핑되어 있어야 합니다.)
- 그라인더 서브루틴은 task_compliance_ctrl() 진입 후 movec()으로 원호 모션을 수행합니다.
  컴플라이언스 구간 진입 전 주변 장애물 여부를 확인하십시오.
- 원본 DRL의 set_singular_handling(DR_AVOID)는 프로그래밍 매뉴얼에 공식 문서화된
  set_singularity_handling(mode)로 치환하였습니다 (동일 기능, 특이점 자동회피).
"""

from __future__ import annotations

# import json  # UI(coffee_status_ui.py) 상태 발행용 — 비활성화
import sys
import time

import rclpy
# from std_msgs.msg import String  # UI 상태 발행용 — 비활성화

import DR_init
from dsr_msgs2.srv import MoveJoint

# =============================================================================
# 사용자 설정
# =============================================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
NODE_NAME = "m0609_coffee_system"

# 티치펜던트에 등록된 이름 (gear.py/move.py와 동일한 이름 사용)
TOOL_NAME = "Tool Weight_gripper"
TCP_NAME = "GripperDA_v1"

# 원본 DRL 하단 전역 설정값
VELJ_DEFAULT = 20.0
ACCJ_DEFAULT = 35.0
VELX_LIN_DEFAULT = 80.0
VELX_ROT_DEFAULT = 25.0
ACCX_LIN_DEFAULT = 300.0
ACCX_ROT_DEFAULT = 100.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# -----------------------------------------------------------------------
# UI(coffee_status_ui.py) 진행 상태 발행 — 비활성화 (아래 전부 주석 처리)
# -----------------------------------------------------------------------
# # UI(coffee_status_ui.py)가 구독하는 진행 상태 토픽. 노드가 namespace=ROBOT_ID로
# # 생성되므로 실제로는 /<ROBOT_ID>/coffee_system/status 로 나간다.
# STATUS_TOPIC = "coffee_system/status"
#
# # 3단계 시퀀스와 각 단계 내부의 세부 동작 라벨 (UI 애니메이션에서 그대로 사용).
# STAGES = ("bean_drop", "grinder", "dripper_in")
# STAGE_LABELS = {
#     "bean_drop": "원두 투입",
#     "grinder": "분쇄",
#     "dripper_in": "드립 추출",
# }
# STEP_LABELS = {
#     "bean_drop": [
#         "그리퍼 초기화",
#         "홈으로 이동",
#         "스푼 위치로 이동 및 파지",
#         "그라인더로 이동",
#         "원두 투입",
#         "스푼 정리",
#         "스푼 내려놓기",
#         "홈으로 복귀",
#     ],
#     "grinder": [
#         "손잡이 위치로 이동 및 파지",
#         "분쇄 준비 자세",
#         "컴플라이언스 진입",
#         "원호 분쇄 동작",
#         "컴플라이언스 해제 및 손잡이 놓기",
#     ],
#     "dripper_in": [
#         "병 위치로 이동 및 파지",
#         "드리퍼로 이동",
#         "주기 운동 추출",
#         "병 복귀",
#         "병 내려놓기",
#     ],
# }
#
#
# class StatusReporter:
#     """coffee_status_ui.py가 구독하는 진행 상태를 STATUS_TOPIC 으로 발행한다.
#
#     UI 쪽이 안 떠 있어도(구독자 없어도) publish는 그냥 버려지므로 이 노드의
#     동작에는 영향이 없다.
#     """
#
#     def __init__(self, node) -> None:
#         self._pub = node.create_publisher(String, STATUS_TOPIC, 10)
#         self._t0 = time.time()
#
#     def _publish(self, payload: dict) -> None:
#         payload["ts"] = time.time()
#         payload["elapsed"] = payload["ts"] - self._t0
#         msg = String()
#         msg.data = json.dumps(payload, ensure_ascii=False)
#         self._pub.publish(msg)
#
#     def step(self, stage: str, step_index: int, state: str = "running") -> None:
#         stage_index = STAGES.index(stage)
#         labels = STEP_LABELS[stage]
#         self._publish({
#             "type": "step",
#             "stage": stage,
#             "stage_index": stage_index,
#             "stage_total": len(STAGES),
#             "step": labels[step_index],
#             "step_index": step_index,
#             "step_total": len(labels),
#             "state": state,
#         })
#
#     def stage_done(self, stage: str) -> None:
#         self._publish({
#             "type": "stage_done",
#             "stage": stage,
#             "stage_index": STAGES.index(stage),
#             "stage_total": len(STAGES),
#         })
#
#     def finished(self) -> None:
#         self._publish({"type": "finished"})
#
#     def error(self, stage: str, message: str) -> None:
#         self._publish({"type": "error", "stage": stage, "message": message})


# =============================================================================
# main
# =============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)

    node = rclpy.create_node(
        NODE_NAME,
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = node

    try:
        import DSR_ROBOT2 as dsr2
    except ImportError as error:
        node.get_logger().error(f"DSR_ROBOT2 모듈을 불러오지 못했습니다: {error}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # DSR_ROBOT2 import 시점에 생성되는 ~100개 서비스 클라이언트가 컨트롤러와 DDS
    # discovery를 마칠 때까지 기다린다. set_singular_handling() 등 일부 API는
    # movej()와 달리 wait_for_service() 없이 곧바로 call_async()를 던지므로,
    # discovery가 끝나기 전에 첫 호출이 나가면 응답이 영영 오지 않아
    # spin_until_future_complete()가 무한 대기에 빠진다. (고정 sleep은 시스템
    # 부하에 따라 부족할 수 있어 실패 사례가 있었다 — 같은 노드로 대표 서비스
    # 하나가 실제로 매칭될 때까지 기다려서 나머지 클라이언트도 함께 뜨게 한다.)
    startup_probe = node.create_client(MoveJoint, "motion/move_joint")
    while not startup_probe.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("컨트롤러 서비스 연결 대기 중...")
    node.destroy_client(startup_probe)

    # -------------------------------------------------------------------
    # API 바인딩 (DRL 원본과 동일한 이름으로 사용)
    # -------------------------------------------------------------------
    movej = dsr2.movej
    movel = dsr2.movel
    movec = dsr2.movec
    amovel = dsr2.amovel
    move_periodic = dsr2.move_periodic
    task_compliance_ctrl = dsr2.task_compliance_ctrl
    release_compliance_ctrl = dsr2.release_compliance_ctrl
    set_stiffnessx = dsr2.set_stiffnessx
    set_digital_output = dsr2.set_digital_output
    set_tool = dsr2.set_tool
    set_tcp = dsr2.set_tcp
    get_tool = dsr2.get_tool
    get_tcp = dsr2.get_tcp
    drl_script_run = dsr2.drl_script_run
    get_drl_state = dsr2.get_drl_state
    get_robot_system = dsr2.get_robot_system
    set_singularity_handling = dsr2.set_singularity_handling
    set_velj = dsr2.set_velj
    set_accj = dsr2.set_accj
    set_velx = dsr2.set_velx
    set_accx = dsr2.set_accx
    wait = dsr2.wait
    posj = dsr2.posj
    posx = dsr2.posx

    DR_BASE = dsr2.DR_BASE
    DR_TOOL = dsr2.DR_TOOL
    DR_MV_MOD_ABS = dsr2.DR_MV_MOD_ABS
    DR_MV_MOD_REL = dsr2.DR_MV_MOD_REL
    DR_MV_RA_DUPLICATE = dsr2.DR_MV_RA_DUPLICATE
    DR_MV_RA_OVERRIDE = dsr2.DR_MV_RA_OVERRIDE
    DR_AVOID = dsr2.DR_AVOID
    # DR_OFF = dsr2.DR_OFF  # 원본에 있었으나 DSR_ROBOT2에 DR_OFF 상수 자체가 없어
    # AttributeError로 즉시 죽었음. set_velx() 3번째 인자로만 쓰였는데 그 인자
    # 자체가 실제 시그니처에 없어서 통째로 제거함 (아래 set_velx() 호출부 참고).
    ON = getattr(dsr2, "ON", 1)
    OFF = getattr(dsr2, "OFF", 0)

    # =====================================================================
    # Teach 포즈 (System.drvar 원본 값)
    # =====================================================================
    System_spoon_j = posj(-69.6, 53.27, 96.56, 25.63, 114.79, -171.03)
    System_spoon_l = posx(251.01, -118.46, 40.17, 87.01, 92.73, -3.35)
    System_grinder_j = posj(-40.58, -9.25, 131.96, 78.44, 107.42, -125.46)
    System_grinder_l = posx(371.82, 168.64, 360.0, 61.05, 86.52, 56.67)
    System_handle_j = posj(15.05, 35.75, 18.15, -1.7, 124.03, 13.38)
    System_handle_l = posx(527.43, 129.24, 278.22, 121.84, -180.0, 120.1)
    System_bottle_j_1 = posj(-19.18, 34.8, 22.24, -2.53, 124.42, -13.34)
    System_bottle_j_2 = posj(-43.18, 55.32, 88.81, 53.44, 114.38, -60.57)
    System_bottle_l = posx(401.53, 167.78, 71.68, 92.22, 89.54, 89.25)
    System_bottle_l2 = posx(401.53, 167.78, 76.68, 92.22, 89.54, 89.25)
    System_drip_j = posj(-26.64, 44.1, 73.65, 89.85, 91.27, -26.21)
    System_drip_l = posx(751.71, 41.51, 238.65, 60.13, 91.27, -92.13)
    System_home = posj(-71.33, 47.91, 97.52, 18.17, 114.49, -168.37)
    System_spoon_l_2 = posx(251.01, -118.46, 50.17, 87.01, 92.73, -3.35)

    # =====================================================================
    # OnRobot RG2 그리퍼 제어 (DO 1, 2 접점 조합)
    # =====================================================================
    def grip_close() -> None:
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def jar_grip_open() -> None:
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def handle_grip_open() -> None:
        set_digital_output(1, ON)
        set_digital_output(2, ON)

    def spoon_cup_grip_open() -> None:
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

    def apply_tcp(name: str, timeout_sec: float = 6.0, poll_interval: float = 0.1) -> bool:
        """set_tcp()를 DRL 프로그램으로 실행해서 TCP를 바꾼다.

        set_tcp()를 ROS2 서비스로 직접 부르면 컨트롤러가 매번
        "this command can only be used in manual mode"로 거부한다 (실측: 6초
        재시도 내내 100% 거부, 반영된 적 0회). 티치펜던트 Task Writer 프로그램은
        같은 명령이 통과되는 걸로 봐서, 로봇 모드(AUTONOMOUS/MANUAL) 자체보다는
        "실행 중인 DRL 프로그램 컨텍스트에서 나온 호출이냐"가 실제 조건으로
        보인다. drl_script_run()으로 set_tcp(...) 한 줄짜리 코드를 컨트롤러에
        네이티브 프로그램처럼 로드해서 실행시켜 이 조건을 맞춘다.
        """
        code = f'set_tcp("{name}")'
        start_ret = drl_script_run(get_robot_system(), code)
        if start_ret != 0:
            node.get_logger().error(f"drl_script_run 시작 실패: {code!r}, ret={start_ret}")
            return False

        deadline = time.monotonic() + timeout_sec
        # get_drl_state(): 0=PLAY, 1=STOP, 2=HOLD, 3=LAST. PLAY를 벗어날 때까지 대기.
        while get_drl_state() == 0:
            if time.monotonic() >= deadline:
                node.get_logger().error(
                    f"drl_script_run({code!r}) {timeout_sec:.0f}초 내에 끝나지 않음"
                )
                return False
            time.sleep(poll_interval)

        active = get_tcp()
        if active != name:
            node.get_logger().error(
                f"drl_script_run으로도 set_tcp('{name}') 반영 안 됨: 실제 활성={active}"
            )
            return False
        return True

    # status = StatusReporter(node)  # UI 상태 발행용 — 비활성화

    # 아래 movel()/amovel() 호출들: 원본에는 전부 app_type=DR_MV_APP_NONE 키워드가
    # 붙어 있었으나, 실제 movel()/amovel() 시그니처에 app_type 파라미터가 없어
    # TypeError가 나서 전부 제거함 (DRL 원본의 app 개념이 Python API 변환 과정에서
    # 잘못 옮겨진 것으로 보임, movec()의 ori 제거와 같은 종류의 수정).
    # =====================================================================
    # 서브 루틴 (Task Writer 서브 프로그램 대응)
    # =====================================================================
    def bean_drop() -> None:
        """스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입."""
        # status.step("bean_drop", 0)  # UI 상태 발행용 — 비활성화
        spoon_cup_grip_open()

        # status.step("bean_drop", 1)  # UI 상태 발행용 — 비활성화
        movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # status.step("bean_drop", 2)  # UI 상태 발행용 — 비활성화
        movej(System_spoon_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_spoon_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        grip_close()
        wait(1.0)

        # status.step("bean_drop", 3)  # UI 상태 발행용 — 비활성화
        movel(
            posx(0.0, 0.0, 100.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_grinder_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("bean_drop", 4)  # UI 상태 발행용 — 비활성화
        amovel(
            posx(32.0, -15.0, 0.0, 0.0, 0.0, 0.0), time=1.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(
            posj(0.0, 0.0, 0.0, 0.0, 0.0, 45.0), time=1.0,
            radius=0.0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("bean_drop", 5)  # UI 상태 발행용 — 비활성화
        movej(System_grinder_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)

        # status.step("bean_drop", 6)  # UI 상태 발행용 — 비활성화
        movel(
            System_spoon_l_2, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        movel(
            posx(0.0, 0.0, -10.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        spoon_cup_grip_open()
        wait(1.0)

        # status.step("bean_drop", 7)  # UI 상태 발행용 — 비활성화
        movej(System_home, radius=0.0, ra=DR_MV_RA_DUPLICATE)

    def grinder() -> None:
        """그라인더 손잡이를 잡고 컴플라이언스 제어로 원호를 그리며 분쇄."""
        # status.step("grinder", 0)  # UI 상태 발행용 — 비활성화
        movej(System_handle_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        jar_grip_open()
        movel(
            System_handle_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        grip_close()

        # status.step("grinder", 1)  # UI 상태 발행용 — 비활성화
        movej(
            posj(13.43, 31.67, 32.98, 0.19, 115.35, 11.45),
            radius=0.0, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("grinder", 2)  # UI 상태 발행용 — 비활성화
        task_compliance_ctrl()
        set_stiffnessx([1500.0, 1500.0, 2500.0, 150.0, 150.0, 200.0], time=0.5)

        # status.step("grinder", 3)  # UI 상태 발행용 — 비활성화
        movec(
            posx(0.0, -124.5, -5.0, 90.0, -179.99, 88.27),
            posx(-124.5, 0.0, -5.0, 90.0, -179.99, 88.27),
            radius=0.0,
            ref=101,  # teach pendant에 등록된 사용자 좌표계 101
            angle=[360.0, 0.0],
            ra=DR_MV_RA_OVERRIDE,
            # 원본에는 ori=DR_MV_ORI_FIXED, app_type=DR_MV_APP_NONE 도 있었으나
            # 실제 movec() 시그니처에 없는 키워드라 TypeError 나서 제거함 (DRL 원본의
            # app/ori 개념이 Python API 변환 과정에서 잘못 옮겨진 것으로 보임)
        )

        # status.step("grinder", 4)  # UI 상태 발행용 — 비활성화
        release_compliance_ctrl()
        jar_grip_open()

    def dripper_in() -> None:
        """드립 병을 잡고 주기 운동으로 커피를 추출."""
        # status.step("dripper_in", 0)  # UI 상태 발행용 — 비활성화
        movej(System_bottle_j_1, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_bottle_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        grip_close()
        movel(
            posx(0.0, 0.0, 5.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("dripper_in", 1)  # UI 상태 발행용 — 비활성화
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            posx(0.0, 0.0, 300.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_drip_j, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_drip_l, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )
        # 반영될 때까지 재시도 (apply_tcp 참고). 끝내 실패해도 move_periodic은
        # 진행한다 — 병을 잡고 있는 상태라 여기서 멈추는 게 더 위험할 수 있다.
        apply_tcp("joint4")

        # status.step("dripper_in", 2)  # UI 상태 발행용 — 비활성화
        move_periodic(
            amp=[0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            period=[1.0, 1.0, 1.0, 0.3, 1.0, 1.0],
            atime=0.0,
            repeat=5,
            ref=DR_TOOL,
        )

        # status.step("dripper_in", 3)  # UI 상태 발행용 — 비활성화
        apply_tcp(TCP_NAME)
        movel(
            posx(0.0, 0.0, -150.0, 0.0, 0.0, 0.0), radius=0.0, ref=DR_TOOL,
            mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE,
        )
        movej(System_bottle_j_2, radius=0.0, ra=DR_MV_RA_DUPLICATE)
        movel(
            System_bottle_l2, radius=0.0, ref=DR_BASE,
            mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE,
        )

        # status.step("dripper_in", 4)  # UI 상태 발행용 — 비활성화
        jar_grip_open()

    # =====================================================================
    # 전역 모션 파라미터 (원본 DRL 하단 설정과 동일)
    # =====================================================================
    set_singularity_handling(DR_AVOID)
    set_velj(VELJ_DEFAULT)
    set_accj(ACCJ_DEFAULT)
    # set_velx()는 실제 시그니처가 (vel1, vel2) 2개 인자까지만 받는데 원본은
    # 3번째 인자로 DR_OFF를 더 넘기고 있었음. DR_OFF는 DSR_ROBOT2에 존재하지도
    # 않는 상수라 AttributeError, 있었다 해도 TypeError가 났을 코드라 제거함.
    set_velx(VELX_LIN_DEFAULT, VELX_ROT_DEFAULT)
    set_accx(ACCX_LIN_DEFAULT, ACCX_ROT_DEFAULT)

    # =====================================================================
    # 메인 시퀀스 실행 (원본 while gLoop < 1: 1회 실행과 동일)
    # =====================================================================
    # current_stage = "bean_drop"  # UI 상태 발행(status.error)용 — 비활성화
    try:
        # 나머지 동작 전에 Tool/TCP부터 티치펜던트에 등록된 이름으로 맞춘다
        # (gear.py/move.py와 동일한 방식). 요청한 이름으로 실제 활성화됐는지
        # get_tool()/get_tcp()로 확인하고, 다르면 여기서 바로 멈춘다.
        tool_ret = set_tool(TOOL_NAME)
        tcp_ret = set_tcp(TCP_NAME)
        active_tool = get_tool()
        active_tcp = get_tcp()
        node.get_logger().info(
            f"set_tool ret={tool_ret}, 요청={TOOL_NAME}, 실제 활성={active_tool}"
        )
        node.get_logger().info(
            f"set_tcp ret={tcp_ret}, 요청={TCP_NAME}, 실제 활성={active_tcp}"
        )
        # # if active_tool != TOOL_NAME or active_tcp != TCP_NAME:
        #     raise RuntimeError(
        #         "Tool/TCP가 요청한 이름으로 활성화되지 않았습니다. "
        #         "티치펜던트에 해당 이름이 등록되어 있는지 확인하세요."
            # )

        node.get_logger().info("커피 추출 시스템 시작: bean_drop -> grinder -> dripper_in")

        node.get_logger().info("[1/3] bean_drop: 원두 투입 중...")
        bean_drop()
        # status.stage_done("bean_drop")  # UI 상태 발행용 — 비활성화

        # current_stage = "grinder"  # UI 상태 발행(status.error)용 — 비활성화
        node.get_logger().info("[2/3] grinder: 분쇄 중...")
        grinder()
        # status.stage_done("grinder")  # UI 상태 발행용 — 비활성화

        # current_stage = "dripper_in"  # UI 상태 발행(status.error)용 — 비활성화
        node.get_logger().info("[3/3] dripper_in: 드립 추출 중...")
        dripper_in()
        # status.stage_done("dripper_in")  # UI 상태 발행용 — 비활성화

        node.get_logger().info("커피 추출 시퀀스 완료.")
        # status.finished()  # UI 상태 발행용 — 비활성화

    except Exception as error:
        node.get_logger().error(f"실행 중 오류 발생: {error}")
        # status.error(current_stage, str(error))  # UI 상태 발행용 — 비활성화
        raise

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()