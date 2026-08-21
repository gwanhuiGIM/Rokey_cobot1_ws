#!/usr/bin/env python3
"""
티칭 헬퍼 - 현재 로봇 포즈를 읽어서 출력한다.

사용법
    1. 로봇 드라이버(bringup)가 떠 있는 상태에서 실행한다.
    2. 로봇을 원하는 위치로 옮긴다.
         - 티치펜던트의 핸드가이딩(직접교시) 버튼을 눌러 손으로 옮기거나
         - control_GUI 로 옮기거나
         - COMPLIANCE_HANDGUIDE = True 로 두면 이 스크립트가 순응제어를 켜서
           로봇을 손으로 밀어 옮길 수 있게 해준다. (주의: 아래 설명 참고)
    3. 터미널에서 Enter 를 누르면 현재 posx / posj 를 출력한다.
    4. 출력된 posx 줄을 그대로 kettle_circle_pour.py 파라미터에 붙여넣으면 된다.
    5. 'q' + Enter 로 종료.
"""

import rclpy
import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight_1"
TCP_NAME = "GripperDA_v1"

# True 로 두면 순응제어(soft)를 켜서 로봇을 손으로 밀 수 있다.
# 단, 완전한 중력보상 핸드가이딩이 아니라 "물렁하게" 만드는 방식이라
# 무거운 주전자를 든 상태에서는 처질 수 있다. 확신 없으면 False 로 두고
# 티치펜던트의 직접교시 버튼을 사용하는 것을 권장.
COMPLIANCE_HANDGUIDE = False


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("teach_helper", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            get_current_posx,
            get_current_posj,
            task_compliance_ctrl,
            set_stiffnessx,
            release_compliance_ctrl,
        )
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    set_tool(TOOL_NAME)
    set_tcp(TCP_NAME)

    compliance_on = False
    if COMPLIANCE_HANDGUIDE:
        task_compliance_ctrl(stx=[500.0, 500.0, 500.0, 100.0, 100.0, 100.0])
        set_stiffnessx([500.0, 500.0, 500.0, 100.0, 100.0, 100.0], time=0.0)
        compliance_on = True
        node.get_logger().info("순응제어 ON - 로봇을 손으로 밀어 옮기세요.")

    captured = []

    print("\n=== 티칭 헬퍼 ===")
    print("로봇을 옮긴 뒤 Enter -> 현재 포즈 캡처 / 'q' + Enter -> 종료\n")

    try:
        while rclpy.ok():
            key = input("Enter=캡처, q=종료 > ").strip().lower()
            if key == "q":
                break

            posx_val, sol = get_current_posx()
            posj_val = get_current_posj()

            px = [round(float(v), 2) for v in posx_val]
            pj = [round(float(v), 2) for v in posj_val]

            idx = len(captured) + 1
            captured.append(px)

            print(f"\n--- 포즈 #{idx} 캡처됨 (solution space={sol}) ---")
            print(f"posx: {px}")
            print(f"posj: {pj}")
            print("붙여넣기용:")
            print(f"    P{idx} = {px}\n")

    except (KeyboardInterrupt, EOFError):
        pass

    finally:
        if compliance_on:
            try:
                release_compliance_ctrl()
            except Exception:
                pass

        if captured:
            print("\n=== 캡처 요약 ===")
            for i, px in enumerate(captured, start=1):
                print(f"P{i} = {px}")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
