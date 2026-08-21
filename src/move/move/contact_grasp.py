#!/usr/bin/env python3
# =============================================================
# contact_grasp.py — 힘제어(순응) 접촉 탐지 → 센터링 → 파지 → 무게검증 (Vision 없이)
# -------------------------------------------------------------
# 실행: ros2 run move contact_grasp   (또는 --selftest 로 순수 로직 자체검증)
# 노드: contact_grasp (ns=dsr01)
# 의존: rclpy, doosan-robot2(DSR_ROBOT2, DR_common2)
# -------------------------------------------------------------
# 대상: M0609 + RG2 | 좌표계: 힘제어/접촉판정 DR_BASE, 무게판정 DR_TOOL
# 전제: 시작 시 로봇 TCP가 대상 예상위치 상공(안전높이)에서 정지·비접촉,
#       그리퍼가 아래(툴 Z ↓)를 향하도록 티칭된 자세일 것.
# 주의: set_external_force_reset()이 이 바인딩에 없어 소프트웨어 tare 사용.
#       파지깊이·바닥하한·존 크기 등 현장값은 아래 TODO 참조.
#       얇고 가벼운 물체(쇠막대 등)는 측면 탐침 시 넘어질 수 있음
#       → ENABLE_CENTERING=False 또는 LATERAL_FORCE_N 축소.
# -------------------------------------------------------------
# 탐지 원리: task_compliance_ctrl + set_desired_force 로 목표하중만큼만 살살
#   밀고, tare 기준 |ΔF|가 임계를 넘으면 접촉으로 판정. 접촉 시 로봇이 순응해
#   부드럽게 멈추고, 물체가 넘어져도 작은 힘으로만 진행하다 바닥 안전하한/
#   최대이동에서 중단하므로 충돌·안전모드 발동을 피한다. (MoveStop 미사용)
# =============================================================

import time as pytime

import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight_gripper"
TCP_NAME = "GripperDA_v1"

# 속도/가속도 (mm/s, deg/s, mm/s^2, deg/s^2) — 경유 이동용
VEL_X_TRANS = 250.0
VEL_X_ROT = 80.625
ACC_X_TRANS = 1000.0
ACC_X_ROT = 322.5
VEL_APPROACH_MM = 60.0   # 경유 이동 속도 (mm/s)

# ---- 힘제어 탐침 파라미터 (접촉판정 임계 < 목표하중 이어야 접촉 시 도달) ----
CONTACT_FORCE_N = 5.0        # 수직 접촉 판정 [N]
PUSH_FORCE_Z_N = 10.0        # 수직 탐침 목표 하중 [N]
LATERAL_FORCE_N = 2.5        # 측면 접촉 판정 [N] — 얇은 물체 넘어짐 방지 위해 낮게
PUSH_FORCE_XY_N = 5.0        # 측면 탐침 목표 하중 [N]

PROBE_STIFFNESS_N_M = 500.0  # 탐침 축 순응 강성 (낮을수록 부드러움) [N/m]
HOLD_STIFFNESS_N_M = 3000.0  # 비탐침 병진축 유지 강성 [N/m]
HOLD_STIFFNESS_ROT = 200.0   # 회전축 유지 강성 [Nm/rad]
COMPLY_TIME_SEC = 0.5        # 순응/힘제어 전환 시간 [s]

FLOOR_Z_MM = -60.0           # TODO(확정 필요): 바닥 안전 하한(이 아래로 안 내려감, mm)
SEARCH_MAX_DEPTH_MM = 80.0   # 미접촉 시 최대 하강 (mm)
PROBE_TIMEOUT_SEC = 12.0     # 단일 탐침 타임아웃 (s)
POLL_DT_SEC = 0.02          # 힘 폴링 주기 (s)

# 측면 센터링
ENABLE_CENTERING = True     # False면 시작 x/y를 그대로 파지 중심으로 사용
ZONE_HALF_MM = 80.0         # 측면 탐침 시작 반경 (예상 물체 최대 반폭보다 크게, mm)
LATERAL_DROP_MM = 20.0      # 상면보다 얼마나 아래에서 측면 탐침할지 (mm)
LATERAL_MAX_TRAVEL_MM = ZONE_HALF_MM + 80.0  # 측면 미접촉 시 최대 진입 (mm)

# 파지
GRASP_DEPTH_MM = 30.0        # TODO(확정 필요): 상면 아래 이 깊이에서 그리퍼 폐쇄 (mm)
LIFT_DISTANCE_MM = 100.0     # 파지 후 들어올릴 높이 (mm)

# 파지 성공 판정 (weight_change_monitor.py와 동일 방식)
GRASP_MASS_THRESHOLD_KG = 0.01
G = 9.81

# 축 인덱스 (get_tool_force / posx: [X,Y,Z,A,B,C])
AXIS_X, AXIS_Y, AXIS_Z = 0, 1, 2

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def contact_exceeded(force, offset, axis, threshold_n):
    """소프트웨어 tare 기준으로 해당 축 외력 크기가 임계를 넘으면 접촉."""
    return abs(force[axis] - offset[axis]) >= threshold_n


def grasp_success(fz_before, fz_after, threshold_kg=GRASP_MASS_THRESHOLD_KG, g=G):
    """들어올리기 전/후 Fz 차이로 실제로 뭔가 잡혔는지 판정."""
    return abs(fz_after - fz_before) / g >= threshold_kg


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("contact_grasp", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_velx,
            set_accx,
            set_singularity_handling,
            set_ref_coord,
            set_digital_output,
            get_tool_force,
            get_current_posx,
            movel,
            wait,
            task_compliance_ctrl,
            set_stiffnessx,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            ON,
            OFF,
            DR_AVOID,
            DR_BASE,
            DR_TOOL,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MV_RA_DUPLICATE,
            DR_FC_MOD_REL,
        )
        from DR_common2 import posx
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # ------------------------------------------------------------------
    # 그리퍼 (디지털 I/O)
    # ------------------------------------------------------------------
    def gripper_off():   # 열기
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def gripper_on():    # 닫기(파지)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    # ------------------------------------------------------------------
    # 좌표/힘 헬퍼
    # ------------------------------------------------------------------
    def posx_now():
        r = get_current_posx(DR_BASE)
        return list(r[0]) if isinstance(r, tuple) else list(r)

    def force_base():
        f = get_tool_force(ref=DR_BASE)   # 실패 시 -1 반환 방어 (repo 관례)
        if f == -1:
            raise RuntimeError("get_tool_force(DR_BASE) 실패")
        return list(f)

    def software_tare(samples=5):
        """정지·비접촉 상태에서 현재 외력을 평균내 offset(6D)으로 사용."""
        acc = [0.0] * 6
        for _ in range(samples):
            acc = [a + b for a, b in zip(acc, force_base())]
            pytime.sleep(POLL_DT_SEC)
        return [a / samples for a in acc]

    def move_linear_abs(target, vel=VEL_APPROACH_MM):
        movel(target, vel=vel, acc=ACC_X_TRANS, radius=0.0,
              ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    def compliant_probe(axis, direction, contact_n, push_n, max_travel_mm, offset):
        """힘제어(순응)로 axis 방향(+1/-1)으로 push_n 만큼 살살 밀며 접촉 탐지.
        접촉(|ΔF|>=contact_n) 시 True. 미접촉이면 max_travel / 바닥하한 / 타임아웃에서
        정지하고 False. 항상 힘제어를 해제하고 복귀(위치제어 재개)."""
        set_ref_coord(DR_BASE)
        stiffness = [HOLD_STIFFNESS_N_M] * 3 + [HOLD_STIFFNESS_ROT] * 3
        stiffness[axis] = PROBE_STIFFNESS_N_M   # 탐침 축만 부드럽게
        task_compliance_ctrl(stx=stiffness, time=COMPLY_TIME_SEC)
        set_stiffnessx(stiffness, time=COMPLY_TIME_SEC)

        fd = [0.0] * 6
        fd[axis] = direction * push_n
        fdir = [0] * 6
        fdir[axis] = 1
        set_desired_force(fd=fd, dir=fdir, time=0.0, mod=DR_FC_MOD_REL)

        start = posx_now()
        contacted = False
        t0 = pytime.monotonic()
        try:
            while rclpy.ok() and pytime.monotonic() - t0 < PROBE_TIMEOUT_SEC:
                if contact_exceeded(force_base(), offset, axis, contact_n):
                    contacted = True
                    break
                cur = posx_now()
                if abs(cur[axis] - start[axis]) >= max_travel_mm:
                    break                                   # 미접촉 최대이동
                if cur[AXIS_Z] <= FLOOR_Z_MM:
                    node.get_logger().warn("바닥 안전 하한 도달 — 탐침 중단")
                    break
                pytime.sleep(POLL_DT_SEC)
        finally:
            release_force(time=0.0)
            release_compliance_ctrl()
        return posx_now(), contacted

    # ==================================================================
    # 파이프라인
    # ==================================================================
    try:
        node.get_logger().info("접촉 기반 물체 감지 + 파지 시작")

        set_tool(TOOL_NAME)
        set_tcp(TCP_NAME)
        set_singularity_handling(DR_AVOID)
        set_velx(VEL_X_TRANS, VEL_X_ROT)
        set_accx(ACC_X_TRANS, ACC_X_ROT)

        gripper_on()   # 닫고 시작: 강체 팁으로 탐침 + tare/무게측정 상태 일치

        # 시작 자세(안전높이 상공, 아래 향함) 캡처 — 이후 접근자세로 재사용
        start_pose = posx_now()
        x0, y0, safe_z = start_pose[0], start_pose[1], start_pose[2]
        approach_abc = start_pose[3:6]

        def pose_at(x, y, z):
            return posx([x, y, z, approach_abc[0], approach_abc[1], approach_abc[2]])

        # 0) 소프트웨어 tare (정지·비접촉 상태에서)
        wait(0.5)
        offset = software_tare()
        node.get_logger().info(
            "tare offset (BASE) Fx=%.2f Fy=%.2f Fz=%.2f N"
            % (offset[0], offset[1], offset[2])
        )

        # 무게검증용 파지 전 Fz (툴 프레임, 빈·닫힌 그리퍼 — fz_after와 상태 일치)
        f_tool_before = get_tool_force(ref=DR_TOOL)
        if f_tool_before == -1:
            raise RuntimeError("get_tool_force(DR_TOOL) 실패")
        fz_before = f_tool_before[2]

        # 1) 수직 탐침(힘제어) → 상면(TOP) Z 확정
        pose, hit = compliant_probe(
            AXIS_Z, -1, CONTACT_FORCE_N, PUSH_FORCE_Z_N, SEARCH_MAX_DEPTH_MM, offset
        )
        if not hit:
            node.get_logger().warn("접촉 없음: 해당 위치에 물체가 없습니다.")
            move_linear_abs(pose_at(x0, y0, safe_z))   # 안전높이 복귀
            return
        z_top = pose[2]
        node.get_logger().info("TOP 접촉: z_top=%.1f mm" % z_top)
        move_linear_abs(pose_at(x0, y0, safe_z))       # 상공 복귀

        # 2) (옵션) 측면 4방향 힘제어 탐침 → 물체 중심(x,y) 산출
        grip_x, grip_y = x0, y0
        if ENABLE_CENTERING:
            probe_z = z_top - LATERAL_DROP_MM
            edges = {}
            # (라벨, 시작x, 시작y, 축, 미는방향)
            lateral_plan = [
                ("LEFT",  x0 - ZONE_HALF_MM, y0, AXIS_X, +1),
                ("RIGHT", x0 + ZONE_HALF_MM, y0, AXIS_X, -1),
                ("FRONT", x0, y0 - ZONE_HALF_MM, AXIS_Y, +1),
                ("BACK",  x0, y0 + ZONE_HALF_MM, AXIS_Y, -1),
            ]
            for label, sx, sy, axis, direction in lateral_plan:
                move_linear_abs(pose_at(sx, sy, safe_z))   # 상공 경유(끌림 방지)
                move_linear_abs(pose_at(sx, sy, probe_z))  # 존 가장자리 하강
                p, h = compliant_probe(
                    axis, direction, LATERAL_FORCE_N, PUSH_FORCE_XY_N,
                    LATERAL_MAX_TRAVEL_MM, offset,
                )
                move_linear_abs(pose_at(sx, sy, probe_z))  # 밀지 않도록 후퇴
                if not h:
                    node.get_logger().warn("%s 측면 접촉 실패 — 종료" % label)
                    move_linear_abs(pose_at(sx, sy, safe_z))   # 안전높이 복귀
                    return
                edges[label] = p[axis]
                node.get_logger().info("%s 접촉: %.1f mm" % (label, p[axis]))

            if "LEFT" in edges and "RIGHT" in edges:
                grip_x = (edges["LEFT"] + edges["RIGHT"]) / 2.0
            if "FRONT" in edges and "BACK" in edges:
                grip_y = (edges["FRONT"] + edges["BACK"]) / 2.0
            node.get_logger().info("파지 중심 = (%.1f, %.1f) mm" % (grip_x, grip_y))

        # 3) 중심 정렬 → 개방(물체 감싸기) → 파지 깊이까지 하강 → 폐쇄
        move_linear_abs(pose_at(grip_x, grip_y, safe_z))
        gripper_off()   # 물체를 감싸도록 개방
        move_linear_abs(pose_at(grip_x, grip_y, z_top - GRASP_DEPTH_MM))
        node.get_logger().info("파지 시도.")
        gripper_on()

        # 4) 들어올려 무게 변화로 파지 성공 검증
        movel([0.0, 0.0, LIFT_DISTANCE_MM, 0.0, 0.0, 0.0],
              vel=VEL_X_TRANS, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)

        f_tool_after = get_tool_force(ref=DR_TOOL)
        if f_tool_after == -1:
            raise RuntimeError("get_tool_force(DR_TOOL) 실패")
        fz_after = f_tool_after[2]

        if grasp_success(fz_before, fz_after):
            node.get_logger().info("파지 성공 확인 (무게 증가 감지).")
        else:
            node.get_logger().warn(
                "파지 실패: 그리퍼가 닫혔지만 물체 무게가 감지되지 않음."
            )
            gripper_off()

    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")
    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _demo():
    # 접촉 판정: tare offset 기준 |ΔF|
    off = [1.0, -0.5, 2.0, 0.0, 0.0, 0.0]
    assert contact_exceeded([1.0, -0.5, 7.5, 0, 0, 0], off, AXIS_Z, 5.0) is True
    assert contact_exceeded([1.0, -0.5, 5.0, 0, 0, 0], off, AXIS_Z, 5.0) is False
    assert contact_exceeded([3.6, -0.5, 2.0, 0, 0, 0], off, AXIS_X, 2.5) is True

    # 300g 물체 → Fz 약 2.94N 증가 → 파지 성공
    assert grasp_success(fz_before=-1.2, fz_after=-1.2 - 0.3 * G) is True
    # 허공 파지 → Fz 변화 없음 → 실패
    assert grasp_success(fz_before=-1.2, fz_after=-1.25) is False
    print("contact_grasp self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo()
    else:
        main()
