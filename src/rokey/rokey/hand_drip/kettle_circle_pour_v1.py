#!/usr/bin/env python3
"""
핸드드립 커피 - 주전자 원 그리기 물 따르기 모듈 (m0609)

동작 순서
    1. 지정 좌표(주전자 position)에서 주전자 손잡이를 잡는다.
    2. 평행(자세)을 유지하며 필터 position 위로 이동한다.
    3. 지정한 각도만큼 주전자를 기울여 따르는 pose를 취한다.
    4. 필터 중심 위에서 원을 그리며 물을 따른다. (뜸들이기 -> 여러 번 붓기)
    5. 주전자를 세우고 원위치(주전자 자리)에 내려놓은 뒤 홈으로 복귀한다.

주둥이(물 떨어지는 점) 기준 처리:
    펜던트에 TCP를 등록하지 않고, 그리퍼 TCP는 그대로 둔 채
    붓기 자세의 방향(A,B,C)으로 주둥이 오프셋을 base 좌표계로 변환해서
    - 기울일 때 주둥이가 필터 위에 고정되도록(주둥이 기준 회전) 하고
    - 원을 주둥이 기준으로 그린다.
    (자세가 일정하면 그리퍼가 그리는 원과 주둥이가 그리는 원은 반지름이 같으므로,
     주둥이 위치만 원 중심으로 맞춰주면 된다.)

좌표값은 teach_helper.py 로 캡처해서 아래 파라미터에 붙여넣으면 된다.
"""

import math

import numpy as np

import rclpy
import DR_init


# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight_1"
TCP_NAME = "GripperDA_v1"

# 주둥이(spout) 오프셋 - 그리퍼 TCP 기준 주둥이 끝 위치 (Tool 좌표계, mm)
#   측정: tool +X 170, +Z 40, 옆(Y) 0.
# 펜던트 TCP 등록 불필요. 코드가 이 오프셋으로 주둥이 위치를 직접 계산한다.
USE_SPOUT_OFFSET = True
# tool +X=위(4cm), tool +Y=앞(주둥이 방향 17cm), Z=0  (조그 검증 결과 반영)
SPOUT_OFFSET_MM = [40.0, 170.0, 0.0]


# =============================================================================
# 속도 / 가속도
# =============================================================================
VEL_J = 20.0            # 조인트 이동 속도 [deg/s]  (저속)
ACC_J = 30.0            # 조인트 이동 가속도 [deg/s^2]

VEL_X_TRANS = 50.0      # 직선 이동 속도 [mm/s]  (저속)
VEL_X_ROT = 20.0        # 회전 속도 [deg/s]

ACC_X_TRANS = 200.0     # 직선 이동 가속도 [mm/s^2]
ACC_X_ROT = 80.0        # 회전 가속도 [deg/s^2]


# =============================================================================
# ★ 사용자 지정 파라미터 ★
# =============================================================================

# 0) 홈 포즈 - 시작/종료 시 안전한 기본 자세 [j1..j6] (deg)
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 1) 주전자 position - 손잡이 잡는 위치 [x, y, z, rx, ry, rz]
KETTLE_GRIP_POS = [277.5, 112.6, 110.9, 83.7, 94.1, -176.4]
KETTLE_LIFT_MM = 120.0           # 잡은 뒤 들어올릴 높이
# 다 붓고 내려놓을 위치. None 이면 잡았던 자리에 그대로 내려놓음.
KETTLE_PLACE_POS = None

# 1-a) 손잡이 충돌 회피 - 직하강 대신 원호(moveC)로 옆에서 감아 들어감
#   GRIP_ARC_DIR : 원호가 들어오는 수평 방향(base 단위벡터). 손잡이를 피하는
#                  쪽으로 조정. 예) [1,0,0]=+X쪽에서, [0,-1,0]=-Y쪽에서 접근
#   GRIP_ARC_RADIUS_MM : 원호 반지름(클수록 완만하게 크게 돌아 들어옴)
# 회전축 Y (base X방향에서 접근). 반대쪽에서 와야 하면 [-1,0,0]
GRIP_ARC_DIR = [1.0, 0.0, 0.0]
GRIP_ARC_RADIUS_MM = 80.0
GRIP_ARC_VEL_MM_S = 25.0

# 2) 필터 position [x, y, z, rx, ry, rz]
FILTER_APPROACH_POS = None       # 선택: 좁으면 경유점 추가
FILTER_POUR_POS = [416.3933, 112.5524, 268.6189, 83.7431, 94.0589, -176.3823]

# 3) 따르는 기울기 [deg] / 기울일 축 ('rx' | 'ry' | 'rz')
#    주둥이가 tool +Y 방향이라, 붓기(주둥이 끝 내림)는 tool Z축(rz) 회전이다.
#    +30°에서 주둥이가 내려간다(붓기). 반대로 나오면 부호를 -로.
POUR_TILT_DEG = 30.0
POUR_TILT_AXIS = "rz"
# 기울이는 속도 - 물이 튀지 않게 천천히 [deg/s], [deg/s^2]
TILT_VEL_ROT = 6.0
TILT_ACC_ROT = 20.0
TILT_VEL_TRANS = 20.0            # 기울일 때 병진 속도 [mm/s]
TILT_ACC_TRANS = 80.0

# 4) 원 - 반지름 [mm], 나선 증가량 [mm/바퀴] (0이면 같은 원 반복)
CIRCLE_RADIUS_MM = 40.0
CIRCLE_SPIRAL_STEP_MM = 0.0

# 5) 원 그리는 방식 - "spiral" | "movec" | "periodic"
#    spiral : 중앙에서 시작 -> 밖으로 커졌다가 -> 다시 중앙으로 (핸드드립)
CIRCLE_METHOD = "spiral"

# 6) 속도 / 붓기
CIRCLE_VEL_MM_S = 20.0           # 원/나선 그리는 속도 (기울인 후 회전)
CIRCLE_PERIOD_SEC = 4.0          # periodic 한 바퀴 주기
POUR_LAPS = 3                    # movec 방식일 때 원 바퀴 수
SPIRAL_REVS = 3                  # spiral 방식일 때 회전 수 (밖으로/안으로 각각)
SPIRAL_STEPS_PER_REV = 24        # 나선 1회전당 점 개수 (클수록 매끄러움)
SPIRAL_BLEND_MM = 5.0            # 나선 점 사이 블렌딩 반경 [mm]


DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# =============================================================================
# 회전 유틸 - posx 방향은 ZYZ 오일러 (A,B,C): R = Rz(A) Ry(B) Rz(C)
# =============================================================================
def _rz(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def zyz_to_matrix(a, b, c):
    return _rz(a) @ _ry(b) @ _rz(c)


def matrix_to_zyz(rot):
    """R = Rz(A)Ry(B)Rz(C) -> (A,B,C) deg."""
    sy = math.sqrt(rot[0, 2] ** 2 + rot[1, 2] ** 2)
    b = math.degrees(math.atan2(sy, rot[2, 2]))
    if sy > 1e-6:
        a = math.degrees(math.atan2(rot[1, 2], rot[0, 2]))
        c = math.degrees(math.atan2(rot[2, 1], -rot[2, 0]))
    else:
        # B ~ 0 또는 180 (특이) - 여기 자세(B~163)에서는 발생하지 않음
        a = 0.0
        if rot[2, 2] > 0:
            c = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
        else:
            c = math.degrees(math.atan2(-rot[1, 0], -rot[0, 0]))
    return a, b, c


def axis_rot(axis, deg):
    if axis == "rx":
        return _rx(deg)
    if axis == "rz":
        return _rz(deg)
    return _ry(deg)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("hand_drip_pour", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_singularity_handling,
            set_velj,
            set_accj,
            set_velx,
            set_accx,
            movej,
            movel,
            movec,
            amove_periodic,
            set_digital_output,
            wait,
            get_current_posx,
            DR_AVOID,
            DR_BASE,
            DR_TOOL,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MV_RA_DUPLICATE,
            ON,
            OFF,
        )

        from DR_common2 import posx, posj

    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    offset = np.array(SPOUT_OFFSET_MM, dtype=float)

    # =========================================================================
    # 그리퍼 (F_3 모듈과 동일한 digital output 방식)
    # =========================================================================
    def gripper_open():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def gripper_close():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    # =========================================================================
    # 모션 헬퍼
    # =========================================================================
    def move_home():
        movej(posj(HOME_J), vel=VEL_J, acc=ACC_J)

    def move_linear_abs(pose):
        movel(posx(list(pose)), vel=VEL_X_TRANS, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    def move_linear_rel(delta):
        """현재 자세 기준 상대 이동(위치만 변경, 평행 유지)."""
        movel(posx(delta), vel=VEL_X_TRANS, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_REL)

    def move_tilt_abs(pose):
        """기울임 전용 저속 절대 이동 (회전 속도를 따로 낮춤)."""
        movel(posx(list(pose)),
              vel=[TILT_VEL_TRANS, TILT_VEL_ROT],
              acc=[TILT_ACC_TRANS, TILT_ACC_ROT],
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    def tilt_kettle_rel(angle_deg, axis):
        """(주둥이 계산을 안 쓸 때) Tool 좌표 상대 회전으로 기울이기 (저속)."""
        rel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rel[{"rx": 3, "ry": 4, "rz": 5}[axis]] = angle_deg
        movel(posx(rel), vel=TILT_VEL_ROT, acc=TILT_ACC_ROT,
              ref=DR_TOOL, mod=DR_MV_MOD_REL)

    def grip_arc_start(pose):
        """원호 접근 시작점(pose 기준 위+옆) 계산."""
        r = GRIP_ARC_RADIUS_MM
        u = np.array(GRIP_ARC_DIR, dtype=float)
        u = u / (np.linalg.norm(u) or 1.0)
        center = np.array(pose[:3], dtype=float) + r * np.array([0.0, 0.0, 1.0])
        start = center + r * u
        return list(start) + list(pose[3:6])

    def grip_arc_to(pose):
        """손잡이 위 직하강 대신 원호(moveC)로 옆에서 감아 pose 로 접근."""
        r = GRIP_ARC_RADIUS_MM
        u = np.array(GRIP_ARC_DIR, dtype=float)
        u = u / (np.linalg.norm(u) or 1.0)
        up = np.array([0.0, 0.0, 1.0])
        orient = list(pose[3:6])
        center = np.array(pose[:3], dtype=float) + r * up
        # 45도 지점을 경유점으로 (start -> via -> pose 원호)
        via = center + r * (math.cos(math.radians(45)) * u
                            - math.sin(math.radians(45)) * up)

        move_linear_abs(grip_arc_start(pose))     # 원호 시작점으로
        movec(posx(list(via) + orient), posx(list(pose)),
              vel=GRIP_ARC_VEL_MM_S, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    def gripper_pose_for_spout(spout_base, rot):
        """주둥이 끝을 spout_base(base)에 두고 방향이 rot 이 되는 그리퍼 posx."""
        p = np.array(spout_base, dtype=float) - rot @ offset
        a, b, c = matrix_to_zyz(rot)
        return [p[0], p[1], p[2], a, b, c]

    def circle_point(center, radius, angle_deg):
        a = math.radians(angle_deg)
        return posx([
            center[0] + radius * math.cos(a),
            center[1] + radius * math.sin(a),
            center[2],
            center[3], center[4], center[5],
        ])

    def draw_circles_movec(center, base_radius, laps):
        """center 중심으로 laps 바퀴. SPIRAL_STEP 만큼 매 바퀴 반지름 증가."""
        radius = base_radius
        movel(circle_point(center, radius, 0.0),
              vel=CIRCLE_VEL_MM_S, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)
        for _ in range(laps):
            movec(circle_point(center, radius, 90.0),
                  circle_point(center, radius, 180.0),
                  vel=CIRCLE_VEL_MM_S, acc=ACC_X_TRANS,
                  ref=DR_BASE, mod=DR_MV_MOD_ABS)
            movec(circle_point(center, radius, 270.0),
                  circle_point(center, radius, 360.0),
                  vel=CIRCLE_VEL_MM_S, acc=ACC_X_TRANS,
                  ref=DR_BASE, mod=DR_MV_MOD_ABS)
            radius += CIRCLE_SPIRAL_STEP_MM

    def draw_circles_periodic(base_radius, laps, period):
        """참고용 periodic. 위상차가 없어 완전한 원은 아님."""
        amove_periodic(
            amp=[base_radius, base_radius, 0.0, 0.0, 0.0, 0.0],
            period=[period, period, 0.0, 0.0, 0.0, 0.0],
            atime=0.2, repeat=laps, ref=DR_BASE,
        )
        wait(period * laps + 0.5)

    def draw_spiral(center, rmax, revs):
        """중앙(center)에서 밖으로 rmax까지 커졌다가 다시 중앙으로 수렴하는 나선.
        이 버전의 move_spiral은 INWARD를 지원하지 않아, 좌표를 직접 생성해서
        movel 블렌딩으로 그린다. 수평면(base X-Y) 나선이라 주둥이가 필터 위를 돈다."""
        steps = SPIRAL_STEPS_PER_REV
        n = max(1, int(revs * steps))
        blend = SPIRAL_BLEND_MM

        def seg(frac):
            return circle_point(center, rmax * frac, 360.0 * revs * frac)

        # 밖으로: 반지름 0 -> rmax
        for i in range(1, n + 1):
            movel(seg(i / n), vel=CIRCLE_VEL_MM_S, acc=ACC_X_TRANS,
                  radius=blend, ref=DR_BASE, mod=DR_MV_MOD_ABS,
                  ra=DR_MV_RA_DUPLICATE)
        # 안으로: 반지름 rmax -> 0
        for i in range(1, n + 1):
            movel(seg(1.0 - i / n), vel=CIRCLE_VEL_MM_S, acc=ACC_X_TRANS,
                  radius=blend, ref=DR_BASE, mod=DR_MV_MOD_ABS,
                  ra=DR_MV_RA_DUPLICATE)
        # 블렌딩 잔여를 없애고 중심으로 확실히 복귀
        movel(posx(list(center)), vel=CIRCLE_VEL_MM_S, acc=ACC_X_TRANS,
              ref=DR_BASE, mod=DR_MV_MOD_ABS)

    def draw_pour(center):
        """붓기 궤적 1회 - 방식에 따라 나선/원."""
        if CIRCLE_METHOD == "spiral":
            draw_spiral(center, CIRCLE_RADIUS_MM, SPIRAL_REVS)
        elif CIRCLE_METHOD == "movec":
            draw_circles_movec(center, CIRCLE_RADIUS_MM, POUR_LAPS)
        else:
            draw_circles_periodic(CIRCLE_RADIUS_MM, POUR_LAPS, CIRCLE_PERIOD_SEC)

    # =========================================================================
    # 메인 시퀀스
    # =========================================================================
    try:
        node.get_logger().info("핸드드립 물 따르기 시작")

        node.get_logger().info("set_tool...")
        set_tool(TOOL_NAME)
        node.get_logger().info("set_tcp...")
        set_tcp(TCP_NAME)
        node.get_logger().info("set_singularity_handling...")
        set_singularity_handling(DR_AVOID)
        node.get_logger().info("set vel/acc...")
        set_velj(VEL_J)
        set_accj(ACC_J)
        set_velx(VEL_X_TRANS, VEL_X_ROT)
        set_accx(ACC_X_TRANS, ACC_X_ROT)

        node.get_logger().info("get_current_posx 확인...")
        cur = get_current_posx()
        node.get_logger().info(f"현재 posx = {cur}")

        node.get_logger().info("move_home 시작...")
        move_home()
        node.get_logger().info("move_home 완료")

        # --- 1) 주전자 잡기 (원호 접근으로 손잡이 충돌 회피) ---------------
        node.get_logger().info("1) 주전자 손잡이 잡기 (원호 접근)")
        gripper_open()

        grip_arc_to(KETTLE_GRIP_POS)       # 옆에서 원호로 감아 들어감

        gripper_close()
        move_linear_rel([0.0, 0.0, KETTLE_LIFT_MM, 0.0, 0.0, 0.0])

        # --- 2) 필터 위로 평행 이동 ----------------------------------------
        node.get_logger().info("2) 필터 위로 평행 이동")
        if FILTER_APPROACH_POS is not None:
            move_linear_abs(FILTER_APPROACH_POS)
        move_linear_abs(FILTER_POUR_POS)

        # --- 3) 기울이기 (주둥이 기준) -------------------------------------
        node.get_logger().info(f"3) {POUR_TILT_DEG}도 기울이기")

        if USE_SPOUT_OFFSET:
            # 현재(수평) 그리퍼 포즈에서 주둥이 위치(base) 계산
            p0 = list(get_current_posx()[0])
            rot0 = zyz_to_matrix(p0[3], p0[4], p0[5])
            spout_base = np.array(p0[:3], dtype=float) + rot0 @ offset

            # 주둥이를 고정한 채 기울인 자세로 이동
            rot_tilt = rot0 @ axis_rot(POUR_TILT_AXIS, POUR_TILT_DEG)
            tilt_pose = gripper_pose_for_spout(spout_base, rot_tilt)
            level_pose = gripper_pose_for_spout(spout_base, rot0)  # == p0

            move_tilt_abs(tilt_pose)       # 천천히 기울임
            center = list(tilt_pose)

            def go_tilt():
                move_tilt_abs(tilt_pose)

            def go_level():
                move_tilt_abs(level_pose)
        else:
            # 주둥이 계산 없이 그리퍼 기준 상대 회전
            tilt_kettle_rel(POUR_TILT_DEG, POUR_TILT_AXIS)
            center = list(get_current_posx()[0])

            def go_tilt():
                tilt_kettle_rel(POUR_TILT_DEG, POUR_TILT_AXIS)

            def go_level():
                tilt_kettle_rel(-POUR_TILT_DEG, POUR_TILT_AXIS)

        # --- 4) 붓기: 기울인 상태로 나선(또는 원) 그리기 ------------------
        node.get_logger().info("4) 붓기 - 나선 그리기")
        draw_pour(center)

        # --- 5) 붓기 멈추고 원위치로 ---------------------------------------
        node.get_logger().info("5) 붓기 멈추고 세워서 복귀")
        move_linear_abs(center)            # 나선 중심으로 복귀
        go_level()                         # 주전자 세움 = 붓기 멈춤

        place = KETTLE_PLACE_POS if KETTLE_PLACE_POS is not None else KETTLE_GRIP_POS
        grip_arc_to(place)                 # 원호로 감아 내려놓기 (손잡이 충돌 회피)
        gripper_open()
        move_linear_abs(grip_arc_start(place))   # 원호 시작점으로 후퇴

        move_home()
        node.get_logger().info("핸드드립 물 따르기 완료")

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
