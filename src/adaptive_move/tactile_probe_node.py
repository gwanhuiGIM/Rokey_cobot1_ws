#!/usr/bin/env python3
# ============================================================
#  tactile_probe_node.py
#  M0609 + RG2 : 촉각 탐침(guarded move) 기반 강체 표면 탐지
#                + RViz2 MarkerArray 디버깅 시각화
#
#  실행 컨텍스트 : ROS2(rclpy) 노드 안에서 DSR_ROBOT2 래퍼로
#                 DRL 명령(movel / get_tool_force 등)을 호출.
#  구조/관례     : __gear_move.py 와 동일한 import·정지·설정 패턴을 따름.
#
#  [전제] 별도 터미널에서 로봇 드라이버(bringup)가 떠 있어야 함:
#    ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
#         mode:=real host:=192.168.137.100 model:=m0609
# ============================================================

import time as pytime

import rclpy
import DR_init


# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# 워크셀에 등록된 이름 (캘리브레이션: 쟁반/그리퍼만 포함, 대상물체 제외)
TOOL_NAME = "Tool Weight_gripper"
TCP_NAME = "GripperDA_v1"

# =============================================================================
# 탐침 파라미터 (현장 튜닝)
# =============================================================================
BASE_FRAME = "base_0"          # RViz Fixed Frame (URDF 루트에 맞출 것)

ZONE_CENTER_X = 400.0          # 150x150 영역 중심 (BASE, mm)
ZONE_CENTER_Y = 0.0
ZONE_HALF_MM = 75.0            # 영역 반폭 (mm)

SAFE_Z_MM = 250.0             # 탐침 시작(안전) 높이 (mm)
FLOOR_Z_MM = 50.0             # 바닥(최저 하강 한계) (mm)
LATERAL_DROP_MM = 10.0        # 상면보다 얼마나 아래에서 측면 탐침할지 (mm)

F_THRESH_Z_N = 5.0            # 수직 접촉 감지 임계 [N]
F_THRESH_XY_N = 3.0          # 측면 접촉 감지 임계 [N] (물체 밀림 방지 위해 낮게)

VEL_J = 60.0
ACC_J = 100.0

VEL_APPROACH_MM = 60.0        # 경유 이동 속도
VEL_PROBE_MM = 20.0          # 탐침 속도 (느리게)
ACC_MM = 200.0

POLL_DT_SEC = 0.02           # 힘 폴링 주기
MOVE_TIMEOUT_SEC = 15.0      # 단일 guarded move 타임아웃

# 하강 시 접근 자세 (쟁반/그리퍼가 수평 아래를 향하도록; 현장값으로 교체)
DOWN_A, DOWN_B, DOWN_C = 0.0, 180.0, 0.0

# 표면별 색상 (r, g, b) : RViz 마커 구분용
SURFACE_COLOR = {
    "TOP":   (0.20, 0.55, 0.85),  # 파랑
    "LEFT":  (0.85, 0.45, 0.15),  # 주황
    "RIGHT": (0.85, 0.65, 0.15),  # 황색
    "FRONT": (0.20, 0.70, 0.45),  # 청록
    "BACK":  (0.55, 0.35, 0.70),  # 보라
}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "m0609_tactile_probe",
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = node

    # ------------------------------------------------------------------
    # 두산 로봇 모듈 import (node 주입 이후에!)
    # ------------------------------------------------------------------
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            set_singularity_handling,
            set_velj,
            set_accj,
            movej,
            movel,
            amovel,
            get_current_posx,
            check_force_condition,
            set_external_force_reset,
            wait,
            DR_AVOID,
            DR_BASE,
            DR_AXIS_X,
            DR_AXIS_Y,
            DR_AXIS_Z,
            DR_MV_MOD_ABS,
            DR_MV_MOD_REL,
            DR_MV_RA_DUPLICATE,
            DR_QSTOP,
        )
        from DR_common2 import posj, posx
        from dsr_msgs2.srv import MoveStop
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from rclpy.qos import QoSProfile, DurabilityPolicy

    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # ==================================================================
    # 비동기 모션 정지 서비스 (__gear_move.py 와 동일 패턴)
    # ==================================================================
    stop_client = node.create_client(MoveStop, "motion/move_stop")

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

    # ==================================================================
    # RViz2 마커 발행 (TRANSIENT_LOCAL: 늦게 구독해도 유지)
    # ==================================================================
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_pub = node.create_publisher(MarkerArray, "probe_markers", qos)
    marker_arr = MarkerArray()
    marker_state = {"id": 0}

    def add_contact_marker(surface, xyz_mm):
        r, g, b = SURFACE_COLOR.get(surface, (0.6, 0.6, 0.6))
        x, y, z = xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0  # mm→m

        sphere = Marker()
        sphere.header.frame_id = BASE_FRAME
        sphere.ns = "contact"
        sphere.id = marker_state["id"]; marker_state["id"] += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=x, y=y, z=z)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = r, g, b, 1.0
        marker_arr.markers.append(sphere)

        label = Marker()
        label.header.frame_id = BASE_FRAME
        label.ns = "label"
        label.id = marker_state["id"]; marker_state["id"] += 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(x=x, y=y, z=z + 0.02)
        label.pose.orientation.w = 1.0
        label.scale.z = 0.015
        label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
        label.text = surface
        marker_arr.markers.append(label)

        marker_pub.publish(marker_arr)
        node.get_logger().info(
            "contact [%-5s] x=%.1f y=%.1f z=%.1f mm"
            % (surface, xyz_mm[0], xyz_mm[1], xyz_mm[2])
        )

    # ==================================================================
    # 좌표/모션 헬퍼
    # ==================================================================
    def posx_now():
        # 버전에 따라 (list, sol) 튜플 또는 list 반환 → 방어적 처리
        r = get_current_posx(DR_BASE)
        return list(r[0]) if isinstance(r, tuple) else list(r)

    def move_joint(target):
        movej(target, vel=VEL_J, acc=ACC_J, radius=0.0, ra=DR_MV_RA_DUPLICATE)

    def move_linear_abs(target, vel=VEL_APPROACH_MM):
        movel(target, vel=vel, acc=ACC_MM, radius=0.0,
              ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    # guarded_move의 axis 인자(0=X,1=Y,2=Z)를 DR_AXIS_* 상수로 매핑
    _AXIS_MAP = {0: DR_AXIS_X, 1: DR_AXIS_Y, 2: DR_AXIS_Z}

    def guarded_move(axis, delta_mm, f_thresh):
        """
        현재 위치에서 axis(0=X,1=Y,2=Z)로 delta_mm 만큼 비동기 직선 이동하되,
        check_force_condition()으로 해당 축 외력 크기가 f_thresh를 넘는 순간
        MoveStop 으로 즉시 정지. (매뉴얼 6.2.13: min/max는 부호 없는 크기 비교)
        반환 : (접촉/종료 시점 태스크좌표, 접촉여부 bool)
        """
        start = posx_now()
        target = list(start)
        target[axis] += delta_mm

        amovel(posx(target), vel=VEL_PROBE_MM, acc=ACC_MM,
               ref=DR_BASE, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

        t0 = pytime.monotonic()
        contacted = False
        dr_axis = _AXIS_MAP[axis]
        while rclpy.ok() and pytime.monotonic() - t0 < MOVE_TIMEOUT_SEC:
            # min=f_thresh만 지정 → "차이가 +이면 True", 즉 |F_axis| > f_thresh 에서 True
            if check_force_condition(dr_axis, min=f_thresh, ref=DR_BASE):
                stop_robot(DR_QSTOP)
                contacted = True
                break
            pytime.sleep(POLL_DT_SEC)

        return posx_now(), contacted

    # ==================================================================
    # 탐침 파이프라인
    # ==================================================================
    try:
        node.get_logger().info("M0609 촉각 탐침 시작")

        set_tool(TOOL_NAME)
        set_tcp(TCP_NAME)
        set_singularity_handling(DR_AVOID)
        set_velj(VEL_J)
        set_accj(ACC_J)

        cx, cy = ZONE_CENTER_X, ZONE_CENTER_Y

        # 0) 영역 중심 상공으로 이동 후 외력 초기화(tare)
        #    반드시 비접촉·정지 상태에서 호출 (매뉴얼 6.1.9 주의사항)
        move_linear_abs(posx([cx, cy, SAFE_Z_MM, DOWN_A, DOWN_B, DOWN_C]))
        wait(0.5)
        set_external_force_reset(1)   # mode=1: 현재 외력을 offset으로 설정
        wait(0.2)

        # 1) 수직 탐침 : 상면(TOP) → Z_top 확정 (높이 변수 제거)
        pose, hit = guarded_move(2, -(SAFE_Z_MM - FLOOR_Z_MM), F_THRESH_Z_N)
        if not hit:
            node.get_logger().warn(
                "중앙 수직 탐침 미접촉 → 물체가 중앙에 없음(격자 탐색 필요)"
            )
            raise RuntimeError("TOP 미검출")
        z_top = pose[2]
        add_contact_marker("TOP", pose[:3])

        probe_z = z_top - LATERAL_DROP_MM

        # 2) 측면 탐침 4방향 : 폭·중심 확정 (XY + 파지폭 변수 제거)
        edges = {}
        lateral_plan = [
            ("LEFT",  cx - ZONE_HALF_MM, cy, 0, +ZONE_HALF_MM),
            ("RIGHT", cx + ZONE_HALF_MM, cy, 0, -ZONE_HALF_MM),
            ("FRONT", cx, cy - ZONE_HALF_MM, 1, +ZONE_HALF_MM),
            ("BACK",  cx, cy + ZONE_HALF_MM, 1, -ZONE_HALF_MM),
        ]
        for label, sx, sy, axis, delta in lateral_plan:
            start_pose = posx([sx, sy, probe_z, DOWN_A, DOWN_B, DOWN_C])
            move_linear_abs(start_pose)                     # 영역 가장자리로
            pose, hit = guarded_move(axis, delta, F_THRESH_XY_N)
            if hit:
                edges[label] = pose[axis]
                add_contact_marker(label, pose[:3])
            move_linear_abs(start_pose)                     # 밀지 않도록 후퇴

        # 3) 중심·폭 계산 → 파지 자세 산출
        if "LEFT" in edges and "RIGHT" in edges:
            grip_cx = (edges["LEFT"] + edges["RIGHT"]) / 2.0
            width_x = abs(edges["RIGHT"] - edges["LEFT"])
        else:
            grip_cx, width_x = cx, None
        if "FRONT" in edges and "BACK" in edges:
            grip_cy = (edges["FRONT"] + edges["BACK"]) / 2.0
            width_y = abs(edges["BACK"] - edges["FRONT"])
        else:
            grip_cy, width_y = cy, None

        node.get_logger().info("=== 탐지 결과 ===")
        node.get_logger().info("Z_top       = %.1f mm" % z_top)
        node.get_logger().info("grip center = (%.1f, %.1f) mm" % (grip_cx, grip_cy))
        node.get_logger().info(
            "width x/y   = %s / %s mm"
            % (("%.1f" % width_x) if width_x else "N/A",
               ("%.1f" % width_y) if width_y else "N/A")
        )
        # 여기서 grip_cx/cy, z_top, width_* 로 RG2 개폐폭·접근 자세를 설정하면 됨.

        node.get_logger().info("탐지 완료. RViz2 probe_markers 확인 (Ctrl+C 종료)")
        rclpy.spin(node)

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
