#!/usr/bin/env python3
"""tray_balance_debug.py -- tray_balance.py가 읽는 실제 힘/모멘트를 눈으로 확인.

핵심 기능
---------
- tray_balance.py와 동일한 tare(measure_tare_offset) + estimate_com_offset을
  그대로 재사용해, 실제 로봇에서 어떤 값이 어떤 방향으로 계산되는지 확인한다.
  (판의 법선 = 툴 +Z(중력 방향), 판은 툴 X-Y 평면 -- tray_balance.py 상단
  README의 그립 가정과 반드시 동일해야 함.)
- 로봇 모션 명령은 전혀 보내지 않는다 (읽기 전용 디버그 툴).
- 콘솔에 raw force, tare 적용 후 force, 추정 offset(dx,dy), 방향 라벨을 출력.
- RViz2에서 보고 싶으면: Marker 토픽 /tray_balance/force_debug/markers 구독
  (ARROW = 판 중심 -> 추정 offset 방향, 판의 X-Y 평면 위에 그려짐, 확대 표시).

실행: ros2 run move tray_balance_debug
전제: tray_balance.py와 동일하게 물체를 판 정중앙에 두고 시작 -- tare 측정 중.
"""

import math
import time as pytime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import DR_init
from move.tray_balance import (
    measure_tare_offset,
    estimate_com_offset,
    check_grip_orientation,
    GOLF_BALL_MASS_KG,
    TRAY_RADIUS_M,
)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TOOL_NAME = "Tool Weight_1"
TCP_NAME = "GripperDA_v1"

LOOP_HZ = 20.0
LOG_PERIOD_SEC = 0.3
DIRECTION_DEADZONE_M = 0.003   # 이 이내면 "CENTER"로 표시

PARENT_FRAME = "link_6"        # 실제 로봇 flange TF 이름으로 맞출 것
TCP_TO_TRAY_EXTENSION_M = 0.06  # tray_balance_viz.py와 동일 (연장된 TCP = 판 중심)
TRAY_MOUNT_OFFSET_M = (0.0, 0.0, TCP_TO_TRAY_EXTENSION_M)
ARROW_SCALE = 20.0               # RViz에서 잘 보이도록 offset을 확대해 그림

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def direction_label(dx, dy, deadzone_m=DIRECTION_DEADZONE_M):
    """dx,dy(tool 좌표계, m) -> 8방향 나침반 라벨. 사람이 바로 읽을 용도."""
    if math.hypot(dx, dy) < deadzone_m:
        return "CENTER"

    angle_deg = math.degrees(math.atan2(dy, dx))
    sectors = ["+X", "+X+Y", "+Y", "-X+Y", "-X", "-X-Y", "-Y", "+X-Y"]
    index = round(angle_deg / 45.0) % 8
    return sectors[index]


class TrayBalanceDebug(Node):
    def __init__(self):
        super().__init__("tray_balance_debug", namespace=ROBOT_ID)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            Marker, "/tray_balance/force_debug/markers", 10
        )

    def publish_tray_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = PARENT_FRAME
        tf.child_frame_id = "tray_debug"
        tf.transform.translation.x = TRAY_MOUNT_OFFSET_M[0]
        tf.transform.translation.y = TRAY_MOUNT_OFFSET_M[1]
        tf.transform.translation.z = TRAY_MOUNT_OFFSET_M[2]
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def publish_force_arrow(self, stamp, dx, dy):
        marker = Marker()
        marker.header.frame_id = "tray_debug"
        marker.header.stamp = stamp
        marker.ns = "tray_balance_debug"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.006   # shaft diameter
        marker.scale.y = 0.012   # head diameter
        marker.scale.z = 0.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.2, 0.1, 1.0

        # 판은 tray_debug 프레임의 X-Y 평면에 놓인다 (법선 = Z) -- 화살표는
        # 그 평면 바로 위(z=0.002)에 그린다.
        start = Point(x=0.0, y=0.0, z=0.002)
        end = Point(x=dx * ARROW_SCALE, y=dy * ARROW_SCALE, z=0.002)
        marker.points = [start, end]
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrayBalanceDebug()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import set_tool, set_tcp, get_tool_force, DR_TOOL
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return

    set_tool(TOOL_NAME)
    set_tcp(TCP_NAME)

    node.get_logger().info(
        "물체를 판 정중앙에 놓고 손을 뗀 뒤 tare 측정을 시작합니다."
    )
    tare_offset = measure_tare_offset(get_tool_force, DR_TOOL)
    node.get_logger().info(
        f"tare 기준값: {[round(v, 4) for v in tare_offset]}"
    )

    orientation_issue = check_grip_orientation(tare_offset, GOLF_BALL_MASS_KG)
    if orientation_issue is not None:
        node.get_logger().warn(
            f"그립 자세/물체 배치 점검 필요 (tray_balance.py는 이 상태로 시작 안 함): "
            f"{orientation_issue}"
        )
    else:
        node.get_logger().info("그립 자세 점검 통과 (Fz가 무게, Fx/Fy는 작음).")

    node.get_logger().info(
        "실시간 힘 방향 디버그 시작 (모션 없음, Ctrl+C로 종료)"
    )

    last_log_time = 0.0
    start_time = pytime.monotonic()

    try:
        while rclpy.ok():
            raw_force = get_tool_force(ref=DR_TOOL)

            if raw_force == -1:
                node.get_logger().error(
                    "get_tool_force 읽기 실패 - 서비스 응답 없음."
                )
                break

            tared_force = [r - t for r, t in zip(raw_force, tare_offset)]
            dx, dy = estimate_com_offset(tared_force, GOLF_BALL_MASS_KG)
            offset_norm = math.hypot(dx, dy)
            label = direction_label(dx, dy)

            stamp = node.get_clock().now().to_msg()
            node.publish_tray_tf(stamp)
            node.publish_force_arrow(stamp, dx, dy)

            now = pytime.monotonic() - start_time
            if now - last_log_time >= LOG_PERIOD_SEC:
                # estimate_com_offset은 판이 거의 안 기울어진 소각 근사를
                # 가정한다 -- 판 반지름보다 큰 offset은 실제 공 위치가 아니라
                # (1) 물체가 안 실렸거나 (2) 지금 tilt가 커서 근사가 깨진
                # 신호이므로 눈에 띄게 경고 표시한다.
                sanity_flag = (
                    " [!] 판 반지름(%.0fmm) 초과 -- 소각 근사 무효 또는 물체 미장착 가능성"
                    % (TRAY_RADIUS_M * 1000)
                    if offset_norm > TRAY_RADIUS_M
                    else ""
                )
                node.get_logger().info(
                    f"raw={[round(v, 3) for v in raw_force]} | "
                    f"tared_txty=({tared_force[3]:+.4f},{tared_force[4]:+.4f})Nm | "
                    f"offset=({dx*1000:+.1f},{dy*1000:+.1f})mm "
                    f"|{offset_norm*1000:.1f}mm| 방향={label}{sanity_flag}"
                )
                last_log_time = now

            pytime.sleep(1.0 / LOOP_HZ)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _demo_direction_label():
    assert direction_label(0.001, 0.001) == "CENTER"
    assert direction_label(0.05, 0.0) == "+X"
    assert direction_label(0.0, 0.05) == "+Y"
    assert direction_label(-0.05, 0.0) == "-X"
    assert direction_label(0.0, -0.05) == "-Y"
    assert direction_label(0.05, 0.05) == "+X+Y"
    print("direction_label self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo_direction_label()
    else:
        main()
