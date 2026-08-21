#!/usr/bin/env python3
"""Static weight-change / on-tray position monitor (robot does NOT move).

Robot holds a fixed pose; reads TCP F/T only. Placing an object anywhere on
the tray changes Fz (mass) and, because the tray is a rigid lever arm off the
TCP, placing it near one edge vs. the opposite edge changes Mx/My too (same
tau = r x F inversion as tray_balance.py's estimate_com_offset). This node
just detects and logs those changes + publishes TF/Marker for RViz2 -- no
control loop, no movel.

Note: estimate_com_offset's sign convention was corrected after confirming
on the real robot that tool +Z points toward gravity (not away from it).
This file inherits that sign since it never commands motion -- worst case
here is a flipped edge/zone marker in RViz, not a safety issue. If this
node's own tray mount ever turns out to have +Z pointing away from gravity,
its dx/dy (and therefore the "edge" direction shown) would be inverted.
"""

import time as pytime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import DR_init
from move.tray_balance import estimate_com_offset

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

TRAY_RADIUS_M = 0.10
G = 9.81
LOOP_HZ = 20.0
FILTER_ALPHA = 0.2

MASS_PRESENT_THRESHOLD_KG = 0.01   # 이 이상이면 "물체 있음"
CENTER_ZONE_M = 0.02               # 중앙 반경
EDGE_ZONE_M = 0.07                 # 이 이상이면 "가장자리" (반지름 10cm 기준)
LOG_PERIOD_SEC = 0.5

PARENT_FRAME = "link_6"            # 실제 로봇 flange TF 이름으로 맞출 것
TRAY_MOUNT_OFFSET_M = (0.0, 0.0, 0.05)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def classify_zone(offset_norm_m):
    if offset_norm_m < CENTER_ZONE_M:
        return "center"
    if offset_norm_m > EDGE_ZONE_M:
        return "edge"
    return "mid"


class WeightChangeMonitor(Node):
    """DSR_ROBOT2 서비스 클라이언트는 import 시점의 g_node에 바인딩되므로,
    이 노드를 먼저 만들어 DR_init.__dsr__node로 지정한 뒤에 import해야 함.
    그래서 get_tool_force는 생성자가 아니라 configure()로 나중에 주입."""

    def __init__(self):
        super().__init__("weight_change_monitor", namespace=ROBOT_ID)
        self.filtered = None  # [fz, mx, my]
        self.mass_kg = 0.0
        self.dx, self.dy = 0.0, 0.0
        self.object_present = False
        self.zone = "center"
        self.last_log_time = 0.0
        self._start_time = pytime.monotonic()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/weight_monitor/markers", 10)

    def configure(self, get_tool_force, dr_tool):
        self._get_tool_force = get_tool_force
        self._dr_tool = dr_tool

        baseline = self._get_tool_force(self._dr_tool)
        self.offset_fz = baseline[2]
        self.offset_mx = baseline[3]
        self.offset_my = baseline[4]
        self.get_logger().info(
            f"tare baseline (빈 쟁반 기준): Fz0={self.offset_fz:.3f}N, "
            f"Mx0={self.offset_mx:.3f}, My0={self.offset_my:.3f}"
        )

        self.create_timer(1.0 / LOOP_HZ, self.tick)
        self.get_logger().info(
            f"weight_change_monitor 시작 (정지 상태 감지 전용, 모션 없음), "
            f"tray_radius={TRAY_RADIUS_M}m"
        )

    def tick(self):
        raw = self._get_tool_force(self._dr_tool)
        if raw == -1:
            self.get_logger().error("get_tool_force 읽기 실패 - 서비스 응답 없음")
            return

        fz = raw[2] - self.offset_fz
        mx = raw[3] - self.offset_mx
        my = raw[4] - self.offset_my

        if self.filtered is None:
            self.filtered = [fz, mx, my]
        else:
            a = FILTER_ALPHA
            self.filtered = [
                a * v + (1 - a) * f for v, f in zip([fz, mx, my], self.filtered)
            ]
        fz_f, mx_f, my_f = self.filtered

        self.mass_kg = abs(fz_f) / G

        was_present = self.object_present
        self.object_present = self.mass_kg >= MASS_PRESENT_THRESHOLD_KG

        if self.object_present:
            self.dx, self.dy = estimate_com_offset([0, 0, 0, mx_f, my_f, 0], self.mass_kg, G)
        else:
            self.dx, self.dy = 0.0, 0.0

        offset_norm = (self.dx ** 2 + self.dy ** 2) ** 0.5
        new_zone = classify_zone(offset_norm) if self.object_present else "center"

        if self.object_present != was_present:
            self.get_logger().info(
                f"{'물체 놓임' if self.object_present else '물체 제거됨'}: "
                f"mass={self.mass_kg*1000:.1f}g"
            )
        if self.object_present and new_zone != self.zone:
            self.get_logger().info(
                f"위치 변화: {self.zone} -> {new_zone} "
                f"(offset=({self.dx*1000:+.1f},{self.dy*1000:+.1f})mm, |offset|={offset_norm*1000:.1f}mm)"
            )
        self.zone = new_zone

        now = pytime.monotonic() - self._start_time
        if now - self.last_log_time >= LOG_PERIOD_SEC:
            self.get_logger().debug(
                f"fz={fz_f:+.3f}N mx={mx_f:+.3f} my={my_f:+.3f} | "
                f"mass={self.mass_kg*1000:.1f}g present={self.object_present} "
                f"offset=({self.dx*1000:+.1f},{self.dy*1000:+.1f})mm zone={self.zone}"
            )
            self.last_log_time = now

        stamp = self.get_clock().now().to_msg()
        self.publish_tray_tf(stamp)
        self.publish_tray_marker(stamp)
        self.publish_object_marker(stamp)

    def publish_tray_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = PARENT_FRAME
        tf.child_frame_id = "tray"
        tf.transform.translation.x = TRAY_MOUNT_OFFSET_M[0]
        tf.transform.translation.y = TRAY_MOUNT_OFFSET_M[1]
        tf.transform.translation.z = TRAY_MOUNT_OFFSET_M[2]
        tf.transform.rotation.w = 1.0  # 로봇 정지 상태 -> tilt 없음
        self.tf_broadcaster.sendTransform(tf)

    def publish_tray_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "tray"
        marker.header.stamp = stamp
        marker.ns = "weight_monitor"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = TRAY_RADIUS_M * 2
        marker.scale.y = TRAY_RADIUS_M * 2
        marker.scale.z = 0.01
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.6, 0.6, 0.6, 0.9
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)

    def publish_object_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "tray"
        marker.header.stamp = stamp
        marker.ns = "weight_monitor"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD if self.object_present else Marker.DELETE
        radius = min(0.03, 0.01 + self.mass_kg * 0.02)
        marker.scale.x = marker.scale.y = marker.scale.z = radius * 2
        zone_color = {"center": (0.1, 0.9, 0.1), "mid": (0.9, 0.9, 0.1), "edge": (0.9, 0.1, 0.1)}
        r, g, b = zone_color.get(self.zone, (1.0, 0.3, 0.1))
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, 1.0
        marker.pose.position.x = self.dx
        marker.pose.position.y = self.dy
        marker.pose.position.z = 0.005 + radius
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WeightChangeMonitor()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import get_tool_force, DR_TOOL
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.configure(get_tool_force, DR_TOOL)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _demo():
    assert classify_zone(0.0) == "center"
    assert classify_zone(0.05) == "mid"
    assert classify_zone(0.09) == "edge"

    # 반지름 10cm 쟁반의 반대편 끝에 물체를 놓았을 때 dx 부호가 뒤집혀야 함
    mass_kg = 0.1
    weight = mass_kg * G
    dx_a, dy_a = estimate_com_offset([0, 0, 0, 0.0, weight * 0.08, 0], mass_kg, G)
    dx_b, dy_b = estimate_com_offset([0, 0, 0, 0.0, -weight * 0.08, 0], mass_kg, G)
    assert dx_a < -0.07 and dx_b > 0.07
    assert abs(dx_a + dx_b) < 1e-9
    print("weight_change_monitor self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo()
    else:
        main()
