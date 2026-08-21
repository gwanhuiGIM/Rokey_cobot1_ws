#!/usr/bin/env python3
"""Publishes the tray_balance_sim.py ball-on-tray simulation to RViz2.

Pure sim state -> TF + Marker, no DSR_ROBOT2 calls. The tray TF is published
as a CHILD of PARENT_FRAME (the robot's real end-effector link), not "world" --
otherwise the tray floats at a fixed point in space while the actual robot
model (from robot_state_publisher) moves independently, and the two never
line up in RViz.

판 장착 가정 (tray_balance.py와 동일):
- 그리퍼가 판을 툴 +Z축이 아래(중력) 방향과 나란하도록 그립했다 (+X는
  BASE +X와 평행). 즉 판의 법선이 툴 Z축이고, 판 자체는 툴 X-Y 평면에
  놓인다.
- 판 중심은 TCP에서 tool +Z(=판의 법선과 같은 축)로
  TCP_TO_TRAY_EXTENSION_M만큼 연장된 위치에 있다. 그 연장된 지점을
  원점으로 삼아 tilt를 적용하므로(publish_tray_tf), 판은 자기 중심을
  축으로만 회전하고 공간 위치는 흔들리지 않는다.
- tilt는 X축 회전(roll, dy 보정)과 Y축 회전(pitch, dx 보정)만 쓴다 --
  Z축은 판의 법선이라 그 축 회전은 기울기에 기여하지 않는다.
- 공은 매 실행마다 판 위 임의 위치에 스폰된다(random_spawn_offset,
  tray_balance.SAFE_RADIUS_M 이내).

PARENT_FRAME must match a frame that actually exists in your /tf tree. Find
it with `ros2 run tf2_tools view_frames` (or `ros2 topic echo /tf`) while the
real robot description is running, and edit the constant below to match --
this repo's m0609 xacro names the flange link "link_6", but a namespace/tf
prefix from your launch file may change the actual published name.

RViz2 setup: Fixed Frame = PARENT_FRAME's root (e.g. "base_link"), then add:
  - TF
  - Marker, topic /tray_balance/markers
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from move.tray_balance import (
    KP,
    KD,
    MAX_TILT_DEG,
    estimate_com_offset,
    GOLF_BALL_MASS_KG,
    BALL_RADIUS_M,
    TRAY_RADIUS_M,
)
from move.tray_balance_sim import TrayBallSim, DT, random_spawn_offset

PARENT_FRAME = "link_6"  # 실제 로봇 flange TF 이름으로 맞출 것 (위 설명 참고)

# 그리퍼가 판을 tool +Z축(=중력 방향)과 나란히 그립했다고 가정 -- 판 중심은
# TCP에서 tool +Z로 TCP_TO_TRAY_EXTENSION_M만큼 연장된 위치. 이 "연장된
# TCP" 지점을 기준으로 판이 그 자리에서만 회전하도록 TF를 구성한다
# (publish_tray_tf 참고).
TCP_TO_TRAY_EXTENSION_M = 0.32
TRAY_MOUNT_OFFSET_M = (0.0, 0.0, TCP_TO_TRAY_EXTENSION_M)
TRAY_THICKNESS_M = 0.01
LOG_PERIOD_SEC = 1.0

# 판의 법선이 툴 Z축이라 RViz Marker.CYLINDER(기본 높이축=로컬 Z)가 별도
# 회전 없이 그대로 판의 법선과 일치한다.


def euler_xy_to_quaternion(roll_rad, pitch_rad):
    # ponytail: yaw=0 special case of the standard ZYX euler->quaternion formula
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    return (
        sr * cp,   # x
        cr * sp,   # y
        -sr * sp,  # z
        cr * cp,   # w
    )


class TrayBalanceViz(Node):
    def __init__(self):
        super().__init__("tray_balance_viz")
        start_dx, start_dy = random_spawn_offset()
        self.sim = TrayBallSim(GOLF_BALL_MASS_KG, start_dx, start_dy)
        self.prev_dx, self.prev_dy = self.sim.dx, self.sim.dy

        self.get_logger().info(
            f"공 스폰 위치: ({start_dx*1000:+.1f}, {start_dy*1000:+.1f})mm"
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, "/tray_balance/markers", 10)
        self.timer = self.create_timer(DT, self.tick)
        self.last_log_time = 0.0

        self.get_logger().info(
            f"tray_balance_viz 시작: parent_frame='{PARENT_FRAME}' "
            f"(로봇 실제 flange TF 이름과 다르면 쟁반이 안 붙어 보임), "
            f"topic=/tray_balance/markers"
        )

    def tick(self):
        dx, dy = estimate_com_offset(self.sim.measured_force(), self.sim.mass)
        dx_dot = (dx - self.prev_dx) / DT
        dy_dot = (dy - self.prev_dy) / DT
        self.prev_dx, self.prev_dy = dx, dy

        self.sim.roll_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, -KP * dy - KD * dy_dot))
        self.sim.pitch_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, KP * dx + KD * dx_dot))
        self.sim.step_physics()

        now = self.get_clock().now().to_msg()
        self.publish_tray_tf(now)
        self.publish_tray_marker(now)
        self.publish_ball_marker(now)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_log_time >= LOG_PERIOD_SEC:
            self.get_logger().info(
                f"ball_offset=({self.sim.dx*1000:+.1f},{self.sim.dy*1000:+.1f})mm "
                f"tilt=(roll={self.sim.roll_deg:+.2f},pitch={self.sim.pitch_deg:+.2f})deg"
            )
            self.last_log_time = now_sec

    def publish_tray_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = PARENT_FRAME
        tf.child_frame_id = "tray"
        tf.transform.translation.x = TRAY_MOUNT_OFFSET_M[0]
        tf.transform.translation.y = TRAY_MOUNT_OFFSET_M[1]
        tf.transform.translation.z = TRAY_MOUNT_OFFSET_M[2]
        qx, qy, qz, qw = euler_xy_to_quaternion(
            math.radians(self.sim.roll_deg), math.radians(self.sim.pitch_deg)
        )
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def publish_tray_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "tray"
        marker.header.stamp = stamp
        marker.ns = "tray_balance"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = TRAY_RADIUS_M * 2
        marker.scale.y = TRAY_RADIUS_M * 2
        marker.scale.z = TRAY_THICKNESS_M
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.6, 0.6, 0.6, 0.9
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)

    def publish_ball_marker(self, stamp):
        marker = Marker()
        marker.header.frame_id = "tray"
        marker.header.stamp = stamp
        marker.ns = "tray_balance"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = BALL_RADIUS_M * 2
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.3, 0.1, 1.0
        marker.pose.position.x = self.sim.dx
        marker.pose.position.y = self.sim.dy
        marker.pose.position.z = TRAY_THICKNESS_M / 2 + BALL_RADIUS_M
        marker.pose.orientation.w = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrayBalanceViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
