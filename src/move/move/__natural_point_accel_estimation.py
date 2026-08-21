#!/usr/bin/env python3
"""M0609 natural-point object acceleration estimation.

Reads the F/T sensor at a fixed ("natural") joint pose and inverts F = m*a to
recover the horizontal acceleration an object on the tray is experiencing
(e.g. from an AGV/vehicle the arm is mounted on).

Ported from m0609_natural_point_accel_estimation.drl.
"""

import math
import time as pytime

import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

G = 9.81
SETTLE_TIME_SEC = 2.0
LOOP_HZ = 100.0
LPF_ALPHA = 0.2
DEADBAND_N = 0.3
DEBUG_LOG_PERIOD_SEC = 0.5
MIN_VALID_MASS_KG = 0.01

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def measure_object_mass(get_tool_force, dr_base):
    """Tool Weight가 '쟁반만' 기준일 때 Fz로부터 물체 질량을 역산."""
    force = get_tool_force(dr_base)
    fz = force[2]
    return abs(fz) / G


def estimate_acceleration_loop(node, get_tool_force, dr_base, mass_kg):
    """tare된 기준선 대비 관성력(F=m*a)을 실시간으로 가속도로 역산."""
    dt = 1.0 / LOOP_HZ
    ax_filtered, ay_filtered = 0.0, 0.0
    last_debug_log = 0.0
    start_time = pytime.monotonic()

    # ponytail: 이 바인딩에 set_external_force_reset()이 없어서 소프트웨어
    # 타어(baseline 샘플을 offset으로 빼는 방식)로 대체. 진짜 펌웨어 tare가
    # 생기면 그걸로 바꾸는 게 더 정확함.
    baseline = get_tool_force(dr_base)
    offset_fx, offset_fy = baseline[0], baseline[1]
    node.get_logger().info(f"tare offset: fx0={offset_fx:.3f}N, fy0={offset_fy:.3f}N")

    node.get_logger().info("---- start acceleration estimation ----")

    while rclpy.ok():
        force = get_tool_force(dr_base)
        fx = force[0] - offset_fx
        fy = force[1] - offset_fy

        if abs(fx) < DEADBAND_N:
            fx = 0.0
        if abs(fy) < DEADBAND_N:
            fy = 0.0

        ax = fx / mass_kg
        ay = fy / mass_kg

        ax_filtered = LPF_ALPHA * ax + (1.0 - LPF_ALPHA) * ax_filtered
        ay_filtered = LPF_ALPHA * ay + (1.0 - LPF_ALPHA) * ay_filtered

        a_mag = math.hypot(ax_filtered, ay_filtered)

        now = pytime.monotonic() - start_time
        if now - last_debug_log >= DEBUG_LOG_PERIOD_SEC:
            node.get_logger().info(
                f"a=({ax_filtered:+.3f}, {ay_filtered:+.3f}) m/s^2 | "
                f"|a|={a_mag:.3f} | Fxy=({fx:+.2f}, {fy:+.2f}) N"
            )
            last_debug_log = now

        pytime.sleep(dt)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("natural_point_accel_estimation", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import set_velj, set_accj, movej, get_tool_force, DR_BASE
        from DR_common2 import posj
    except ImportError as error:
        node.get_logger().error(f"두산 로봇 모듈 import 실패: {error}")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 자연 지점(중립 자세): 현장에서 직접교시로 잡은 값으로 교체할 것.
    natural_pose = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

    try:
        set_velj(30.0)
        set_accj(60.0)
        movej(natural_pose)

        pytime.sleep(SETTLE_TIME_SEC)
        mass_kg = measure_object_mass(get_tool_force, DR_BASE)
        node.get_logger().info(f"Measured object mass = {mass_kg:.4f} kg")

        if mass_kg < MIN_VALID_MASS_KG:
            node.get_logger().warn("object not detected (mass too small). abort.")
            return

        pytime.sleep(SETTLE_TIME_SEC)
        estimate_acceleration_loop(node, get_tool_force, DR_BASE, mass_kg)

    except KeyboardInterrupt:
        node.get_logger().info("사용자 요청으로 정지")

    except Exception as error:
        node.get_logger().error(f"Robot Error: {error}")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _demo_measure_object_mass():
    def fake_get_tool_force(ref):
        return [0, 0, -4.905, 0, 0, 0]

    mass_kg = measure_object_mass(fake_get_tool_force, "DR_BASE")
    assert abs(mass_kg - 0.5) < 1e-6
    print("measure_object_mass self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo_measure_object_mass()
    else:
        main()
