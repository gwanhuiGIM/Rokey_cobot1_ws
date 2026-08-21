#!/usr/bin/env python3
"""Ball-on-tilted-tray physics mock for tray_balance.py — no ROS, no robot.

Runs the same control law as tray_balance.main() (KP/KD/MAX_TILT_DEG imported,
not copied) against a simple point-mass-on-inclined-plane model, so gains and
sign conventions can be tuned before ever touching the real robot.

Grip assumption (must match tray_balance.py's docstring): tool +Z points down
(gravity axis), tray lies in the tool X-Y plane, tilt is commanded via
rotation about tool X/roll (corrects dy) and tool Y/pitch (corrects dx).

This is how the sign bug in the original P-only controller was caught: it
diverged in this mock before it ever ran on hardware. (The signs below were
re-derived and re-verified here after confirming on the real robot that tool
+Z points toward gravity, not away from it -- flipping every sign relative
to the previous "away from gravity" assumption.)
"""

import math
import random

from move.tray_balance import (
    estimate_com_offset,
    KP,
    KD,
    MAX_TILT_DEG,
    GOLF_BALL_MASS_KG,
    SAFE_RADIUS_M as TRAY_RADIUS_M,
)

G = 9.81
DT = 0.02                 # 50 Hz, matches tray_balance.LOOP_HZ
FRICTION = 1.5             # ponytail: linear velocity damping, not real rolling friction
# TRAY_RADIUS_M = tray_balance.SAFE_RADIUS_M (판 반지름 - 공 반지름), 실기와 동일 기준

SPAWN_MARGIN_RATIO = 0.9   # 판 가장자리에 바로 스폰되지 않도록 안전반경의 90%까지만 사용


def random_spawn_offset(max_radius_m=None):
    """판 위 임의 지점에 공을 스폰 -- 극좌표로 균일 분포."""
    if max_radius_m is None:
        max_radius_m = TRAY_RADIUS_M * SPAWN_MARGIN_RATIO
    radius = max_radius_m * math.sqrt(random.uniform(0.0, 1.0))
    angle = random.uniform(0.0, 2 * math.pi)
    return radius * math.cos(angle), radius * math.sin(angle)


class TrayBallSim:
    def __init__(self, ball_mass_kg, start_dx, start_dy):
        self.mass = ball_mass_kg
        self.dx = start_dx
        self.dy = start_dy
        self.vx = 0.0
        self.vy = 0.0
        self.roll_deg = 0.0    # rotation about tool X -> corrects dy
        self.pitch_deg = 0.0   # rotation about tool Y -> corrects dx

    def measured_force(self):
        # Inverse of estimate_com_offset's tau = r x F model (tool +Z down,
        # tray in X-Y plane): reuses the exact relationship the controller
        # assumes, so the loop is internally consistent with tray_balance.py.
        weight = self.mass * G
        tx = self.dy * weight
        ty = -self.dx * weight
        return [0, 0, 0, tx, ty, 0]

    def step_physics(self):
        # Small-angle incline: lateral accel ~= g * sin(tilt). Z points
        # toward gravity here, so the signs are opposite of the old
        # "Z away from gravity" version.
        ax = -G * math.sin(math.radians(self.pitch_deg))
        ay = G * math.sin(math.radians(self.roll_deg))

        self.vx += ax * DT
        self.vy += ay * DT
        self.vx *= max(0.0, 1.0 - FRICTION * DT)
        self.vy *= max(0.0, 1.0 - FRICTION * DT)

        self.dx += self.vx * DT
        self.dy += self.vy * DT

    def offset_norm(self):
        return math.hypot(self.dx, self.dy)


def run(
    ball_mass_kg=GOLF_BALL_MASS_KG,
    start_dx=None,
    start_dy=None,
    steps=1500,
    log_every=100,
    quiet=False,
):
    if start_dx is None or start_dy is None:
        start_dx, start_dy = random_spawn_offset()
        if not quiet:
            print(f"spawn point: ({start_dx*1000:+.1f}, {start_dy*1000:+.1f})mm")

    sim = TrayBallSim(ball_mass_kg, start_dx, start_dy)
    prev_dx, prev_dy = start_dx, start_dy

    for step in range(steps):
        dx, dy = estimate_com_offset(sim.measured_force(), ball_mass_kg)
        dx_dot = (dx - prev_dx) / DT
        dy_dot = (dy - prev_dy) / DT
        prev_dx, prev_dy = dx, dy

        # Same law as tray_balance.py's control loop.
        sim.roll_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, -KP * dy - KD * dy_dot))
        sim.pitch_deg = max(-MAX_TILT_DEG, min(MAX_TILT_DEG, KP * dx + KD * dx_dot))

        sim.step_physics()

        if not quiet and step % log_every == 0:
            print(
                f"t={step*DT:5.2f}s  offset=({sim.dx*1000:+6.1f}, {sim.dy*1000:+6.1f})mm "
                f"|{sim.offset_norm()*1000:5.1f}mm  tilt=(roll={sim.roll_deg:+5.2f}, pitch={sim.pitch_deg:+5.2f})deg"
            )

        if sim.offset_norm() > TRAY_RADIUS_M:
            if not quiet:
                print(f"FAILED: ball left the tray at t={step*DT:.2f}s")
            return False

    if not quiet:
        verdict = "CONVERGED" if sim.offset_norm() < 0.005 else "DID NOT CONVERGE"
        print(f"{verdict}: final offset={sim.offset_norm()*1000:.2f}mm after {steps*DT:.1f}s")

    return sim.offset_norm() < 0.005


def main(args=None):
    """ros2 run 진입점 -- 매번 판 위 임의 지점에 공을 스폰해 데모로 실행."""
    ok = run()
    print("result:", "CONVERGED" if ok else "FAILED (판을 벗어났거나 수렴하지 않음)")
    if not ok:
        raise SystemExit(1)


def _selftest():
    ok = run(start_dx=0.05, start_dy=0.03, quiet=True)
    assert ok, "회귀: 고정 시작점(58.3mm)에서 수렴하지 못함 -- KP/KD 또는 부호를 확인할 것"
    print("tray_balance_sim self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _selftest()
    else:
        main()
