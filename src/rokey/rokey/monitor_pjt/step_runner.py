# =============================================================
# step_runner.py — "스텝 순서 + 실패 시 버튼 한 번으로 재시도" 실행기
# -------------------------------------------------------------
# 실행: 라이브러리 (자체 점검: python3 -m rokey.monitor_pjt.step_runner --selftest)
# 구독: <ns>/io/ctrl_box_digital_input_state (UInt8MultiArray, SensorDataQoS)
#       -> DI n 상승엣지 = "마지막 스텝 재시도"
# 의존: rclpy, std_msgs (process_state는 선택 — report 콜백으로만 연결)
# -------------------------------------------------------------
# 쓰는 법 (자동화 노드에서):
#
#   from rokey.monitor_pjt.step_runner import RetryButton, run_steps
#   retry = RetryButton(node, di=15)
#   steps = [
#       ("safe_posture",   lambda: move_joint(...)),
#       ("grasp_approach", lambda: move_line(grasp, ...), lambda: move_line(entry, ...)),
#       #                   ^ 실패하면 3번째(복귀 동작)를 먼저 하고 스텝을 다시 실행
#   ]
#   ok = run_steps(node, steps, retry)
#
# 알고리즘 요약
# - 스텝 리스트가 곧 상태머신이다. 인덱스 i가 상태, 예외가 전이 실패.
#   성공 -> i+1, 예외 -> i 유지한 채 재시도 버튼 대기 -> 같은 i를 다시 실행.
# - 재시도 전 복귀동작(before_retry)이 핵심이다. 실패한 자리에서 곧바로 같은
#   movel을 다시 쏘면 진입 도중 멈춘 상태에서 재진입하다 충돌한다. 스텝이
#   "어디서 시작해도 안전한 지점"을 스스로 알고 있어야 재시도가 성립한다.
# - 버튼은 폴링이 아니라 DI 토픽 상승엣지다(system_monitor가 쓰는 그 토픽).
#   블로킹 서비스 호출(get_digital_input) 대신 구독 + spin_once로 대기한다.
#
# ponytail: 재시도 횟수 제한·타임아웃·자동복구 없음. 사람이 버튼을 누를
#   때까지 기다리는 게 요구사항이고, 무인 운전을 하게 되면 그때 넣는다.
# =============================================================

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8MultiArray

DI_TOPIC = "io/ctrl_box_digital_input_state"   # 노드 네임스페이스(dsr01) 기준 상대이름
RETRY_DI = 15                                  # process_state.DI_LABELS의 "예비" 자리


class RetryButton:
    """DI n의 상승엣지를 1회성 플래그로 모아두는 구독자.

    DRL의 get_digital_input(n)은 1-based, 토픽 data[i]는 DI[i+1]
    -> 배열 인덱스 = n - 1 (process_state.DI_LABELS 주석과 동일 규약).
    """

    def __init__(self, node, di=RETRY_DI, topic=DI_TOPIC):
        self.number = di
        self._index = di - 1
        self._prev = False
        self._pressed = False
        self._sub = node.create_subscription(
            UInt8MultiArray, topic, self._on_io, qos_profile_sensor_data)

    def _on_io(self, msg):
        now = self._index < len(msg.data) and bool(msg.data[self._index])
        if now and not self._prev:      # 상승엣지만 채택(누르고 있는 동안 연타 방지)
            self._pressed = True
        self._prev = now

    def take(self):
        """눌렸으면 True를 반환하고 플래그를 소비한다."""
        pressed, self._pressed = self._pressed, False
        return pressed


def run_steps(node, steps, retry, report=None, spin_sec=0.05):
    """steps를 순서대로 실행. 예외가 나면 재시도 버튼을 기다렸다 그 스텝을 다시 한다.

    steps: (key, fn) 또는 (key, fn, before_retry) 튜플의 리스트.
           fn은 실패 시 예외를 던져야 한다(반환값은 보지 않는다).
           before_retry는 재시도 직전에 실행할 복귀 동작(안전한 시작점으로).
    retry: .take() -> bool 을 가진 객체(RetryButton, 또는 테스트용 가짜).
    report: 선택. report(key, status, message) 콜백 — ProcessReporter 연결용.
    반환: 전부 성공하면 True, 사용자가 Ctrl+C로 중단하면 False.
    """
    logger = node.get_logger()

    def tell(key, status, message=""):
        if report:
            report(key, status, message)

    index = 0
    while index < len(steps):
        key, fn = steps[index][0], steps[index][1]
        before_retry = steps[index][2] if len(steps[index]) > 2 else None
        try:
            logger.info(f"[step {index + 1}/{len(steps)}] {key}")
            tell(key, "RUNNING")
            fn()
            index += 1
        except KeyboardInterrupt:
            logger.warning(f"[step {key}] 사용자 중단")
            tell(key, "ERROR", "사용자 중단")
            return False
        except Exception as exc:
            logger.error(f"[step {key}] 실패: {exc}")
            tell(key, "ERROR", str(exc))
            logger.warning(
                f"DI{getattr(retry, 'number', '?')} 버튼을 누르면 '{key}'를 다시 시도합니다"
                " (Ctrl+C = 중단)")
            if not _wait_for_press(node, retry, spin_sec):
                return False
            if before_retry is not None:
                logger.info(f"[step {key}] 재시도 전 복귀 동작")
                try:
                    before_retry()
                except Exception as recover_exc:   # 복귀조차 실패 = 사람이 봐야 한다
                    logger.error(f"[step {key}] 복귀 실패, 중단: {recover_exc}")
                    tell(key, "ERROR", f"복귀 실패: {recover_exc}")
                    return False
    return True


def _wait_for_press(node, retry, spin_sec):
    """재시도 버튼이 눌릴 때까지 spin. Ctrl+C면 False."""
    try:
        while True:
            if retry.take():
                return True
            if not rclpy.ok():
                return False
            rclpy.spin_once(node, timeout_sec=spin_sec)
    except KeyboardInterrupt:
        return False


def _demo():
    """ROS 없이 도는 자체 점검 — 가짜 node/버튼으로 재시도 경로를 검증."""

    class FakeLogger:
        def info(self, _m): pass
        def warning(self, _m): pass
        def error(self, _m): pass

    class FakeNode:
        def get_logger(self): return FakeLogger()

    class AlwaysPressed:
        number = RETRY_DI
        def take(self): return True

    calls = []
    fail_left = [2]

    def flaky():
        calls.append("flaky")
        if fail_left[0]:
            fail_left[0] -= 1
            raise RuntimeError("movel was rejected")

    reported = []
    steps = [
        ("a", lambda: calls.append("a")),
        ("b", flaky, lambda: calls.append("recover_b")),
        ("c", lambda: calls.append("c")),
    ]
    assert run_steps(FakeNode(), steps, AlwaysPressed(),
                     report=lambda k, s, m="": reported.append((k, s)))
    # b는 3번 실행(2번 실패 + 성공), 실패할 때마다 복귀동작이 먼저 돈다
    assert calls == ["a", "flaky", "recover_b", "flaky", "recover_b", "flaky", "c"], calls
    assert reported.count(("b", "ERROR")) == 2, reported
    assert reported[-1] == ("c", "RUNNING"), reported

    # 버튼을 안 누르면(=Ctrl+C 대용) 실패 지점에서 중단하고 다음 스텝으로 안 넘어간다
    class NeverPressed:
        number = RETRY_DI
        def take(self): raise KeyboardInterrupt

    calls.clear()
    fail_left[0] = 1
    assert not run_steps(FakeNode(), steps, NeverPressed())
    assert "c" not in calls, calls

    # 복귀 동작이 실패하면 재시도하지 않고 멈춘다(사람이 봐야 하는 상황)
    def boom(): raise RuntimeError("recover failed")
    fail_left[0] = 1
    assert not run_steps(FakeNode(), [("b", flaky, boom)], AlwaysPressed())

    # DI 인덱스 규약: DI15 -> data[14]
    button = RetryButton.__new__(RetryButton)
    button.number, button._index, button._prev, button._pressed = 15, 14, False, False
    button._on_io(UInt8MultiArray(data=[0] * 14 + [1]))
    assert button.take() and not button.take()      # 상승엣지 1회만
    button._on_io(UInt8MultiArray(data=[0] * 14 + [1]))
    assert not button.take()                        # 계속 눌려 있으면 재발화 없음
    print("step_runner self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo()
