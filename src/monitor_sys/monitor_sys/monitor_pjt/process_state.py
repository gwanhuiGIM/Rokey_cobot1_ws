# =============================================================
# process_state.py — 핸드드립 커피 공정 상태머신 정의 + 보고 메시지 규약
# -------------------------------------------------------------
# 실행: 라이브러리 (자체 점검: python3 -m monitor_sys.monitor_pjt.process_state --selftest)
# 토픽: /coffee_process/state — std_msgs/String (JSON 1줄)
#       QoS: RELIABLE / TRANSIENT_LOCAL / depth 10
#       -> 모니터를 나중에 켜도 마지막 상태를 바로 받는다
# 의존: rclpy, std_msgs
# -------------------------------------------------------------
# 자동화 노드(로봇을 실제로 움직이는 쪽)에서 쓰는 법:
#
#   from monitor_pjt.process_state import ProcessReporter, STEP
#   report = ProcessReporter(node)
#   report.set(STEP.HOME)                       # 홈 이동 시작
#   report.set(STEP.IDLE, waiting="DI13")       # 사용자 입력 대기
#   ...  di13 = get_digital_input(13)  ...
#   report.set(STEP.PICK_BEANS)
#   report.set(STEP.GRINDING, progress_in_step=0.4, message="grinding 40%")
#   report.error("gripper timeout")             # 이상 발생
#
# ponytail: 커스텀 .msg 대신 String+JSON. monitor_pjt가 ament_python이라
#   .msg를 만들려면 별도 인터페이스 패키지가 필요하고, 필드가 아직 굳지
#   않았다. 규약이 굳고 타입 안전성이 필요해지면 그때 msgs 패키지로 승격.
#
# 알고리즘 요약
# - 상태머신 정의는 STEPS 리스트 하나뿐이다(단일 진실 소스). 순서를 바꾸면
#   전체 진행률(step_progress)·모니터의 "건너뜀 감지"가 자동으로 같이 바뀐다.
# - step_progress()는 스텝 인덱스를 0~100%로 선형 매핑한다. progress_in_step
#   (그라인딩·드립처럼 오래 걸리는 스텝의 내부 0~1 진행도)을 주면 그 스텝
#   구간 안에서 다시 선형 보간한다 — 그라인딩 절반이면 "그라인딩 진행률 +
#   (다음 스텝-현재 스텝 간격)/2"가 된다.
# - is_expected_transition()이 상태머신 규칙을 검사한다: 첫 보고, 같은
#   스텝 반복, 바로 다음 스텝, INIT/IDLE 복귀(중단·재시작·사이클 반복)만
#   "정상"이고 그 외(스텝 건너뜀)는 모니터가 WARN으로 남긴다.
# =============================================================

import json
import time

from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import String

TOPIC = "/coffee_process/state"

# 공정 순서 (key, 화면 라벨, 진행 조건). 이 리스트가 상태머신의 유일한 정의다.
STEPS = [
    ("INIT",          "초기화",              ""),
    ("HOME",          "홈 위치 이동",        ""),
    ("IDLE",          "대기",                "DI13 입력 대기"),
    ("PICK_BEANS",    "원두 집기",           ""),
    ("POUR_BEANS",    "그라인더에 붓기",     ""),
    ("WAIT_LID",      "뚜껑 닫힘 대기",      "DI14 입력 대기"),
    ("GRINDING",      "그라인딩",            ""),
    ("MOVE_DRIPPER",  "원두 → 드리퍼 이동",  ""),
    ("PICK_KETTLE",   "물컵 그립",           ""),
    ("DRIPPING",      "핸드드립 진행",       ""),
    ("RETURN_KETTLE", "물컵 원위치",         ""),
    ("DONE",          "1 사이클 완료",       ""),
]
STEP_KEYS = [key for key, _label, _hint in STEPS]
STEP_INDEX = {key: i for i, key in enumerate(STEP_KEYS)}
STEP_LABEL = {key: label for key, label, _hint in STEPS}
STEP_HINT = {key: hint for key, _label, hint in STEPS}


class STEP:
    """오타 방지용 상수 네임스페이스 (STEP.GRINDING 처럼 사용)."""


for _key in STEP_KEYS:
    setattr(STEP, _key, _key)

STATUSES = ("RUNNING", "WAITING", "DONE", "ERROR")

# 컨트롤박스 디지털 입력 배치.
# 주의: DRL의 get_digital_input(n)은 1-based, ROS 토픽
# io/ctrl_box_digital_input_state의 data[i]는 DI[i+1] -> 배열 인덱스 = n-1.
DI_LABELS = {
    13: "시작 (원두 투입)",
    14: "뚜껑 닫힘 확인",
    15: "예비",
    16: "예비",
}
DI_TRIGGER = {"IDLE": 13, "WAIT_LID": 14}   # 각 대기 상태를 푸는 버튼


def step_progress(step):
    """공정 전체 진행률 [%] — 스텝 순서에서 자동 계산 (매직넘버 없음)."""
    return round(100.0 * STEP_INDEX[step] / (len(STEPS) - 1), 1)


def is_expected_transition(prev_step, step):
    """상태머신 검증: 정의된 순서대로 왔는가.

    허용: 첫 보고 / 같은 스텝 재보고 / 바로 다음 스텝 /
          INIT·IDLE 복귀(중단·재시작·사이클 반복은 언제든 가능).
    그 외는 건너뛴 것이므로 모니터가 경고를 남긴다.
    """
    if prev_step is None or prev_step == step:
        return True
    if step in ("INIT", "IDLE"):
        return True
    return STEP_INDEX.get(step, -1) == STEP_INDEX.get(prev_step, -99) + 1


def encode(step, status="RUNNING", message="", progress_in_step=None):
    """공정 보고 1건 -> JSON 문자열.

    progress_in_step: 그라인딩·드립처럼 오래 걸리는 스텝의 내부 진행도
    (0.0~1.0). 주면 전체 진행률이 다음 스텝까지 그만큼 선형 보간된다.
    """
    if step not in STEP_INDEX:
        raise ValueError(f"unknown step: {step}")
    if status not in STATUSES:
        raise ValueError(f"unknown status: {status}")

    progress = step_progress(step)
    if progress_in_step is not None:
        span = 100.0 / (len(STEPS) - 1)
        progress = round(progress + span * min(1.0, max(0.0, progress_in_step)), 1)
    return json.dumps({
        "step": step,
        "index": STEP_INDEX[step],
        "total": len(STEPS),
        "label": STEP_LABEL[step],
        "hint": STEP_HINT[step],
        "status": status,
        "message": message,
        "progress": progress,
        "stamp": time.time(),
    })


def decode(text):
    payload = json.loads(text)
    if payload.get("step") not in STEP_INDEX:
        raise ValueError(f"unknown step: {payload.get('step')}")
    if payload.get("status") not in STATUSES:
        raise ValueError(f"unknown status: {payload.get('status')}")
    return payload


def qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class ProcessReporter:
    """자동화 노드가 공정 상태를 모니터로 보내는 한 줄짜리 창구."""

    def __init__(self, node, topic=TOPIC):
        self.node = node
        self.pub = node.create_publisher(String, topic, qos())
        self.step = None

    def set(self, step, message="", progress_in_step=None, waiting=None):
        """waiting에 'DI13' 같은 문자열을 주면 상태가 WAITING으로 나간다."""
        status = "WAITING" if waiting else ("DONE" if step == "DONE" else "RUNNING")
        if waiting and not message:
            message = f"{waiting} 입력 대기"
        self.pub.publish(String(data=encode(step, status, message, progress_in_step)))
        self.step = step

    def error(self, message):
        """현재 스텝에서 이상 발생. 스텝은 유지한 채 상태만 ERROR."""
        step = self.step or "IDLE"
        self.pub.publish(String(data=encode(step, "ERROR", message)))


def _demo():
    assert STEP.GRINDING == "GRINDING"
    assert step_progress("INIT") == 0.0
    assert step_progress("DONE") == 100.0

    payload = decode(encode(STEP.GRINDING, "RUNNING", "test"))
    assert payload["step"] == "GRINDING"
    assert payload["label"] == "그라인딩"
    assert payload["index"] == STEP_INDEX["GRINDING"]

    # 스텝 내부 진행도는 다음 스텝 사이를 선형 보간
    half = decode(encode(STEP.GRINDING, progress_in_step=0.5))["progress"]
    assert step_progress("GRINDING") < half < step_progress("MOVE_DRIPPER"), half
    full = decode(encode(STEP.GRINDING, progress_in_step=1.0))["progress"]
    assert abs(full - step_progress("MOVE_DRIPPER")) < 0.2, full

    for bad in (lambda: encode("NOPE"), lambda: encode("IDLE", "BOOM"),
                lambda: decode('{"step":"NOPE","status":"RUNNING"}')):
        try:
            bad()
        except ValueError:
            pass
        else:
            raise AssertionError("검증 없이 통과됨")

    assert is_expected_transition(None, "INIT")
    assert is_expected_transition("IDLE", "PICK_BEANS")
    assert is_expected_transition("GRINDING", "GRINDING")
    assert is_expected_transition("DONE", "IDLE")        # 사이클 반복
    assert is_expected_transition("DRIPPING", "IDLE")    # 중단 후 대기 복귀
    assert not is_expected_transition("IDLE", "GRINDING")  # 스텝 건너뜀

    assert DI_TRIGGER["IDLE"] == 13 and DI_TRIGGER["WAIT_LID"] == 14
    print("process_state self-check OK")


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        _demo()
