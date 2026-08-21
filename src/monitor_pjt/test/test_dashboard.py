# =============================================================
# test_dashboard.py — 로봇/ROS 없이 대시보드 로직 검증 (offscreen Qt)
# -------------------------------------------------------------
# 실행: pytest src/monitor_pjt/test/test_dashboard.py
# 검증: 배너 우선순위, 제어 게이트(E-Stop 예외), jog press/release,
#       로그 중복 출력 방지
# =============================================================

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication

from monitor_pjt import process_state as ps
from monitor_pjt.dashboard import Dashboard
from monitor_pjt.snapshot import Snapshot


class FakeNode:
    """SystemMonitor가 대시보드에 노출하는 부분만 흉내낸다."""

    def __init__(self):
        self.snap = Snapshot()
        self.logs = []
        self.calls = []

    def snapshot(self):
        return self.snap

    def set_control_enabled(self, enabled):
        self.calls.append(("enable", enabled))

    def emergency_stop(self):
        self.calls.append(("estop",))

    def jog(self, axis, speed):
        self.calls.append(("jog", axis, speed))

    def jog_stop(self, axis):
        self.calls.append(("jog_stop", axis))


@pytest.fixture(scope="module")
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def gui(app):
    node = FakeNode()
    return Dashboard(node), node


def test_refresh_and_log(gui):
    window, node = gui
    node.snap.robot_state_str = "STANDBY"
    node.snap.link_state = "OK"
    node.snap.joint_pos_deg = [10.0, -20.0, 30.0, 0.0, 5.0, -5.0]
    node.snap.task_progress = 42.0
    node.logs.append("12:00:00 [ERROR] boom")
    window.refresh()

    assert "정상" in window.lbl_banner.text()
    assert window.bar_task.value() == 42
    assert window.txt_log.toPlainText().count("boom") == 1
    window.refresh()                      # 같은 줄이 다시 찍히면 안 됨
    assert window.txt_log.toPlainText().count("boom") == 1


def test_banner_priority(gui):
    window, node = gui
    node.snap.link_state = "STALE"
    node.snap.estop = True
    window.refresh()
    assert window.lbl_banner.text() == "EMERGENCY STOP"


def test_control_gate_never_disables_estop(gui):
    window, node = gui
    assert not window.btn_start.isEnabled()
    assert window.btn_estop.isEnabled()

    window.chk_control.setChecked(True)
    assert window.btn_start.isEnabled() and window.jog_buttons[0].isEnabled()
    assert ("enable", True) in node.calls

    window.chk_control.setChecked(False)
    assert not window.jog_buttons[0].isEnabled()
    assert window.btn_estop.isEnabled()

    window.btn_estop.click()
    assert ("estop",) in node.calls       # 게이트 OFF여도 E-Stop은 나간다


def test_process_panel_tracks_step(gui):
    window, node = gui
    node.snap.step = "GRINDING"
    node.snap.step_index = ps.STEP_INDEX["GRINDING"]
    node.snap.step_label = ps.STEP_LABEL["GRINDING"]
    node.snap.step_status = "RUNNING"
    node.snap.task_progress = ps.step_progress("GRINDING")
    node.snap.di = [0] * 16
    window.refresh()

    assert "그라인딩" in window.lbl_step.text()
    current = window.chips[ps.STEP_INDEX["GRINDING"]].styleSheet()
    before = window.chips[ps.STEP_INDEX["IDLE"]].styleSheet()
    after = window.chips[ps.STEP_INDEX["DRIPPING"]].styleSheet()
    assert current != before and current != after   # 현재/완료/대기가 구분됨

    # WAIT_LID면 DI14 램프가 "대기중"으로 강조된다
    node.snap.step = "WAIT_LID"
    node.snap.step_index = ps.STEP_INDEX["WAIT_LID"]
    node.snap.step_status = "WAITING"
    window.refresh()
    assert window.leds_button[14].styleSheet() != window.leds_button[15].styleSheet()

    # 공정 ERROR는 배너에 붉게 뜬다
    node.snap.step_status = "ERROR"
    node.snap.step_message = "gripper timeout"
    window.refresh()
    assert "공정 오류" in window.lbl_banner.text()


def test_jog_press_release(gui):
    window, node = gui
    window.chk_control.setChecked(True)
    window.sld_speed.setValue(7)
    assert node.calls[-1] == ("enable", True)   # 슬라이더 이동만으론 명령 없음

    window.jog_buttons[0].pressed.emit()
    window.jog_buttons[0].released.emit()
    assert node.calls[-2] == ("jog", 6, 7)
    assert node.calls[-1] == ("jog_stop", 6)
