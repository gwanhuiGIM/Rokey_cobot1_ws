# =============================================================
# snapshot.py — 시스템 모니터가 GUI로 내보내는 상태 스냅샷 정의
# -------------------------------------------------------------
# 실행: 라이브러리 (단독 실행 없음)
# 의존: dataclasses만 — dsr_msgs2/rclpy를 몰라도 되게 일부러 분리했다.
# -------------------------------------------------------------
# system_monitor.py(로봇 쪽)와 dashboard.py(GUI 쪽)는 서로 다른 프로세스로
# 뜨고 JSON 문자열(~/status 토픽)로만 통신한다. 이 dataclass가 그 JSON의
# 스키마다: system_monitor는 asdict(snapshot)으로 직렬화해서 보내고,
# dashboard는 Snapshot(**json.loads(...))으로 그대로 복원한다 — 필드 이름이
# 어긋나면 그 자리에서 TypeError로 바로 드러난다.
# =============================================================

from dataclasses import dataclass, field


@dataclass
class Snapshot:
    """GUI 한 프레임이 필요로 하는 전부. UI는 이 dict(JSON)만 보면 된다."""
    stamp: float = 0.0
    # 1. 로봇 상태
    robot_state: int = -1
    robot_state_str: str = "UNKNOWN"
    robot_mode: str = "UNKNOWN"
    speed_mode: str = "UNKNOWN"          # NORMAL / REDUCED (안전 속도 제한)
    # 공정 진행정보 (자동화 노드가 /coffee_process/state로 보고, process_state.py)
    step: str = ""                       # STEP key (IDLE, GRINDING, ...)
    step_label: str = "(공정 보고 없음)"
    step_index: int = -1
    step_status: str = ""                # RUNNING / WAITING / DONE / ERROR
    step_message: str = ""
    step_elapsed: float = 0.0            # 현재 스텝 경과 [s]
    task_progress: float = 0.0           # 공정 전체 진행률 0~100 [%]
    joint_pos_deg: list = field(default_factory=list)
    tcp_posx: list = field(default_factory=list)
    linear_speed: float = 0.0            # TCP 병진 속도 [mm/s]
    angular_speed: float = 0.0           # TCP 회전 속도 [deg/s]
    # 2. 통신
    link_state: str = "DISCONNECTED"     # OK / STALE / DISCONNECTED
    link_age_ms: float = -1.0
    link_hz: float = 0.0
    link_latency_ms: float = 0.0
    link_loss_rate: float = 0.0
    service_ready: bool = False          # 드라이버 서비스 응답 여부
    gripper_connected: bool = False
    # 3. 안전
    estop: bool = False
    safety_stop: bool = False            # SAFE_STOP = 충돌/안전정지 의심
    collision_suspected: bool = False
    fault: bool = False
    last_error: str = ""
    last_error_level: str = ""
    # 4. IO / 그리퍼
    di: list = field(default_factory=list)
    do: list = field(default_factory=list)
    gripper_width_mm: float = 0.0
    gripper_busy: bool = False
    gripper_grip_detected: bool = False
    # 제어 게이트 상태
    control_enabled: bool = False
