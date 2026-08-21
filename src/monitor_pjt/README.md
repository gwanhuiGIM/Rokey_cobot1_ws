# monitor_pjt — M0609 핸드드립 공정 시스템 모니터

Doosan **M0609** + OnRobot **RG2**로 핸드드립 커피를 만드는 공정 자동화
시스템의 상태를 실시간으로 보여주는 ROS 2 패키지. 로봇을 직접 움직이는
"자동화 노드"는 이 패키지 밖에 있고, 여기는 **모니터링 + (게이트를 통한)
원격 제어**만 담당한다.

> 하드웨어: M0609(6축, 가반 6kg) + RG2 | SW: ROS 2 Humble, `doosan-robot2`

---

## 아키텍처

**두 개의 독립 프로세스**가 토픽으로만 통신한다. 어느 한쪽이 죽어도
다른 쪽은 그대로 살아 있고, 대시보드만 여러 대 띄워도 된다.

```
┌────────────────────┐   /system_monitor/status (10Hz)   ┌─────────────────────┐
│   system_monitor    │ ─────────────────────────────────▶│   dashboard (Qt)     │
│ (system_monitor.py) │   /system_monitor/log             │  MonitorClient +     │
│ dsr_msgs2, DRL 토픽/ │ ─────────────────────────────────▶│  Dashboard 위젯      │
│ 서비스를 만지는       │                                     │  (dashboard.py)      │
│ 유일한 노드           │◀───────────────────────────────── │                       │
└────────────────────┘   /system_monitor/cmd             └─────────────────────┘
          ▲
          │ /coffee_process/state (TRANSIENT_LOCAL)
          │
┌────────────────────┐
│  자동화 노드 (외부)   │  예: rokey_move — DRL로 로봇을 실제로 움직이는 노드
│  process_state.py의 │  ProcessReporter로 공정 진행을 보고
│  ProcessReporter 사용 │
└────────────────────┘
```

- **system_monitor**만 `dsr_msgs2`를 import한다. `dashboard`는 `std_msgs`,
  `rclpy`, `PyQt5`만 있으면 뜨므로 로봇 SDK가 없는 PC에서도 원격으로
  모니터 창을 띄울 수 있다.
- 제어 명령(`~/cmd`)의 게이트(`control_enabled`)는 **항상 system_monitor
  안에서** 검사한다 — 대시보드의 체크박스는 표시일 뿐, 실제 차단은
  서버(system_monitor) 쪽이다.

---

## 노드 (entry points)

| 실행 | 파일 | 역할 |
|---|---|---|
| `ros2 run monitor_pjt system_monitor` | [system_monitor.py](monitor_pjt/system_monitor.py) | 로봇 상태 폴링/구독, 안전 판정, 공정 상태머신 감시 |
| `ros2 run monitor_pjt dashboard` | [dashboard.py](monitor_pjt/dashboard.py) | PyQt5 모니터 창 (읽기 전용 + 게이트가 걸린 제어) |
| (라이브러리) | [process_state.py](monitor_pjt/process_state.py) | 핸드드립 공정 상태머신 정의 + 보고 규약 (`ProcessReporter`) |
| (라이브러리) | [snapshot.py](monitor_pjt/snapshot.py) | 두 프로세스가 주고받는 JSON 스키마(`Snapshot`) |

빌드 및 실행:
```bash
cd ~/cobot1_ws
colcon build --symlink-install --packages-select monitor_pjt
source install/setup.bash

ros2 launch monitor_pjt monitor.launch.py                      # 권장: 두 노드 함께
ros2 launch monitor_pjt monitor.launch.py control_enabled:=true
```

로봇 없이 로직만 검증:
```bash
python3 -m monitor_pjt.system_monitor --selftest   # 통신품질/안전판정 알고리즘
python3 -m monitor_pjt.process_state --selftest    # 공정 상태머신
python3 src/monitor_pjt/monitor_pjt/dashboard.py    # 레이아웃만 (ROS 불필요)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/monitor_pjt/test/test_dashboard.py -q
```

---

## 화면 구성

| 영역 | 표시 항목 |
|---|---|
| 배너 | E-Stop > 충돌의심 > fault > 공정 ERROR > 통신이상 > 정상 순으로 가장 위험한 것 하나만 |
| 핸드드립 공정 진행 | 12단계 칩(완료/현재/대기 색 구분), 전체 진행률 바, 현재 스텝 경과시간 |
| 1. 로봇 상태 | 상태 문자열, 운전모드(AUTO/MANUAL), 속도모드(NORMAL/REDUCED), TCP 속도 |
| 2. 로봇 위치 | J1~J6(관절 한계 대비 바) + TCP posx |
| 3. 통신 상태 | ROS/서비스/그리퍼 연결 LED, OK·STALE·DISCONNECTED, Hz/latency/유실률, latency 그래프 |
| 4. 제어 | E-STOP(항상 활성), 제어 활성화 체크, START/STOP, MANUAL/AUTO, Jog 속도+방향 버튼 |
| 5. IO / 그리퍼 | DI/DO 16채널 LED + DI13/14 공정 버튼(대기 중이면 강조), RG2 폭/busy/파지 |
| 6. 로그 | INFO/WARN/ERROR 필터, 색상, 2000줄 상한 |

---

## 토픽/서비스 규약

### GUI ↔ system_monitor

| 토픽 | 타입 | 방향 | 내용 |
|---|---|---|---|
| `/system_monitor/status` | `std_msgs/String` (JSON, [snapshot.Snapshot](monitor_pjt/snapshot.py) 스키마) | monitor → GUI | 10Hz 전체 스냅샷 |
| `/system_monitor/log` | `std_msgs/String` | monitor → GUI | 이벤트 로그 1줄 |
| `/system_monitor/cmd` | `std_msgs/String` (JSON `{"cmd": ...}`) | GUI → monitor | 제어 명령 (아래 표) |

`cmd` 종류: `estop`(게이트 무시, 항상 실행) / `stop` / `start` /
`set_control_enabled{enabled}` / `set_mode{mode}` / `jog{axis,speed}` /
`jog_stop{axis}`.

### 자동화 노드 → system_monitor (핸드드립 공정 보고)

| 토픽 | 타입 | QoS | 내용 |
|---|---|---|---|
| `/coffee_process/state` | `std_msgs/String` (JSON) | RELIABLE + TRANSIENT_LOCAL | 공정 스텝/상태/진행률 — [process_state.py](monitor_pjt/process_state.py) |

TRANSIENT_LOCAL이라 모니터를 나중에 켜도 마지막 상태를 바로 받는다.
자동화 노드 쪽 사용법은 `process_state.py` 상단 주석과 `ProcessReporter`
docstring 참고.

### system_monitor → 로봇 드라이버 (doosan-robot2)

구독: `joint_states`, `error`, `robot_disconnection`,
`io/ctrl_box_digital_input_state`. 서비스 폴링(5Hz): `get_robot_state`,
`get_robot_mode`, `get_robot_speed_mode`, `get_current_posx/velx`. 제어
서비스: `set_robot_mode`, `set_robot_control`, `move_stop`, `jog`.
전부 이 워크스페이스의 doosan-robot2 소스(`dsr_controller2.cpp`)에서
실제 이름을 확인했다 — RobotState **토픽**은 이 버전에 없어서 서비스
폴링으로 대체했다.

---

## 안전 설계

- 기본은 **읽기 전용**(`control_enabled:=false`). 모션 명령은 대시보드의
  "제어 활성화" 체크 + system_monitor 파라미터가 모두 켜져야 나간다.
- **E-Stop만 게이트를 무시**하고 항상 즉시 실행된다.
- Jog 속도는 `MAX_JOG_SPEED_PCT`(20%)로 서버 쪽에서 하드 클램프한다 —
  GUI 슬라이더 상한을 우회해도 로봇에 그 이상 명령이 나가지 않는다.
- **소프트웨어 E-Stop은 물리 E-Stop의 대체재가 아니다.** 화면에도 명시.
- 충돌 전용 토픽이 없어 SAFE_STOP 진입 + SAFETY 그룹 에러로 "충돌 의심"을
  추정한다 (`TODO(verify)`, `system_monitor.py`의 `evaluate_safety` 참고) —
  실기에서 실제 충돌 시 에러 코드를 확인해 좁힐 것.
