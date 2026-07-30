# rokey

Doosan M0609 + OnRobot RG2 협동로봇의 자동 커피 추출 태스크, 그 웹 모니터링/제어
UI, 판 위 물체 중심 이탈 감지 GUI로 구성된 ROS 2 (ament_python) 패키지.

## 빌드 및 실행

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rokey
source install/setup.bash
```

두산 M0609 브링업(네임스페이스 `dsr01`)이 먼저 떠 있어야 한다. `web_ui`는 `fastapi`·`uvicorn`(pip)이 추가로 필요하다.

- `ros2 run rokey coffee_system` — 자동 커피 추출 태스크
- `ros2 run rokey web_ui` — 웹 대시보드/제어 UI (`http://localhost:8000`)
- `ros2 run rokey plate_monitor` — 판 중심 이탈 감지 GUI

## `coffee_system.py` — 자동 커피 추출 태스크 노드

원두 선택부터 드립까지 전체 공정을 상태 머신으로 실행하는 메인 로봇 제어 노드.

**공정 순서**
1. 원두 선택 — DI 13~16 물리 버튼 입력 대기
2. `bean_drop` — 스푼으로 원두를 퍼서 그라인더 호퍼에 투입
3. 분쇄 회전수 선택 — DI 13~16 버튼으로 3/5/7/10회전 선택
4. `grinder` — 선택 회전수만큼 원호(movec) 모션으로 분쇄
5. `dripper_in` — 분쇄 원두가 든 병을 필터에 투입
6. `spiral_pour` — 주전자를 파지하고 보상 내향 스파이럴 드립 후 원위치 복귀
7. `final_drip` — mug TCP 기준으로 한 번 기울인 뒤 저장된 시작 자세로 복귀

**그리퍼 안전 감시(`GripMonitor`)**: `/OnRobotRGInput` 구독으로 파지 상태를 실시간
감시하며, 파지 실패(`GripFailureError`)나 그리퍼 신호 유실(`GripperSignalLostError`)
발생 시 `motion/move_stop` 서비스로 즉시 정지시킨다.

**단계 테스트 모드**: 전체 공정 외에 개별 단계(`bean_drop`, `grinder`, `dripper_in`,
`spiral_pour`, `final_drip`, `gripper_open`, `gripper_close`)만 단독 실행 가능.

**ROS 인터페이스** (네임스페이스 `dsr01`)
- 발행 `/coffee_system/status` (`std_msgs/String`, JSON): 현재 화면/진행률, 선택된
  원두·분쇄수, 스파이럴/최종드립 진행 단계, 힘 센서 값, 테스트 모드 결과 등을 실시간 보고
- 구독 `/coffee_system/control` (`std_msgs/String`, JSON): `{"cmd": "start_test", ...}`
  로 단계 테스트 시작, `{"cmd": "set_speed", "speed_percent": N}` 로 실행 속도 변경
- `motion/change_operation_speed` 서비스로 공정 실행 중에도 속도를 실시간 반영

## `web_ui.py` — 브라우저 대시보드 / 제어 UI (FastAPI, port 8000)

`coffee_system.py`가 발행하는 상태를 구독해 웹으로 노출하고, 명령을 다시 그쪽으로
전달하는 중계 서버.

| 경로 | 기능 |
|---|---|
| `GET /` | 커피 공정 진행 대시보드 (원두/분쇄 선택 표시, 진행률, 스파이럴·최종드립 단계) |
| `GET /api/state` | 대시보드 상태 JSON |
| `GET /test` | 단계별 테스트 실행 페이지 (로컬 접속만 허용) |
| `POST /api/test/start` | 지정 단계(`stage`, `grind_turns`, `gripper_open_mode`)만 단독 실행 요청 |
| `POST /api/control/speed` | 전체 공정 실행 속도 변경 (10~100%) |
| `GET /admin` | 로컬 전용 수동 조작 페이지: 로봇 상태/조인트/TCP/DI·DO 표시, e-stop, jog 이동(`move_task`), J6 틸트(`move_joint6`), 그리퍼 개폐 |
| `GET /api/admin/state` | 관리자 페이지 상태 JSON (`/system_monitor/status` 구독 기반) |
| `POST /api/admin/cmd` | 관리자 명령 실행: `estop`, `stop`, `start`, `set_control_enabled`, `set_mode`, `move_task`, `move_joint6`, `gripper` |
| `WS /ws` | 대시보드 상태를 0.2초 주기로 변경분만 push |

`/test`, `/admin` 계열 엔드포인트는 요청 클라이언트가 `127.0.0.1`/`::1`이 아니면
403으로 거부한다(로봇 PC 로컬 접속 전용).

## `plate_monitor.py` — 판 위 물체 중심 이탈 감지 GUI (tkinter)

로봇 모션 명령을 전혀 보내지 않고, `get_external_torque()` 값만으로 반지름 100mm
원형 판 위에 놓인 물체가 중심에서 얼마나 벗어났는지 실시간 추정하는 독립 GUI 도구.

**측정 절차**: 빈 판 기준 측정 → 동일 물체를 중심에 놓고 중심 기준 측정 → (선택)
Tool ±Y/±Z 4방향 보정 → 모니터 시작. 4방향 보정을 마치면 실제 mm 단위 이탈 방향/거리를
표시하고, 보정 전에도 이탈 크기와 증가/감소 추세는 표시한다. 측정 결과는 CSV와
모델 JSON으로 자동 저장된다.

**ROS 인터페이스** (네임스페이스 `dsr01`)
- 발행 `/{ROBOT_ID}/plate_position/features` (`std_msgs/Float64MultiArray`)
- 발행 `/{ROBOT_ID}/plate_position/status` (`std_msgs/String`, JSON)
- 구독 `/{ROBOT_ID}/plate_position/command` (`std_msgs/String`)
