# M0609 웹 관리자 모니터

`monitor_pjt`는 M0609의 상태를 웹에서 확인하고 제한된 수동 제어 명령을
ROS 2 서비스로 전달하는 관리자 모듈이다.

## 구성

| 파일 | 역할 |
| --- | --- |
| `web_ui.py` | FastAPI 웹 화면과 `SystemMonitor`를 함께 실행하는 진입점 |
| `system_monitor.py` | M0609 상태 수집 및 실제 제어 서비스 호출 |
| `snapshot.py` | 관리자 화면으로 전달할 상태 데이터 모델 |
| `process_state.py` | 커피 공정 단계와 상태 메시지 규약 |

웹 UI와 모니터는 다음 ROS 토픽으로 통신한다.

| 토픽 | 방향 | 내용 |
| --- | --- | --- |
| `/system_monitor/status` | 모니터 → 웹 | 로봇·통신·IO 상태 JSON |
| `/system_monitor/log` | 모니터 → 웹 | 관리자 이벤트 로그 |
| `/system_monitor/cmd` | 웹 → 모니터 | 이동·모드·정지·그리퍼 명령 JSON |

## 설치 및 빌드

```bash
python3 -m pip install "fastapi>=0.95" "uvicorn>=0.20"

cd ~/ws_cobot_pjt/ws_dsr
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

## 실행

M0609 bringup을 먼저 실행하고 별도 터미널에서 웹 UI 하나만 실행한다.
`web_ui`가 내부에서 `SystemMonitor`도 자동으로 시작한다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 launch rokey web_admin.launch.py robot_id:=dsr01
```

로봇 PC의 브라우저에서 다음 주소로 접속한다.

```text
http://127.0.0.1:8000
```

launch 없이 실행할 때도 `web_ui` 하나만 실행하면 된다.

```bash
ros2 run rokey web_ui
```

소스 파일을 직접 실행할 수도 있다.

```bash
python3 rokey/rokey/monitor_pjt/web_ui.py
```

`system_monitor`를 별도로 실행하면 같은 노드가 중복되므로 웹 UI와 동시에
실행하지 않는다.

## 제어 안전 조건

- 관리자 API는 로봇 PC의 localhost 접속만 허용한다.
- 이동·모드·그리퍼 명령은 `제어 활성화`를 체크한 뒤 사용할 수 있다.
- 처음에는 작은 이동 간격과 낮은 속도로 동작을 확인한다.
- `START`는 일반 공정 시작이 아니라 `SAFE_STOP` 상태 복귀 명령이다.
- 웹의 E-STOP은 소프트 정지 기능이며 물리 E-Stop을 대체하지 않는다.
