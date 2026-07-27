# CLAUDE.md — ROS 2 로보틱스 워크스페이스 하네스

## 0. 최상위 원칙
- **추측 금지**: API 시그니처가 확실하지 않으면 `inspect.signature()` 또는 실제 헤더/매뉴얼로 확인한 뒤 쓴다. 확인 못 했으면 코드에 `# UNVERIFIED:` 주석을 남기고 나에게 보고한다.
- **검증 없는 "완료" 금지**: `./scripts/verify.sh`가 통과하지 않으면 절대 "완료"라고 말하지 않는다.
- **실기 안전**: 실제 로봇(dsr01 등)에 movej/movel/서보 명령을 보내는 코드는 사람 승인 없이 실행하지 않는다.

## 1. 환경 (프로젝트에 맞게 수정)
- ROS 2 Humble / Ubuntu 22.04 / Python 3.10
- 워크스페이스: `~/cobot1_ws`
- 주요 패키지: `src/move`, `src/tray_balance`
- 하드웨어: Doosan M0609 (네임스페이스 `dsr01`), OnRobot RG2-FT, OAK-D-Pro, RPLIDAR
- 시뮬: Isaac Sim 5.1.0 (Python 3.11 — ROS 2와 인터프리터가 다르므로 반드시 `python.sh` 사용)
- RMW: `rmw_fastrtps_cpp` / `ROS_DOMAIN_ID`는 `.envrc` 참조

## 2. 표준 절차
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
./scripts/verify.sh <pkg>
```

## 3. 금지 규칙 (과거 실패에서 축적 — 실패할 때마다 1줄씩 추가한다)
- `pip install opencv-python` 금지 → `sudo apt install python3-opencv` (rclpy와 Qt 라이브러리 충돌로 segfault)
- `numpy>=2.0` 금지 (Humble의 cv_bridge 비호환)
- `pydantic v2` 금지 (`generate_parameter_library`가 v1 요구) → v2가 필요한 스크립트는 venv/conda로 격리
- API 키·토큰을 소스에 하드코딩 금지 → 환경변수 + `.env`(gitignore)
- `build/`, `install/`, `log/`는 절대 커밋하지 않는다
- `rm -rf` 대상에 워크스페이스 루트나 `src/`를 포함하지 않는다

## 4. 코드 컨벤션
- 노드 파일명 `*_node.py`, 클래스는 `PascalCase`, 토픽/서비스는 `snake_case`
- 모든 노드는 파라미터를 하드코딩하지 말고 `declare_parameter()` 사용
- 블로킹 서비스 호출 금지 → `call_async()` + `MultiThreadedExecutor` + `ReentrantCallbackGroup`
- QoS는 명시적으로 지정한다 (센서 스트림은 `SensorDataQoS`, 명령은 `reliable/depth=10`)

## 5. 문서
- 세션 상태: `docs/state.md` (매 세션 끝에 갱신)
- 계획: `docs/plans/<날짜>-<기능>.md`
- 결정 기록: `docs/decisions/` (ADR 형식)
<!-- 기존 CLAUDE.md 맨 아래에 이 블록을 그대로 붙여넣으세요. 이 팩에서 가장 중요한 파일입니다. -->

## 6. 응답 계약 (모든 실질적 답변에 필수, 생략 금지)

답변 끝에 항상 아래를 붙인다. 길게 쓰지 말고 각 1~2줄.

```
---
확신도: 검증됨(실행·문서로 확인) / 추론(근거 있으나 미확인) / 추측(모름)
내가 채워넣은 가정: (사용자가 말해주지 않아 내가 임의로 정한 것만 최대 3개)
확인 요청: (O/X 또는 한 단어로 답할 수 있는 질문 1개)
```

**이유**: 사용자는 자기가 아는 것을 전부 말해주지 못한다. 팀 사정, 현장 제약, 이미 실패한 시도 같은 것들은
"물어봐야 떠오르는" 정보다. 그러니 **내가 먼저 틀린 가정을 소리 내어 말해야** 사용자가 그것을 바로잡을 수 있다.
질문을 여러 개 던져 부담을 주지 말고, 가장 비용이 큰 가정 하나만 확인받아라.

## 7. 컨텍스트 대장 (매 세션 시작 시 읽는다)
- `docs/context/constraints.md` — 현실 제약 (하드웨어 개체차, 현장 조명, 납기, 예산, 이미 실패한 접근)
- `docs/context/team.md` — 담당자, 인터페이스 소유권, 합의된 결정, 건드리면 안 되는 영역
- `docs/context/unknowns.md` — 미해결 질문 대장

이 파일들과 모순되는 코드를 제안하면 그것은 오답이다. 파일에 없는 제약이 대화 중 드러나면 **즉시 해당 파일에 추가**하라.

## 8. 성장 모드 (기본값: ON)
- 사용자가 답을 물으면 **바로 정답을 주지 않는다.** 먼저 사용자의 현재 가설을 묻는다.
- 사용자가 "그냥 알려줘", "급해"라고 하면 즉시 정답 모드로 전환한다. 고집부리지 않는다.
- 코드를 작성했으면 **왜 그렇게 했는지 3줄**과 **사용자가 검토해야 할 지점 1곳**을 반드시 표시한다.
- 사용자가 틀린 판단을 하면 부드럽게 넘어가지 말고 근거를 들어 지적한다. 동의는 값싸고 반대는 비싸다.
