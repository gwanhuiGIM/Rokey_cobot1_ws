# CLAUDE.md — cobot1_ws

> 공통 규칙(빌드 게이트·금지 규칙·응답 계약·문서 규칙)은 `~/.claude/CLAUDE.md`에 있다. 여기엔 **이 ws에서만 참인 것**만 둔다.
> Claude/Codex 멀티에이전트 위임·리뷰 절차는 `~/vault/ai/MULTI_AGENT_POLICY.md`(여러 ws 공유 단일 출처)를 따른다.

## 1. 환경
- ROS 2 Humble / Ubuntu 22.04 / Python 3.10 / RMW `rmw_fastrtps_cpp`
- 워크스페이스: `~/cobot1_ws`
- 하드웨어: Doosan **M0609** (네임스페이스 `dsr01`), OnRobot **RG2** / **RG6** 그리퍼, OAK-D-Pro, RPLIDAR
  - RG2는 **F/T 내장 모델이 아니다** (3절 참조). "RG2-FT"로 가정하고 실측 힘을 읽으려 하면 틀린다.
- 시뮬: Isaac Sim 5.1.0 (Python 3.11 — ROS 2와 인터프리터가 달라 반드시 `python.sh` 사용)

## 2. 패키지 지도 (`src/`)
| 패키지 | 역할 |
|---|---|
| `move` | M0609 모션 노드 + **`.drl` 스크립트** (tray_balance, m0609_gear, accel_estimation) |
| `tray_balance_kkh` | F/T 기반 쟁반 무게중심 추정 |
| `cobot_rg2` | 그리퍼 브링업 (`doosan-robot2` 서브트리 포함, `m0609_rg2_bringup/launch/bringup.launch.py`) |
| `cup_detect` | 컵 인식 + `probe_grip_v*` 접촉 탐지 |
| `coffee_system`, `rokey` | 커피 시퀀스 상위 로직, 웹 UI |
| `monitor_pjt`, `monitor_sys`, `dooy_monitor_spiral` | 모니터링 / 웹 어드민 |
| `adaptive_move` | — |

별도 트리: `coffee_pipeline_ws/`, `DooSan_Robotics_Cobot_Project/` (독립 ws, 루트에서 빌드하면 안 됨)

## 3. 실기에서 확인한 사실 (문서·직관과 다르다 — 재발견하지 말 것)
- **`movel(..., ref=DR_TOOL, mod=DR_MV_MOD_REL)`의 자세 슬롯 `[rx,ry,rz]`는 ZYZ 오일러가 아니라 툴축 회전벡터(axis-angle, deg)다.** 실기 검증(2026-07): `movel([0,0,0,5,0,0], REL, DR_TOOL)` → 툴 **X축** 회전. 회전벡터를 그대로 넣으면 되고 오일러 변환은 불필요. (ABS 모드/다른 ref는 미확인 — ZYZ일 수 있음)
- **RG2에는 실측 힘 피드백이 없다.** `/onrobot_joint_states`의 `effort`는 `busy ? force : 0`인 **명령값**이다. 파지 판정은 `position` plateau + `effort≠0`(stall)로 한다. 파손 감지 지표는 **width(=position) 차분**. 종이컵에 40N은 과하다(RG2 최소 ~3N).
- 하드웨어 grip 비트는 gSTA 상태워드(register 268 = `response[10]`)의 **bit1**. `comModbusTcp.getStatus`의 라벨이 틀려 `'grip'` 키는 항상 true다 → `(int(status['busy'])>>1)&1`로 뽑는다.
- **힘 기반 노드(`probe_grip*`, `tray_balance*`)가 방향성 편향·오검출을 보이면 코드가 아니라 펜던트를 먼저 본다.** DART의 Tool Weight 프리셋이 안 걸리면 그리퍼 자중(RG6 = 1.25kg ≈ 12.3N)이 외력으로 읽힌다. 노드의 `No Tool payload preset active` WARN 유무 / `GetCurrentTool` 응답으로 먼저 확인. 부호·threshold를 만지는 건 증상만 가린다.
- F/T 기반 무게중심 감지의 하한: **골프공(46g)은 감지 불가**(모멘트 변화 ~0.035Nm < 노이즈). 실제 대상(접시+음료 0.5~1.5kg)으로 테스트해야 한다.
- `tare`는 pose 고정 전제. tare 후 직접교시 등으로 관절을 움직이면 기준값이 무효다.

## 4. DRL (Doosan Robot Language) 작업 규칙
- `.drl`은 컨트롤러(DART)에서 도는 별개 언어다. **rclpy 노드가 아니고 colcon 빌드 대상도 아니다** — `python3 foo.drl`로 문법 검사하려 하면 안 된다.
- 파이썬 노드 ↔ DRL 값 전달은 `drvar`를 쓴다. 어느 쪽이 원천인지 코드 상단에 주석으로 명시한다.
- DRL에서만 되는 것: `get_external_torque()` 같은 컨트롤러 내부 실시간 신호. ROS 토픽으로는 주기·지연이 다르다.
- 파이썬 노드를 DRL로 옮기기 전에 **그 노드가 쓰는 API가 DRL에 존재하는지 매뉴얼로 확인**하고, 없으면 옮기지 말고 보고한다.

## 5. 검증
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
./scripts/verify.sh <pkg>   # 보조: 환경·시크릿·빌드·테스트·런치 스모크. SMOKE_LAUNCH 설정 시 런치까지
```
`verify.sh`는 이 ws에만 있다. 완료 판정의 필수 게이트는 `colcon build`이고, verify.sh는 그 위의 선택 점검이다.

## 6. 컨텍스트 대장
`docs/context/{constraints,team,unknowns}.md` — 현재 **빈 템플릿**이다. 3절의 사실들이 원래 여기 들어갈 내용이었다. 대화 중 새 제약이 드러나면 3절이나 이 파일들에 적는다(둘 다 적을 필요 없다).
