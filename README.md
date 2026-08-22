# cobot1_ws

Doosan **M0609** 협동로봇 + OnRobot **RG2/RG6** 그리퍼로 커피 서빙 태스크(원두 투입 →
그라인딩 → 드립 → 서빙 중 쟁반 균형 유지)를 자동화하는 개인 개발 ROS 2 (Humble) 워크스페이스.

> ⚠️ **개인 개발 진행 코드입니다.** 패키지별로 따로 개발되어 있어 "원두 투입 →
> 그라인딩 → 드립 → 서빙"을 하나로 잇는 통합 파이프라인은 없습니다. 각 패키지는
> 독립적으로 실행/검증되는 단계이고, 패키지 간 연결(단계 전환, 공통 상태 관리)은
> 아직 비어 있습니다. 아래 표의 "상태"는 그 공백을 표시한 것입니다.

## 패키지 지도

| 패키지 | 역할 | 상태 |
|---|---|---|
| `cobot_rg2` | M0609 + RG2 브링업 (`doosan-robot2` 서브트리 포함) | 브링업 확인됨 |
| `coffee_system` | 원두 투입 → 그라인딩 → 드립 (단일 파일 노드, 1회 실행) | 개별 동작 확인, 반복/예외처리 없음 |
| `rokey` | 커피 추출 + 웹 UI + 판 중심 이탈 감지 GUI 묶음 패키지 | `coffee_system`과 로직 중복 있음 (정리 안 됨) |
| `move` | 쟁반 위 물체 중심 유지 (F/T 피드백, `.drl` 스크립트 포함) | `tray_balance_kkh`와 별도로 진행 중인 버전 |
| `tray_balance_kkh` | 쟁반 위 물체 중심 유지 (F/T 기반) | **미완성** — 정상상태 완전 안정화 미달성 (`src/tray_balance_kkh/README.md` 참고) |
| `cup_detect` | 컵 인식 + 접촉 탐지(`probe_grip_v*`) | README 없음 — 코드가 최신 상태 |
| `monitor_pjt` | 공정 상태 실시간 모니터링 + 게이트 통한 원격 제어 | 동작 확인됨 |
| `monitor_sys`, `dooy_monitor_spiral` | 모니터링/웹 어드민 (변형 버전) | `monitor_pjt`와 관계 미정리 |
| `adaptive_move` | (역할 미상) | 미문서화 |

패키지별 상세 실행 방법·파라미터·검증 결과는 각 `src/<pkg>/README.md`를 본다
(있는 패키지만).

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
./scripts/verify.sh <pkg>   # 선택: 환경/시크릿/빌드/테스트/런치 스모크
```

전체 빌드(`colcon build`, 패키지 지정 없이)는 `coffee_pipeline_ws/`,
`DooSan_Robotics_Cobot_Project/` 등 독립 서브 워크스페이스를 끌어들이지 않도록
반드시 `--packages-select`로 범위를 좁힌다.

## 실기에서 확인한 것들

하드웨어 특이사항(RG2 힘 피드백 부재, `movel` REL+TOOL 자세 표현, F/T 감지 한계 등)은
`CLAUDE.md` 3절에 정리되어 있다. 재확인 없이 재발견하지 말 것.
