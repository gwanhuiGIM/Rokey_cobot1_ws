---
updated: 2026-08-19
---

# cobot1_ws 패키지 README 인덱스

`docs/src` → `../src` 심링크로 vault에서 열린다. 링크는 `ws/cobot1/src/...` 경로.

## 우리가 쓴 것 (`src/`)

| 패키지 | README | 한 줄 |
|---|---|---|
| `move` | [[ws/cobot1/src/move/README\|move]] (119L) | 쟁반 잡고 F/T 피드백으로 판 기울여 물체 중심 유지 |
| `tray_balance_kkh` | [[ws/cobot1/src/tray_balance_kkh/README\|tray_balance_kkh]] (273L) | 위와 같은 주제의 확장판 — 실제로 쓰는 쪽 |
| `cobot_rg2` | [[ws/cobot1/src/cobot_rg2/README\|cobot_rg2]] (241L) | M0609 + RG2 브링업 워크스페이스 전체 안내 |
| `coffee_system` | [[ws/cobot1/src/coffee_system/README\|coffee_system]] (44L) | DRL `m0609_coffe_system.drl` → 단일 파이썬 노드 변환 |
| `rokey` | [[ws/cobot1/src/rokey/README\|rokey]] (80L) | 커피 추출 태스크 + 웹 모니터링/제어 UI + 중심이탈 GUI |
| `monitor_pjt` | [[ws/cobot1/src/monitor_pjt/README\|monitor_pjt]] (134L) | 핸드드립 공정 상태 실시간 모니터 + 게이트 통한 원격제어 |
| `monitor_sys` | [[ws/cobot1/src/monitor_sys/monitor_sys/monitor_pjt/README\|monitor_sys/monitor_pjt]] (73L) | 웹 관리자 모니터 — 상태 확인 + 제한된 수동제어 서비스 |
| `dooy_monitor_spiral` | [[ws/cobot1/src/dooy_monitor_spiral/README\|dooy_monitor_spiral]] (312L) | `cobot_rg2` README 복사본 (내용 중복) |
| ↳ 내부 | [[ws/cobot1/src/dooy_monitor_spiral/dooy_spiral_monitor/dooy_spiral_monitor/README\|dooy_spiral_monitor 개발노트]] (198L) | 주전자 잡고 필터 위 나선 물따르기 모듈 — **이쪽이 실내용** |

**README 없음**: `cup_detect`, `adaptive_move`

## 업스트림 서브트리 (수정 대상 아님)

- [[ws/cobot1/src/cobot_rg2/doosan-robot2/README\|doosan-robot2]] (191L) — Doosan 공식 ROS 2 패키지
- [[ws/cobot1/src/cobot_rg2/onrobot-ros2/README\|onrobot-ros2]] (125L) — ABC-iRobotics OnRobot 컨트롤러
- dsr_example / dsr_realtime_control / dsr_visualservoing / dsr_mujoco — 공식 예제

## 별도 트리 (vault 밖, 루트에서 빌드 금지)

- `coffee_pipeline_ws/README.md` (363L) — 커피공정+컵파지+모니터+웹UI를 한 상태머신으로 합친 작업 ws
- `DooSan_Robotics_Cobot_Project/src/rokey/README.md` (80L) — `src/rokey` README와 동일 내용
