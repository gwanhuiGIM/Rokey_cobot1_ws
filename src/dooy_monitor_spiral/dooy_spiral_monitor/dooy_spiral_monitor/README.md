# 핸드드립 물 따르기 모듈 (M0609) — 개발 노트

로봇 팔(Doosan **M0609** + GripperDA)로 주전자 손잡이를 잡고, 필터 위에서
**나선을 그리며 물을 따르는** 핸드드립 커피 자동화 모듈. 전체 핸드드립 공정
중 **PICK_KETTLE → DRIPPING → RETURN_KETTLE** 구간을 담당한다.

---

## 1. 파일 구성

| 파일 | 역할 | 실행 |
|---|---|---|
| [kettle_circle_pour.py](kettle_circle_pour.py) | 메인 - 잡기/이동/기울임/나선 붓기/복귀 | `ros2 run rokey hand_drip_pour` |
| [kettle_circle_pour_v4.py](kettle_circle_pour_v4.py) | 최신 - 붓기 전용(잡기/내려놓기 없음), 주둥이 피벗 고정 계산 | `ros2 run rokey kettle_circle_pour_v4` |
| [path_viz.py](path_viz.py) | 필터와 TCP/주둥이 실시간 이동경로 패널 | 메인 실행 시 `--viz` |
| [teach_helper.py](teach_helper.py) | 현재 포즈(posx/posj) 캡처 티칭 헬퍼 | `ros2 run rokey hand_drip_teach` |

버전별 반복 개발 파일(`kettle_circle_pour_v1.py`~`v3.py`, `kettle_circle_pour_test.py`)은
과거 시도/가상 제어기 재현용이라 위 표에서 생략했다. 티치펜던트에 직접 올리는
DRL 버전은 [handDrip_DRL/](handDrip_DRL/)(§10) 참고.

레퍼런스 컨벤션: `DSR_ROBOT2`(movej/movel/movec), 그리퍼 = digital output,
`F_3_drl_sun_gear.py` 스타일.

관련: 모니터링 패키지 `rokey/monitor_pjt/`(system_monitor + dashboard)를
`hand_drip` 옆에 통합해 두었다. → `ros2 launch rokey monitor.launch.py`

---

## 2. 동작 순서

```
홈(joint) → 주전자 잡기(원호 접근) → 들어올림
   → 필터 위로 수평 이동
   → 기울임(천천히, 주둥이 기준)
   → 나선 그리기: 중앙→밖(rmax)→다시 중앙, 각 3회전
   → 세워서 붓기 멈춤 → 제자리에 내려놓기(원호) → 홈
```

---

## 3. 핵심 파라미터 (실기 튜닝 완료값)

```python
# 좌표 (control GUI / 대시보드 TCP 값으로 티칭)
KETTLE_GRIP_POS = [277.5, 112.6, 110.9, 83.7, 94.1, -176.4]   # 잡기=놓기 자리
FILTER_POUR_POS = [416.3933, 112.5524, 268.6189, 83.7431, 94.0589, -176.3823]
#   → 주전자 주둥이가 필터 조금 위에 온 상태의 그리퍼 TCP

# 주둥이 오프셋 (그리퍼 TCP 기준, Tool 좌표계 mm)
SPOUT_OFFSET_MM = [40.0, 170.0, 0.0]   # tool +X=위4cm, tool +Y=앞(주둥이)17cm

# 기울임 (붓기)
POUR_TILT_DEG  = 30.0
POUR_TILT_AXIS = "rz"      # 주둥이가 tool +Y라, 주둥이 끝 내림 = tool Z축 회전
TILT_VEL_ROT   = 6.0       # 기울임 회전 속도(천천히) [deg/s]

# 나선
CIRCLE_METHOD  = "spiral"
CIRCLE_RADIUS_MM = 40.0    # 나선 최대 반지름
SPIRAL_REVS      = 3       # 회전수(밖/안 각각)
SPIRAL_STEPS_PER_REV = 24  # 회전당 점 개수
SPIRAL_BLEND_MM  = 5.0     # 점 사이 블렌딩
CIRCLE_VEL_MM_S  = 20.0    # 나선 속도

# 잡기 원호(손잡이 충돌 회피)
GRIP_ARC_DIR      = [1.0, 0.0, 0.0]  # 회전축 Y (base X방향 접근). 반대면 [-1,0,0]
GRIP_ARC_RADIUS_MM = 80.0
GRIP_ARC_VEL_MM_S  = 25.0

# 전체 저속
VEL_J=20, VEL_X_TRANS=50, VEL_X_ROT=20
```

---

## 4. 좌표계 / 주둥이 오프셋 (중요 개념)

- **Tool(공구) 좌표계**: 플랜지 기준 고정. TCP는 정적 오프셋이라 그리퍼가
  열리든 닫히든 안 변한다.
- **주둥이를 TCP로 안 쓰고 계산으로 처리**: 펜던트 TCP 등록 없이, 붓기 자세의
  방향(A,B,C)으로 주둥이 오프셋을 base로 변환. 자세가 일정하면 그리퍼가 그리는
  나선 = 주둥이가 그리는 나선(반지름 동일)이라, 주둥이가 필터 위에 오도록만
  맞추면 된다.
- **축 검증(중요한 함정)**: "앞/위"가 tool 어느 축인지 Tool 좌표 조그로 확인해야
  한다. 이 자세에서는 **tool +X = base 위**, **tool +Y = base 앞(수평)**,
  tool +Z = base 옆. 그래서 주둥이(앞 17cm) = tool **+Y**.
- **붓기 축**: 주둥이가 tool +Y이므로, 기울여 붓기(주둥이 끝 내림)는
  tool **Z축(rz)** 회전이다. (ry로 하면 주둥이 축으로 헛돎 → 안 부어짐)

---

## 5. 티칭 워크플로우 (TP 제어 + PC 모니터링)

- "제어권 없는 브링업"은 없다. 브링업은 평소대로(`mode:=real`) 켠다.
- 제어권 = 로봇 **운전모드**: **Manual**=티치펜던트 제어(ROS는 읽기만),
  **Auto**=ROS 제어.
- 모니터(`system_monitor`, `control_enabled=false`)는 명령을 안 보내므로
  TP가 제어권을 가져도 충돌 없이 관절/TCP를 실시간 표시.
- 좌표 티칭: 대시보드/펜던트의 TCP(x,y,z,A,B,C)를 읽어 파라미터에 붙여넣기.
  (`get_current_posx`는 제어권과 무관한 읽기 전용)

---

## 6. 겪은 이슈 & 해결

| 이슈 | 해결 |
|---|---|
| 첫 실행이 첫 모션에서 멈춤 | 로봇을 모션 받는 상태(Auto/서보온)로. 단계별 로그로 위치 확인 |
| 잡을 때 손잡이와 충돌 | 직하강 대신 **원호(moveC) 접근**. 회전축 Y(base X방향) |
| 붓는 방향(기울임) 틀림 | 기울임 축을 ry→**rz**로. 주둥이 끝이 실제로 내려가야 붓기 |
| 주둥이 오프셋이 위로 계산됨 | 축 라벨 오류. tool +X는 위, 앞은 tool +Y → `[40,170,0]` |
| `DR_SPIRAL_OUTWARD` import 실패 | 설치된 `move_spiral`이 구버전(방향 인자 없음). **나선 좌표를 직접 생성**해 movel 블렌딩으로 그림 |
| `--symlink-install` 빌드 에러 (launch 추가 후) | `rm -rf build/rokey install/rokey` 후 재빌드 |

---

## 7. 빌드 & 실행

```bash
cd ~/ws_cobot_pjt/ws_dsr
colcon build --symlink-install --packages-select rokey
source install/setup.bash

ros2 run rokey hand_drip_pour     # 물 따르기 (첫 테스트는 저속·물 없이 권장)
ros2 run rokey hand_drip_pour --viz  # 실시간 필터/TCP 경로 패널과 함께 실행
ros2 run rokey hand_drip_teach    # 좌표 티칭
ros2 launch rokey monitor.launch.py   # 상태 모니터 (읽기전용)
```
> `.py` 수정은 `--symlink-install`이라 재빌드 없이 반영. `setup.py`/launch/
> package.xml 변경 시에는 재빌드 필요.

---

## 8. 가상 제어기에서 실패 궤적 영상 촬영

실제 로봇 없이 DRCF 에뮬레이터가 계산하는 TCP를 `path_viz`에서 읽어, 과거의
두 실패 장면을 재현할 수 있다. 테스트는 `/dsr01/virtual_node`가 발견되고
실기 전용 노드, 중복 컨트롤러, 로컬 `mode:=real` bringup 및 다른 붓기 모션
프로세스가 없을 때만 모션을 허용하며 Tool/TCP 등록과 그리퍼 출력은 변경하지
않는다.

```bash
# 터미널 1
cd ~/ws_cobot_pjt/ws_dsr
source install/setup.bash
# 먼저 기존 mode:=real/virtual bringup을 모두 Ctrl+C로 종료한다.
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual

# 터미널 2: MoveJ 기울임 뒤 밀려난 중심에서 MoveC
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 run rokey kettle_circle_pour_test --case movec --viz --keep-viz

# 터미널 2: 잘못된 오프셋으로 계산한 spiral + 점진 기울임
ros2 run rokey kettle_circle_pour_test --case spiral --viz --keep-viz
```

- `movec`: 초록색 필터 중심은 기울이기 전 주둥이 위치에 고정된다. 주황색
  주둥이 경로가 MoveJ 중 밖으로 이동하고, MoveC 원도 밀려난 곳에 그려진다.
- `spiral`: 시작 순간에는 주황색 경로와 중심이 일치하지만 추가 기울기가
  누적될수록 잘못 가정한 오프셋 때문에 중심이 서서히 이탈한다.
- `--keep-viz`는 Enter를 누를 때까지 결과 패널을 유지한다. 자동 종료하려면
  대신 `--hold 15`처럼 유지 시간을 지정한다.

---

## 9. 남은 튜닝 포인트

- 나선이 끊기면 `SPIRAL_BLEND_MM`↑ 또는 `SPIRAL_STEPS_PER_REV`↑
- 잡기 원호 방향이 반대면 `GRIP_ARC_DIR = [-1,0,0]`
- 기울임 부호가 반대면 `POUR_TILT_DEG = -30`
- (선택) `monitor_pjt`의 `ProcessReporter` 연동으로 대시보드에 단계 표시

---

## 10. 티치펜던트 DRL 프로그램 (handDrip_DRL/)

[kettle_circle_pour_v4.py](kettle_circle_pour_v4.py)의 붓기 로직을 ROS2 없이
펜던트에서 바로 실행할 수 있게 옮긴 DRL 4쌍(Task Program `.drl` + Sub
Program `_sub.drl`, 총 8개). `.py`처럼 실행 인자로 파라미터를 못 바꾸므로
좌표/속도/반지름 등은 각 `_sub.drl` 상단 상수를 펜던트에서 직접 고쳐야 한다.

| 파일 | 궤적 | 상태 |
|---|---|---|
| [kettle_pour_circle.drl](handDrip_DRL/kettle_pour_circle.drl) / [_sub](handDrip_DRL/kettle_pour_circle_sub.drl) | 고정 반지름 원 | v4.py 그대로 이식 |
| [kettle_pour_circle_updown.drl](handDrip_DRL/kettle_pour_circle_updown.drl) / [_sub](handDrip_DRL/kettle_pour_circle_updown_sub.drl) | 고정 반지름 원 + Z축 상하운동 | v4.py `--updown` 이식 |
| [kettle_pour_spiral.drl](handDrip_DRL/kettle_pour_spiral.drl) / [_sub](handDrip_DRL/kettle_pour_spiral_sub.drl) | 나선(반지름 0→최대→0) | ⚠️ 새로 설계, 실기 미검증 |
| [kettle_pour_spiral_updown.drl](handDrip_DRL/kettle_pour_spiral_updown.drl) / [_sub](handDrip_DRL/kettle_pour_spiral_updown_sub.drl) | 나선 + Z축 상하운동 | ⚠️ 새로 설계, 실기 미검증 |

- **circle 계열**: v4.py의 numpy 행렬/비동기 `amovel`+`check_motion` 폴링을
  펜던트 네이티브 환경에 맞게 순수 3x3 행렬 연산 + 동기 `movel`/`movesx`로
  바꾼 것 외에는 로직이 동일하다.
- **spiral 계열**: v4.py에는 없던 궤적이다. v3의 나선 형상(반지름
  0→`SPIRAL_RADIUS_MM`→0, 연속 회전)을 v4의 드리프트 없는 피벗 역산
  방식(매 경유점마다 주둥이가 정확히 그 점에 있도록 TCP를 계산)으로 다시
  구현했다. **반드시 물 없이 저속으로 궤적부터 확인할 것.**
- 사용법: main `.drl`과 대응하는 `_sub.drl`을 같은 프로젝트에 Sub Program으로
  등록한 뒤, main `.drl`을 Play. main은 `run_...()` 함수 한 줄 호출뿐이다.
