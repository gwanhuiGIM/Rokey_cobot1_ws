# move — M0609 서빙 자동화 (쟁반 위 물체 중심 유지)

Doosan **M0609** + **OnRobot RG2** 그리퍼로 쟁반(판)을 잡고, 판 위 물체(공/음료)가
가장자리로 굴러가지 않게 **TCP 힘/토크 피드백으로 판을 기울여 중심을 유지**하는 ROS 2 패키지.

> 하드웨어: M0609(6축, 가반 6kg) + RG2 + 힘/토크 센서 | SW: ROS 2 Humble, `doosan-robot2`
> 좌표계는 별도 표기 없으면 `DR_TOOL`(TCP) 기준. 자세는 ZYZ 오일러(A,B,C).

---

## 핵심 아이디어

물체가 판 중심에서 벗어나면 그 무게가 TCP에 **모멘트(tau = r × F)** 를 만든다.
이 토크를 역산해 물체의 평면 내 위치 오프셋 `(a, b)` 를 구하고, **PD 제어**로 판을
반대로 기울여(tilt) 물체를 중심으로 되돌린다.

- 중력 방향을 **하드코딩하지 않는다.** 정지 상태(tare) 힘 벡터 자체를 중력 방향으로
  써서 그립/TCP가 바뀌어도 그대로 동작한다 (`calibrate_gravity_frame`).
- **레버 팬텀 보상**(phantom 버전): 판을 기울이면 TCP→판중심 연장팔이 만드는
  모멘트가 tilt에 따라 변해 가짜 오프셋으로 잡힌다. 이걸 예측·상쇄해 자기진동/발산을 막는다.

---

## 노드 (entry points)

| 실행 | 파일 | 용도 |
|---|---|---|
| `ros2 run move tray_balance_phantom` | [tray_balance_phantom.py](move/tray_balance_phantom.py) | **메인.** 레버 팬텀 보상 포함 실기 제어 |
| `ros2 run move tray_balance` | [tray_balance.py](move/tray_balance.py) | 팬텀 보상 없는 기본판(수학/필터/상수의 원본) |
| `ros2 run move tray_balance_viz` | [tray_balance_viz.py](move/tray_balance_viz.py) | rviz2 시각화 보조 |
| `ros2 run move tray_balance_sim` | [tray_balance_sim.py](move/tray_balance_sim.py) | 시뮬(구 "Z가 아래" 가정 — 실제 노드와 불일치, 참고용) |
| `ros2 run move tray_balance_debug` | [tray_balance_debug.py](move/tray_balance_debug.py) | 센서/오프셋 디버그 |

> ⚠️ tray_balance 계열 노드는 **한 번에 하나만** 실행할 것 (같은 토픽/서비스 공유).

빌드:
```bash
cd ~/cobot1_ws
colcon build --packages-select move
source install/setup.bash
```

수학·부호 자체 검증(로봇 없이):
```bash
python3 -m move.tray_balance --selftest          # 중력프레임/오프셋/폐루프 수렴
python3 -m move.tray_balance_phantom --selftest  # 레버팔 복원/팬텀 보상 폐형식
```

---

## 실행 순서

1. **그리퍼로 판을 잡고**, 물체를 **판 정중앙**에 올린 뒤 손을 뗀다.
   (grip 구조상 TCP와 판중심이 중력방향 같은 높이 → 연장팔은 수평.)
2. 노드 실행. 시작 시 약 **3초간 tare(영점)** 측정 — 이 동안 판/물체를 흔들지 말 것.
3. 시작 로그에서 **tare 노이즈(std)** 와 **weight/레버** 값을 확인한다.
4. 물체를 밀어 중심에서 벗어나게 하면 판이 기울어 되돌린다.
5. 오프셋이 `SAFE_RADIUS_M`(판 반지름)을 넘으면 **이탈로 보고 즉시 정지**한다.

---

## 실행 전 반드시 맞출 상수 (`tray_balance.py` 상단)

값이 틀리면 오프셋 배율·안전판정이 전부 어긋난다. **한 번은 실측해서 넣을 것.**

| 상수 | 의미 | 확정 방법 |
|---|---|---|
| **`OBJECT_WEIGHT_N`** | **판에 올리는 물체만의 무게[N].** offset = 토크변화 ÷ 이 무게 | **저울로 실측.** 전체무게(판+그리퍼 ~5N)로 나누면 실제 이동이 ~12배 축소돼 보상이 약해짐 |
| `TRAY_RADIUS_M` / `BALL_RADIUS_M` | 판 반지름 / 물체 반지름[m] | 자로 실측. `SAFE_RADIUS_M` 계산에 사용 |
| `D_TCP_TO_TRAY_M` *(phantom)* | TCP→판중심 **중력방향** 거리[m] | 같은 높이 grip이면 **0**. 판이 아래로 매달리면 그 수직거리(아래로 +) |
| `G_HAT0_TOOL` *(phantom, 선택)* | 툴프레임 중력 단위벡터 고정값 | joint6 정렬을 매번 재현하면 그 값을 박아 run간 흔들림 제거. None이면 매 run 측정 |

---

## 튜닝 파라미터 (보상 세기)

`OBJECT_WEIGHT_N`을 먼저 맞춘 뒤, 아래를 **하나씩** 조정한다.

| 파라미터 | 역할 | 방향 |
|---|---|---|
| `KP` | 오프셋 1m당 tilt[deg] | 보상 세게 → ↑ (30mm를 5deg로 되돌리려면 ~167) |
| `KD` | 오프셋 속도 감쇠 | `KP` 올린 만큼 같이 ↑ (P만 키우면 중심 오가며 진동) |
| `MAX_TILT_DEG` | tilt 상한 | tilt가 상한에 saturate될 때만 ↑ |
| `CONTROL_DEADBAND_M` | 이 안쪽이면 명령 생략(채터링 방지) | 민감하게 → ↓ (너무 낮추면 노이즈에 떨림) |
| `COM_OK_THRESHOLD_M` | "balanced" **로그 판정**용 (제어는 안 멈춤) | 표시용 |
| `FILTER_ALPHA` / `MEDIAN_WINDOW` | EMA 계수 / 중앙값 창(스파이크 제거) | 노이즈 심하면 알파↓·창↑ (지연 증가 주의) |
| `TARE_SAMPLE_COUNT` | tare 평균 샘플 수(×0.02s=시간) | 영점 노이즈 크면 ↑ (드리프트는 안 줄어듦) |

---

## 실시간 디버깅 토픽

| 토픽/TF | 내용 | 도구 |
|---|---|---|
| `~/offset_debug_mm` (Vector3Stamped) | x=a, y=b, z=offset_norm [mm] | rqt_plot, PlotJuggler |
| `~/control_debug_deg` (Vector3Stamped) | 목표 tilt rx,ry,rz [deg, TCP] | rqt_plot, PlotJuggler |
| `~/com_point_marker` (Marker) | 무게 작용점 구 (녹=중심OK/적=벗어남) | rviz2 |
| TF `link_6` → `com_axes` | (u_hat,v_hat,g_hat) 축 | rviz2 TF > Axes |

(네임스페이스 `dsr01` 접두어가 붙는다: `/dsr01/offset_debug_mm` …)

phantom 노드 상태 로그는 매 0.5초:
```
[보상 ON] offset=  4.8mm (a=+2.7,b=-4.0) | raw=  4.9mm 팬텀제거= 0.1mm | tilt목표(u=+0.12,v=+0.08)deg
```
`raw`=팬텀 보상 전, `offset`=보상 후, `팬텀제거`=둘의 차(제거된 팬텀 크기).

---

## 안전·한계 (매뉴얼 V2.12 근거, 현장 투입 전 확인)

- 총 부하(그리퍼+판+물체) **6 kg 이내**. 초과 금지.
- 힘/컴플라이언스 제어는 TCP 환산 길이 **≤ 500mm**에서만(300mm 초과 시 진동 경고).
- 특이점 영역에서 힘 제어 비권장.
- **센서 SNR 한계**: 물체가 전체 하중(판+그리퍼)보다 훨씬 가벼우면(예: 골프공 0.45N vs
  전체 ~5N) 신호가 노이즈 바닥에 묻힌다. 무거운 물체일수록 유리. → [tray-balance 감지 한계 메모] 참고.
- `tray_balance_sim.py`/`viz.py`/`debug.py`는 구 "Z가 아래" 가정이라 실제 노드와 부호가
  다를 수 있다(참고용).
- 마찰/구름 모델은 소각 근사라 게인은 **실기 튜닝** 필요.
