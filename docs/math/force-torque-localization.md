---
tags: [math, 면접, 힘토크, 좌표변환]
축: 1 (내 코드에 있는 수학)
---

# 힘/토크 기반 위치 추정 — 내 코드에 실제로 있는 선형대수

> 이 문서는 `cobot1_ws` 저장소의 코드에서 **실제로 확인된 것만** 적는다.
> NotebookLM 소스로 올릴 때는 이 파일 그대로 올린다.

## 0. 한 장 요약 — 판 위 물체를 F/T 센서만으로 찾아 되돌리는 루프

```
get_tool_force(ref=DR_TOOL)  (raw force/torque, 6D)
        │  ① calibrate_gravity_frame — tare 시점 힘벡터 = 중력 방향
        ▼
정규직교 기저 (g_hat, u_hat, v_hat)   ← 특정 축이 "아래"라고 가정 안 함
        │  ② estimate_plane_offset — τ = r × F 를 이 기저로 역산
        ▼
판 평면 내 오프셋 (a, b)
        │  ③ PD 제어 (KP·offset) → u_hat/v_hat 축 회전량
        ▼
movel([0,0,0,rx,ry,rz], ref=DR_TOOL, mod=REL)  → 물체를 중앙으로 되돌림
```

`src/tray_balance_kkh/tray_balance_kkh/tray_balance.py`가 이 전체 루프의 원본이다.

**면접에서 이 그림 하나를 그릴 수 있으면 절반은 끝난다.**

---

## 1. τ = r × F 역산 (토크에서 위치를 구하기)

`tray_balance.py:272-286` (`estimate_plane_offset`)

물체가 판 위 오프셋 `r`(중심 기준)에 있고, 무게 `F = weight_n · g_hat`가 그 지점에 실리면 센서가 읽는 토크는

```
τ = r × F = weight_n · (r × g_hat)
```

`r`이 판 평면 내(=`g_hat`에 수직)에 있다는 구속조건과 삼중외적 항등식 `(r × g_hat) × g_hat = -r`(r⊥g_hat일 때)을 쓰면 역으로 풀린다:

```
r = (g_hat × τ) / weight_n
```

즉 **토크 벡터 하나에서 2D 평면 위치를 직접 역산**한다 — 반복 탐색이나 최적화가 없다. 코드는 이 `r`을 `u_hat`, `v_hat`에 내적해 스칼라 `(a, b)`로 뽑는다(`tray_balance.py:283-286`).

이 관계는 `_demo_calibrate_and_estimate_plane_offset`(`tray_balance.py:723-746`)에서 알려진 `r0`로 `τ`를 합성한 뒤 역산해 되돌아오는지(round-trip)로 자체 검증된다.

---

## 2. 중력 방향 실측 보정 — 축을 가정하지 않는 정규직교 기저

`tray_balance.py:252-269` (`calibrate_gravity_frame`)

이전 버전은 "툴 +Z가 아래"처럼 특정 축이 중력과 나란하다고 가정했지만, 실기에서 TCP 좌표계의 중력 방향이 어느 축과도 깨끗이 정렬되지 않는다는 게 드러났다(§0 주석, `tray_balance.py:6-13`). 그래서 **tare 측정값(정지 상태의 raw force) 자체를 중력 벡터로 직접 쓴다**:

```python
g_hat = normalize(tare_force_xyz)          # 중력 방향(단위벡터)
weight_n = |tare_force_xyz|                # 무게(스칼라)
```

여기서 `u_hat`, `v_hat`(판 평면 내 기저)을 만드는 방법이 **그람-슈미트 직교화**다:

```python
reference = (1,0,0) if |g_hat.x| < 0.9 else (0,1,0)   # g_hat과 거의 평행하지 않은 아무 벡터
u_hat = normalize(reference - (reference·g_hat)·g_hat)  # reference에서 g_hat 성분 제거
v_hat = g_hat × u_hat
```

`reference`를 축에 매번 고정하지 않고 `g_hat`과의 정렬도로 골라 쓰는 이유: `g_hat`이 `(1,0,0)`에 가까우면 그 reference를 빼는 순간 벡터가 거의 0이 되어(수치적으로 불안정) 정규화가 깨진다. `_demo_calibrate_and_estimate_plane_offset`이 `u_hat⊥v_hat⊥g_hat`, 우수좌표계(`u_hat×v_hat=g_hat`), 단위벡터 여부를 전부 assert로 확인한다(`tray_balance.py:729-737`).

---

## 3. 회전행렬 → 쿼터니언 변환 (Shepperd's method)

`tray_balance.py:218-249` (`_basis_to_quaternion`)

`(u_hat, v_hat, g_hat)`을 열로 하는 3×3 회전행렬을 rviz2 TF(쿼터니언)로 보내기 위해 trace 기반 변환을 쓴다. trace(`m00+m11+m22`)가 0 근처거나 음수면 `sqrt(1+trace)`가 0에 가까워 수치오차가 커지므로, **대각 성분 중 가장 큰 것을 기준으로 분기**한다(4가지 케이스). 이게 표준 라이브러리(`scipy.spatial.transform.Rotation`, `tf_transformations`) 없이 직접 짠 이유이자, 짤 때 분기 없이 하나의 공식만 쓰면 특정 회전 근처에서 깨지는 이유이기도 하다.

---

## 4. 접촉 기반 위치 추정 — 반지름 오프셋 (probe_grip_v4)

`src/cup_detect/cup_detect/probe_grip_v4.py:390-420` (`run_once` 내 `x_center`/`y_center` 계산)

F/T가 아니라 **접촉 이벤트 하나**(첫 충돌 지점)만으로 물체 중심을 구하는 다른 접근:

```
x_center = contact_x + object_radius_mm   # +X로 접근해 부딪힌 지점 + 가정 반지름
y_center = contact_y - object_radius_mm   # -Y로 접근해 부딪힌 지점 - 가정 반지름
```

v3(같은 축 양쪽에서 2회 접촉해 평균)에서 v4(축당 1회 + 반지름 가정)로 바꾼 트레이드오프가 파일 헤더에 그대로 남아있다(`probe_grip_v4.py:8-11`): **접촉 횟수는 줄지만, 실제 반지름이 `object_radius_mm`(하드코딩 55mm, `probe_grip_v4.py:96`)과 다르면 그 오차가 그대로 중심 추정 오차로 들어간다.** 접촉력의 부호(`contact_force_dir_x/y`)도 축마다 실기로 따로 검증해야 한다는 걸 코드가 `# UNVERIFIED` 주석으로 명시한다(`probe_grip_v4.py:24-25, 89`) — §1의 F/T 역산과 달리 이건 기하학적 가정(반지름) 하나로 물리를 대체한 것이라 정확도의 성격이 다르다.

---

## 5. 왜 측정 주기와 명령 주기를 분리했나 (제어이론 관점)

`tray_balance.py:96-105`

F/T 센서는 중력에 의한 토크뿐 아니라 서보 모션 자체의 관성·진동도 같이 잡는다. 루프가 `LOOP_HZ`(70Hz)로 계속 움직이며 명령을 쏘면, 그 진동이 다시 센서에 섞이고 게인이 그걸 증폭해 리밋사이클(진동이 안 죽고 유지되는 상태)이 생긴다. 해법은 **관측과 액추에이션의 주파수를 분리**하는 것:

- 오프셋 추정은 `LOOP_HZ`(70Hz)로 계속
- `movel` 명령은 `COMMAND_HZ`(30Hz, `tray_balance.py:102`)로만 — 이전 명령이 정착(settle)된 뒤의 샘플만 실제 이동에 반영

이게 "센서 대역폭을 낮춰서 노이즈를 줄인다"가 아니라 **"액추에이터가 만든 자기 자신의 노이즈를 안 보이게 시점을 늦춘다"**는 점이 면접에서 헷갈리기 쉬운 부분이다.

---

## 6. 면접 예상 질문 — 내 코드 기준 답

| 질문 | 답의 뼈대 |
|---|---|
| 토크 벡터 하나로 어떻게 2D 위치를 구하나요 | `r = (g_hat × τ) / weight_n` — 평면 구속(r⊥g_hat) + 삼중외적 항등식 |
| 왜 특정 축이 "아래"라고 가정하지 않나요 | 실측(tare)해보니 TCP 좌표계에서 중력이 축과 정렬 안 됨(~40°) — 하드코딩하면 그립/TCP가 바뀔 때마다 깨짐 |
| 정규직교 기저는 어떻게 만드나요 | 그람-슈미트: 임의 reference에서 `g_hat` 성분 제거 → `u_hat`, 외적으로 `v_hat` |
| reference 벡터를 왜 조건부로 고르나요 | `g_hat`과 거의 평행한 reference를 쓰면 직교화 후 벡터 크기가 0에 가까워져 수치적으로 불안정 |
| 회전행렬을 쿼터니언으로 바꿀 때 왜 분기가 필요한가요 | trace가 작거나 음수면 `sqrt(1+trace)` 근처에서 수치오차 커짐 → 대각 성분 최대값 기준 분기(Shepperd's method) |
| 접촉 탐지로 중심을 구할 때의 오차 요인은 | 반지름을 실측이 아니라 가정값으로 쓰므로, 실제 물체 반지름과의 차이가 그대로 중심 오차 |
| 왜 센서 측정 주기와 명령 발사 주기를 다르게 뒀나요 | 서보 자체 진동이 F/T에 섞여 게인이 증폭 → 리밋사이클. 명령을 저주파로 늦춰 진동이 가라앉은 샘플만 반영 |
| PD에서 D게인을 왜 0으로 뒀나요(`KD=0.0`) | 실기에서 P게인만으로 시작 — 진동이 확인되면 KP와 같이 올리는 걸 전제(`tray_balance.py:86-87` 주석) |
