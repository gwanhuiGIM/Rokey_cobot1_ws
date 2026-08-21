# 그리퍼 파지(grip) 판단 로직 추가 — 작업 인수 문서

대상 코드: `src/coffee_system/coffee_system/m0609_coffee_system_v2.py`
대상 하드웨어: M0609 + OnRobot RG2
작성일: 2026-07-25

이 문서는 **"그리퍼가 실제로 물체를 잡았는지"를 판단하는 로직**을 커피 시스템에
넣기 위해 필요한 수정 지점, 이미 끝난 선행 작업, 남은 TODO를 정리한 것이다.
실기 로그로 검증한 내용을 근거로 한다.

---

## 0. 배경 — RG2가 주는 신호와 한계

RG2 상태는 `onrobot_rg_control` 노드(Modbus TCP)가 읽어서 발행한다.

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/onrobot_joint_states` | `sensor_msgs/JointState` | `position`(관절각, rad, 실측), `effort`(명령 힘값 — 실측 아님) |
| `/onrobot/grip_detected` | `std_msgs/Bool` | RG2 하드웨어 grip 비트 (선행 작업에서 추가) |

**중요한 한계**: RG2는 **실시간 측정 힘(force)을 주지 않는다.** `effort`는 우리가
명령한 힘 설정값이며 `gSTA 상태워드가 0이 아닐 때만 그 값, 아니면 0`이다.
→ "얼마나 세게 쥐었나"를 힘으로 직접 읽을 수 없다. 변형/파손 감지는 **width(=position)
차분**으로 계산해야 한다(6장).

---

## 1. ⚠️ 통합 시 가장 먼저 알아야 할 것 — 명령 경로와 모니터링 경로가 다르다

`m0609_coffee_system_v2.py`는 그리퍼를 **제어박스 DO 1/2 접점 조합**으로 여닫는다
([line 578~592](coffee_system/coffee_system/m0609_coffee_system_v2.py#L578)):

```python
def grip_close():        set_digital_output(1, ON);  set_digital_output(2, OFF)
def jar_grip_open():     set_digital_output(1, OFF); set_digital_output(2, ON)
def handle_grip_open():  set_digital_output(1, ON);  set_digital_output(2, ON)
def spoon_cup_grip_open():set_digital_output(1, OFF);set_digital_output(2, OFF)
```

- **명령**은 DO(접점)로 나간다 → RG2의 폭·힘은 **컴퓨트박스/웹UI에 사전 설정된 값**을
  쓴다. 코드에서 힘을 바꾸지 않는다.
- **모니터링**(위 토픽)은 별도로 `onrobot_rg_control` 노드가 Modbus로 읽는다.
- 따라서 **파지 판단을 하려면 `onrobot_rg_control` 노드(= bringup)가 반드시 함께 떠
  있어야** 한다. DO만으로는 상태 피드백이 없다.

정리: **DO로 닫고 → 토픽으로 확인**하는 구조. 둘은 독립이며 공존 가능하다.

---

## 2. 이미 끝난 선행 작업 (재빌드만 하면 됨)

협업자가 다시 할 필요는 없고, **환경에 반영됐는지 확인만** 하면 된다.

### 2-1. pymodbus 버전 핀
드라이버가 `slave=` 인자를 쓰므로 pymodbus 3.6.9 필요(최신 3.14는 `device_id=`로 바뀜).
```bash
pip3 install 'pymodbus==3.6.9'
```
> TODO(권장): `onrobot_rg_control/package.xml` 또는 requirements에 버전 핀 고정.

### 2-2. 하드웨어 grip 비트 노출 (`/onrobot/grip_detected`)
`onrobot_rg_control/.../OnRobotRGControllerServer.py`에 발행부를 추가했다.
**핵심 함정**: RG2의 grip 감지는 별도 레지스터가 아니라 **gSTA 상태워드(register 268)의
bit1**이다. 드라이버 `getStatus()`의 `'grip'` 라벨(register 269)은 grip이 아니라 항상
non-zero라 `bool()`이 늘 `true`가 된다. 그래서 gSTA에서 bit를 직접 뽑는다:

```python
# getStatus() 내 발행부
gsta = int(self.status['busy'])   # status['busy']엔 gSTA 전체워드가 담겨있음
grip = bool((gsta >> 1) & 1)      # bit0=busy, bit1=grip, bit2~=safety
self.grip_pub.publish(Bool(data=grip))
```
반영:
```bash
colcon build --packages-select onrobot_rg_control && source install/setup.bash
ros2 topic echo /onrobot/grip_detected   # 물체 유무에 따라 true/false 갈리는지 확인
```

---

## 3. 실기 검증값 (force=40N, 최대치로 닫기)

| 상황 | 정착 position(rad) | 정착 시 effort |
|---|---|---|
| 빈손 닫힘 | **0.7587** (끝까지 닫힘) | **0.0** (gSTA=0, 완주) |
| 얇은 강체 물체 | **0.7141** (덜 닫힘, stall) | **40.0** (gSTA에 grip비트 → 유지) |

- position은 **클수록 닫히는 방향**(widthToJointValue의 arccos가 감소함수).
- 빈손·물체 **position 갭 ≈ 0.045 rad** — 얇은 물체도 구분 가능.
- effort 0/비0 이 더 깨끗한 이진 신호(빈손 완주=0, 파지=비0).

---

## 4. 추가할 파지 판단 로직 (핵심)

`grip_close()` **호출 직후** 파지 성공을 확인하는 단계를 넣는다. 판정 우선순위:

1. **1순위 — 하드웨어 grip 비트**: `/onrobot/grip_detected == true`
   (RG2 자체 판정, 토크 기반. 가장 신뢰도 높음)
2. **2순위(보조) — effort/position**: 닫기 후 position이 plateau가 됐을 때
   `effort != 0` (gSTA 비0 = stall/grip) 이면 파지, position이 빈손값(≈0.7587)까지
   가면 놓침.

### 넣을 위치
`grip_close()`가 불리는 각 지점 뒤 (예: [line 747](coffee_system/coffee_system/m0609_coffee_system_v2.py#L747),
[803](coffee_system/coffee_system/m0609_coffee_system_v2.py#L803),
[841](coffee_system/coffee_system/m0609_coffee_system_v2.py#L841)).
매번 쓰기 좋게 `grip_close()`에 확인을 합치거나, `verify_grip()` 헬퍼로 분리.

### 참고 구현 (헬퍼, position 토픽 기반 — 하드웨어 비트 우선 사용 권장)
```python
from std_msgs.msg import Bool

# TODO(확정 필요): 실측 교정. 빈손 닫기 1회 → POS_CLOSED_EMPTY.
POS_CLOSED_EMPTY = 0.7587   # rad, 위 3장 값(실환경에서 재측정 권장)
POS_MARGIN       = 0.02     # rad

def verify_grip(node, timeout_sec=2.0):
    """grip_close() 후 호출. 하드웨어 비트 우선, 없으면 effort/position."""
    latest = {"grip": None, "eff": None, "pos": None}

    def on_bit(m):  latest["grip"] = m.data
    def on_js(m):
        if m.position: latest["pos"] = m.position[0]
        if m.effort:   latest["eff"] = m.effort[0]

    s1 = node.create_subscription(Bool, '/onrobot/grip_detected', on_bit, 3)
    s2 = node.create_subscription(JointState, '/onrobot_joint_states', on_js, 3)
    deadline = time.monotonic() + timeout_sec
    time.sleep(0.5)                              # 모션 정착 대기
    while latest["grip"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(s1); node.destroy_subscription(s2)

    if latest["grip"] is not None:               # 1순위: 하드웨어 비트
        return latest["grip"]
    if latest["eff"] is not None:                # 2순위: effort(gSTA 비0)
        return latest["eff"] != 0.0
    if latest["pos"] is not None:                # 3순위: position 갭
        return latest["pos"] < (POS_CLOSED_EMPTY - POS_MARGIN)
    node.get_logger().warn("그리퍼 상태 수신 실패 → 판정 불가")
    return False
```

### 실패 시 처리 (설계 결정 필요)
파지 실패로 판정되면 무엇을 할지 결정해야 한다: 재시도 / 정지 / 사용자 알림.
현재 커피 시퀀스는 실패해도 다음 모션으로 진행 → **놓친 채 이동하면 낙하 위험**.
→ TODO(협의): 실패 시 `move_stop` 후 시퀀스 중단 권장.

---

## 5. ROS2 콜백/블로킹 주의 (프로젝트 규칙)

- `verify_grip()`의 `spin_once` 폴링은 짧게(≤2s) 유지. 긴 블로킹은 규칙 위반.
- 이 노드가 이미 별도 실행 스레드/콜백 구조를 쓰면, 파지 상태를 **상시 구독**해
  최신값만 읽는 방식(멤버 변수 캐싱)이 더 깔끔하다. 통합 방식은 기존 노드 구조에 맞춰 결정.

---

## 6. 파손 민감 모니터링 (종이컵 등) — 별도 지표

파지 "유무"와 별개로 **과압착/파손**을 잡으려면:

- **1차 예방은 힘 설정**: 현재 DO 모드라 힘은 컴퓨트박스/웹UI 사전값. 종이컵 등
  약한 물체엔 40N(최대)은 과함 → **RG2 최소치(약 3N) 부근으로 낮춘 프리셋** 사용.
- **모니터링 지표(계산)**: 접촉 후 `dposition/dt`가 계속 닫히는 방향이면 = 재료가
  눌리는 중 = 크러시. position이 딱 멈추면 강체(안전). position은 이미 50Hz로 나오므로
  추가 배관 없이 차분으로 계산 가능.
- 압축량 δ = (물체 자유폭 → position) − (정착 position). 물체 자유폭을 알면 정적 지표.

> 이 크립 감시는 파지 판단(4장)과 별개 기능. 필요 시 별도 모니터 노드로 분리 권장.

---

## 7. 남은 TODO 체크리스트

- [ ] `onrobot_rg_control` 재빌드 + `/onrobot/grip_detected` 동작 확인 (2장)
- [ ] `POS_CLOSED_EMPTY` 실환경 재측정 (빈손 닫기 후 `ros2 topic echo /onrobot_joint_states`)
- [ ] `verify_grip()` 를 `grip_close()` 지점들에 통합 (4장)
- [ ] 파지 실패 시 동작 정의 (재시도/중단/알림) — 협의 필요
- [ ] (선택) 종이컵용 저(低)힘 프리셋 + 크립 감시 (6장)
- [ ] (권장) pymodbus 3.6.9 버전 핀을 패키지 메타에 고정


    def grip_close() -> None: # width= 5mm, Force = 40N# 
        set_digital_output(1, ON)
        set_digital_output(2, OFF)

    def jar_grip_open() -> None: # width= 70mm, Force = 40N
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def handle_grip_open() -> None: # width= 104(max)mm, Force = 40N
        set_digital_output(1, ON)
        set_digital_output(2, ON)

    def spoon_cup_grip_open() -> None: # width= 35mm, Force = 40N
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
