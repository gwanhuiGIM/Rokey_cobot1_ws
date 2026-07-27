---
description: 개발 — 4단계 체계적 디버깅. 이해하기 전에 고치는 것을 금지한다
argument-hint: [증상 또는 에러 메시지]
allowed-tools: Read, Grep, Glob, Bash
disable-model-invocation: true
---

# 체계적 디버깅: $ARGUMENTS

**금지: 원인을 특정하기 전에 코드를 수정하는 것.** "일단 이렇게 해보죠"는 금지어다.

## Phase 1 — 재현 (수정 금지)
1. 최소 재현 절차를 명령어 단위로 확정하라.
2. 결정론적인가, 산발적인가? 산발적이면 몇 번 중 몇 번인가?
3. 언제부터 발생했는가? `git log`로 마지막 정상 커밋을 찾아라.

## Phase 2 — 관측 (수정 금지)
ROS 2 진단을 **실제로 실행하고** 결과를 붙여라. 추측 금지.
```bash
ros2 node list && ros2 node info <node>
ros2 topic list && ros2 topic info <topic> --verbose   # QoS 확인
ros2 topic hz <topic> ; ros2 topic echo <topic> --once
ros2 param list <node>
ros2 service list -t
ros2 doctor --report
env | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY|ROS_DISCOVERY_SERVER'
ros2 run tf2_tools view_frames    # TF 문제일 때
```
Python 의존성 의심 시:
```bash
python3 -c "import numpy, cv2, sys; print(numpy.__version__, cv2.__file__, sys.executable)"
pip list 2>/dev/null | grep -E 'numpy|opencv|pydantic'
```

## Phase 3 — 가설과 배제
최소 3개 가설을 세우고 표로 정리한 뒤, **각각을 배제하는 실험**을 설계하라.

| 가설 | 근거 | 이 가설이 참이면 관측될 것 | 배제 실험 | 결과 |
|---|---|---|---|---|

ROS 2에서 흔한 원인 후보(먼저 의심할 것):
- DDS: `ROS_DOMAIN_ID` 불일치 / RMW 구현 불일치 / Discovery Server vs Simple Discovery / 멀티캐스트 차단
- QoS: reliability·durability·depth 불일치로 매칭 실패
- 라이프사이클: 서비스 서버 노드가 안 떠 있음 / 콜백 그룹 데드락 (단일 스레드 executor에서 동기 호출)
- 환경: 인터프리터 불일치, `LD_LIBRARY_PATH` 우선순위, 미소스된 오버레이
- 하드웨어: USB 권한/udev 규칙, 전원, 케이블

## Phase 4 — 수정과 재발 방지
1. **최소 수정**만 한다. 관련 없는 리팩터링 금지.
2. 이 버그를 잡아내는 **회귀 테스트**를 추가한다.
3. `./scripts/verify.sh` 통과 확인.
4. 마지막에 `/lesson`을 실행할 수 있도록 아래를 출력하라:
   - 증상 / 근본 원인 / 수정 / **CLAUDE.md에 추가할 규칙 1줄**
