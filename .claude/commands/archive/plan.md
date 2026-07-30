---
description: 기획 → 실행 계획. 파일 단위 태스크로 쪼개고 승인 전까지 코드를 쓰지 않는다
argument-hint: [사양 문서 경로 또는 기능명]
allowed-tools: Read, Grep, Glob, Bash(git status:*), Bash(git log:*), Bash(ros2 interface show:*)
disable-model-invocation: true
---

# 실행 계획 수립: $ARGUMENTS

## 컨텍스트
- 현재 브랜치 상태: !`git status --short`
- 최근 커밋: !`git log --oneline -10`

## 지시
사양 문서($ARGUMENTS)를 읽고 실행 계획을 만든다. **파일은 아직 수정하지 않는다.**

### 1) 사전 조사 (반드시 먼저)
- 관련 기존 코드를 Grep/Read로 실제로 읽어라. 파일명만 보고 추측하지 마라.
- 사용할 메시지 타입은 `ros2 interface show`로 실제 필드를 확인하라.
- 외부 API(Doosan, Isaac Sim, OnRobot 등)는 **버전 명시된 매뉴얼 근거**가 없으면 `UNVERIFIED`로 표시하고 나에게 확인을 요청하라.

### 2) 계획 표
| # | 태스크 | 수정/생성 파일 | 변경 요지 | 검증 방법 | 예상 시간 |
|---|---|---|---|---|---|

규칙:
- 태스크 1개는 **5분 이내** 분량으로 쪼갠다. 넘으면 더 쪼갠다.
- 각 태스크는 **독립적으로 검증 가능**해야 한다.
- 순서는 "인터페이스 정의 → 테스트 작성 → 구현 → 통합" 순을 지킨다.

### 3) 리스크 표
| 리스크 | 발생 조건 | 탐지 방법 | 완화책 |
|---|---|---|---|

특히 다음을 반드시 검토하라: DDS 디스커버리/도메인 불일치, QoS 불일치, 파이썬 의존성 충돌(numpy/pydantic/opencv), 실기 안전, 좌표계·TF 프레임 불일치, 단위(m/mm, rad/deg) 혼동.

### 4) 롤백 계획
어느 커밋으로 되돌리면 되는지, 하드웨어 상태는 어떻게 복구하는지.

---
계획을 출력한 뒤 **멈춘다**. 내가 `GO`라고 답하기 전에는 Edit/Write를 사용하지 마라.
계획은 `docs/plans/`에 저장한다.
