# coffee_system

M0609 + OnRobot RG2 협동로봇 자동 커피 추출 ROS2 패키지.
원본 Task Writer(DRL) 프로그램(`m0609_coffe_system.drl`, `System.drvar`)을
[m0609_coffee.py](m0609_coffee.py) 단일 파일 노드로 변환한 것입니다.

## 동작 순서 (1회 실행, 반복 없음)

1. **bean_drop** — 스푼을 잡고 원두를 퍼서 그라인더 호퍼에 투입
2. **grinder** — 그라인더 손잡이를 잡고 컴플라이언스 제어로 원호를 그리며 분쇄
3. **dripper_in** — 드립 병을 잡고 주기 운동(`move_periodic`)으로 커피 추출

## 실행

`setup.py`에 entry_point가 등록되어 있지 않아 `ros2 run`으로는 실행되지 않습니다.

```bash
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609
python3 src/coffee_system/m0609_coffee.py
```

## 좌표계 / Tool·TCP

- 좌표계: `DR_BASE` (그라인더 컴플라이언스 원호 구간만 teach pendant 사용자 좌표계 `101`)
- Tool: `Tool Weight_gripper` / TCP: `GripperDA_v1` (시작 시 `set_tool`/`set_tcp`로 강제 설정 후 `get_tool`/`get_tcp`로 반영 확인, 불일치 시 예외 발생)
- 드립 추출 구간은 병 축 정렬을 위해 TCP를 일시적으로 `joint4`로 전환했다가 완료 후 `GripperDA_v1`로 복귀

## 주의

- 노드는 시작과 동시에 실제 로봇 모션을 수행합니다. 실행 전 그라인더·드리퍼·병·컵·스푼 위치가 교시 당시(`System.drvar`)와 동일한지 반드시 확인하세요.
- OnRobot RG2 그리퍼는 컨트롤러 Digital Output 1, 2번 접점 조합으로 제어됩니다 (그리퍼 컨트롤 박스가 해당 접점에 매핑되어 있어야 함).
- 그라인더 서브루틴은 `task_compliance_ctrl()` 진입 후 `movec()`으로 원호 모션을 수행합니다. 컴플라이언스 구간 진입 전 주변 장애물 여부를 확인하세요.

## DRL 원본 대비 수정 사항

Python API(`DSR_ROBOT2`) 변환 과정에서 원본 DRL에는 있었으나 실제 시그니처에 없어 제거/치환한 항목 (코드 내 인라인 주석 참고):

- `set_singular_handling(DR_AVOID)` → `set_singularity_handling(DR_AVOID)`로 치환
- `movel()`/`amovel()`/`movec()`의 `app_type`, `ori` 키워드 인자 제거 (실제 시그니처에 없음)
- `set_velx()` 3번째 인자 `DR_OFF` 제거 (`DR_OFF` 상수 자체가 `DSR_ROBOT2`에 없음)

## 비활성 기능

`coffee_status_ui.py`가 구독하는 진행 상태 발행(`StatusReporter`, `coffee_system/status` 토픽)은 코드 내 전부 주석 처리되어 비활성 상태입니다.
