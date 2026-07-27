#!/usr/bin/env bash
# PreToolUse(Bash) 훅. stdin으로 JSON을 받고, exit 2로 도구 실행을 차단한다.
INPUT=$(cat)
CMD=$(printf '%s' "$INPUT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('tool_input',{}).get('command',''))" 2>/dev/null)

block() { echo "BLOCKED by harness: $1" >&2; exit 2; }

# 1) 워크스페이스 파괴 방지
echo "$CMD" | grep -qE 'rm[[:space:]]+-[a-zA-Z]*r[a-zA-Z]*f?[[:space:]]+(/|~|\$HOME|.*src/?[[:space:]]*$)' \
  && block "src/ 또는 홈 디렉토리에 대한 재귀 삭제는 금지됩니다."

# 2) 알려진 의존성 충돌 유발 설치 차단
echo "$CMD" | grep -qE 'pip[0-9]*[[:space:]]+install.*(opencv-python|numpy[=><]*2|pydantic[[:space:]=><]*2)' \
  && block "CLAUDE.md 금지 규칙: opencv-python / numpy>=2 / pydantic v2 설치는 ROS 2 Humble을 깨뜨립니다. apt 패키지 또는 격리된 venv를 사용하세요."

# 3) 실기 모션 명령 차단 (사람이 직접 실행)
echo "$CMD" | grep -qiE 'ros2[[:space:]]+service[[:space:]]+call.*(move_j|move_l|movej|movel|servo|jog)' \
  && block "실기 모션 명령은 에이전트가 실행할 수 없습니다. /hw-gate 점검 후 사람이 실행하세요."

# 4) 빌드 산출물 커밋 차단
echo "$CMD" | grep -qE 'git[[:space:]]+add.*(build/|install/|log/)' \
  && block "build/ install/ log/ 는 커밋하지 않습니다."

exit 0
