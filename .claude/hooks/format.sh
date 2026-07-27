#!/usr/bin/env bash
# PostToolUse(Edit|Write) 훅. 수정된 파이썬 파일을 자동 정리한다. 실패해도 작업은 막지 않는다.
INPUT=$(cat)
FILE=$(printf '%s' "$INPUT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('tool_input',{}).get('file_path',''))" 2>/dev/null)
case "$FILE" in
  *.py)
    command -v ruff >/dev/null && ruff format "$FILE" >/dev/null 2>&1
    command -v ruff >/dev/null && ruff check --fix "$FILE" >/dev/null 2>&1
    ;;
esac
exit 0
