#!/usr/bin/env bash
# ROS 2 하네스 검증 게이트. 에이전트는 이 스크립트 통과 전까지 "완료"라고 말할 수 없다.
set -uo pipefail

PKG="${1:-}"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT" || exit 1
FAIL=0
step() { echo -e "\n===== [$1] ====="; }
ok()   { echo "  PASS: $1"; }
ng()   { echo "  FAIL: $1"; FAIL=1; }

step "0. 환경 확인"
[ -n "${ROS_DISTRO:-}" ] && ok "ROS_DISTRO=$ROS_DISTRO" || ng "ROS 미소스 (source /opt/ros/humble/setup.bash)"
python3 -c "import numpy,sys; sys.exit(0 if numpy.__version__ < '2' else 1)" 2>/dev/null \
  && ok "numpy<2.0" || ng "numpy>=2.0 감지 — cv_bridge 비호환"
python3 -c "import cv2, os, sys; sys.exit(0 if 'dist-packages' in os.path.dirname(cv2.__file__) else 1)" 2>/dev/null \
  && ok "opencv=apt(python3-opencv)" || echo "  WARN: cv2가 pip 설치본일 수 있음 (Qt 충돌 위험)"

step "1. 시크릿 스캔"
if git rev-parse --git-dir >/dev/null 2>&1; then
  if git grep -nIE '(api[_-]?key|secret|token|passwd|password)[[:space:]]*=[[:space:]]*["'"'"'][A-Za-z0-9_\-]{16,}' -- 'src/*' >/dev/null 2>&1; then
    ng "하드코딩된 자격증명 의심"; git grep -nIE '(api[_-]?key|secret|token)[[:space:]]*=' -- 'src/*' | head -5
  else ok "하드코딩된 자격증명 없음"; fi
fi

step "2. 빌드"
if [ -n "$PKG" ]; then
  colcon build --symlink-install --packages-select "$PKG" >/tmp/hb_build.log 2>&1
else
  colcon build --symlink-install >/tmp/hb_build.log 2>&1
fi
[ $? -eq 0 ] && ok "colcon build" || { ng "colcon build"; tail -30 /tmp/hb_build.log; }

# shellcheck disable=SC1091
# ROS/colcon setup 파일은 미설정 변수를 참조하므로 set -u를 잠시 끈다
[ -f install/setup.bash ] && { set +u; source install/setup.bash; set -u; }

step "3. 테스트"
# -p no:anyio: ~/.local의 pip anyio 플러그인이 apt pytest(6.x)와 비호환 → 전 패키지 pytest 붕괴
# -k: ament 템플릿 스타일 테스트는 게이트에서 제외하고 step 5에서 WARN으로만 본다
PYTEST_ARGS=(-p no:anyio -k "not flake8 and not pep257 and not copyright")
if [ -n "$PKG" ]; then colcon test --packages-select "$PKG" --pytest-args "${PYTEST_ARGS[@]}" >/tmp/hb_test.log 2>&1
else colcon test --pytest-args "${PYTEST_ARGS[@]}" >/tmp/hb_test.log 2>&1; fi
colcon test-result --verbose >/tmp/hb_result.log 2>&1 \
  && ok "colcon test" || { ng "colcon test"; tail -30 /tmp/hb_result.log; }

step "4. 런치 스모크 테스트 (5초 spin 후 그래프 확인)"
if [ -n "${SMOKE_LAUNCH:-}" ]; then
  timeout 12s ros2 launch $SMOKE_LAUNCH >/tmp/hb_launch.log 2>&1 &
  LPID=$!; sleep 5
  NODES=$(ros2 node list 2>/dev/null | wc -l)
  TOPICS=$(ros2 topic list 2>/dev/null | wc -l)
  kill $LPID 2>/dev/null; wait $LPID 2>/dev/null
  [ "$NODES" -gt 0 ] && ok "노드 $NODES개 / 토픽 $TOPICS개 기동" || { ng "노드 미기동"; tail -30 /tmp/hb_launch.log; }
else
  echo "  SKIP: SMOKE_LAUNCH 환경변수 미설정 (예: export SMOKE_LAUNCH='move bringup.launch.py')"
fi

step "5. 정적 검사 (WARN 전용 — 게이트 아님)"
LINT_TARGET="src${PKG:+/$PKG}"
if command -v ruff >/dev/null; then
  ruff check "$LINT_TARGET" >/tmp/hb_lint.log 2>&1 && ok "ruff" || { echo "  WARN: ruff $(wc -l </tmp/hb_lint.log)줄"; head -10 /tmp/hb_lint.log; }
elif python3 -m flake8 --version >/dev/null 2>&1; then
  python3 -m flake8 --max-line-length=99 "$LINT_TARGET" >/tmp/hb_lint.log 2>&1 && ok "flake8" || { echo "  WARN: flake8 위반 $(wc -l </tmp/hb_lint.log)건"; head -10 /tmp/hb_lint.log; }
else echo "  SKIP: ruff/flake8 미설치"; fi

echo -e "\n=============================="
[ $FAIL -eq 0 ] && echo "VERIFY: PASS" || echo "VERIFY: FAIL"
exit $FAIL
