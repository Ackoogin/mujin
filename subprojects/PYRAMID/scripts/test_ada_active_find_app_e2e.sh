#!/usr/bin/env bash
# test_ada_active_find_app_e2e.sh -- Ada ActiveFind against tactical_objects_app.
#
# Ada binaries must be pre-built via:  cmake --build --target pyramid_ada_all
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

APP_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tactical_objects/tactical_objects_app"
CLIENT_BIN="${PYRAMID_ROOT}/tests/ada/bin/ada_active_find_e2e"
PORT=19305
CONTENT_TYPE="application/json"
TIMEOUT=25
PORT_FILE=$(mktemp /tmp/tobj_app_af.XXXXXX)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --app-bin) APP_BIN="$2"; shift 2 ;;
    --client-bin) CLIENT_BIN="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --content-type) CONTENT_TYPE="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    *) shift ;;
  esac
done

cleanup() {
  if [[ -n "${APP_PID:-}" ]] && kill -0 "$APP_PID" 2>/dev/null; then
    kill "$APP_PID" 2>/dev/null || true
    wait "$APP_PID" 2>/dev/null || true
  fi
  rm -f "$PORT_FILE"
}
trap cleanup EXIT

echo "=== Ada ActiveFind Real-App E2E ($CONTENT_TYPE) ==="

if [[ ! -x "$CLIENT_BIN" ]]; then
  echo "[driver] SKIP: Ada client binary not found at $CLIENT_BIN"
  echo "[driver]   Run: cmake --build --target pyramid_ada_all"
  exit 0
fi
if [[ ! -x "$APP_BIN" ]]; then
  echo "[driver] FAIL: tactical_objects_app not found at $APP_BIN"
  exit 1
fi

echo "[driver] Starting tactical_objects_app on port $PORT..."
"$APP_BIN" --port "$PORT" --port-file "$PORT_FILE" \
           --timeout "$TIMEOUT" --content-type "$CONTENT_TYPE" &
APP_PID=$!

for _ in $(seq 1 50); do
  [[ -s "$PORT_FILE" ]] && break
  sleep 0.1
done

if [[ ! -s "$PORT_FILE" ]]; then
  echo "[driver] FAIL: tactical_objects_app did not start" >&2
  exit 1
fi

ACTUAL_PORT=$(cat "$PORT_FILE")
echo "[driver] App ready on port $ACTUAL_PORT"
echo "[driver] Starting Ada active-find client..."
"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_PORT" \
              --content-type "$CONTENT_TYPE"
CLIENT_EXIT=$?

if [[ $CLIENT_EXIT -eq 0 ]]; then
  echo "[driver] PASS: Ada ActiveFind real-app E2E succeeded"
  exit 0
else
  echo "[driver] FAIL: Ada active-find client exited with code $CLIENT_EXIT"
  exit 1
fi
