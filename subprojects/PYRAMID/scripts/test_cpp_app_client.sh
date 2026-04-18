#!/usr/bin/env bash
# test_cpp_app_client.sh -- C++ client ↔ tactical_objects_app integration test.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

APP_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tactical_objects/tactical_objects_app"
CLIENT_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tactical_objects/tactical_objects_test_client"
PORT=19295
CONTENT_TYPE="application/json"
TIMEOUT=20
PORT_FILE=$(mktemp /tmp/tobj_cpp_app_frontend.XXXXXX)

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

echo "=== C++ App Client Test ($CONTENT_TYPE) ==="

"$APP_BIN" --port "$PORT" --port-file "$PORT_FILE" \
           --timeout "$TIMEOUT" --content-type "$CONTENT_TYPE" &
APP_PID=$!

for i in $(seq 1 50); do
  [[ -s "$PORT_FILE" ]] && break
  sleep 0.1
done

if [[ ! -s "$PORT_FILE" ]]; then
  echo "[driver] FAIL: tactical_objects_app did not start" >&2
  exit 1
fi

ACTUAL_PORT=$(cat "$PORT_FILE")

"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_PORT" \
              --content-type "$CONTENT_TYPE"
