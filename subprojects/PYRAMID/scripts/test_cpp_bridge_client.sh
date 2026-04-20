#!/usr/bin/env bash
# test_cpp_bridge_client.sh -- C++ client ↔ standalone bridge integration test.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

SERVER_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tests/tobj_socket_server"
BRIDGE_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tests/standalone_bridge"
CLIENT_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tactical_objects/tactical_objects_test_client"
BACKEND_PORT=19285
FRONTEND_PORT=19286
CONTENT_TYPE="application/json"
TIMEOUT=20
PORT_FILE=$(mktemp /tmp/tobj_cpp_bridge_server.XXXXXX)
BRIDGE_PORT_FILE=$(mktemp /tmp/tobj_cpp_bridge_frontend.XXXXXX)

while [[ $# -gt 0 ]]; do
  case "$1" in
    --server-bin) SERVER_BIN="$2"; shift 2 ;;
    --bridge-bin) BRIDGE_BIN="$2"; shift 2 ;;
    --client-bin) CLIENT_BIN="$2"; shift 2 ;;
    --backend-port) BACKEND_PORT="$2"; shift 2 ;;
    --frontend-port) FRONTEND_PORT="$2"; shift 2 ;;
    --content-type) CONTENT_TYPE="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    *) shift ;;
  esac
done

cleanup() {
  if [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
  fi
  if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -f "$PORT_FILE" "$BRIDGE_PORT_FILE"
}
trap cleanup EXIT

echo "=== C++ Bridge Client Test ($CONTENT_TYPE) ==="

"$SERVER_BIN" --port "$BACKEND_PORT" --port-file "$PORT_FILE" \
              --timeout "$TIMEOUT" --no-entity --no-bridge &
SERVER_PID=$!

for i in $(seq 1 50); do
  [[ -s "$PORT_FILE" ]] && break
  sleep 0.1
done

if [[ ! -s "$PORT_FILE" ]]; then
  echo "[driver] FAIL: backend server did not start" >&2
  exit 1
fi

ACTUAL_BACKEND_PORT=$(cat "$PORT_FILE")

"$BRIDGE_BIN" --backend-host 127.0.0.1 --backend-port "$ACTUAL_BACKEND_PORT" \
              --frontend-port "$FRONTEND_PORT" --port-file "$BRIDGE_PORT_FILE" \
              --frontend-content-type "$CONTENT_TYPE" --timeout "$TIMEOUT" &
BRIDGE_PID=$!

for i in $(seq 1 50); do
  [[ -s "$BRIDGE_PORT_FILE" ]] && break
  sleep 0.1
done

if [[ ! -s "$BRIDGE_PORT_FILE" ]]; then
  echo "[driver] FAIL: frontend bridge did not start" >&2
  exit 1
fi

ACTUAL_FRONTEND_PORT=$(cat "$BRIDGE_PORT_FILE")

"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_FRONTEND_PORT" \
              --content-type "$CONTENT_TYPE"
