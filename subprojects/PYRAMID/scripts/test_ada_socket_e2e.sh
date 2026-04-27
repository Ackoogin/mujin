#!/usr/bin/env bash
# test_ada_socket_e2e.sh — Cross-process E2E test driver.
#
# Orchestrates:
#   1. Start C++ server (tobj_socket_server)
#   2. Wait for server to be ready (port file appears)
#   3. Start Ada client (ada_tobj_client)
#   4. Wait for Ada client to exit
#   5. Terminate server
#   6. Report pass/fail
#
# Ada binaries must be pre-built via:  cmake --build --target pyramid_ada_all
#
# Usage: scripts/test_ada_socket_e2e.sh [--server-bin PATH] [--client-bin PATH]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

# Defaults
SERVER_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tests/tobj_socket_server"
CLIENT_BIN="${PYRAMID_ROOT}/examples/ada/bin/ada_tobj_client"
PORT=19234
PORT_FILE=$(mktemp /tmp/tobj_port.XXXXXX)
TIMEOUT=15

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --server-bin) SERVER_BIN="$2"; shift 2 ;;
    --client-bin) CLIENT_BIN="$2"; shift 2 ;;
    --port)       PORT="$2"; shift 2 ;;
    *)            shift ;;
  esac
done

cleanup() {
  if [[ -n "${SERVER_PID:-}" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -f "$PORT_FILE"
}
trap cleanup EXIT

echo "=== Ada Socket E2E Test ==="

if [[ ! -x "$CLIENT_BIN" ]]; then
  echo "[driver] SKIP: Ada client binary not found at $CLIENT_BIN"
  echo "[driver]   Run: cmake --build --target pyramid_ada_all"
  exit 0
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  echo "[driver] FAIL: Server binary not found at $SERVER_BIN"
  exit 1
fi

# Step 1: Start server
echo "[driver] Starting server on port $PORT..."
"$SERVER_BIN" --port "$PORT" --port-file "$PORT_FILE" --timeout "$TIMEOUT" &
SERVER_PID=$!

# Step 2: Wait for port file
echo "[driver] Waiting for server to write port file..."
for i in $(seq 1 50); do
  if [[ -s "$PORT_FILE" ]]; then
    break
  fi
  sleep 0.1
done

if [[ ! -s "$PORT_FILE" ]]; then
  echo "[driver] FAIL: Server did not write port file within 5 seconds"
  exit 1
fi

ACTUAL_PORT=$(cat "$PORT_FILE")
echo "[driver] Server ready on port $ACTUAL_PORT"

# Step 3: Brief delay so server enters accept()
sleep 0.2

# Step 4: Start Ada client
echo "[driver] Starting Ada client..."
"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_PORT"
CLIENT_EXIT=$?

# Step 5: Stop server
if kill -0 "$SERVER_PID" 2>/dev/null; then
  kill "$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
fi

# Step 6: Report
if [[ $CLIENT_EXIT -eq 0 ]]; then
  echo "[driver] PASS: Ada client received entity updates over socket transport"
  exit 0
else
  echo "[driver] FAIL: Ada client exited with code $CLIENT_EXIT"
  exit 1
fi
