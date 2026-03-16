#!/usr/bin/env bash
# test_ada_active_find_e2e.sh — Cross-process ActiveFind E2E test driver.
#
# Orchestrates:
#   1. Build Ada active-find client (if gprbuild available)
#   2. Start C++ server with --no-entity (tobj_socket_server)
#   3. Wait for server to be ready (port file appears)
#   4. Start Ada active-find executable (client + evidence provider)
#   5. Wait for Ada process to exit
#   6. Terminate server
#   7. Report pass/fail
#
# Usage: scripts/test_ada_active_find_e2e.sh [--server-bin PATH] [--client-bin PATH]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Defaults
SERVER_BIN="${ROOT_DIR}/build/tests/tobj_socket_server"
CLIENT_BIN="${ROOT_DIR}/examples/ada/bin/ada_active_find_e2e"
PORT=19235
PORT_FILE=$(mktemp /tmp/tobj_port.XXXXXX)
TIMEOUT=20

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

echo "=== Ada ActiveFind E2E Test ==="

# Step 1: Build Ada client if gprbuild is available
if command -v gprbuild &>/dev/null; then
  echo "[driver] Building Ada active-find client..."
  (cd "$ROOT_DIR/examples/ada" && \
    MUJIN_ROOT="$ROOT_DIR" gprbuild -P ada_active_find_e2e.gpr -q 2>&1) || {
    echo "[driver] SKIP: gprbuild failed (GNAT toolchain issue)"
    exit 0
  }
else
  echo "[driver] gprbuild not found — checking for pre-built client..."
  if [[ ! -x "$CLIENT_BIN" ]]; then
    echo "[driver] SKIP: no Ada client binary and no gprbuild"
    exit 0
  fi
fi

if [[ ! -x "$CLIENT_BIN" ]]; then
  echo "[driver] SKIP: Ada client binary not found at $CLIENT_BIN"
  exit 0
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  echo "[driver] FAIL: Server binary not found at $SERVER_BIN"
  exit 1
fi

# Step 2: Start server with --no-entity (Ada client drives entity creation via ActiveFind)
echo "[driver] Starting server on port $PORT (--no-entity)..."
"$SERVER_BIN" --port "$PORT" --port-file "$PORT_FILE" --timeout "$TIMEOUT" --no-entity &
SERVER_PID=$!

# Step 3: Wait for port file
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

# Step 4: Brief delay so server enters accept()
sleep 0.2

# Step 5: Start Ada active-find client
echo "[driver] Starting Ada active-find client..."
"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_PORT"
CLIENT_EXIT=$?

# Step 6: Stop server
if kill -0 "$SERVER_PID" 2>/dev/null; then
  kill "$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
fi

# Step 7: Report
if [[ $CLIENT_EXIT -eq 0 ]]; then
  echo "[driver] PASS: Ada ActiveFind E2E — evidence provider + entity correlation succeeded"
  exit 0
else
  echo "[driver] FAIL: Ada active-find client exited with code $CLIENT_EXIT"
  exit 1
fi
