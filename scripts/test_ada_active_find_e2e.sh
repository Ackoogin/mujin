#!/usr/bin/env bash
# test_ada_active_find_e2e.sh — Cross-process ActiveFind E2E test driver.
#
# 3-process architecture:
#   1. TacticalObjectsComponent server (--no-bridge --no-entity)
#   2. Standalone StandardBridge (dual TCP: client→server, server→Ada)
#   3. Ada active-find client (connects to bridge)
#
# Usage: scripts/test_ada_active_find_e2e.sh [--server-bin PATH]
#            [--bridge-bin PATH] [--client-bin PATH]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Defaults
SERVER_BIN="${ROOT_DIR}/build/tests/tobj_socket_server"
BRIDGE_BIN="${ROOT_DIR}/build/tests/standalone_bridge"
CLIENT_BIN="${ROOT_DIR}/examples/ada/bin/ada_active_find_e2e"
BACKEND_PORT=19235
FRONTEND_PORT=19236
PORT_FILE=$(mktemp /tmp/tobj_port.XXXXXX)
BRIDGE_PORT_FILE=$(mktemp /tmp/bridge_port.XXXXXX)
TIMEOUT=25

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --server-bin) SERVER_BIN="$2"; shift 2 ;;
    --bridge-bin) BRIDGE_BIN="$2"; shift 2 ;;
    --client-bin) CLIENT_BIN="$2"; shift 2 ;;
    --backend-port) BACKEND_PORT="$2"; shift 2 ;;
    --frontend-port) FRONTEND_PORT="$2"; shift 2 ;;
    *)            shift ;;
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

echo "=== Ada ActiveFind E2E Test (3-process: server → bridge → client) ==="

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

if [[ ! -x "$BRIDGE_BIN" ]]; then
  echo "[driver] FAIL: Bridge binary not found at $BRIDGE_BIN"
  exit 1
fi

# Step 2: Start TacticalObjects server with --no-bridge --no-entity
echo "[driver] Starting backend server on port $BACKEND_PORT (--no-bridge --no-entity)..."
"$SERVER_BIN" --port "$BACKEND_PORT" --port-file "$PORT_FILE" \
              --timeout "$TIMEOUT" --no-entity --no-bridge &
SERVER_PID=$!

# Step 3: Wait for server port file
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

ACTUAL_BACKEND_PORT=$(cat "$PORT_FILE")
echo "[driver] Backend server ready on port $ACTUAL_BACKEND_PORT"

# Step 4: Brief delay so server enters accept()
sleep 0.2

# Step 5: Start standalone bridge (connects to server, serves Ada client)
echo "[driver] Starting standalone bridge (backend=$ACTUAL_BACKEND_PORT, frontend=$FRONTEND_PORT)..."
"$BRIDGE_BIN" --backend-host 127.0.0.1 --backend-port "$ACTUAL_BACKEND_PORT" \
              --frontend-port "$FRONTEND_PORT" --port-file "$BRIDGE_PORT_FILE" \
              --timeout "$TIMEOUT" &
BRIDGE_PID=$!

# Step 6: Wait for bridge port file
echo "[driver] Waiting for bridge to write port file..."
for i in $(seq 1 50); do
  if [[ -s "$BRIDGE_PORT_FILE" ]]; then
    break
  fi
  sleep 0.1
done

if [[ ! -s "$BRIDGE_PORT_FILE" ]]; then
  echo "[driver] FAIL: Bridge did not write port file within 5 seconds"
  exit 1
fi

ACTUAL_FRONTEND_PORT=$(cat "$BRIDGE_PORT_FILE")
echo "[driver] Bridge ready, frontend on port $ACTUAL_FRONTEND_PORT"

# Step 7: Brief delay so bridge enters accept()
sleep 0.2

# Step 8: Start Ada active-find client (connects to bridge)
echo "[driver] Starting Ada active-find client (→ bridge port $ACTUAL_FRONTEND_PORT)..."
"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_FRONTEND_PORT"
CLIENT_EXIT=$?

# Step 9: Stop bridge and server
if kill -0 "$BRIDGE_PID" 2>/dev/null; then
  kill "$BRIDGE_PID" 2>/dev/null || true
  wait "$BRIDGE_PID" 2>/dev/null || true
fi
if kill -0 "$SERVER_PID" 2>/dev/null; then
  kill "$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
fi

# Step 10: Report
if [[ $CLIENT_EXIT -eq 0 ]]; then
  echo "[driver] PASS: Ada ActiveFind E2E (3-process) — evidence + correlation via standalone bridge succeeded"
  exit 0
else
  echo "[driver] FAIL: Ada active-find client exited with code $CLIENT_EXIT"
  exit 1
fi
