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
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

# Defaults
SERVER_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tests/tobj_socket_server"
BRIDGE_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tests/standalone_bridge"
CLIENT_BIN="${PYRAMID_ROOT}/examples/ada/bin/ada_active_find_e2e"
BACKEND_PORT=19235
FRONTEND_PORT=19236
CONTENT_TYPE="application/json"
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
    --content-type) CONTENT_TYPE="$2"; shift 2 ;;
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

# Step 0: Generate Ada bindings from proto
if command -v python3 &>/dev/null || command -v python &>/dev/null; then
  echo "[driver] Generating Ada bindings from proto..."
  bash "$PYRAMID_ROOT/scripts/generate_bindings.sh" --ada
else
  echo "[driver] python not found — skipping Ada stub generation"
fi

# Step 1: Build Ada client if gprbuild is available
if command -v gprbuild &>/dev/null; then
  echo "[driver] Building Ada active-find client..."
  ADA_PCL_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pcl"
  if [[ ! -f "$ADA_PCL_LIB_DIR/libpcl_core.a" || ! -f "$ADA_PCL_LIB_DIR/libpcl_transport_socket.a" ]]; then
    echo "[driver] Preparing GNAT-compatible PCL static archives..."
    "$WORKSPACE_ROOT/subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh" "$ADA_PCL_LIB_DIR" || {
      echo "[driver] SKIP: unable to build GNAT-compatible PCL static archives"
      exit 0
    }
  fi

  (cd "$PYRAMID_ROOT/examples/ada" && \
    MUJIN_ROOT="$WORKSPACE_ROOT" gprbuild -P ada_active_find_e2e.gpr -q \
      -XMUJIN_ROOT="$WORKSPACE_ROOT" \
      -XPCL_INCLUDE_DIR="$WORKSPACE_ROOT/subprojects/PCL/include" \
      -XPCL_LIB_DIR="$ADA_PCL_LIB_DIR" \
      -XPCL_LIB_NAME=pcl_core \
      -XPCL_SOCKET_LIB_NAME=pcl_transport_socket 2>&1) || {
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
              --frontend-content-type "$CONTENT_TYPE" --timeout "$TIMEOUT" &
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
"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_FRONTEND_PORT" \
              --content-type "$CONTENT_TYPE"
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
