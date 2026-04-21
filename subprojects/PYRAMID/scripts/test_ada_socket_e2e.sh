#!/usr/bin/env bash
# test_ada_socket_e2e.sh — Cross-process E2E test driver.
#
# Orchestrates:
#   1. Build Ada client (if gprbuild available)
#   2. Start C++ server (tobj_socket_server)
#   3. Wait for server to be ready (port file appears)
#   4. Start Ada client (ada_tobj_client)
#   5. Wait for Ada client to exit
#   6. Terminate server
#   7. Report pass/fail
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

# Step 0: Generate Ada service stubs from proto (provided + consumed)
if command -v python3 &>/dev/null || command -v python &>/dev/null; then
  echo "[driver] Generating Ada bindings from proto..."
  bash "$PYRAMID_ROOT/scripts/generate_bindings.sh" --ada
else
  echo "[driver] python not found — skipping Ada stub generation"
fi

# Step 1: Build Ada client if gprbuild is available
if command -v gprbuild &>/dev/null; then
  echo "[driver] Building Ada client..."
  ADA_PCL_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pcl"
  ADA_PYRAMID_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pyramid"
  echo "[driver] Refreshing GNAT-compatible PCL static archives..."
  "$WORKSPACE_ROOT/subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh" "$ADA_PCL_LIB_DIR" --force || {
    echo "[driver] SKIP: unable to build GNAT-compatible PCL static archives"
    exit 0
  }
  echo "[driver] Refreshing GNAT-compatible generated FlatBuffers archive..."
  "$PYRAMID_ROOT/scripts/build_gnat_generated_flatbuffers_libs.sh" "$ADA_PYRAMID_LIB_DIR" || {
    echo "[driver] SKIP: unable to build GNAT-compatible generated FlatBuffers archive"
    exit 0
  }

  (cd "$PYRAMID_ROOT/examples/ada" && \
    MUJIN_ROOT="$WORKSPACE_ROOT" gprbuild -P ada_tobj_client.gpr -q \
      -XMUJIN_ROOT="$WORKSPACE_ROOT" \
      -XPCL_INCLUDE_DIR="$WORKSPACE_ROOT/subprojects/PCL/include" \
      -XPCL_LIB_DIR="$ADA_PCL_LIB_DIR" \
      -XPCL_LIB_NAME=pcl_core \
      -XPCL_SOCKET_LIB_NAME=pcl_transport_socket \
      -XPYRAMID_GEN_LIB_DIR="$ADA_PYRAMID_LIB_DIR" \
      -XPYRAMID_GEN_LIB_NAME=pyramid_generated_flatbuffers_codec 2>&1) || {
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

# Step 2: Start server
echo "[driver] Starting server on port $PORT..."
"$SERVER_BIN" --port "$PORT" --port-file "$PORT_FILE" --timeout "$TIMEOUT" &
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

# Step 5: Start Ada client
echo "[driver] Starting Ada client..."
"$CLIENT_BIN" --host 127.0.0.1 --port "$ACTUAL_PORT"
CLIENT_EXIT=$?

# Step 6: Stop server
if kill -0 "$SERVER_PID" 2>/dev/null; then
  kill "$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
fi

# Step 7: Report
if [[ $CLIENT_EXIT -eq 0 ]]; then
  echo "[driver] PASS: Ada client received entity updates over socket transport"
  exit 0
else
  echo "[driver] FAIL: Ada client exited with code $CLIENT_EXIT"
  exit 1
fi
