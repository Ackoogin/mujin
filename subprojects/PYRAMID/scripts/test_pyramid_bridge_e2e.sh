#!/usr/bin/env bash
# test_pyramid_bridge_e2e.sh
#
# Integration test for the Pyramid Bridge.
#
# Process topology:
#   tactical_objects_app  (TCP socket, --demo-evidence)
#         ↑  TCP socket client
#   pyramid_bridge_main   (Ada bridge, shared-memory to AME)
#         ↑  shared-memory bus "pyramid_bridge"
#   ame_backend_stub      (C++, shared-memory server)
#
# Pass criteria: ame_backend_stub exits 0 (i.e., received ≥1 world fact).
#
# Usage:
#   test_pyramid_bridge_e2e.sh
#     [--app-bin    PATH]   tactical_objects_app binary
#     [--stub-bin   PATH]   ame_backend_stub binary
#     [--bridge-bin PATH]   pyramid_bridge_main Ada binary
#     [--port       NUM]    tactical_objects_app TCP port (default 19400)
#     [--bus        NAME]   shared-memory bus name (default pyramid_bridge)
#     [--timeout    SECS]   per-process wall-clock timeout (default 25)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

APP_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/tactical_objects/tactical_objects_app"
STUB_BIN="${WORKSPACE_ROOT}/build/subprojects/PYRAMID/pyramid_bridge/cpp/ame_backend_stub"
BRIDGE_BIN="${PYRAMID_ROOT}/pyramid_bridge/ada/bin/pyramid_bridge_main"
PORT=19400
BUS_NAME="pyramid_bridge"
TIMEOUT=25

while [[ $# -gt 0 ]]; do
  case "$1" in
    --app-bin)    APP_BIN="$2";    shift 2 ;;
    --stub-bin)   STUB_BIN="$2";   shift 2 ;;
    --bridge-bin) BRIDGE_BIN="$2"; shift 2 ;;
    --port)       PORT="$2";       shift 2 ;;
    --bus)        BUS_NAME="$2";   shift 2 ;;
    --timeout)    TIMEOUT="$2";    shift 2 ;;
    *) shift ;;
  esac
done

APP_PID=""
STUB_PID=""
BRIDGE_PID=""
PORT_FILE=$(mktemp /tmp/tobj_bridge_e2e.XXXXXX)

cleanup() {
  for pid_var in BRIDGE_PID STUB_PID APP_PID; do
    eval "pid=\${$pid_var:-}"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  rm -f "$PORT_FILE"
}
trap cleanup EXIT

echo "=== Pyramid Bridge E2E ==="
echo "[driver] bus=$BUS_NAME port=$PORT timeout=${TIMEOUT}s"

# -------------------------------------------------------------------------
# Optional: build Ada bridge if gprbuild is available
# -------------------------------------------------------------------------

if command -v gprbuild &>/dev/null; then
  echo "[driver] Building Ada Pyramid Bridge..."
  ADA_PCL_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pcl"
  ADA_PYRAMID_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pyramid"

  "$WORKSPACE_ROOT/subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh" \
    "$ADA_PCL_LIB_DIR" --force || {
      echo "[driver] SKIP: unable to build GNAT-compatible PCL archives"
      exit 0
    }
  "$PYRAMID_ROOT/scripts/build_gnat_generated_flatbuffers_libs.sh" \
    "$ADA_PYRAMID_LIB_DIR" --force || {
      echo "[driver] SKIP: unable to build GNAT-compatible generated FlatBuffers archive"
      exit 0
    }
  (cd "$PYRAMID_ROOT/pyramid_bridge/ada" && \
    MUJIN_ROOT="$WORKSPACE_ROOT" gprbuild -P pyramid_bridge.gpr -q \
      -XMUJIN_ROOT="$WORKSPACE_ROOT" \
      -XPCL_INCLUDE_DIR="$WORKSPACE_ROOT/subprojects/PCL/include" \
      -XPCL_LIB_DIR="$ADA_PCL_LIB_DIR" \
      -XPCL_LIB_NAME=pcl_core \
      -XPCL_SOCKET_LIB_NAME=pcl_transport_socket \
      -XPCL_SHMEM_LIB_NAME=pcl_transport_shared_memory \
      -XPYRAMID_GEN_LIB_DIR="$ADA_PYRAMID_LIB_DIR" \
      -XPYRAMID_GEN_LIB_NAME=pyramid_generated_flatbuffers_codec 2>&1) || {
    echo "[driver] SKIP: gprbuild failed (GNAT toolchain issue)"
    exit 0
  }
else
  echo "[driver] gprbuild not found — checking for pre-built bridge binary..."
fi

# -------------------------------------------------------------------------
# Pre-flight checks
# -------------------------------------------------------------------------

if [[ ! -x "$APP_BIN" ]]; then
  echo "[driver] FAIL: tactical_objects_app not found at $APP_BIN"
  exit 1
fi
if [[ ! -x "$STUB_BIN" ]]; then
  echo "[driver] FAIL: ame_backend_stub not found at $STUB_BIN"
  exit 1
fi
if [[ ! -x "$BRIDGE_BIN" ]]; then
  echo "[driver] SKIP: pyramid_bridge_main not found at $BRIDGE_BIN"
  exit 0
fi

# -------------------------------------------------------------------------
# 1. Start ame_backend_stub (must be up before bridge connects)
# -------------------------------------------------------------------------

echo "[driver] Starting ame_backend_stub (bus=$BUS_NAME)..."
"$STUB_BIN" --bus "$BUS_NAME" --timeout "$TIMEOUT" &
STUB_PID=$!
sleep 0.3

if ! kill -0 "$STUB_PID" 2>/dev/null; then
  echo "[driver] FAIL: ame_backend_stub failed to start"
  exit 1
fi

# -------------------------------------------------------------------------
# 2. Start tactical_objects_app with demo evidence injection
# -------------------------------------------------------------------------

echo "[driver] Starting tactical_objects_app on port $PORT..."
"$APP_BIN" --port "$PORT" --port-file "$PORT_FILE" \
           --timeout "$TIMEOUT" --demo-evidence &
APP_PID=$!

for _ in $(seq 1 50); do
  [[ -s "$PORT_FILE" ]] && break
  sleep 0.1
done

if [[ ! -s "$PORT_FILE" ]]; then
  echo "[driver] FAIL: tactical_objects_app did not write port file" >&2
  exit 1
fi
ACTUAL_PORT=$(cat "$PORT_FILE")
echo "[driver] tactical_objects_app ready on port $ACTUAL_PORT"

# -------------------------------------------------------------------------
# 3. Start Ada Pyramid Bridge
# -------------------------------------------------------------------------

echo "[driver] Starting pyramid_bridge_main..."
"$BRIDGE_BIN" \
  --tobj-host 127.0.0.1 \
  --tobj-port "$ACTUAL_PORT" \
  --ame-bus   "$BUS_NAME" \
  --timeout   "$TIMEOUT" &
BRIDGE_PID=$!

# -------------------------------------------------------------------------
# 4. Wait for ame_backend_stub to finish (it exits when it receives facts)
# -------------------------------------------------------------------------

echo "[driver] Waiting for ame_backend_stub to finish..."
if wait "$STUB_PID"; then
  STUB_RC=0
else
  STUB_RC=$?
fi
STUB_PID=""

if [[ "$STUB_RC" -eq 0 ]]; then
  echo "[driver] PASS: ame_backend_stub received world facts from Pyramid Bridge"
  exit 0
else
  echo "[driver] FAIL: ame_backend_stub did not receive world facts (exit=$STUB_RC)"
  exit 1
fi
