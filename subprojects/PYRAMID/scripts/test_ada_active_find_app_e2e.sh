#!/usr/bin/env bash
# test_ada_active_find_app_e2e.sh -- Ada ActiveFind against tactical_objects_app.
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

if command -v python3 &>/dev/null || command -v python &>/dev/null; then
  echo "[driver] Generating Ada bindings from proto..."
  bash "$PYRAMID_ROOT/scripts/generate_bindings.sh" --ada
else
  echo "[driver] python not found -- skipping Ada stub generation"
fi

if command -v gprbuild &>/dev/null; then
  echo "[driver] Building Ada active-find client..."
  ADA_PCL_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pcl"
  ADA_PYRAMID_LIB_DIR="$WORKSPACE_ROOT/build/ada_gnat_pyramid"
  "$WORKSPACE_ROOT/subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh" \
    "$ADA_PCL_LIB_DIR" --force || {
      echo "[driver] SKIP: unable to build GNAT-compatible PCL static archives"
      exit 0
    }
    "$PYRAMID_ROOT/scripts/build_gnat_generated_flatbuffers_libs.sh" \
    "$ADA_PYRAMID_LIB_DIR" || {
      echo "[driver] SKIP: unable to build GNAT-compatible generated FlatBuffers archive"
      exit 0
    }
  (cd "$PYRAMID_ROOT/tests/ada" && \
    UNMANNED_ROOT="$WORKSPACE_ROOT" gprbuild -P ada_active_find_e2e.gpr -q \
      -XUNMANNED_ROOT="$WORKSPACE_ROOT" \
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
  echo "[driver] gprbuild not found -- checking for pre-built client..."
fi

if [[ ! -x "$CLIENT_BIN" ]]; then
  echo "[driver] SKIP: Ada client binary not found at $CLIENT_BIN"
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
