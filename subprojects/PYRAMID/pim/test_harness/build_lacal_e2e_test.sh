#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/../../../.." && pwd)
BUILD_DIR=${PYRAMID_BUILD_DIR:-${ROOT_DIR}/build-flatbuffers-only}
BIN=${LACAL_E2E_BIN:-${BUILD_DIR}/subprojects/PYRAMID/tests/lacal_e2e_test}
TMP_DIR=$(mktemp -d)
SLEET_PID=""
SUB_PID=""

cleanup() {
  if [[ -n "${SUB_PID}" ]]; then kill "${SUB_PID}" 2>/dev/null || true; fi
  if [[ -n "${SLEET_PID}" ]]; then kill "${SLEET_PID}" 2>/dev/null || true; fi
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DPYRAMID_ENABLE_OWP=ON
cmake --build "${BUILD_DIR}" --target lacal_e2e_test --parallel

if [[ -n "${SLEET_BIN:-}" ]]; then
  if [[ ! -x "${SLEET_BIN}" ]]; then
    echo "SKIP: SLEET_BIN is not executable: ${SLEET_BIN}"
    exit 0
  fi
  if [[ -z "${SLEET_SCHEMA_PATH:-}" || ! -f "${SLEET_SCHEMA_PATH:-}" ]]; then
    echo "SKIP: SLEET_SCHEMA_PATH must name the pinned UCI 2.5 XSD"
    exit 0
  fi
  SLEET_PORT=${SLEET_PORT:-21402}
  SLEET_URL="ws://127.0.0.1:${SLEET_PORT}"
  cat >"${TMP_DIR}/server.toml" <<EOF
bind_addr = "127.0.0.1:${SLEET_PORT}"
server_id = "pyramid-lacal-e2e"
system_label = "PYRAMID LA-CAL E2E"
schema_path = "${SLEET_SCHEMA_PATH}"
schema_version = "002.5.0"
system_uuid = "123e4567-e89b-12d3-a456-426614174000"
subsystem_uuid = "123e4567-e89b-12d3-a456-426614174001"
services_dir = "${SCRIPT_DIR}/lacal/services"
idle_timeout_seconds = 0
EOF
  "${SLEET_BIN}" "${TMP_DIR}/server.toml" >"${TMP_DIR}/sleet.log" 2>&1 &
  SLEET_PID=$!
elif [[ -z "${SLEET_URL:-}" ]]; then
  echo "SKIP: set SLEET_URL, or set SLEET_BIN and SLEET_SCHEMA_PATH"
  exit 0
fi

HOST_PORT=${SLEET_URL#ws://}
HOST_PORT=${HOST_PORT%%/*}
HOST=${HOST_PORT%:*}
PORT=${HOST_PORT##*:}
if [[ "${HOST}" == "${HOST_PORT}" || -z "${PORT}" ]]; then
  echo "FAIL: SLEET_URL must be ws://host:port[/path]"
  exit 2
fi

reachable=false
for _ in $(seq 1 50); do
  if python3 - "${HOST}" "${PORT}" <<'PY'
import socket
import sys

try:
    with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.2):
        pass
except OSError:
    raise SystemExit(1)
PY
  then
    reachable=true
    break
  fi
  sleep 0.1
done

if [[ "${reachable}" != true ]]; then
  echo "SKIP: Sleet is unreachable at ${SLEET_URL}"
  exit 0
fi

READY=${TMP_DIR}/subscriber.ready
OUTPUT=${TMP_DIR}/subscriber.out
"${BIN}" subscriber "${SLEET_URL}" "${READY}" "${OUTPUT}" 10 &
SUB_PID=$!

for _ in $(seq 1 50); do
  if [[ -f "${READY}" ]]; then break; fi
  if ! kill -0 "${SUB_PID}" 2>/dev/null; then
    wait "${SUB_PID}"
    echo "FAIL: subscriber exited before becoming ready"
    exit 2
  fi
  sleep 0.1
done

if [[ ! -f "${READY}" ]]; then
  echo "FAIL: subscriber did not become ready"
  exit 2
fi

"${BIN}" publisher "${SLEET_URL}"
wait "${SUB_PID}"
SUB_PID=""

if [[ ! -f "${OUTPUT}" ]] || ! grep -q '^position-ok$' "${OUTPUT}"; then
  echo "FAIL: subscriber did not decode the expected PositionReport"
  exit 2
fi

echo "PASS: cross-process PCL -> Sleet -> PCL PositionReport delivery"
