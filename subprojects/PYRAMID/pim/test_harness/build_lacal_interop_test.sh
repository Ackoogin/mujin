#!/usr/bin/env bash
# LA-CAL Phase 4 foreign-peer interop: PCL <-> Sleet <-> AMS GRA la-cal-harness.
#
# Two directions, both must pass:
#   A. we consume foreign  -- harness OWP client + XSD-derived OMS JSON PUBs a
#      PositionReport; the PCL subscriber typed-decodes it.
#   B. foreign consumes us  -- PCL publishes; the harness OWP client subscribes,
#      receives, and validates the OMS JSON.
#
# SKIPs (exit 0) when the live peers are unavailable: no reachable Sleet, no
# harness-capable Python, or no UCI 2.5 XSD. A reachable Sleet that then
# rejects registration, subscription, a schema-valid payload, or delivery is a
# FAIL, not a SKIP (same contract as build_lacal_e2e_test.sh).
#
# Required env:
#   SLEET_URL             ws://host:port of a running Sleet with the
#                         lacal/services/*.toml registrations loaded.
#   LACAL_HARNESS_PYTHON  python interpreter (>=3.12) with `la_cal_harness`
#                         importable (e.g. a venv: pip install -e la-cal-harness).
#   UCI_XSD_PATH          pinned UCI_MessageDefinitions_v2_5_0.xsd.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/../../../.." && pwd)
BUILD_DIR=${PYRAMID_BUILD_DIR:-${ROOT_DIR}/build-flatbuffers-only}
BIN=${LACAL_E2E_BIN:-${BUILD_DIR}/subprojects/PYRAMID/tests/lacal_e2e_test}
DRIVER=${SCRIPT_DIR}/lacal/lacal_interop_driver.py
TMP_DIR=$(mktemp -d)
SUB_PID=""

cleanup() {
  if [[ -n "${SUB_PID}" ]]; then kill "${SUB_PID}" 2>/dev/null || true; fi
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

if [[ -z "${SLEET_URL:-}" ]]; then
  echo "SKIP: set SLEET_URL to a running Sleet with lacal/services loaded"
  exit 0
fi
if [[ -z "${LACAL_HARNESS_PYTHON:-}" ]] || ! "${LACAL_HARNESS_PYTHON}" -c \
    "import la_cal_harness" >/dev/null 2>&1; then
  echo "SKIP: set LACAL_HARNESS_PYTHON to a python with la_cal_harness installed"
  exit 0
fi
if [[ -z "${UCI_XSD_PATH:-}" || ! -f "${UCI_XSD_PATH}" ]]; then
  echo "SKIP: set UCI_XSD_PATH to the pinned UCI 2.5 XSD"
  exit 0
fi

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DPYRAMID_ENABLE_OWP=ON >/dev/null
cmake --build "${BUILD_DIR}" --target lacal_e2e_test --parallel >/dev/null

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

# --- Direction A: we consume foreign ---------------------------------------
READY_A=${TMP_DIR}/a.ready
OUT_A=${TMP_DIR}/a.out
"${BIN}" subscriber "${SLEET_URL}" "${READY_A}" "${OUT_A}" 15 &
SUB_PID=$!
for _ in $(seq 1 100); do
  if [[ -f "${READY_A}" ]]; then break; fi
  if ! kill -0 "${SUB_PID}" 2>/dev/null; then
    wait "${SUB_PID}"; echo "FAIL: PCL subscriber exited before ready"; exit 2
  fi
  sleep 0.1
done
if [[ ! -f "${READY_A}" ]]; then echo "FAIL: PCL subscriber never ready"; exit 2; fi

if ! "${LACAL_HARNESS_PYTHON}" "${DRIVER}" publish \
    --url "${SLEET_URL}" --xsd "${UCI_XSD_PATH}"; then
  echo "FAIL: harness publish (direction A) failed"; exit 2
fi
wait "${SUB_PID}"; SUB_PID=""
if [[ ! -f "${OUT_A}" ]] || ! grep -q '^position-ok$' "${OUT_A}"; then
  echo "FAIL: PCL subscriber did not decode the foreign PositionReport"; exit 2
fi
echo "PASS A: harness -> Sleet -> PCL PositionReport (we consume foreign)"

# --- Direction B: foreign consumes us --------------------------------------
READY_B=${TMP_DIR}/b.ready
"${LACAL_HARNESS_PYTHON}" "${DRIVER}" subscribe \
  --url "${SLEET_URL}" --ready "${READY_B}" --timeout 15 &
SUB_PID=$!
for _ in $(seq 1 150); do
  if [[ -f "${READY_B}" ]]; then break; fi
  if ! kill -0 "${SUB_PID}" 2>/dev/null; then
    wait "${SUB_PID}"; echo "FAIL: harness subscriber exited before ready"; exit 2
  fi
  sleep 0.1
done
if [[ ! -f "${READY_B}" ]]; then echo "FAIL: harness subscriber never ready"; exit 2; fi

"${BIN}" publisher "${SLEET_URL}"
if ! wait "${SUB_PID}"; then
  SUB_PID=""; echo "FAIL: harness subscriber did not validate the PCL PositionReport"; exit 2
fi
SUB_PID=""
echo "PASS B: PCL -> Sleet -> harness PositionReport (foreign consumes us)"

echo "PASS: LA-CAL Phase 4 foreign-peer interop (both directions)"
