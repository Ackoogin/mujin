#!/usr/bin/env bash
# LA-CAL Phase 5 positive: a correlated request/requirement interaction realized
# pub/sub on both legs over the LA-CAL transport, through real Sleet, using the
# UCI ActionCommand / ActionCommandStatus pair.
#
# The consumer (seam-c2) publishes an ActionCommand and subscribes the
# requirement topic; the provider (seam-ma) subscribes the request topic and
# publishes correlated ActionCommandStatus transitions (RECEIVED, ACCEPTED).
# Both legs route over the LA-CAL (owp/asb) peer. SKIPs (exit 0) when Sleet is
# unavailable; a reachable-but-rejecting Sleet is a FAIL.
#
# The seam-ma / seam-c2 service registrations (lacal/services) must be loaded by
# the Sleet the SLEET_URL points at.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/../../../.." && pwd)
BUILD_DIR=${PYRAMID_BUILD_DIR:-${ROOT_DIR}/build-flatbuffers-only}
BIN=${LACAL_SEAM_BIN:-${BUILD_DIR}/subprojects/PYRAMID/tests/lacal_seam_test}
TMP_DIR=$(mktemp -d)
PROV_PID=""

cleanup() {
  if [[ -n "${PROV_PID}" ]]; then kill "${PROV_PID}" 2>/dev/null || true; fi
  rm -rf "${TMP_DIR}"
}
trap cleanup EXIT

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DPYRAMID_ENABLE_OWP=ON >/dev/null
cmake --build "${BUILD_DIR}" --target lacal_seam_test --parallel >/dev/null

if [[ -z "${SLEET_URL:-}" ]]; then
  echo "SKIP: set SLEET_URL to a running Sleet with lacal/services loaded"
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

READY=${TMP_DIR}/provider.ready
OUTPUT=${TMP_DIR}/consumer.out

# Provider first: its request-topic subscription must be active before the
# consumer publishes the ActionCommand.
"${BIN}" provider "${SLEET_URL}" "${READY}" 12 &
PROV_PID=$!
for _ in $(seq 1 100); do
  if [[ -f "${READY}" ]]; then break; fi
  if ! kill -0 "${PROV_PID}" 2>/dev/null; then
    wait "${PROV_PID}"; echo "FAIL: provider exited before becoming ready"; exit 2
  fi
  sleep 0.1
done
if [[ ! -f "${READY}" ]]; then echo "FAIL: provider did not become ready"; exit 2; fi

"${BIN}" consumer "${SLEET_URL}" "${OUTPUT}" 12
if ! wait "${PROV_PID}"; then
  PROV_PID=""; echo "FAIL: provider did not observe the request"; exit 2
fi
PROV_PID=""

if [[ ! -f "${OUTPUT}" ]] || ! grep -q '^seam-ok$' "${OUTPUT}"; then
  echo "FAIL: consumer did not receive the correlated requirement transitions"
  exit 2
fi

echo "PASS: LA-CAL Phase 5 request/requirement seam over Sleet (both legs pub/sub)"
