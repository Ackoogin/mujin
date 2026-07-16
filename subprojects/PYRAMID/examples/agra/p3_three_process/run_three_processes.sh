#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT=""
BUILD_DIR=""
PROFILE="shared_memory"
DURATION_SECONDS=15

usage() {
  echo "Usage: $0 --sdk-root DIR --build-dir DIR [--profile shared_memory|tcp] [--duration-seconds N]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sdk-root) SDK_ROOT="$2"; shift 2 ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --duration-seconds) DURATION_SECONDS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${SDK_ROOT}" || -z "${BUILD_DIR}" ]]; then
  usage >&2
  exit 2
fi
case "${PROFILE}" in
  shared_memory|tcp) ;;
  *) echo "Profile must be 'shared_memory' or 'tcp'." >&2; exit 2 ;;
esac
if [[ ! "${DURATION_SECONDS}" =~ ^[0-9]+$ ]] ||
   (( DURATION_SECONDS < 3 || DURATION_SECONDS > 300 )); then
  echo "Duration must be an integer from 3 to 300 seconds." >&2
  exit 2
fi

SDK_ROOT="$(cd "${SDK_ROOT}" && pwd)"
BUILD_DIR="$(cd "${BUILD_DIR}" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/configs/linux/${PROFILE}"

COORDINATOR="${BUILD_DIR}/agra_p3_mission_coordinator"
SENSOR="${BUILD_DIR}/agra_p3_sensor_planner"
VEHICLE="${BUILD_DIR}/agra_p3_vehicle_planner"
for executable in "${COORDINATOR}" "${SENSOR}" "${VEHICLE}"; do
  if [[ ! -x "${executable}" ]]; then
    echo "Missing example executable: ${executable}" >&2
    exit 1
  fi
done

PIDS=()
cleanup() {
  for pid in "${PIDS[@]-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

DURATION_ARGUMENT="--duration-seconds=${DURATION_SECONDS}"
echo "Starting ${PROFILE} profile from ${SDK_ROOT}"
(
  cd "${SDK_ROOT}"
  exec "${COORDINATOR}" "${CONFIG_DIR}/mission_coordinator.routes" "${DURATION_ARGUMENT}"
) &
PIDS+=("$!")
sleep 1
(
  cd "${SDK_ROOT}"
  exec "${SENSOR}" "${CONFIG_DIR}/sensor_planner.routes" "${DURATION_ARGUMENT}"
) &
PIDS+=("$!")
(
  cd "${SDK_ROOT}"
  exec "${VEHICLE}" "${CONFIG_DIR}/vehicle_planner.routes" "${DURATION_ARGUMENT}"
) &
PIDS+=("$!")

FAILED=0
for pid in "${PIDS[@]}"; do
  if ! wait "${pid}"; then
    FAILED=1
  fi
done
PIDS=()

if [[ ${FAILED} -ne 0 ]]; then
  echo "One or more example processes failed." >&2
  exit 1
fi
echo "All three processes completed successfully."
