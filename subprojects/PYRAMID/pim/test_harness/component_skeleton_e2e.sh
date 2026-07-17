#!/usr/bin/env bash
# Prove that generated sensors skeleton code is unchanged across RPC/pub-sub.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYRAMID_ROOT="$(cd "${HERE}/../.." && pwd)"
SDK_ROOT="${PYRAMID_SDK:-}"
PROTO_DIR="${PYRAMID_ROOT}/pim/test/pyramid"

usage() {
  echo "Usage: $0 --sdk-root DIR [--proto-dir DIR]"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sdk-root) SDK_ROOT="$2"; shift 2 ;;
    --proto-dir) PROTO_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ -z "${SDK_ROOT}" ]]; then
  usage >&2
  exit 2
fi
SDK_ROOT="$(cd "${SDK_ROOT}" && pwd)"
PROTO_DIR="$(cd "${PROTO_DIR}" && pwd)"

TRANSPORT_PLUGIN="${SDK_ROOT}/plugins/libpcl_transport_shared_memory_plugin.so"
if [[ ! -f "${TRANSPORT_PLUGIN}" ]]; then
  echo "Missing shared-memory transport plugin: ${TRANSPORT_PLUGIN}" >&2
  exit 2
fi

SCRATCH="$(mktemp -d "${TMPDIR:-/tmp}/pyramid-skeleton-e2e.XXXXXX")"
PROVIDER_PID=""
cleanup() {
  if [[ -n "${PROVIDER_PID}" ]] &&
     kill -0 "${PROVIDER_PID}" 2>/dev/null; then
    kill "${PROVIDER_PID}" 2>/dev/null || true
    wait "${PROVIDER_PID}" 2>/dev/null || true
  fi
  rm -rf "${SCRATCH}"
}
trap cleanup EXIT INT TERM

echo "== generating the sensors skeleton =="
# Generate into the scratch directory so the SDK's own generated
# contract tree is left untouched.
GENERATED_CPP="${SCRATCH}/generated"
python3 "${PYRAMID_ROOT}/pim/generate_bindings.py" \
  "${PROTO_DIR}" "${GENERATED_CPP}" --languages cpp --backends json \
  --component-skeletons --components pim_osprey.sensors

echo "== building the filled provider stub and client harness =="
cmake -S "${HERE}/component_skeleton_e2e" \
  -B "${SCRATCH}/build" -DPYRAMID_SDK="${SDK_ROOT}" \
  -DSDK_GENERATED_DIR="${GENERATED_CPP}"
cmake --build "${SCRATCH}/build" \
  --target component_skeleton_e2e --parallel
E2E="${SCRATCH}/build/component_skeleton_e2e"

write_provider_config() {
  local mode="$1"
  local bus="$2"
  local output="$3"
  local config="{\"bus_name\":\"${bus}\",\"participant_id\":\"provider\"}"
  {
    echo "port authorisation_dependency_request rpc client ${TRANSPORT_PLUGIN} ${config}"
    echo "port capability_evidence_information pubsub client ${TRANSPORT_PLUGIN} ${config}"
    echo "port capability_information pubsub client ${TRANSPORT_PLUGIN} ${config}"
    echo "port sen_requirement_request ${mode} client ${TRANSPORT_PLUGIN} ${config}"
    echo "port sen_requirement_transition ${mode} client ${TRANSPORT_PLUGIN} ${config}"
  } > "${output}"
}

write_client_config() {
  local mode="$1"
  local bus="$2"
  local output="$3"
  local config="{\"bus_name\":\"${bus}\",\"participant_id\":\"client\"}"
  {
    echo "port sen_requirement_request ${mode} provider ${TRANSPORT_PLUGIN} ${config}"
    echo "port sen_requirement_transition ${mode} provider ${TRANSPORT_PLUGIN} ${config}"
  } > "${output}"
}

for mode in rpc pubsub; do
  echo "== running ${mode} realization =="
  bus="pyramid_component_skeleton_${mode}_$$"
  provider_config="${SCRATCH}/${mode}_provider.ports"
  client_config="${SCRATCH}/${mode}_client.ports"
  provider_log="${SCRATCH}/${mode}_provider.log"
  client_log="${SCRATCH}/${mode}_client.log"
  write_provider_config "${mode}" "${bus}" "${provider_config}"
  write_client_config "${mode}" "${bus}" "${client_config}"

  "${E2E}" provider "${provider_config}" --duration-seconds=10 \
    > "${provider_log}" 2>&1 &
  PROVIDER_PID="$!"
  sleep 1
  if ! "${E2E}" client "${client_config}" --duration-seconds=8 \
      > "${client_log}" 2>&1; then
    echo "${mode} client process failed." >&2
    cat "${provider_log}" "${client_log}" >&2
    exit 1
  fi
  kill "${PROVIDER_PID}" 2>/dev/null || true
  wait "${PROVIDER_PID}" || true
  PROVIDER_PID=""

  if ! grep -q '^ACK_OBSERVED$' "${client_log}" ||
     ! grep -q '^TRANSITION_OBSERVED$' "${client_log}"; then
    echo "${mode} realization did not observe both acknowledgements." >&2
    cat "${provider_log}" "${client_log}" >&2
    exit 1
  fi
  if grep -q 'E2E_FAIL' "${client_log}" "${provider_log}"; then
    echo "${mode} realization reported an E2E failure." >&2
    cat "${provider_log}" "${client_log}" >&2
    exit 1
  fi
done

echo "Component skeleton E2E passed for rpc and pubsub."
