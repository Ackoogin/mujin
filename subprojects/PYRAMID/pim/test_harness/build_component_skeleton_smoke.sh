#!/usr/bin/env bash
# Generate and compile the untouched sensors component scaffold.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYRAMID_ROOT="$(cd "${HERE}/../.." && pwd)"
SDK_ROOT="${PYRAMID_SDK:-}"

usage() {
  echo "Usage: $0 --sdk-root DIR [--proto-dir DIR]"
}

PROTO_DIR="${PYRAMID_ROOT}/pim/test/pyramid"
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

SCRATCH="$(mktemp -d "${TMPDIR:-/tmp}/pyramid-skeleton-smoke.XXXXXX")"
trap 'rm -rf "${SCRATCH}"' EXIT
SCAFFOLD="${SCRATCH}/scaffold"
# Generate into the scratch directory so the SDK's own generated
# contract tree is left untouched.
GENERATED_CPP="${SCRATCH}/generated"
GENERATED_ADA="${SCRATCH}/generated_ada"

echo "== generating C++ and Ada skeletons and scaffolds =="
python3 "${PYRAMID_ROOT}/pim/generate_bindings.py" \
  "${PROTO_DIR}" "${GENERATED_CPP}" --languages cpp --backends json \
  --component-skeletons --scaffold-dir "${SCAFFOLD}"
python3 "${PYRAMID_ROOT}/pim/generate_bindings.py" \
  "${PROTO_DIR}" "${GENERATED_ADA}" --languages ada --backends json \
  --component-skeletons --scaffold-dir "${SCAFFOLD}"

echo "== building the untouched C++ sensors process against pcl_core =="
cmake -S "${SCAFFOLD}/pim_osprey_sensors" \
  -B "${SCRATCH}/cpp-build" \
  -DPYRAMID_SDK="${SDK_ROOT}" \
  -DSDK_GENERATED_DIR="${GENERATED_CPP}" \
  -DPYRAMID_SDK_ENABLE_FLATBUFFERS=OFF
cmake --build "${SCRATCH}/cpp-build" \
  --target pim_osprey_sensors --parallel

if command -v gprbuild >/dev/null 2>&1; then
  echo "== object-compiling the untouched Ada sensors process =="
  mkdir -p \
    "${SCAFFOLD}/pim_osprey_sensors/ada/obj" \
    "${SCAFFOLD}/pim_osprey_sensors/ada/bin" \
    "${SDK_ROOT}/gnat/obj"
  GPR_PROJECT_PATH="${SDK_ROOT}/gnat${GPR_PROJECT_PATH:+:${GPR_PROJECT_PATH}}" \
    gprbuild -c -q \
      -P "${SCAFFOLD}/pim_osprey_sensors/ada/sensors.gpr" \
      -XPYRAMID_SDK_ROOT="${SDK_ROOT}" \
      -XPYRAMID_GENERATED_ADA_DIR="${GENERATED_ADA}" \
      -XGNATCOLL_OS=unix -XOS=linux
else
  echo "== Ada smoke skipped: gprbuild is not on PATH =="
fi

echo "Component skeleton compile smoke passed."
