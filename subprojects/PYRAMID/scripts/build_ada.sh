#!/usr/bin/env bash
#
# build_ada.sh -- end-to-end ".proto -> Ada client/bridge binaries" build.
#
# The Ada counterpart to build_plugins.sh. Ada does NOT produce plugins: the
# codec/transport plugins are language-neutral C-ABI .so files built by the C++
# chain (build_plugins.sh / the `pyramid_plugins` target). Ada clients and the
# Ada bridge *consume* those same .so files at run time (codec via
# PYRAMID_CODEC_PLUGINS, transport via PCL_TRANSPORT_PLUGIN). This script builds
# the Ada *consumers*.
#
# Pipeline:
#
#   proto/**.proto
#     |                          (--regen: generate_bindings.sh --ada -> build-local Ada bindings)
#     v
#   <build>/generated/pyramid_ada_bindings/**.{ads,adb} (compiled from source by GNAT)
#     |                          (CMake + gprbuild; GNAT FlatBuffers archive built automatically)
#     v
#   Ada binaries: ada_tobj_client, ada_active_find_e2e, pyramid_bridge (Ada), ...
#     + the C++ codec/transport plugins they load at run time (pyramid_plugins)
#
# Differences from the C++ chain (build_plugins.sh):
#   - Toolchain: GNAT `gprbuild` via .gpr projects (needs gnat/gprbuild on PATH).
#   - Ada and C++ bindings regenerate into the build tree. Pass --regen to
#     refresh the build-local Ada bindings before CMake config/build.
#   - The Ada targets are excluded from the default build; this script builds the
#     `pyramid_ada_all` aggregate (which also auto-builds the GNAT FlatBuffers
#     support archive and the GNAT-compatible PCL archives).
#
# Usage:
#   build_ada.sh [--build-dir DIR] [--regen] [--backends LIST]
#                [--jobs N] [--clean] [--test]
#
# Options:
#   --build-dir DIR  CMake build directory       (default: <repo>/build-ada)
#   --regen          regenerate build-local Ada bindings from proto before building
#   --backends LIST  backends for --regen         (default: json,flatbuffers)
#   --jobs N         parallel build jobs          (default: nproc)
#   --clean          delete the build dir before configuring
#   --test           run the Ada test suite (ctest -L ada) after building
#
# CI example:
#   subprojects/PYRAMID/scripts/build_ada.sh --regen --test
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

BUILD_DIR="${REPO_ROOT}/build-ada"
BACKENDS="json,flatbuffers"
DO_REGEN=0
DO_TEST=0
CLEAN=0
JOBS="$( (command -v nproc >/dev/null && nproc) || echo 4)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --regen)     DO_REGEN=1; shift ;;
    --backends)  BACKENDS="$2"; shift 2 ;;
    --jobs)      JOBS="$2"; shift 2 ;;
    --clean)     CLEAN=1; shift ;;
    --test)      DO_TEST=1; shift ;;
    -h|--help)   sed -n '2,52p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[build_ada] unknown arg: $1" >&2; exit 2 ;;
  esac
done

if ! command -v gprbuild >/dev/null 2>&1; then
  echo "[build_ada] FAIL: gprbuild not found on PATH (Ada builds need a GNAT toolchain)." >&2
  exit 1
fi

echo "[build_ada] repo      : ${REPO_ROOT}"
echo "[build_ada] build-dir : ${BUILD_DIR}"
echo "[build_ada] regen     : $([[ ${DO_REGEN} -eq 1 ]] && echo "on (${BACKENDS})" || echo off)"
echo "[build_ada] jobs      : ${JOBS}"

if [[ ${CLEAN} -eq 1 ]]; then
  echo "[build_ada] cleaning ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

if [[ ${DO_REGEN} -eq 1 ]]; then
  echo "[build_ada] regenerating build-local Ada bindings from proto ..."
  "${SCRIPT_DIR}/generate_bindings.sh" \
    --ada \
    --backends "${BACKENDS}" \
    --ada-out "${BUILD_DIR}/generated/pyramid_ada_bindings"
fi

# Lean configuration matching build_plugins.sh (PYRAMID only, FlatBuffers+JSON,
# no AME/Foxglove/ROS2). The Ada targets are picked up when gprbuild is found.
echo "[build_ada] configure ..."
cmake \
  -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUNMANNED_BUILD_PYRAMID=ON \
  -DUNMANNED_BUILD_AME=OFF \
  -DAME_FOXGLOVE=OFF \
  -DPYRAMID_ENABLE_ROS2=OFF \
  -DPYRAMID_ENABLE_FLATBUFFERS=ON

# pyramid_plugins  -> the codec/transport .so the Ada binaries load at run time
# pyramid_ada_all  -> the Ada binaries (auto-builds the GNAT FlatBuffers archive
#                     and GNAT-compatible PCL archives)
echo "[build_ada] build pyramid_plugins + pyramid_ada_all ..."
cmake --build "${BUILD_DIR}" --target pyramid_plugins pyramid_ada_all --parallel "${JOBS}"

echo
echo "[build_ada] produced Ada binaries:"
find "${REPO_ROOT}/subprojects/PYRAMID" -type d -name bin 2>/dev/null \
  | while read -r d; do
      find "${d}" -maxdepth 1 -type f -executable 2>/dev/null
    done | sort | sed "s|${REPO_ROOT}/|  |" || true

if [[ ${DO_TEST} -eq 1 ]]; then
  echo
  echo "[build_ada] running Ada test suite (ctest -L ada) ..."
  ctest --test-dir "${BUILD_DIR}" -L ada --output-on-failure
fi

echo
echo "[build_ada] PASS"
