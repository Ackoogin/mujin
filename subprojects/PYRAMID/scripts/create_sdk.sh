#!/usr/bin/env bash
#
# create_sdk.sh -- one command to build the monorepo and package the offline
#                  PCL/PYRAMID SDK.
#
# Maintainer-run, from inside this repo. Collapses the multi-step "Maintainer:
# create a distribution" flow from sdk_template/README.md into a single command:
#
#   1. cmake --preset flatbuffers-only                 (configure)
#   2. cmake --build --preset flatbuffers-only-<cfg>   (build repo + C++ bindings)
#   3. scripts/build_plugins.sh --build-dir build-flatbuffers-only
#   4. [Ada] subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh <dir>/ada_gnat_pcl
#            (skipped with --no-ada or when no GNAT toolchain is on PATH)
#   5. scripts/package_sdk.sh --build-dir build-flatbuffers-only --clean --out <out>
#   6. [--verify] run the downstream verify flow inside the packaged SDK
#
# The result is a self-contained deploy directory (default dist/pcl_pyramid_sdk)
# that a downstream user builds with scripts/build_sdk.sh -- no monorepo, no
# network.
#
# Usage:
#   create_sdk.sh [--out DIR] [--config Release|Debug] [--no-ada] [--skip-build]
#                 [--build-dir DIR] [--verify] [--jobs N]
#
# Defaults: --out dist/pcl_pyramid_sdk, --config Release, Ada libs built when a
#           GNAT toolchain is on PATH, full configure+build, build dir
#           build-flatbuffers-only. --build-dir overrides the build/package dir
#           and is meant to be paired with --skip-build to reuse an existing,
#           already-configured build (it must have FlatBuffers + C++ bindings on).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PCL_ROOT="${REPO_ROOT}/subprojects/PCL"
BUILD_DIR="${REPO_ROOT}/build-flatbuffers-only"

OUT_DIR="${REPO_ROOT}/dist/pcl_pyramid_sdk"
CONFIG="Release"
WITH_ADA=1
SKIP_BUILD=0
VERIFY=0
JOBS="$( (command -v nproc >/dev/null && nproc) || echo 4)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out)        OUT_DIR="$2"; shift 2 ;;
    --build-dir)  BUILD_DIR="$2"; shift 2 ;;
    --config)     CONFIG="$2"; shift 2 ;;
    --no-ada)     WITH_ADA=0; shift ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --verify)     VERIFY=1; shift ;;
    --jobs)       JOBS="$2"; shift 2 ;;
    -h|--help)    sed -n '2,36p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[create_sdk] unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "${CONFIG}" in
  Release) BUILD_PRESET="flatbuffers-only-release" ;;
  Debug)   BUILD_PRESET="flatbuffers-only-debug" ;;
  *) echo "[create_sdk] ERROR: --config must be Release or Debug (got '${CONFIG}')" >&2; exit 2 ;;
esac

step() { echo; echo "==== [create_sdk] $* ===="; }

if [[ "${SKIP_BUILD}" -eq 1 ]]; then
  step "step 1-2/6: skipping configure+build (--skip-build); reusing ${BUILD_DIR}"
  [[ -d "${BUILD_DIR}" ]] || { echo "[create_sdk] ERROR: --skip-build but ${BUILD_DIR} does not exist." >&2; exit 1; }
else
  step "step 1/6: configure (cmake --preset flatbuffers-only)"
  cmake --preset flatbuffers-only

  step "step 2/6: build repo + C++ bindings (cmake --build --preset ${BUILD_PRESET})"
  cmake --build --preset "${BUILD_PRESET}" --parallel "${JOBS}"
fi

step "step 3/6: build codec/transport plugins (build_plugins.sh)"
"${SCRIPT_DIR}/build_plugins.sh" --build-dir "${BUILD_DIR}"

if [[ "${WITH_ADA}" -eq 1 ]]; then
  if command -v gnatmake >/dev/null 2>&1 || command -v gcc >/dev/null 2>&1; then
    step "step 4/6: build GNAT PCL static libs (Ada consumers)"
    "${PCL_ROOT}/scripts/build_gnat_pcl_static_libs.sh" "${BUILD_DIR}/ada_gnat_pcl" || \
      echo "[create_sdk] WARN: GNAT PCL lib build failed; package_sdk will retry / Ada may be unavailable."
  else
    echo "[create_sdk] step 4/6: no GNAT toolchain on PATH -- skipping Ada libs (use --no-ada to silence)."
  fi
else
  step "step 4/6: Ada libs skipped (--no-ada)"
fi

step "step 5/6: package the SDK (package_sdk.sh --clean --out ${OUT_DIR})"
"${SCRIPT_DIR}/package_sdk.sh" --build-dir "${BUILD_DIR}" --out "${OUT_DIR}" --clean

if [[ "${VERIFY}" -eq 1 ]]; then
  step "step 6/6: verify the packaged SDK end-to-end (build_sdk.sh)"
  "${OUT_DIR}/scripts/build_sdk.sh"
else
  echo "[create_sdk] step 6/6: verification skipped (pass --verify to build+smoke-test the package)."
fi

echo
echo "[create_sdk] Done. Offline SDK: ${OUT_DIR}"
echo "[create_sdk] Ship that directory; downstream builds it with scripts/build_sdk.sh."
