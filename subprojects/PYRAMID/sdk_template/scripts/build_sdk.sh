#!/usr/bin/env bash
#
# build_sdk.sh -- one command to turn proto/ into runnable codec plugins.
#
# The downstream end-to-end for this offline SDK. Chains the individual steps
# from README.md ("Downstream: verify the distribution" / "build your
# contracts") into a single command:
#
#   1. scripts/generate_bindings.sh          (proto/ -> generated bindings)
#   2. scripts/build_plugins.sh [--smoke-tests]
#                                            (build codec plugin .so's; run
#                                             the SDK smoke tests by default)
#   3. [--ada] scripts/build_ada.sh          (build the C-ABI marshal archive
#                                             for Ada clients)
#
# No network access and no parent monorepo are required. Edit proto/ first (or
# leave the starter contracts to try the toolchain), then run this.
#
# Usage:
#   build_sdk.sh [--no-smoke] [--ada] [--clean] [--config Release|Debug]
#                [--jobs N] [--cpp-only|--ada-only]
#
# Defaults: generate both C++ and Ada bindings, build C++ codec plugins, run
#           the smoke tests, skip the Ada archive build (add --ada).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

RUN_SMOKE=1
BUILD_ADA=0
CLEAN=0
CONFIG="Release"
JOBS="$( (command -v nproc >/dev/null && nproc) || echo 4)"
GEN_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-smoke)  RUN_SMOKE=0; shift ;;
    --ada)       BUILD_ADA=1; shift ;;
    --clean)     CLEAN=1; shift ;;
    --config)    CONFIG="$2"; shift 2 ;;
    --jobs)      JOBS="$2"; shift 2 ;;
    --cpp-only)  GEN_ARGS=(--cpp); shift ;;
    --ada-only)  GEN_ARGS=(--ada); BUILD_ADA=1; shift ;;
    -h|--help)   sed -n '2,29p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[build_sdk] unknown arg: $1" >&2; exit 2 ;;
  esac
done

step() { echo; echo "==== [build_sdk] $* ===="; }

step "step 1/3: generate bindings from proto/"
"${SCRIPT_DIR}/generate_bindings.sh" "${GEN_ARGS[@]}"

# --ada-only skips the C++ plugin build entirely.
if [[ "${#GEN_ARGS[@]}" -gt 0 && "${GEN_ARGS[0]}" == "--ada" ]]; then
  echo "[build_sdk] step 2/3: C++ plugin build skipped (--ada-only)."
else
  PLUGIN_ARGS=(--build-dir "${SDK_ROOT}/build" --config "${CONFIG}" --jobs "${JOBS}")
  [[ "${CLEAN}" -eq 1 ]] && PLUGIN_ARGS+=(--clean)
  [[ "${RUN_SMOKE}" -eq 1 ]] && PLUGIN_ARGS+=(--smoke-tests)
  step "step 2/3: build C++ codec plugins$([[ "${RUN_SMOKE}" -eq 1 ]] && echo ' + smoke tests')"
  "${SCRIPT_DIR}/build_plugins.sh" "${PLUGIN_ARGS[@]}"
fi

if [[ "${BUILD_ADA}" -eq 1 ]]; then
  step "step 3/3: build Ada C-ABI marshal archive"
  "${SCRIPT_DIR}/build_ada.sh"
else
  echo "[build_sdk] step 3/3: Ada archive skipped (add --ada to build it)."
fi

echo
echo "[build_sdk] Done. Codec plugins are under ${SDK_ROOT}/build."
echo "[build_sdk] Load them alongside plugins/ per README.md 'Running against the plugins'."
