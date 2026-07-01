#!/usr/bin/env bash
# build_plugins.sh -- build codec plugin .so's from generated bindings.
#
# Standalone: configures/builds sdk_project (no FetchContent, no network, no
# parent monorepo). Run scripts/generate_bindings.sh first.
#
# Usage:
#   build_plugins.sh [--build-dir DIR] [--config Release|Debug] [--clean] [--jobs N]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_DIR="$SDK_ROOT/build"
CONFIG="Release"
CLEAN=0
JOBS="$( (command -v nproc >/dev/null && nproc) || echo 4)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --config)    CONFIG="$2"; shift 2 ;;
    --clean)     CLEAN=1; shift ;;
    --jobs)      JOBS="$2"; shift 2 ;;
    *) echo "[build_plugins] unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ ! -d "$SDK_ROOT/sdk_project/generated" ]]; then
  echo "[build_plugins] FAIL: no generated bindings at $SDK_ROOT/sdk_project/generated -- run scripts/generate_bindings.sh first." >&2
  exit 1
fi

if [[ "$CLEAN" -eq 1 ]]; then
  echo "[build_plugins] cleaning $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi

echo "[build_plugins] configure ..."
cmake -S "$SDK_ROOT/sdk_project" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$CONFIG"

echo "[build_plugins] build ..."
cmake --build "$BUILD_DIR" --target pyramid_sdk_plugins --parallel "$JOBS"

echo
echo "[build_plugins] produced plugins:"
find "$BUILD_DIR" -name 'pyramid_codec_*.so' -o -name 'pyramid_codec_*.dll' 2>/dev/null | sed "s|$BUILD_DIR/|  |" || true

echo
echo "[build_plugins] PASS"
