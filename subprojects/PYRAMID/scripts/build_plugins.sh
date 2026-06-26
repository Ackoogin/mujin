#!/usr/bin/env bash
#
# build_plugins.sh -- end-to-end ".proto -> runtime plugin .so" build.
#
# Regenerates the C++ bindings from the .proto data model and builds every
# runtime codec/transport plugin via the CMake `pyramid_plugins` target. Suitable
# as a CI/CD stage or a manual engineer step.
#
# Pipeline:
#
#   proto/**.proto
#     |                                  (CMake configure, PYRAMID_GENERATE_CPP_BINDINGS=ON)
#     v
#   generated C++ bindings  ── pim/generate_bindings.py
#     |                                  (C++ compile -> shared modules)
#     v
#   plugins (.so):
#     libpyramid_codec_json_<component>.so
#     libpyramid_codec_flatbuffers_<component>.so
#     libpcl_transport_socket_plugin.so
#     libpcl_transport_shared_memory_plugin.so
#     [libpyramid_grpc_coupled_plugin.so]   (with --grpc)
#
#   optionally:  stage per-component deployment dirs (stage_plugin_deploy.sh)
#
# The codec plugin is the single cross-language .so (it consumes the frozen
# pyramid_<Type>_c C struct) and is loaded from both C++ and Ada clients.
#
# Usage:
#   build_plugins.sh [--build-dir DIR] [--grpc] [--jobs N] [--clean]
#                    [--stage] [--stage-out DIR]
#
# Options:
#   --build-dir DIR  CMake build directory          (default: <repo>/build-plugins)
#   --grpc           also build protobuf + the coupled gRPC target plugin
#                    (fetches gRPC from source on first configure -- needs network)
#   --jobs N         parallel build jobs            (default: nproc)
#   --clean          delete the build dir before configuring
#   --stage          after building, stage per-component deployment dirs
#   --stage-out DIR  staging output dir             (default: <repo>/dist/plugin_deploy)
#
# CI example:
#   subprojects/PYRAMID/scripts/build_plugins.sh --grpc --stage
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

BUILD_DIR="${REPO_ROOT}/build-plugins"
STAGE_OUT="${REPO_ROOT}/dist/plugin_deploy"
ENABLE_GRPC=0
DO_STAGE=0
CLEAN=0
JOBS="$( (command -v nproc >/dev/null && nproc) || echo 4)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --grpc)      ENABLE_GRPC=1; shift ;;
    --jobs)      JOBS="$2"; shift 2 ;;
    --clean)     CLEAN=1; shift ;;
    --stage)     DO_STAGE=1; shift ;;
    --stage-out) STAGE_OUT="$2"; shift 2 ;;
    -h|--help)   sed -n '2,46p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[build_plugins] unknown arg: $1" >&2; exit 2 ;;
  esac
done

echo "[build_plugins] repo      : ${REPO_ROOT}"
echo "[build_plugins] build-dir : ${BUILD_DIR}"
echo "[build_plugins] grpc      : $([[ ${ENABLE_GRPC} -eq 1 ]] && echo on || echo off)"
echo "[build_plugins] jobs      : ${JOBS}"

if [[ ${CLEAN} -eq 1 ]]; then
  echo "[build_plugins] cleaning ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

# Lean, plugin-focused configuration:
#   - PYRAMID only (no AME), no Foxglove, no ROS2
#   - FlatBuffers + JSON codecs always; PYRAMID_GENERATE_CPP_BINDINGS regenerates
#     the C++ bindings from .proto so this really is "proto -> plugins"
#   - --grpc adds protobuf + the coupled gRPC target plugin
CMAKE_ARGS=(
  -S "${REPO_ROOT}" -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE=Release
  -DUNMANNED_BUILD_PYRAMID=ON
  -DUNMANNED_BUILD_AME=OFF
  -DAME_FOXGLOVE=OFF
  -DPYRAMID_ENABLE_ROS2=OFF
  -DPYRAMID_ENABLE_FLATBUFFERS=ON
  -DPYRAMID_GENERATE_CPP_BINDINGS=ON
)
if [[ ${ENABLE_GRPC} -eq 1 ]]; then
  CMAKE_ARGS+=(-DPYRAMID_ENABLE_PROTOBUF=ON -DPYRAMID_ENABLE_GRPC=ON)
else
  CMAKE_ARGS+=(-DPYRAMID_ENABLE_PROTOBUF=OFF -DPYRAMID_ENABLE_GRPC=OFF)
fi

echo "[build_plugins] configure ..."
cmake "${CMAKE_ARGS[@]}"

echo "[build_plugins] build pyramid_plugins ..."
cmake --build "${BUILD_DIR}" --target pyramid_plugins --parallel "${JOBS}"

echo
echo "[build_plugins] produced plugins:"
# Codec plugins are named libpyramid_codec_<codec>_<component>.so (no "plugin"
# in the name); transport/coupled plugins carry "_plugin".
find "${BUILD_DIR}/subprojects" \
     \( -name 'libpyramid_codec_*.so' \
        -o -name 'libpcl_transport_*_plugin.so' \
        -o -name 'libpyramid_grpc_coupled_plugin.so' \) \
     2>/dev/null | sort | sed "s|${BUILD_DIR}/|  |"

if [[ ${DO_STAGE} -eq 1 ]]; then
  echo
  echo "[build_plugins] staging deployment -> ${STAGE_OUT}"
  "${SCRIPT_DIR}/stage_plugin_deploy.sh" --build-dir "${BUILD_DIR}" --out "${STAGE_OUT}" --clean
fi

echo
echo "[build_plugins] PASS"
