#!/usr/bin/env bash
#
# build_ros2_transport.sh -- build the PYRAMID ROS2 runtime transport package
# with colcon (Linux / ament; the counterpart to build_ros2_transport.bat).
#
# This also builds the coupled ROS2 target plugin (libpyramid_ros2_coupled_plugin),
# a single MODULE .so exposing both a PCL transport vtable and a codec vtable under
# application/ros2. It is part of the ament package, so colcon builds it
# automatically.
#
# Pipeline (no committed generated files -- cf. WIP §2.G.2):
#
#   proto/**.proto
#     |                          (CMake configure, PYRAMID_ENABLE_ROS2=ON)
#     v
#   <host-build>/generated/pyramid_cpp_bindings/ros2/cpp/**   (build tree)
#     |                          (colcon + ament; PYRAMID_CPP_BINDINGS_DIR points here)
#     v
#   install/pyramid_ros2_transport/lib/libpyramid_ros2_coupled_plugin.so
#
# The host build (plain CMake) generates the C++ bindings + the ROS2 transport
# support layer and builds pcl_core; colcon then compiles the ament package
# against those, so nothing generated needs to be committed.
#
# Prerequisites: a sourced ROS2 (e.g. /opt/ros/humble/setup.bash), colcon, and a
# C++ toolchain. gprbuild/GNAT are NOT required (this is the C++/ROS2 path).
#
# Usage:
#   build_ros2_transport.sh [--host-build-dir DIR] [--ros2-setup PATH]
#                           [--jobs N] [--test] [--skip-host]
#
# Options:
#   --host-build-dir DIR  CMake host build dir       (default: <repo>/build-ros2)
#   --ros2-setup PATH     ROS2 setup.bash to source  (default: $ROS2_SETUP or
#                                                      /opt/ros/humble/setup.bash)
#   --jobs N              parallel build jobs         (default: nproc)
#   --test                run `colcon test` after building
#   --skip-host           reuse an existing host build (skip the CMake step)
#
# CI example:
#   subprojects/PYRAMID/scripts/build_ros2_transport.sh --test
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYRAMID_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PYRAMID_ROOT}/../.." && pwd)"

HOST_BUILD_DIR="${REPO_ROOT}/build-ros2"
ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
JOBS="$(nproc 2>/dev/null || echo 4)"
DO_TEST=0
SKIP_HOST=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host-build-dir) HOST_BUILD_DIR="$2"; shift 2 ;;
    --ros2-setup)     ROS2_SETUP="$2"; shift 2 ;;
    --jobs)           JOBS="$2"; shift 2 ;;
    --test)           DO_TEST=1; shift ;;
    --skip-host)      SKIP_HOST=1; shift ;;
    *) echo "[build_ros2] unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ ! -f "${ROS2_SETUP}" ]]; then
  echo "[build_ros2] ROS2 setup not found: ${ROS2_SETUP}" >&2
  echo "             Pass --ros2-setup PATH or set ROS2_SETUP." >&2
  exit 1
fi

BINDINGS_DIR="${HOST_BUILD_DIR}/generated/pyramid_cpp_bindings"
PCL_CORE_LIB="${HOST_BUILD_DIR}/subprojects/PCL/src/libpcl_core.a"
AMENT_BASE="${REPO_ROOT}/build-ros2-ament"

if [[ "${SKIP_HOST}" -eq 0 ]]; then
  echo "=== Step 1: host build (generate bindings + pcl_core + ros2 support) ==="
  # Disable ament discovery for the host build: this step is plain CMake and only
  # needs the generated support + pcl_core, not the ROS2 toolchain.
  cmake -S "${REPO_ROOT}" -B "${HOST_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYRAMID_ENABLE_ROS2=ON \
    -DPYRAMID_GENERATE_CPP_BINDINGS=ON \
    -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
  cmake --build "${HOST_BUILD_DIR}" --target pcl_core pyramid_ros2_transport -j"${JOBS}"
fi

if [[ ! -f "${PCL_CORE_LIB}" ]]; then
  echo "[build_ros2] missing ${PCL_CORE_LIB}; run without --skip-host first." >&2
  exit 1
fi

# The ROS2 setup scripts reference unbound vars (e.g. AMENT_TRACE_SETUP_FILES),
# so relax nounset just for the source.
set +u
# shellcheck disable=SC1090
source "${ROS2_SETUP}"
set -u
echo "[build_ros2] using ROS_DISTRO=${ROS_DISTRO:-?}"

echo "=== Step 2: colcon build pyramid_ros2_transport ==="
colcon build --packages-select pyramid_ros2_transport \
  --base-paths "${PYRAMID_ROOT}/ros2" \
  --build-base "${AMENT_BASE}/build" \
  --install-base "${AMENT_BASE}/install" \
  --cmake-args \
    "-DUNMANNED_ROOT=${REPO_ROOT}" \
    "-DPCL_CORE_LIB=${PCL_CORE_LIB}" \
    "-DPCL_CORE_INCLUDE_DIR=${REPO_ROOT}/subprojects/PCL/include" \
    "-DPYRAMID_CPP_BINDINGS_DIR=${BINDINGS_DIR}"

if [[ "${DO_TEST}" -eq 1 ]]; then
  echo "=== Step 3: colcon test ==="
  colcon test --packages-select pyramid_ros2_transport \
    --base-paths "${PYRAMID_ROOT}/ros2" \
    --build-base "${AMENT_BASE}/build" \
    --install-base "${AMENT_BASE}/install"
  colcon test-result --test-result-base "${AMENT_BASE}/build" --all
fi

echo "=== Build complete ==="
echo "Coupled ROS2 plugin: ${AMENT_BASE}/install/pyramid_ros2_transport/lib/libpyramid_ros2_coupled_plugin.so"
