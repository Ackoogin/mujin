#!/usr/bin/env bash
# Generate source statement coverage report for PCL (PYRAMID Composition Library).
# Requires: GCC or Clang with gcov, gcovr (pip install gcovr)
#
# Output: coverage_pcl/ (HTML and txt summary)
#
# Coverage target: 100% of reachable statements. Lines that cannot be exercised
# under normal testing (allocation failures, OS/kernel faults, adversarial peer
# timing, defensive preconditions) are marked with GCOVR_EXCL_LINE/START/STOP
# in the source, with a comment justifying the exclusion.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
cd "$ROOT"

BUILD_DIR="build-coverage-pcl"
COV_OUT="coverage_pcl"
FAIL_UNDER=100

if ! command -v gcovr &>/dev/null; then
  echo "ERROR: gcovr not found. Install with: pip install gcovr"
  exit 1
fi

echo "=== Configuring (coverage build, PCL only) ==="
cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DUNMANNED_BUILD_PYRAMID=OFF \
  -DUNMANNED_BUILD_AME=OFF \
  -DPYRAMID_ENABLE_FLATBUFFERS=OFF \
  -DCMAKE_C_FLAGS="--coverage -O0 -g" \
  -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
  -DCMAKE_SHARED_LINKER_FLAGS="--coverage" \
  -DCMAKE_MODULE_LINKER_FLAGS="--coverage"

echo ""
echo "=== Building all PCL tests ==="
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo ""
echo "=== Running tests (serially: parallel runs corrupt shared .gcda files) ==="
find "$BUILD_DIR" -name '*.gcda' -delete
(cd "$BUILD_DIR" && ctest --output-on-failure -j1)

echo ""
echo "=== Generating coverage report ==="
mkdir -p "$COV_OUT"
gcovr --root . \
  --filter "subprojects/PCL/src/.*" \
  --filter "subprojects/PCL/include/.*" \
  --filter "subprojects/PCL/bindings/apos/.*" \
  --object-directory "$BUILD_DIR" \
  --gcov-ignore-errors source_not_found \
  --gcov-ignore-errors no_working_dir_found \
  --gcov-ignore-parse-errors negative_hits.warn \
  --html-details="$COV_OUT/index.html" \
  --txt="$COV_OUT/summary.txt" \
  --fail-under-line "$FAIL_UNDER"

echo ""
echo "Coverage report: $COV_OUT/index.html"
echo "Summary: $COV_OUT/summary.txt"
