#!/usr/bin/env bash
# Generate source statement coverage report for PCL (PYRAMID Container Library).
# Requires: GCC or Clang with gcov, gcovr (pip install gcovr)
#
# Output: coverage_pcl/ (HTML and txt summary)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

BUILD_DIR="build-coverage-pcl"
COV_OUT="coverage_pcl"

if ! command -v gcovr &>/dev/null; then
  echo "ERROR: gcovr not found. Install with: pip install gcovr"
  exit 1
fi

echo "=== Configuring (coverage build) ==="
cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DAME_FOXGLOVE=OFF \
  -DCMAKE_C_FLAGS="--coverage -O0" \
  -DCMAKE_CXX_FLAGS="--coverage -O0" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"

echo ""
echo "=== Building PCL tests ==="
cmake --build "$BUILD_DIR" -j"$(nproc)" \
  --target test_pcl_lifecycle test_pcl_executor test_pcl_log test_pcl_robustness test_pcl_dining

echo ""
echo "=== Running tests ==="
"$BUILD_DIR/tests/test_pcl_lifecycle" || true
"$BUILD_DIR/tests/test_pcl_executor" || true
"$BUILD_DIR/tests/test_pcl_log" || true
"$BUILD_DIR/tests/test_pcl_robustness" || true
"$BUILD_DIR/tests/test_pcl_dining" || true

echo ""
echo "=== Generating coverage report ==="
mkdir -p "$COV_OUT"
gcovr --root . \
  --filter "src/pcl/.*" \
  --object-directory "$BUILD_DIR" \
  --gcov-ignore-errors source_not_found \
  --gcov-ignore-errors no_working_dir_found \
  --gcov-ignore-parse-errors negative_hits.warn \
  --html-details="$COV_OUT/index.html" \
  --txt="$COV_OUT/summary.txt"

echo ""
echo "Coverage report: $COV_OUT/index.html"
echo "Summary: $COV_OUT/summary.txt"
