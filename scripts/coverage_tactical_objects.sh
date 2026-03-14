#!/usr/bin/env bash
# Generate source statement coverage report for tactical_objects module.
# Requires: GCC or Clang with gcov, gcovr (pip install gcovr)
#
# Output: coverage_tactical_objects/ (HTML and txt summary)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

BUILD_DIR="build-coverage-tactical_objects"
COV_OUT="coverage_tactical_objects"

if ! command -v gcovr &>/dev/null; then
  echo "ERROR: gcovr not found. Install with: pip install gcovr"
  exit 1
fi

echo "=== Configuring (coverage build) ==="
cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DAME_FOXGLOVE=OFF \
  -DCMAKE_C_FLAGS="--coverage -O0" \
  -DCMAKE_CXX_FLAGS="--coverage -O0 -fno-elide-constructors" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"

echo ""
echo "=== Building tactical_objects tests ==="
cmake --build "$BUILD_DIR" -j"$(nproc)" \
  --target test_tobj_types test_tobj_object_store test_tobj_milclass test_tobj_spatial \
  test_tobj_zone test_tobj_correlation test_tobj_query test_tobj_relationship \
  test_tobj_codec test_tobj_interest test_tobj_history test_tobj_runtime \
  test_tobj_zone_perf test_tobj_component test_tobj_component_robustness test_tobj_component_hlr \
  test_tobj_streaming_codec test_tobj_runtime_streaming test_tobj_interest_matching

echo ""
echo "=== Running tests ==="
for t in test_tobj_types test_tobj_object_store test_tobj_milclass test_tobj_spatial \
  test_tobj_zone test_tobj_correlation test_tobj_query test_tobj_relationship \
  test_tobj_codec test_tobj_interest test_tobj_history test_tobj_runtime \
  test_tobj_zone_perf test_tobj_component test_tobj_component_robustness test_tobj_component_hlr \
  test_tobj_streaming_codec test_tobj_runtime_streaming test_tobj_interest_matching; do
  [ -x "$BUILD_DIR/tests/$t" ] && "$BUILD_DIR/tests/$t" || true
done

echo ""
echo "=== Generating coverage report ==="
mkdir -p "$COV_OUT"
gcovr --root . \
  --filter "pyramid/tactical_objects/src/.*" \
  --object-directory "$BUILD_DIR" \
  --gcov-ignore-errors source_not_found \
  --gcov-ignore-errors no_working_dir_found \
  --gcov-ignore-parse-errors negative_hits.warn \
  --html-details="$COV_OUT/index.html" \
  --txt="$COV_OUT/summary.txt"

echo ""
echo "Coverage report: $COV_OUT/index.html"
echo "Summary: $COV_OUT/summary.txt"
