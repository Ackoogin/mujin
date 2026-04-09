#!/usr/bin/env bash
# test_generated_codec_dispatch.sh -- Regenerate bindings, rebuild codec
# dispatch test, and prove a non-JSON runtime codec selection works.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

FILTER="CodecDispatchE2E.FlatBuffersPubSubRoundTrip"
BUILD_DIR="$WORKSPACE_ROOT/build"
CONFIG="Release"
TEST_BIN=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --filter) FILTER="$2"; shift 2 ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --config) CONFIG="$2"; shift 2 ;;
    --test-bin) TEST_BIN="$2"; shift 2 ;;
    *)
      echo "[driver] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "$TEST_BIN" ]]; then
  if [[ -x "$BUILD_DIR/subprojects/PYRAMID/tests/test_codec_dispatch_e2e" ]]; then
    TEST_BIN="$BUILD_DIR/subprojects/PYRAMID/tests/test_codec_dispatch_e2e"
  else
    TEST_BIN="$BUILD_DIR/subprojects/PYRAMID/tests/$CONFIG/test_codec_dispatch_e2e"
  fi
fi

echo "=== Generated Codec Dispatch Test ==="
bash "$PYRAMID_ROOT/scripts/generate_bindings.sh" --cpp --backends "json,flatbuffers"

echo "[driver] Rebuilding test_codec_dispatch_e2e..."
cmake --build "$BUILD_DIR" --config "$CONFIG" --target test_codec_dispatch_e2e -j4

if [[ ! -x "$TEST_BIN" ]]; then
  echo "[driver] FAIL: test binary not found at $TEST_BIN" >&2
  exit 1
fi

echo "[driver] Running $FILTER..."
"$TEST_BIN" --gtest_filter="$FILTER"

echo "[driver] PASS: generated bindings rebuilt and non-JSON runtime dispatch succeeded"
