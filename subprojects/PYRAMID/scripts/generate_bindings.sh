#!/usr/bin/env bash
# generate_bindings.sh -- Wrapper around pim/generate_bindings.py.
#
# Usage:
#   scripts/generate_bindings.sh [--cpp] [--ada]
#                                [--backends LIST]
#                                [--proto-dir DIR]
#                                [--cpp-out DIR]
#                                [--ada-out DIR]
#
# If neither --cpp nor --ada is supplied, both languages are generated.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DO_CPP=0
DO_ADA=0
BACKENDS="json,flatbuffers"
PROTO_DIR="$PYRAMID_ROOT/proto"
CPP_OUT="$PYRAMID_ROOT/bindings/cpp/generated"
ADA_OUT="$PYRAMID_ROOT/bindings/ada/generated"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cpp) DO_CPP=1; shift ;;
    --ada) DO_ADA=1; shift ;;
    --backends) BACKENDS="$2"; shift 2 ;;
    --proto-dir) PROTO_DIR="$2"; shift 2 ;;
    --cpp-out) CPP_OUT="$2"; shift 2 ;;
    --ada-out) ADA_OUT="$2"; shift 2 ;;
    *)
      echo "[generate] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ $DO_CPP -eq 0 && $DO_ADA -eq 0 ]]; then
  DO_CPP=1
  DO_ADA=1
fi

PYTHON_BIN="$(command -v python3 || command -v python || true)"
if [[ -z "$PYTHON_BIN" ]]; then
  echo "[generate] FAIL: python not found" >&2
  exit 1
fi

GEN_SCRIPT="$PYRAMID_ROOT/pim/generate_bindings.py"
if [[ ! -f "$GEN_SCRIPT" ]]; then
  echo "[generate] FAIL: generator not found at $GEN_SCRIPT" >&2
  exit 1
fi

if [[ $DO_CPP -eq 1 ]]; then
  echo "[generate] C++ bindings -> $CPP_OUT"
  "$PYTHON_BIN" "$GEN_SCRIPT" "$PROTO_DIR" "$CPP_OUT" --languages cpp --backends "$BACKENDS"
fi

if [[ $DO_ADA -eq 1 ]]; then
  echo "[generate] Ada bindings -> $ADA_OUT"
  "$PYTHON_BIN" "$GEN_SCRIPT" "$PROTO_DIR" "$ADA_OUT" --languages ada --backends "$BACKENDS"
fi

echo "[generate] PASS"
