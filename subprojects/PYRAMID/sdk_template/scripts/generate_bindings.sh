#!/usr/bin/env bash
# generate_bindings.sh -- proto -> generated bindings, offline SDK edition.
#
# Wraps generator/generate_bindings.py against the SDK's own proto/ tree.
# No network access, no parent monorepo required.
#
# Usage:
#   scripts/generate_bindings.sh [--cpp] [--ada] [--backends LIST]
#                                 [--proto-dir DIR] [--cpp-out DIR] [--ada-out DIR]
#
# If neither --cpp nor --ada is supplied, both languages are generated.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DO_CPP=0
DO_ADA=0
BACKENDS="json,flatbuffers"
PROTO_DIR=""
CPP_OUT=""
ADA_OUT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cpp) DO_CPP=1; shift ;;
    --ada) DO_ADA=1; shift ;;
    --backends) BACKENDS="$2"; shift 2 ;;
    --proto-dir) PROTO_DIR="$2"; shift 2 ;;
    --cpp-out) CPP_OUT="$2"; shift 2 ;;
    --ada-out) ADA_OUT="$2"; shift 2 ;;
    *) echo "[generate] Unknown argument: $1" >&2; exit 1 ;;
  esac
done

if [[ "$DO_CPP" -eq 0 && "$DO_ADA" -eq 0 ]]; then
  DO_CPP=1
  DO_ADA=1
fi

[[ -z "$PROTO_DIR" ]] && PROTO_DIR="$SDK_ROOT/proto"
[[ -z "$CPP_OUT" ]] && CPP_OUT="$SDK_ROOT/sdk_project/generated"
[[ -z "$ADA_OUT" ]] && ADA_OUT="$SDK_ROOT/gnat/generated_ada"

PY_CMD="$(command -v python3 || command -v python || true)"
if [[ -z "$PY_CMD" ]]; then
  echo "[generate] FAIL: python3 not found on PATH" >&2
  exit 1
fi

GEN_SCRIPT="$SDK_ROOT/generator/generate_bindings.py"
if [[ ! -f "$GEN_SCRIPT" ]]; then
  echo "[generate] FAIL: generator not found at $GEN_SCRIPT" >&2
  exit 1
fi
if [[ ! -d "$PROTO_DIR" ]]; then
  echo "[generate] FAIL: proto dir not found at $PROTO_DIR" >&2
  exit 1
fi

if [[ "$DO_CPP" -eq 1 ]]; then
  echo "[generate] C++ bindings -> $CPP_OUT"
  "$PY_CMD" "$GEN_SCRIPT" "$PROTO_DIR" "$CPP_OUT" --languages cpp --backends "$BACKENDS"
fi

if [[ "$DO_ADA" -eq 1 ]]; then
  echo "[generate] Ada bindings -> $ADA_OUT"
  "$PY_CMD" "$GEN_SCRIPT" "$PROTO_DIR" "$ADA_OUT" --languages ada --backends "$BACKENDS"
fi

echo "[generate] PASS"
