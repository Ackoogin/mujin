#!/usr/bin/env bash
# build_ada.sh -- build the generated Ada C-ABI marshal archive, offline SDK edition.
#
# Runs gnat/build_gnat_pyramid_cabi_marshal_libs.sh (compiles the
# proto-dependent C-ABI marshalling sources into a GNAT-compatible archive;
# Ada depends on PCL + this C-ABI layer only, never codec-specific content)
# then leaves gprbuild to the caller's own client project, which should
# `with` gnat/pyramid_sdk_ada.gpr.
#
# Prerequisite: scripts/generate_bindings.sh --ada (or no-arg, which does both).
#
# Usage: build_ada.sh [--force]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if ! command -v gprbuild >/dev/null 2>&1; then
  echo "[build_ada] FAIL: gprbuild not found on PATH (Ada builds need a GNAT toolchain)." >&2
  exit 1
fi

if [[ ! -d "$SDK_ROOT/gnat/generated_ada" ]]; then
  echo "[build_ada] FAIL: no generated Ada bindings at $SDK_ROOT/gnat/generated_ada -- run scripts/generate_bindings.sh first." >&2
  exit 1
fi

echo "[build_ada] building GNAT-compatible C-ABI marshal archive ..."
"$SDK_ROOT/gnat/build_gnat_pyramid_cabi_marshal_libs.sh" "$@"

echo
echo "[build_ada] Ready. Build your own client with:"
echo "[build_ada]   gprbuild -P your_client.gpr"
echo "[build_ada] where your_client.gpr contains:"
echo "[build_ada]   with \"$SDK_ROOT/gnat/pyramid_sdk_ada.gpr\";"
echo
echo "[build_ada] PASS"
