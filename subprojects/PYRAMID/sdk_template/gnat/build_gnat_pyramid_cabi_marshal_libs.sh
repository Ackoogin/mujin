#!/usr/bin/env bash
# build_gnat_pyramid_cabi_marshal_libs.sh -- SDK counterpart of the
# monorepo's subprojects/PYRAMID/scripts/build_gnat_pyramid_cabi_marshal_libs.sh.
#
# Compiles the GENERATED (proto-dependent) C-ABI marshalling sources into a
# GNAT-compatible static archive. Cannot be prebuilt by the SDK maintainer
# because its content depends on the user's own proto/ tree; run
# scripts/generate_bindings.sh first.
#
# Usage: gnat/build_gnat_pyramid_cabi_marshal_libs.sh [OUT_DIR] [--force]
# Default OUT_DIR: <sdk_root>/gnat/build
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

OUT_DIR="${1:-$SDK_ROOT/gnat/build}"
OBJ_DIR="$OUT_DIR/obj"
FORCE_REBUILD=0

if [[ $# -gt 0 ]]; then
  shift
fi
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force) FORCE_REBUILD="--force"; shift ;;
    *) shift ;;
  esac
done

LIB_NAME="pyramid_generated_cabi_marshal"
LIB_FILE="$OUT_DIR/lib${LIB_NAME}.a"
MANIFEST_FILE="$OUT_DIR/lib${LIB_NAME}.manifest"
RAND_SUFFIX="${RANDOM}${RANDOM}"
TMP_LIB_FILE="$OUT_DIR/lib${LIB_NAME}_${RAND_SUFFIX}.a"
TMP_MANIFEST_FILE="$OUT_DIR/lib${LIB_NAME}_${RAND_SUFFIX}.manifest"

GXX_PATH="$(command -v g++ 2>/dev/null || true)"
if [[ -z "$GXX_PATH" ]]; then
  echo "[sdk-ada] ERROR: g++ not found in PATH" >&2
  exit 1
fi
CXX="$GXX_PATH"
AR="$(command -v gcc-ar 2>/dev/null || true)"
if [[ -z "$AR" ]]; then
  AR="$(command -v ar 2>/dev/null || true)"
fi
if [[ -z "$AR" ]]; then
  echo "[sdk-ada] ERROR: ar not found in PATH" >&2
  exit 1
fi

echo "[sdk-ada] Compiler : $CXX"
echo "[sdk-ada] Archiver : $AR"

mkdir -p "$OBJ_DIR"

if [[ "$FORCE_REBUILD" == "--force" ]]; then
  rm -f "$LIB_FILE" "$MANIFEST_FILE" \
        "$OUT_DIR"/lib${LIB_NAME}_*.a \
        "$OUT_DIR"/lib${LIB_NAME}_*.manifest \
        "$OBJ_DIR"/*.o
fi

# NOTE: this archive covers to_c/from_c/_c_free only. Ada never calls the
# wire-codec (toJson/fromJson/toBinary/fromBinary) functions directly -- it
# has its own pure-Ada JSON codec, and wire encode/decode goes through a
# loaded codec plugin .so -- so wire-codec sources are not compiled here.
GEN_DIR="$SDK_ROOT/sdk_project/generated"
CORE_EXTERNAL_INCLUDE="$SDK_ROOT/include/external"
PCL_INCLUDE="$SDK_ROOT/include"

if [[ ! -d "$GEN_DIR" ]]; then
  echo "[sdk-ada] ERROR: no generated bindings at $GEN_DIR -- run scripts/generate_bindings first." >&2
  exit 1
fi

CXXFLAGS=(-std=c++17 -O2 -I"$GEN_DIR" -I"$CORE_EXTERNAL_INCLUDE" -I"$PCL_INCLUDE")

shopt -s nullglob
sources=(
  "$GEN_DIR"/pyramid_data_model_*_cabi_marshal.cpp
  "$GEN_DIR"/pyramid_components_*_cabi_marshal.cpp
)
deps=(
  "${sources[@]}"
  "$GEN_DIR"/pyramid_data_model_*_cabi_marshal.hpp
  "$GEN_DIR"/pyramid_components_*_cabi_marshal.hpp
)
shopt -u nullglob

if [[ "${#sources[@]}" -eq 0 ]]; then
  echo "[sdk-ada] ERROR: no generated C-ABI marshal sources found under $GEN_DIR" >&2
  exit 1
fi

file_fingerprint() {
  if stat -c '%n|%s|%Y' "$1" >/dev/null 2>&1; then
    stat -c '%n|%s|%Y' "$1"
  else
    stat -f '%N|%z|%m' "$1"
  fi
}

{
  printf 'CXX=%s\n' "$CXX"
  printf 'AR=%s\n' "$AR"
  printf 'CXXFLAGS=%s\n' "${CXXFLAGS[*]}"
  for dep in "${deps[@]}"; do
    [[ -f "$dep" ]] && file_fingerprint "$dep"
  done
} > "$TMP_MANIFEST_FILE"

need_rebuild=0
if [[ "$FORCE_REBUILD" == "--force" || ! -f "$LIB_FILE" || ! -f "$MANIFEST_FILE" ]]; then
  need_rebuild=1
elif ! cmp -s "$MANIFEST_FILE" "$TMP_MANIFEST_FILE"; then
  need_rebuild=1
fi

if [[ "$need_rebuild" -eq 0 ]]; then
  echo "[sdk-ada] GNAT C-ABI marshal archive is up to date in $OUT_DIR"
  rm -f "$TMP_MANIFEST_FILE"
else
  echo "[sdk-ada] Building GNAT-compatible C-ABI marshal archive in $OUT_DIR"

  object_files=()
  for src in "${sources[@]}"; do
    obj="$OBJ_DIR/$(basename "${src%.cpp}").o"
    echo "[sdk-ada]   compiling $(basename "$src")"
    "$CXX" "${CXXFLAGS[@]}" -c "$src" -o "$obj"
    object_files+=("$obj")
  done

  rm -f "$TMP_LIB_FILE"
  "$AR" rcs "$TMP_LIB_FILE" "${object_files[@]}"

  rm -f "$LIB_FILE"
  mv "$TMP_LIB_FILE" "$LIB_FILE"
  mv "$TMP_MANIFEST_FILE" "$MANIFEST_FILE"
fi

LIBSTDCXX="$("$CXX" -print-file-name=libstdc++.a 2>/dev/null || true)"
if [[ -n "$LIBSTDCXX" && "$LIBSTDCXX" != "libstdc++.a" && -f "$LIBSTDCXX" ]]; then
  echo "[sdk-ada] Copying libstdc++ from $LIBSTDCXX to $OUT_DIR"
  cp -f "$LIBSTDCXX" "$OUT_DIR/"
  LIBSTDCXX_DIR="$(dirname "$LIBSTDCXX")"
  [[ -f "$LIBSTDCXX_DIR/libstdc++.dll.a" ]] && cp -f "$LIBSTDCXX_DIR/libstdc++.dll.a" "$OUT_DIR/" || true
else
  echo "[sdk-ada] WARNING: libstdc++.a not found via '$CXX -print-file-name'; link may fail with cannot find -lstdc++"
fi

echo "[sdk-ada] Ready:"
echo "[sdk-ada]   $LIB_FILE"
