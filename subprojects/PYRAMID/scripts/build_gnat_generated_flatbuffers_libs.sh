#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

OUT_DIR="${1:-$WORKSPACE_ROOT/build/ada_gnat_pyramid}"
OBJ_DIR="$OUT_DIR/obj"
FORCE_REBUILD=0
BUILD_DIR="${PYRAMID_BUILD_DIR:-$WORKSPACE_ROOT/build}"

if [[ $# -gt 0 ]]; then
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --force)
      FORCE_REBUILD="--force"
      shift
      ;;
    --build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    --config)
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

LIB_NAME="pyramid_generated_flatbuffers_codec"
LIB_FILE="$OUT_DIR/lib${LIB_NAME}.a"
MANIFEST_FILE="$OUT_DIR/lib${LIB_NAME}.manifest"
RAND_SUFFIX="${RANDOM}${RANDOM}"
TMP_LIB_FILE="$OUT_DIR/lib${LIB_NAME}_${RAND_SUFFIX}.a"
TMP_MANIFEST_FILE="$OUT_DIR/lib${LIB_NAME}_${RAND_SUFFIX}.manifest"

# Use g++ and all companion tools (ar) from the same bin directory so that the
# compiler and archiver are always from the same toolchain and produce
# mutually-compatible object/archive formats.
GXX_PATH="$(command -v g++ 2>/dev/null || true)"
if [[ -z "$GXX_PATH" ]]; then
  echo "[ada-pyramid] ERROR: g++ not found in PATH" >&2
  exit 1
fi
GXX_DIR="$(dirname "$GXX_PATH")"

CXX="$GXX_DIR/g++"
AR="$(command -v gcc-ar 2>/dev/null || true)"
if [[ -z "$AR" ]]; then
  AR="$(command -v ar 2>/dev/null || true)"
fi
if [[ -z "$AR" ]]; then
  echo "[ada-pyramid] ERROR: ar not found in PATH" >&2
  exit 1
fi

echo "[ada-pyramid] Compiler : $CXX"
echo "[ada-pyramid] Archiver : $AR"

mkdir -p "$OBJ_DIR"

if [[ "$FORCE_REBUILD" == "--force" ]]; then
  rm -f "$LIB_FILE" "$MANIFEST_FILE" \
        "$OUT_DIR"/lib${LIB_NAME}_*.a \
        "$OUT_DIR"/lib${LIB_NAME}_*.manifest \
        "$OBJ_DIR"/*.o
fi

GEN_DIR="$PYRAMID_ROOT/bindings/cpp/generated"
GEN_FB_DIR="$GEN_DIR/flatbuffers/cpp"
BUILD_FB_DIR="$BUILD_DIR/generated/flatbuffers/cpp"
FLATBUFFERS_INCLUDE="$BUILD_DIR/_deps/flatbuffers-src/include"
NLOHMANN_INCLUDE="${NLOHMANN_JSON_INCLUDE:-$BUILD_DIR/_deps/nlohmann_json-src/include}"
if [[ ! -d "$NLOHMANN_INCLUDE" ]]; then
  NLOHMANN_INCLUDE="$PYRAMID_ROOT/core/external"
fi

if [[ ! -d "$BUILD_FB_DIR" ]]; then
  if ! command -v cmake >/dev/null 2>&1; then
    echo "[ada-pyramid] ERROR: missing flatc-generated headers and cmake not available" >&2
    exit 1
  fi
  echo "[ada-pyramid] Refreshing flatc-generated headers..."
  cmake --build "$BUILD_DIR" --config Release --target pyramid_flatbuffers_codegen -j4
fi

CXXFLAGS=(
  -std=c++17 -O2
  -I"$GEN_DIR"
  -I"$GEN_FB_DIR"
  -I"$BUILD_FB_DIR"
  -I"$FLATBUFFERS_INCLUDE"
  -I"$NLOHMANN_INCLUDE"
)

shopt -s nullglob
sources=(
  "$GEN_DIR"/pyramid_data_model_*_codec.cpp
  "$GEN_FB_DIR"/*_flatbuffers_codec.cpp
)
deps=(
  "${sources[@]}"
  "$GEN_DIR"/pyramid_data_model_*_codec.hpp
  "$GEN_FB_DIR"/*_flatbuffers_codec.hpp
  "$BUILD_FB_DIR"/*_generated.h
)
shopt -u nullglob

if [[ "${#sources[@]}" -eq 0 ]]; then
  echo "[ada-pyramid] ERROR: no generated codec sources found" >&2
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
  echo "[ada-pyramid] GNAT FlatBuffers archive is up to date in $OUT_DIR"
  rm -f "$TMP_MANIFEST_FILE"
else
  echo "[ada-pyramid] Building GNAT-compatible generated FlatBuffers archive in $OUT_DIR"

  object_files=()
  for src in "${sources[@]}"; do
    obj="$OBJ_DIR/$(basename "${src%.cpp}").o"
    echo "[ada-pyramid]   compiling $(basename "$src")"
    "$CXX" "${CXXFLAGS[@]}" -c "$src" -o "$obj"
    object_files+=("$obj")
  done

  rm -f "$TMP_LIB_FILE"

  "$AR" rcs "$TMP_LIB_FILE" "${object_files[@]}"

  rm -f "$LIB_FILE"
  mv "$TMP_LIB_FILE" "$LIB_FILE"
  mv "$TMP_MANIFEST_FILE" "$MANIFEST_FILE"
fi

# OUT_DIR is on GNAT's linker -L path but GNAT's ld doesn't know about the
# g++ installation's lib directory.  Copy libstdc++.a from g++'s own lib into
# OUT_DIR so -lstdc++ resolves there.
LIBSTDCXX="$("$CXX" -print-file-name=libstdc++.a 2>/dev/null || true)"
if [[ -n "$LIBSTDCXX" && "$LIBSTDCXX" != "libstdc++.a" && -f "$LIBSTDCXX" ]]; then
  echo "[ada-pyramid] Copying libstdc++ from $LIBSTDCXX to $OUT_DIR"
  cp -f "$LIBSTDCXX" "$OUT_DIR/"
  LIBSTDCXX_DIR="$(dirname "$LIBSTDCXX")"
  [[ -f "$LIBSTDCXX_DIR/libstdc++.dll.a" ]] && cp -f "$LIBSTDCXX_DIR/libstdc++.dll.a" "$OUT_DIR/" || true
else
  echo "[ada-pyramid] WARNING: libstdc++.a not found via '$CXX -print-file-name'; link may fail with cannot find -lstdc++"
fi

echo "[ada-pyramid] Ready:"
echo "[ada-pyramid]   $LIB_FILE"
