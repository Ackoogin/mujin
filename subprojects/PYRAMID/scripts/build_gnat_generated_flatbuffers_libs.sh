#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYRAMID_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PYRAMID_ROOT/../.." && pwd)"

OUT_DIR="${1:-$WORKSPACE_ROOT/build/ada_gnat_pyramid}"
OBJ_DIR="$OUT_DIR/obj"
FORCE_REBUILD="${2:-0}"

LIB_NAME="pyramid_generated_flatbuffers_codec"
LIB_FILE="$OUT_DIR/lib${LIB_NAME}.a"
RAND_SUFFIX="${RANDOM}${RANDOM}"
TMP_LIB_FILE="$OUT_DIR/lib${LIB_NAME}_${RAND_SUFFIX}.a"

if [[ "$FORCE_REBUILD" != "--force" && -f "$LIB_FILE" ]]; then
  echo "[ada-pyramid] GNAT FlatBuffers archive already present in $OUT_DIR"
  exit 0
fi

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
AR="$GXX_DIR/ar"

echo "[ada-pyramid] Compiler : $CXX"
echo "[ada-pyramid] Archiver : $AR"

mkdir -p "$OBJ_DIR"

if [[ "$FORCE_REBUILD" == "--force" ]]; then
  rm -f "$LIB_FILE" "$OUT_DIR"/lib${LIB_NAME}_*.a "$OBJ_DIR"/*.o
fi

GEN_DIR="$PYRAMID_ROOT/bindings/cpp/generated"
GEN_FB_DIR="$GEN_DIR/flatbuffers/cpp"
BUILD_FB_DIR="$WORKSPACE_ROOT/build/generated/flatbuffers/cpp"
FLATBUFFERS_INCLUDE="$WORKSPACE_ROOT/build/_deps/flatbuffers-src/include"
NLOHMANN_INCLUDE="${NLOHMANN_JSON_INCLUDE:-$WORKSPACE_ROOT/build/_deps/nlohmann_json-src/include}"
if [[ ! -d "$NLOHMANN_INCLUDE" ]]; then
  NLOHMANN_INCLUDE="$PYRAMID_ROOT/core/external"
fi

if [[ ! -f "$BUILD_FB_DIR/pyramid_services_tactical_objects_generated.h" ]] || \
   [[ ! -f "$BUILD_FB_DIR/pyramid_services_autonomy_backend_generated.h" ]]; then
  if ! command -v cmake >/dev/null 2>&1; then
    echo "[ada-pyramid] ERROR: missing flatc-generated header and cmake not available" >&2
    exit 1
  fi
  echo "[ada-pyramid] Refreshing flatc-generated headers..."
  cmake --build "$WORKSPACE_ROOT/build" --config Release --target pyramid_flatbuffers_codegen -j4
fi

echo "[ada-pyramid] Building GNAT-compatible generated FlatBuffers archive in $OUT_DIR"

CXXFLAGS=(
  -std=c++17 -O2
  -I"$GEN_DIR"
  -I"$GEN_FB_DIR"
  -I"$BUILD_FB_DIR"
  -I"$FLATBUFFERS_INCLUDE"
  -I"$NLOHMANN_INCLUDE"
)

"$CXX" "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_base_codec.cpp"     -o "$OBJ_DIR/pyramid_data_model_base_codec.o"
"$CXX" "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_common_codec.cpp"   -o "$OBJ_DIR/pyramid_data_model_common_codec.o"
"$CXX" "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_tactical_codec.cpp" -o "$OBJ_DIR/pyramid_data_model_tactical_codec.o"
"$CXX" "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_autonomy_codec.cpp" -o "$OBJ_DIR/pyramid_data_model_autonomy_codec.o"
"$CXX" "${CXXFLAGS[@]}" -c "$GEN_FB_DIR/pyramid_services_tactical_objects_flatbuffers_codec.cpp"  -o "$OBJ_DIR/pyramid_services_tactical_objects_flatbuffers_codec.o"
"$CXX" "${CXXFLAGS[@]}" -c "$GEN_FB_DIR/pyramid_services_autonomy_backend_flatbuffers_codec.cpp" -o "$OBJ_DIR/pyramid_services_autonomy_backend_flatbuffers_codec.o"

rm -f "$TMP_LIB_FILE"

"$AR" rcs "$TMP_LIB_FILE" \
  "$OBJ_DIR/pyramid_data_model_base_codec.o" \
  "$OBJ_DIR/pyramid_data_model_common_codec.o" \
  "$OBJ_DIR/pyramid_data_model_tactical_codec.o" \
  "$OBJ_DIR/pyramid_data_model_autonomy_codec.o" \
  "$OBJ_DIR/pyramid_services_tactical_objects_flatbuffers_codec.o" \
  "$OBJ_DIR/pyramid_services_autonomy_backend_flatbuffers_codec.o"

rm -f "$LIB_FILE"
mv "$TMP_LIB_FILE" "$LIB_FILE"

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

echo "[ada-pyramid] Built:"
echo "[ada-pyramid]   $LIB_FILE"
