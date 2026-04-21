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

# Locate GNAT toolchain (next to gprbuild).  GNAT gcc is Ada/C only (no cc1plus),
# so system g++ is used for C++ compilation.  We query GNAT gcc for its target
# triple to derive the right -m32/-m64 flag, ensuring the objects match the
# architecture GNAT links for.  GNAT's ar is used for a compatible archive index.
# libstdc++.a from the g++ installation is copied into OUT_DIR so GNAT's linker
# can satisfy -lstdc++ without needing to know the MinGW lib paths.
AR_CMD=""
ARCH_FLAG=""
if command -v gprbuild >/dev/null 2>&1; then
  GNAT_BIN="$(dirname "$(command -v gprbuild)")"
  if [[ -x "$GNAT_BIN/gcc-ar" ]]; then AR_CMD="$GNAT_BIN/gcc-ar"
  elif [[ -x "$GNAT_BIN/ar" ]];    then AR_CMD="$GNAT_BIN/ar"; fi

  if [[ -x "$GNAT_BIN/gcc" ]]; then
    GNAT_TARGET="$("$GNAT_BIN/gcc" -dumpmachine 2>/dev/null || true)"
    echo "[ada-pyramid] GNAT target : $GNAT_TARGET"
    case "$GNAT_TARGET" in
      i686*|i386*|i486*|i586*) ARCH_FLAG="-m32" ;;
      x86_64*|amd64*)           ARCH_FLAG="-m64" ;;
    esac
  fi
fi

if [[ -z "$AR_CMD" ]]; then
  if ! command -v ar >/dev/null 2>&1; then
    echo "[ada-pyramid] ERROR: no ar found (checked GNAT bin and PATH)" >&2
    exit 1
  fi
  AR_CMD="ar"
fi

if ! command -v g++ >/dev/null 2>&1; then
  echo "[ada-pyramid] ERROR: g++ not found in PATH" >&2
  exit 1
fi

echo "[ada-pyramid] Compiler : g++ $ARCH_FLAG"
echo "[ada-pyramid] Archiver : $AR_CMD"

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
  $ARCH_FLAG -std=c++17 -O2
  -I"$GEN_DIR"
  -I"$GEN_FB_DIR"
  -I"$BUILD_FB_DIR"
  -I"$FLATBUFFERS_INCLUDE"
  -I"$NLOHMANN_INCLUDE"
)

g++ "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_base_codec.cpp"     -o "$OBJ_DIR/pyramid_data_model_base_codec.o"
g++ "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_common_codec.cpp"   -o "$OBJ_DIR/pyramid_data_model_common_codec.o"
g++ "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_tactical_codec.cpp" -o "$OBJ_DIR/pyramid_data_model_tactical_codec.o"
g++ "${CXXFLAGS[@]}" -c "$GEN_DIR/pyramid_data_model_autonomy_codec.cpp" -o "$OBJ_DIR/pyramid_data_model_autonomy_codec.o"
g++ "${CXXFLAGS[@]}" -c "$GEN_FB_DIR/pyramid_services_tactical_objects_flatbuffers_codec.cpp"  -o "$OBJ_DIR/pyramid_services_tactical_objects_flatbuffers_codec.o"
g++ "${CXXFLAGS[@]}" -c "$GEN_FB_DIR/pyramid_services_autonomy_backend_flatbuffers_codec.cpp" -o "$OBJ_DIR/pyramid_services_autonomy_backend_flatbuffers_codec.o"

rm -f "$TMP_LIB_FILE"

"$AR_CMD" rcs "$TMP_LIB_FILE" \
  "$OBJ_DIR/pyramid_data_model_base_codec.o" \
  "$OBJ_DIR/pyramid_data_model_common_codec.o" \
  "$OBJ_DIR/pyramid_data_model_tactical_codec.o" \
  "$OBJ_DIR/pyramid_data_model_autonomy_codec.o" \
  "$OBJ_DIR/pyramid_services_tactical_objects_flatbuffers_codec.o" \
  "$OBJ_DIR/pyramid_services_autonomy_backend_flatbuffers_codec.o"

rm -f "$LIB_FILE"
mv "$TMP_LIB_FILE" "$LIB_FILE"

# Copy libstdc++ from g++'s own installation into OUT_DIR so GNAT's linker
# can satisfy -lstdc++ without needing to know the MinGW lib paths.
GXX_LIBSTDCXX="$(g++ ${ARCH_FLAG} -print-file-name=libstdc++.a 2>/dev/null || true)"
if [[ -n "$GXX_LIBSTDCXX" && "$GXX_LIBSTDCXX" != "libstdc++.a" && -f "$GXX_LIBSTDCXX" ]]; then
  GXX_LIB_DIR="$(dirname "$GXX_LIBSTDCXX")"
  echo "[ada-pyramid] Copying libstdc++ from $GXX_LIB_DIR to $OUT_DIR"
  cp -f "$GXX_LIB_DIR/libstdc++.a"     "$OUT_DIR/" 2>/dev/null || true
  cp -f "$GXX_LIB_DIR/libstdc++.dll.a" "$OUT_DIR/" 2>/dev/null || true
else
  echo "[ada-pyramid] WARNING: libstdc++.a not found via 'g++ ${ARCH_FLAG} -print-file-name'; link may fail with cannot find -lstdc++"
fi

echo "[ada-pyramid] Built:"
echo "[ada-pyramid]   $LIB_FILE"
