#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

OUT_DIR="${1:-$ROOT_DIR/build/ada_gnat_pcl}"
OBJ_DIR="$OUT_DIR/obj"
FORCE_REBUILD="${2:-0}"

CORE_LIB="$OUT_DIR/libpcl_core.a"
SOCKET_LIB="$OUT_DIR/libpcl_transport_socket.a"
SHMEM_LIB="$OUT_DIR/libpcl_transport_shared_memory.a"
RAND_SUFFIX="${RANDOM}${RANDOM}"
CORE_TMP="$OUT_DIR/libpcl_core_${RAND_SUFFIX}.a"
SOCKET_TMP="$OUT_DIR/libpcl_transport_socket_${RAND_SUFFIX}.a"
SHMEM_TMP="$OUT_DIR/libpcl_transport_shared_memory_${RAND_SUFFIX}.a"

if [[ "$FORCE_REBUILD" != "--force" && -f "$CORE_LIB" && -f "$SOCKET_LIB" && -f "$SHMEM_LIB" ]]; then
  echo "[ada-pcl] GNAT static archives already present in $OUT_DIR"
  exit 0
fi

# Use gcc and all companion tools (ar) from the same bin directory as g++ so
# that the compiler and archiver are always from the same toolchain and produce
# mutually-compatible object/archive formats.
GXX_PATH="$(command -v g++ 2>/dev/null || true)"
if [[ -z "$GXX_PATH" ]]; then
  echo "[ada-pcl] ERROR: g++ not found in PATH" >&2
  exit 1
fi
GXX_DIR="$(dirname "$GXX_PATH")"

CC="$GXX_DIR/gcc"
AR="$GXX_DIR/ar"

echo "[ada-pcl] Compiler : $CC"
echo "[ada-pcl] Archiver : $AR"

mkdir -p "$OBJ_DIR"

if [[ "$FORCE_REBUILD" == "--force" ]]; then
  rm -f "$CORE_LIB" "$SOCKET_LIB" "$SHMEM_LIB" \
        "$OUT_DIR"/libpcl_core_*.a \
        "$OUT_DIR"/libpcl_transport_socket_*.a \
        "$OUT_DIR"/libpcl_transport_shared_memory_*.a \
        "$OBJ_DIR"/*.o
fi

echo "[ada-pcl] Building GNAT-compatible PCL archives in $OUT_DIR"

CFLAGS=(-std=c11 -O2 -I"$ROOT_DIR/include" -I"$ROOT_DIR/src")

"$CC" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_container.c"               -o "$OBJ_DIR/pcl_container.o"
"$CC" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_executor.c"                -o "$OBJ_DIR/pcl_executor.o"
"$CC" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_log.c"                     -o "$OBJ_DIR/pcl_log.o"
"$CC" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_bridge.c"                  -o "$OBJ_DIR/pcl_bridge.o"
"$CC" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_transport_socket.c"        -o "$OBJ_DIR/pcl_transport_socket.o"
"$CC" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_transport_shared_memory.c" -o "$OBJ_DIR/pcl_transport_shared_memory.o"

rm -f "$CORE_TMP" "$SOCKET_TMP" "$SHMEM_TMP"

"$AR" rcs "$CORE_TMP" \
  "$OBJ_DIR/pcl_container.o" \
  "$OBJ_DIR/pcl_executor.o" \
  "$OBJ_DIR/pcl_log.o" \
  "$OBJ_DIR/pcl_bridge.o"

"$AR" rcs "$SOCKET_TMP" "$OBJ_DIR/pcl_transport_socket.o"
"$AR" rcs "$SHMEM_TMP"  "$OBJ_DIR/pcl_transport_shared_memory.o"

rm -f "$CORE_LIB" "$SOCKET_LIB" "$SHMEM_LIB"
mv "$CORE_TMP"   "$CORE_LIB"
mv "$SOCKET_TMP" "$SOCKET_LIB"
mv "$SHMEM_TMP"  "$SHMEM_LIB"

echo "[ada-pcl] Built:"
echo "[ada-pcl]   $CORE_LIB"
echo "[ada-pcl]   $SOCKET_LIB"
echo "[ada-pcl]   $SHMEM_LIB"
