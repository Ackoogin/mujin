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

if ! command -v gcc >/dev/null 2>&1; then
  echo "[ada-pcl] ERROR: gcc not found in PATH" >&2
  exit 1
fi

if ! command -v ar >/dev/null 2>&1; then
  echo "[ada-pcl] ERROR: ar not found in PATH" >&2
  exit 1
fi

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

gcc "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_container.c" -o "$OBJ_DIR/pcl_container.o"
gcc "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_executor.c" -o "$OBJ_DIR/pcl_executor.o"
gcc "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_log.c" -o "$OBJ_DIR/pcl_log.o"
gcc "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_bridge.c" -o "$OBJ_DIR/pcl_bridge.o"
gcc "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_transport_socket.c" -o "$OBJ_DIR/pcl_transport_socket.o"
gcc "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_transport_shared_memory.c" -o "$OBJ_DIR/pcl_transport_shared_memory.o"

rm -f "$CORE_TMP" "$SOCKET_TMP" "$SHMEM_TMP"

ar rcs "$CORE_TMP" \
  "$OBJ_DIR/pcl_container.o" \
  "$OBJ_DIR/pcl_executor.o" \
  "$OBJ_DIR/pcl_log.o" \
  "$OBJ_DIR/pcl_bridge.o"

ar rcs "$SOCKET_TMP" "$OBJ_DIR/pcl_transport_socket.o"
ar rcs "$SHMEM_TMP"  "$OBJ_DIR/pcl_transport_shared_memory.o"

rm -f "$CORE_LIB" "$SOCKET_LIB" "$SHMEM_LIB"
mv "$CORE_TMP"   "$CORE_LIB"
mv "$SOCKET_TMP" "$SOCKET_LIB"
mv "$SHMEM_TMP"  "$SHMEM_LIB"

echo "[ada-pcl] Built:"
echo "[ada-pcl]   $CORE_LIB"
echo "[ada-pcl]   $SOCKET_LIB"
echo "[ada-pcl]   $SHMEM_LIB"
