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

# Prefer GNAT's own gcc and ar so objects and archive index are in the format
# that GNAT's linker expects.  GNAT gcc supports C (cc1) even though it lacks
# C++ (cc1plus), so it can compile all PCL sources directly.
CC_CMD=""
AR_CMD=""
ARCH_FLAG=""
if command -v gprbuild >/dev/null 2>&1; then
  GNAT_BIN="$(dirname "$(command -v gprbuild)")"
  if [[ -x "$GNAT_BIN/gcc" ]];    then CC_CMD="$GNAT_BIN/gcc"; fi
  if [[ -x "$GNAT_BIN/gcc-ar" ]]; then AR_CMD="$GNAT_BIN/gcc-ar"
  elif [[ -x "$GNAT_BIN/ar" ]];   then AR_CMD="$GNAT_BIN/ar"; fi

  if [[ -x "$GNAT_BIN/gcc" ]]; then
    GNAT_TARGET="$("$GNAT_BIN/gcc" -dumpmachine 2>/dev/null || true)"
    echo "[ada-pcl] GNAT target : $GNAT_TARGET"
    case "$GNAT_TARGET" in
      i686*|i386*|i486*|i586*) ARCH_FLAG="-m32" ;;
      x86_64*|amd64*)           ARCH_FLAG="-m64" ;;
    esac
  fi
fi

if [[ -z "$CC_CMD" ]]; then
  if ! command -v gcc >/dev/null 2>&1; then
    echo "[ada-pcl] ERROR: GNAT gcc not found and gcc not found in PATH" >&2
    exit 1
  fi
  CC_CMD="gcc"
  echo "[ada-pcl] WARNING: GNAT gcc not found; using system gcc - link may fail if it targets a different architecture"
fi

if [[ -z "$AR_CMD" ]]; then
  if ! command -v ar >/dev/null 2>&1; then
    echo "[ada-pcl] ERROR: no ar found (checked GNAT bin and PATH)" >&2
    exit 1
  fi
  AR_CMD="ar"
fi

echo "[ada-pcl] Compiler : $CC_CMD"
echo "[ada-pcl] Archiver : $AR_CMD"

mkdir -p "$OBJ_DIR"

if [[ "$FORCE_REBUILD" == "--force" ]]; then
  rm -f "$CORE_LIB" "$SOCKET_LIB" "$SHMEM_LIB" \
        "$OUT_DIR"/libpcl_core_*.a \
        "$OUT_DIR"/libpcl_transport_socket_*.a \
        "$OUT_DIR"/libpcl_transport_shared_memory_*.a \
        "$OBJ_DIR"/*.o
fi

echo "[ada-pcl] Building GNAT-compatible PCL archives in $OUT_DIR"

CFLAGS=($ARCH_FLAG -std=c11 -O2 -I"$ROOT_DIR/include" -I"$ROOT_DIR/src")

"$CC_CMD" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_container.c"               -o "$OBJ_DIR/pcl_container.o"
"$CC_CMD" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_executor.c"                -o "$OBJ_DIR/pcl_executor.o"
"$CC_CMD" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_log.c"                     -o "$OBJ_DIR/pcl_log.o"
"$CC_CMD" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_bridge.c"                  -o "$OBJ_DIR/pcl_bridge.o"
"$CC_CMD" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_transport_socket.c"        -o "$OBJ_DIR/pcl_transport_socket.o"
"$CC_CMD" "${CFLAGS[@]}" -c "$ROOT_DIR/src/pcl_transport_shared_memory.c" -o "$OBJ_DIR/pcl_transport_shared_memory.o"

rm -f "$CORE_TMP" "$SOCKET_TMP" "$SHMEM_TMP"

"$AR_CMD" rcs "$CORE_TMP" \
  "$OBJ_DIR/pcl_container.o" \
  "$OBJ_DIR/pcl_executor.o" \
  "$OBJ_DIR/pcl_log.o" \
  "$OBJ_DIR/pcl_bridge.o"

"$AR_CMD" rcs "$SOCKET_TMP" "$OBJ_DIR/pcl_transport_socket.o"
"$AR_CMD" rcs "$SHMEM_TMP"  "$OBJ_DIR/pcl_transport_shared_memory.o"

rm -f "$CORE_LIB" "$SOCKET_LIB" "$SHMEM_LIB"
mv "$CORE_TMP"   "$CORE_LIB"
mv "$SOCKET_TMP" "$SOCKET_LIB"
mv "$SHMEM_TMP"  "$SHMEM_LIB"

echo "[ada-pcl] Built:"
echo "[ada-pcl]   $CORE_LIB"
echo "[ada-pcl]   $SOCKET_LIB"
echo "[ada-pcl]   $SHMEM_LIB"
