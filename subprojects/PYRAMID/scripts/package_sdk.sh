#!/usr/bin/env bash
#
# package_sdk.sh -- produce the offline PCL/PYRAMID SDK deploy directory.
#
# Maintainer-run, from inside this repo. Packages prebuilt PCL static libs +
# headers + a prebuilt flatc alongside the pure-Python generator, starter
# .proto contracts, and a standalone CMake/GNAT project template, so a
# downstream developer can build their OWN proto -> plugin on a firewalled
# machine with zero network access and no dependency on this monorepo.
#
# Preconditions:
#   --build-dir must already be configured+built with
#   PYRAMID_ENABLE_FLATBUFFERS=ON, PYRAMID_GENERATE_CPP_BINDINGS=ON, e.g.:
#     subprojects/PYRAMID/scripts/build_plugins.sh
#   For the GNAT/Ada libs, run first (or let this script attempt it if
#   gcc/ar are on PATH):
#     subprojects/PCL/scripts/build_gnat_pcl_static_libs.sh
#
# Usage:
#   package_sdk.sh [--build-dir DIR] [--gnat-pcl-dir DIR] [--out DIR] [--clean]
#
# Defaults: --build-dir build-flatbuffers-only
#           --gnat-pcl-dir <build-dir>/ada_gnat_pcl
#           --out dist/pcl_pyramid_sdk
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

BUILD_DIR="${REPO_ROOT}/build-flatbuffers-only"
GNAT_PCL_DIR=""
OUT_DIR="${REPO_ROOT}/dist/pcl_pyramid_sdk"
CLEAN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)    BUILD_DIR="$2"; shift 2 ;;
    --gnat-pcl-dir) GNAT_PCL_DIR="$2"; shift 2 ;;
    --out)          OUT_DIR="$2"; shift 2 ;;
    --clean)        CLEAN=1; shift ;;
    -h|--help)      sed -n '2,24p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "[package_sdk] unknown arg: $1" >&2; exit 2 ;;
  esac
done

[[ -d "${BUILD_DIR}" ]] || { echo "[package_sdk] ERROR: build dir not found: ${BUILD_DIR} -- run subprojects/PYRAMID/scripts/build_plugins.sh first." >&2; exit 1; }
BUILD_DIR="$(cd "${BUILD_DIR}" && pwd)"
[[ -n "${GNAT_PCL_DIR}" ]] || GNAT_PCL_DIR="${BUILD_DIR}/ada_gnat_pcl"

mkdir -p "${OUT_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd)"

PYRAMID_ROOT="${REPO_ROOT}/subprojects/PYRAMID"
PCL_ROOT="${REPO_ROOT}/subprojects/PCL"
SDK_TEMPLATE="${PYRAMID_ROOT}/sdk_template"

echo "[package_sdk] repo         : ${REPO_ROOT}"
echo "[package_sdk] build-dir    : ${BUILD_DIR}"
echo "[package_sdk] gnat-pcl-dir : ${GNAT_PCL_DIR}"
echo "[package_sdk] out          : ${OUT_DIR}"

if [[ ${CLEAN} -eq 1 ]]; then
  echo "[package_sdk] cleaning ${OUT_DIR}"
  rm -rf "${OUT_DIR:?}"/*
fi

# --- Locate prebuilt artifacts ---------------------------------------------
PCL_LIB="$(find "${BUILD_DIR}" -name 'libpcl_core.a' 2>/dev/null | head -1 || true)"
if [[ -z "${PCL_LIB}" ]]; then
  echo "[package_sdk] ERROR: no libpcl_core.a found under ${BUILD_DIR}" >&2
  exit 1
fi

FLATC_BIN="$(find "${BUILD_DIR}" -name 'flatc' -type f -perm -u+x 2>/dev/null | head -1 || true)"
if [[ -z "${FLATC_BIN}" ]]; then
  echo "[package_sdk] ERROR: flatc not found under ${BUILD_DIR} -- build with PYRAMID_ENABLE_FLATBUFFERS=ON." >&2
  exit 1
fi

FLATBUFFERS_INCLUDE="$(find "${BUILD_DIR}/_deps" -maxdepth 4 -type d -path '*flatbuffers-src/include/flatbuffers' 2>/dev/null | head -1 || true)"
if [[ -z "${FLATBUFFERS_INCLUDE}" ]]; then
  echo "[package_sdk] ERROR: flatbuffers runtime headers not found under ${BUILD_DIR}/_deps/flatbuffers-src" >&2
  exit 1
fi

TRANSPORT_SOCKET_SO="$(find "${BUILD_DIR}" -name 'libpcl_transport_socket_plugin.so' 2>/dev/null | head -1 || true)"
TRANSPORT_SHM_SO="$(find "${BUILD_DIR}" -name 'libpcl_transport_shared_memory_plugin.so' 2>/dev/null | head -1 || true)"
[[ -n "${TRANSPORT_SOCKET_SO}" ]] || echo "[package_sdk] WARN: libpcl_transport_socket_plugin.so not found under ${BUILD_DIR}"
[[ -n "${TRANSPORT_SHM_SO}" ]]    || echo "[package_sdk] WARN: libpcl_transport_shared_memory_plugin.so not found under ${BUILD_DIR}"

if [[ ! -f "${GNAT_PCL_DIR}/libpcl_core.a" ]]; then
  if command -v gcc >/dev/null 2>&1; then
    echo "[package_sdk] building GNAT PCL static libs ..."
    "${PCL_ROOT}/scripts/build_gnat_pcl_static_libs.sh" "${GNAT_PCL_DIR}" || \
      echo "[package_sdk] WARN: GNAT PCL static lib build failed -- skipping GNAT/Ada libs."
  else
    echo "[package_sdk] WARN: no GNAT PCL archives at ${GNAT_PCL_DIR} and gcc not on PATH -- skipping GNAT/Ada libs."
  fi
fi

# --- Lay out the deploy directory ------------------------------------------
mkdir -p "${OUT_DIR}"/{include/pcl,include/external,lib/linux,lib/gnat,plugins,tools,generator/backends,proto,gnat/pcl_bindings}

echo "[package_sdk] copying PCL headers ..."
cp -f "${PCL_ROOT}"/include/pcl/* "${OUT_DIR}/include/pcl/"

echo "[package_sdk] copying vendored header-only deps ..."
mkdir -p "${OUT_DIR}/include/external/nlohmann" "${OUT_DIR}/include/external/tl" "${OUT_DIR}/include/external/flatbuffers"
cp -f "${PYRAMID_ROOT}/core/external/nlohmann/"* "${OUT_DIR}/include/external/nlohmann/"
cp -f "${PYRAMID_ROOT}/core/external/tl/"* "${OUT_DIR}/include/external/tl/"
cp -rf "${FLATBUFFERS_INCLUDE}/"* "${OUT_DIR}/include/external/flatbuffers/"

echo "[package_sdk] copying prebuilt libs (static, single-config toolchain) ..."
cp -f "${PCL_LIB}" "${OUT_DIR}/lib/linux/"

if [[ -f "${GNAT_PCL_DIR}/libpcl_core.a" ]]; then
  echo "[package_sdk] copying prebuilt GNAT libs ..."
  cp -f "${GNAT_PCL_DIR}"/lib*.a "${OUT_DIR}/lib/gnat/"
else
  echo "[package_sdk] copying prebuilt GNAT libs ... (none -- Ada support unavailable in this deploy)"
fi

echo "[package_sdk] copying prebuilt transport plugins ..."
[[ -n "${TRANSPORT_SOCKET_SO}" ]] && cp -f "${TRANSPORT_SOCKET_SO}" "${OUT_DIR}/plugins/"
[[ -n "${TRANSPORT_SHM_SO}" ]]    && cp -f "${TRANSPORT_SHM_SO}"    "${OUT_DIR}/plugins/"

echo "[package_sdk] copying flatc ..."
cp -f "${FLATC_BIN}" "${OUT_DIR}/tools/flatc"
chmod +x "${OUT_DIR}/tools/flatc"

echo "[package_sdk] copying generator (pim/*.py) ..."
cp -f "${PYRAMID_ROOT}"/pim/*.py "${OUT_DIR}/generator/"
cp -f "${PYRAMID_ROOT}"/pim/backends/*.py "${OUT_DIR}/generator/backends/"
mkdir -p "${OUT_DIR}/generator/topic_metadata"
cp -f "${PYRAMID_ROOT}"/pim/topic_metadata/*.json "${OUT_DIR}/generator/topic_metadata/"

echo "[package_sdk] copying starter proto contracts ..."
cp -rf "${PYRAMID_ROOT}/proto/pyramid" "${OUT_DIR}/proto/"

echo "[package_sdk] copying Ada PCL bindings (source only) ..."
cp -f "${PCL_ROOT}"/bindings/ada/*.ads "${OUT_DIR}/gnat/pcl_bindings/" 2>/dev/null || true
cp -f "${PCL_ROOT}"/bindings/ada/*.adb "${OUT_DIR}/gnat/pcl_bindings/" 2>/dev/null || true

echo "[package_sdk] copying GNATCOLL JSON (Ada JSON codec dependency) ..."
mkdir -p "${OUT_DIR}/gnat/external"
cp -rf "${PYRAMID_ROOT}/core/external/gnatcoll-core" "${OUT_DIR}/gnat/external/"

echo "[package_sdk] copying SDK project template (CMake + GNAT + scripts + README) ..."
cp -f "${SDK_TEMPLATE}/README.md" "${OUT_DIR}/"
rm -rf "${OUT_DIR}/sdk_project"
cp -rf "${SDK_TEMPLATE}/sdk_project" "${OUT_DIR}/"
cp -f "${SDK_TEMPLATE}/gnat/pyramid_sdk_ada.gpr" "${OUT_DIR}/gnat/"
cp -f "${SDK_TEMPLATE}/gnat/build_gnat_pyramid_cabi_marshal_libs.sh" "${OUT_DIR}/gnat/"
cp -f "${SDK_TEMPLATE}/gnat/build_gnat_pyramid_cabi_marshal_libs.bat" "${OUT_DIR}/gnat/"
chmod +x "${OUT_DIR}/gnat/build_gnat_pyramid_cabi_marshal_libs.sh"
mkdir -p "${OUT_DIR}/scripts"
cp -f "${SDK_TEMPLATE}"/scripts/* "${OUT_DIR}/scripts/"
chmod +x "${OUT_DIR}"/scripts/*.sh

# --- Manifest ---------------------------------------------------------------
{
  echo "# PCL/PYRAMID offline SDK -- packaged from ${BUILD_DIR}"
  echo "# gnat-pcl-dir: ${GNAT_PCL_DIR}"
  find "${OUT_DIR}" -type f | sed "s|${OUT_DIR}/||" | sort
} > "${OUT_DIR}/MANIFEST.txt"

echo
echo "[package_sdk] Done. Deploy directory: ${OUT_DIR}"
echo "[package_sdk] See ${OUT_DIR}/README.md for the downstream quick start."
