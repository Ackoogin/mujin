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
#   package_sdk.sh [--build-dir DIR] [--gnat-pcl-dir DIR] [--proto-dir DIR]
#                  [--gra] [--out DIR] [--clean]
#
# Defaults: --build-dir build-flatbuffers-only
#           --gnat-pcl-dir <build-dir>/ada_gnat_pcl
#           --proto-dir subprojects/PYRAMID/proto
#           --out dist/pcl_pyramid_sdk
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

BUILD_DIR="${REPO_ROOT}/build-flatbuffers-only"
GNAT_PCL_DIR=""
PROTO_DIR=""
OUT_DIR="${REPO_ROOT}/dist/pcl_pyramid_sdk"
CLEAN=0
GRA=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)    BUILD_DIR="$2"; shift 2 ;;
    --gnat-pcl-dir) GNAT_PCL_DIR="$2"; shift 2 ;;
    --proto-dir)    PROTO_DIR="$2"; shift 2 ;;
    --gra)          GRA=1; shift ;;
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
[[ -n "${PROTO_DIR}" ]] || PROTO_DIR="${PYRAMID_ROOT}/proto"
[[ -d "${PROTO_DIR}" ]] || { echo "[package_sdk] ERROR: proto dir not found: ${PROTO_DIR}" >&2; exit 1; }
PROTO_DIR="$(cd "${PROTO_DIR}" && pwd)"
[[ -n "$(find "${PROTO_DIR}" -name '*.proto' -type f -print -quit)" ]] || {
  echo "[package_sdk] ERROR: no .proto contracts found under ${PROTO_DIR}" >&2
  exit 1
}

echo "[package_sdk] repo         : ${REPO_ROOT}"
echo "[package_sdk] build-dir    : ${BUILD_DIR}"
echo "[package_sdk] gnat-pcl-dir : ${GNAT_PCL_DIR}"
echo "[package_sdk] proto-dir    : ${PROTO_DIR}"
echo "[package_sdk] gra profile  : ${GRA}"
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

# The OMS codecs generated from the selected contract. The hand-written
# UCI 2.5 starter subset (..._uci_starter) is deliberately excluded: it is a
# frozen byte-equivalence fixture for the generated UCI codec, and shipping
# it as "the" OMS codec of an A-GRA distribution is exactly the confusion
# this packaging step exists to prevent.
OMS_CODEC_SOS=()
while IFS= read -r _so; do
  [[ -n "${_so}" ]] && OMS_CODEC_SOS+=("${_so}")
done < <(find "${BUILD_DIR}" -name 'libpyramid_codec_oms_json_*.so' \
              ! -name 'libpyramid_codec_oms_json_uci_starter.so' \
              2>/dev/null | sort)
case "${PROTO_DIR}" in
  *agra_p3_seam)
    OMS_CODEC_SOS=()
    while IFS= read -r _so; do
      [[ -n "${_so}" ]] && OMS_CODEC_SOS+=("${_so}")
    done < <(find "${BUILD_DIR}" -name 'libpyramid_codec_oms_json_agra_core_mms.so' 2>/dev/null | sort)
    ;;
  *agra_p2_seam)
    OMS_CODEC_SOS=()
    while IFS= read -r _so; do
      [[ -n "${_so}" ]] && OMS_CODEC_SOS+=("${_so}")
    done < <(find "${BUILD_DIR}" -name 'libpyramid_codec_oms_json_agra.so' 2>/dev/null | sort)
    ;;
esac
LACAL_TRANSPORT_SO="$(find "${BUILD_DIR}" -name 'libpyramid_lacal_transport_plugin.so' 2>/dev/null | head -1 || true)"
# Whether an OMS codec is *expected* is the contract's answer, not a fixed
# rule: the generator emits one only for a UCI-shaped data model. Ask the
# binding manifest, so a contract that declares a codec but failed to build it
# fails the package, while a port-grammar contract that declares none packages
# cleanly with a warning rather than a spurious error.
BINDING_MANIFEST="${BUILD_DIR}/generated/pyramid_cpp_bindings/binding_manifest.json"
OMS_CODEC_DECLARED=0
if [[ -f "${BINDING_MANIFEST}" ]] && grep -q '"application/oms-json"' "${BINDING_MANIFEST}"; then
  OMS_CODEC_DECLARED=1
fi
if [[ ${GRA} -eq 1 && ${#OMS_CODEC_SOS[@]} -eq 0 ]]; then
  if [[ ${OMS_CODEC_DECLARED} -eq 1 ]]; then
    echo "[package_sdk] ERROR: the contract declares an OMS codec, but none was built." >&2
    echo "[package_sdk]   Rebuild with: build_plugins.sh --gra --proto-dir <contract>" >&2
    exit 1
  fi
  echo "[package_sdk] WARN: this contract declares no OMS codec, so the package will" >&2
  echo "[package_sdk]   contain none. The codec is generated only from an XSD-derived," >&2
  echo "[package_sdk]   UCI-shaped data model. For the formal A-GRA 5.0a P2 profile use:" >&2
  echo "[package_sdk]     --proto-dir subprojects/PYRAMID/pim/agra_p2_seam" >&2
fi
if [[ ${GRA} -eq 1 && -z "${LACAL_TRANSPORT_SO}" ]]; then
  echo "[package_sdk] ERROR: --gra requires libpyramid_lacal_transport_plugin.so; rebuild with build_plugins.sh --gra." >&2
  exit 1
fi

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
if [[ ${GRA} -eq 1 ]]; then
  for _so in "${OMS_CODEC_SOS[@]-}"; do
    [[ -n "${_so}" ]] || continue
    echo "[package_sdk]   OMS codec: $(basename "${_so}")"
    cp -f "${_so}" "${OUT_DIR}/plugins/"
  done
  cp -f "${LACAL_TRANSPORT_SO}" "${OUT_DIR}/plugins/"
fi

echo "[package_sdk] copying flatc ..."
cp -f "${FLATC_BIN}" "${OUT_DIR}/tools/flatc"
chmod +x "${OUT_DIR}/tools/flatc"

echo "[package_sdk] copying generator (pim/*.py) ..."
cp -f "${PYRAMID_ROOT}"/pim/*.py "${OUT_DIR}/generator/"
cp -f "${PYRAMID_ROOT}"/pim/backends/*.py "${OUT_DIR}/generator/backends/"
mkdir -p "${OUT_DIR}/generator/cpp"
cp -f "${PYRAMID_ROOT}"/pim/cpp/*.py "${OUT_DIR}/generator/cpp/"
mkdir -p "${OUT_DIR}/generator/ada"
cp -f "${PYRAMID_ROOT}"/pim/ada/*.py "${OUT_DIR}/generator/ada/"
for _skeleton_generator in \
  "${OUT_DIR}/generator/cpp/component_skeleton_gen.py" \
  "${OUT_DIR}/generator/ada/component_skeleton_gen.py"; do
  if [[ ! -f "${_skeleton_generator}" ]]; then
    echo "[package_sdk] FAIL: missing ${_skeleton_generator}" >&2
    exit 1
  fi
done
mkdir -p "${OUT_DIR}/generator/topic_metadata"
cp -f "${PYRAMID_ROOT}"/pim/topic_metadata/*.json "${OUT_DIR}/generator/topic_metadata/"

echo "[package_sdk] copying starter proto contracts ..."
cp -rf "${PROTO_DIR}/." "${OUT_DIR}/proto/"

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
  echo "# proto-dir: ${PROTO_DIR}"
  echo "# gra-profile: ${GRA}"
  find "${OUT_DIR}" -type f | sed "s|${OUT_DIR}/||" | sort
} > "${OUT_DIR}/MANIFEST.txt"

echo
echo "[package_sdk] Done. Deploy directory: ${OUT_DIR}"
echo "[package_sdk] See ${OUT_DIR}/README.md for the downstream quick start."
