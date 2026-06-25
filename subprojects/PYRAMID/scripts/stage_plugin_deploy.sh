#!/usr/bin/env bash
#
# stage_plugin_deploy.sh
#
# Stage the runtime plugin binding artifacts into per-component deployment
# directories so the output can be reviewed and a client can be built against
# them.  For each PYRAMID component it collects:
#
#   <out>/<component>/
#     plugins/   codec plugin .so(s) for the component  + the transport plugin .so
#     include/   client-facing headers   (pcl/*  and  pyramid/*  generated facade)
#     src/       the generated contract facade the client compiles against
#     lib/       static libraries a client links (.a)
#     MANIFEST.txt   plugin .so paths (feed to --codec-plugin / PCL_CODEC_PLUGIN_PATH)
#     README.md      how to build a client and run it against the plugin
#
# The codec plugin is the single cross-language .so (it consumes the frozen
# pyramid_<Type>_c C struct); the same file is loaded from C++ and Ada.
#
# Usage:
#   stage_plugin_deploy.sh [--build-dir DIR] [--out DIR] [--clean]
#
# Defaults: --build-dir build-flatbuffers-only   --out dist/plugin_deploy
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

BUILD_DIR="${REPO_ROOT}/build-flatbuffers-only"
OUT_DIR="${REPO_ROOT}/dist/plugin_deploy"
CLEAN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --out)       OUT_DIR="$2";   shift 2 ;;
    --clean)     CLEAN=1;        shift ;;
    -h|--help)   sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

# Resolve to absolute paths.
mkdir -p "${OUT_DIR}"
BUILD_DIR="$(cd "${BUILD_DIR}" && pwd)"
OUT_DIR="$(cd "${OUT_DIR}" && pwd)"

PCL_INC="${REPO_ROOT}/subprojects/PCL/include/pcl"
GEN_DIR="${BUILD_DIR}/generated/pyramid_cpp_bindings"
PYRAMID_LIBDIR="${BUILD_DIR}/subprojects/PYRAMID"
PCL_SRCDIR="${BUILD_DIR}/subprojects/PCL/src"
TRANSPORT_PLUGIN="${PCL_SRCDIR}/libpcl_transport_socket_plugin.so"

[[ -d "${GEN_DIR}" ]]   || { echo "ERROR: generated bindings not found at ${GEN_DIR} -- build first." >&2; exit 1; }
[[ -d "${PCL_INC}" ]]   || { echo "ERROR: PCL include dir not found at ${PCL_INC}" >&2; exit 1; }

echo "build-dir : ${BUILD_DIR}"
echo "out       : ${OUT_DIR}"
[[ ${CLEAN} -eq 1 ]] && { echo "cleaning ${OUT_DIR}"; rm -rf "${OUT_DIR:?}"/*; }

# Static libraries a plugin-only client links (codec comes from the .so at run time).
LINK_LIBS=(
  "${PCL_SRCDIR}/libpcl_core.a"
  "${PCL_SRCDIR}/libpcl_transport_socket.a"
  "${PYRAMID_LIBDIR}/libpyramid_generated_codecs.a"   # native<->C-struct marshalling
)
[[ -f "${PYRAMID_LIBDIR}/libpyramid_flatbuffers_support.a" ]] && \
  LINK_LIBS+=("${PYRAMID_LIBDIR}/libpyramid_flatbuffers_support.a")

# Discover components from the built codec plugin .so names:
#   libpyramid_codec_<codec>_<component>.so
mapfile -t COMPONENTS < <(
  find "${PYRAMID_LIBDIR}" -maxdepth 1 -name 'libpyramid_codec_*.so' -printf '%f\n' 2>/dev/null \
    | sed -E 's/^libpyramid_codec_(json|flatbuffers|protobuf)_(.*)\.so$/\2/' \
    | sort -u
)
[[ ${#COMPONENTS[@]} -gt 0 ]] || { echo "ERROR: no codec plugin .so found in ${PYRAMID_LIBDIR}" >&2; exit 1; }

echo "components: ${COMPONENTS[*]}"
echo

for comp in "${COMPONENTS[@]}"; do
  dest="${OUT_DIR}/${comp}"
  echo "==> ${comp}  ->  ${dest}"
  mkdir -p "${dest}/plugins" "${dest}/include/pcl" "${dest}/include/pyramid" "${dest}/src" "${dest}/lib"

  # --- plugins: the per-component codec .so(s) + the shared transport plugin ---
  found_codec=0
  for codec in json flatbuffers protobuf; do
    so="${PYRAMID_LIBDIR}/libpyramid_codec_${codec}_${comp}.so"
    if [[ -f "${so}" ]]; then cp -f "${so}" "${dest}/plugins/"; found_codec=1; fi
  done
  [[ ${found_codec} -eq 1 ]] || echo "   WARN: no codec .so for ${comp}"
  if [[ -f "${TRANSPORT_PLUGIN}" ]]; then
    cp -f "${TRANSPORT_PLUGIN}" "${dest}/plugins/"
  else
    echo "   WARN: transport plugin not found at ${TRANSPORT_PLUGIN} (build target pcl_transport_socket_plugin)"
  fi

  # --- include/pcl: the public PCL headers a client uses ---
  cp -f "${PCL_INC}"/*.h "${PCL_INC}"/*.hpp "${dest}/include/pcl/" 2>/dev/null || true

  # --- include/pyramid: generated facade + data-model + C-ABI headers ---
  # (the data-model header set is shared; copy it whole so any closure resolves)
  cp -f "${GEN_DIR}"/pyramid_data_model_*.hpp "${dest}/include/pyramid/" 2>/dev/null || true
  cp -f "${GEN_DIR}"/pyramid_data_model_*.h   "${dest}/include/pyramid/" 2>/dev/null || true
  cp -f "${GEN_DIR}"/pyramid_datamodel_cabi.h "${dest}/include/pyramid/" 2>/dev/null || true
  # this component's service facade headers
  cp -f "${GEN_DIR}"/pyramid_services_"${comp}"_*.hpp "${dest}/include/pyramid/" 2>/dev/null || true

  # --- src: the contract facade the client compiles against ---
  # (exclude *_codec_plugin.cpp -- that is the plugin's own source, already
  #  built into the .so; a client must not recompile it)
  for f in "${GEN_DIR}"/pyramid_services_"${comp}"_*.cpp; do
    [[ -f "${f}" ]] || continue
    case "${f}" in *_codec_plugin.cpp) continue ;; esac
    cp -f "${f}" "${dest}/src/"
  done

  # --- lib: static link libraries ---
  for l in "${LINK_LIBS[@]}"; do [[ -f "${l}" ]] && cp -f "${l}" "${dest}/lib/"; done

  # --- MANIFEST: absolute plugin paths (for --codec-plugin / PCL_CODEC_PLUGIN_PATH) ---
  ( cd "${dest}/plugins" && ls -1 ./*.so 2>/dev/null | sed "s|^\./|${dest}/plugins/|" ) > "${dest}/MANIFEST.txt" || true

  # --- README ---
  cat > "${dest}/README.md" <<EOF
# Deployment: ${comp}

Generated by \`stage_plugin_deploy.sh\` from \`${BUILD_DIR##*/}\`.

\`\`\`
plugins/   codec plugin .so(s) for ${comp} + the socket transport plugin .so
include/   pcl/*  (PCL public API)   pyramid/*  (generated typed contract + C-ABI)
src/       generated contract facade (.cpp) to compile into the client
lib/       static libraries to link (codec is loaded from plugins/ at run time)
MANIFEST.txt  absolute plugin .so paths
\`\`\`

## Plugins (the runtime binding)
$(cd "${dest}/plugins" 2>/dev/null && for f in *.so; do echo "- \`$f\`"; done)

The codec plugin is the **single cross-language** \`.so\` — it consumes the frozen
\`pyramid_<Type>_c\` C struct and is loaded from both C++ and Ada clients.

## Build a client (plugin-only: no codec linked, loaded at runtime)
\`\`\`sh
g++ -std=c++17 my_client.cpp src/*.cpp \\
    -Iinclude -Iinclude/pyramid -I<nlohmann_json_include> -I<flatbuffers_include> \\
    lib/libpcl_core.a lib/libpcl_transport_socket.a \\
    lib/libpyramid_generated_codecs.a lib/libpyramid_flatbuffers_support.a \\
    -lpthread -o my_client
\`\`\`
(nlohmann/flatbuffers headers live under the build tree \`_deps/\`.)

## Run against the plugin
\`\`\`sh
./my_client --codec-plugin "\$(head -1 MANIFEST.txt)"
# or point the default-load at the dir:
PCL_CODEC_PLUGIN_PATH="\$(pwd)/plugins" ./my_client
\`\`\`
The client calls the generated service with native types; the loaded \`.so\`
supplies the codec. With no plugin loaded the C++ facade fails closed.
EOF
done

echo
echo "Done. Layout:"
find "${OUT_DIR}" -maxdepth 2 -mindepth 1 | sort | sed "s|${OUT_DIR}|<out>|"
