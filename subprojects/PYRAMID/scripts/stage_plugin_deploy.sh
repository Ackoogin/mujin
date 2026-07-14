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
#     codec_manifest.txt      codec .so paths (point PCL_CODEC_MANIFEST /
#                             --codec-manifest at it, or list via --codec-plugin
#                             / the PYRAMID_CODEC_PLUGINS env var for Ada)
#     transport_manifest.txt  transport .so paths (pass via --transport-plugin)
#     MANIFEST.txt   all staged plugin .so paths (review/audit listing)
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
    -h|--help)   sed -n '2,27p' "${BASH_SOURCE[0]}"; exit 0 ;;
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
# The coupled ROS2 plugin is built out-of-tree via colcon (ament package), so it
# lives under the ROS2 install tree rather than the main build dir. It is staged
# only when that colcon build has produced it. The path must match the
# --install-base that scripts/build_ros2_transport.sh passes to colcon
# (build-ros2-ament/install), or the plugin is silently never staged.
ROS2_INSTALL_LIBDIR="${REPO_ROOT}/build-ros2-ament/install/pyramid_ros2_transport/lib"
TRANSPORT_PLUGINS=(
  "${PCL_SRCDIR}/libpcl_transport_socket_plugin.so"
  "${PCL_SRCDIR}/libpcl_transport_shared_memory_plugin.so"
  "${PCL_SRCDIR}/libpcl_transport_udp_plugin.so"
  "${ROS2_INSTALL_LIBDIR}/libpyramid_ros2_coupled_plugin.so"
)

[[ -d "${GEN_DIR}" ]]   || { echo "ERROR: generated bindings not found at ${GEN_DIR} -- build first." >&2; exit 1; }
[[ -d "${PCL_INC}" ]]   || { echo "ERROR: PCL include dir not found at ${PCL_INC}" >&2; exit 1; }

echo "build-dir : ${BUILD_DIR}"
echo "out       : ${OUT_DIR}"
[[ ${CLEAN} -eq 1 ]] && { echo "cleaning ${OUT_DIR}"; rm -rf "${OUT_DIR:?}"/*; }

# Framework library every plugin-only client links. Both the codec
# (JSON/FlatBuffers) AND the transport (socket/shm) are loaded from .so plugins
# at run time, so the client links ONLY the framework (pcl_core) and the
# native<->C-struct marshalling for its data-model module closure (computed
# per component below) -- no wire codec, no FlatBuffers, no transport library.
LINK_LIBS=(
  "${PCL_SRCDIR}/libpcl_core.a"
)

# Discover components from the built codec plugin .so names:
#   libpyramid_codec_<codec>_<component>.so
mapfile -t COMPONENTS < <(
  find "${PYRAMID_LIBDIR}" -maxdepth 1 -name 'libpyramid_codec_*.so' -printf '%f\n' 2>/dev/null \
    | sed -nE 's/^libpyramid_codec_(json|flatbuffers|protobuf)_(.*)\.so$/\2/p' \
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
  found_transport=0
  for tp in "${TRANSPORT_PLUGINS[@]}"; do
    if [[ -f "${tp}" ]]; then cp -f "${tp}" "${dest}/plugins/"; found_transport=1; fi
  done
  [[ ${found_transport} -eq 1 ]] || \
    echo "   WARN: no transport plugin found (build pcl_transport_socket_plugin / pcl_transport_shared_memory_plugin / pcl_transport_udp_plugin)"

  # --- include/pcl: the public PCL headers a client uses ---
  cp -f "${PCL_INC}"/*.h "${PCL_INC}"/*.hpp "${dest}/include/pcl/" 2>/dev/null || true

  # --- include/pyramid: generated facade + data-model + C-ABI headers ---
  # (the data-model header set is shared; copy it whole so any closure resolves)
  cp -f "${GEN_DIR}"/pyramid_data_model_*.hpp "${dest}/include/pyramid/" 2>/dev/null || true
  cp -f "${GEN_DIR}"/pyramid_data_model_*.h   "${dest}/include/pyramid/" 2>/dev/null || true
  cp -f "${GEN_DIR}"/pyramid_datamodel_cabi.h "${dest}/include/pyramid/" 2>/dev/null || true
  # this component's service facade headers
  cp -f "${GEN_DIR}"/pyramid_services_"${comp}"_*.hpp "${dest}/include/pyramid/" 2>/dev/null || true
  # header-only dep the generated data-model headers include (tl::optional)
  mkdir -p "${dest}/include/tl"
  cp -f "${REPO_ROOT}/subprojects/PYRAMID/core/external/tl/optional.hpp" \
        "${dest}/include/tl/" 2>/dev/null || true

  # --- src: the contract facade the client compiles against ---
  # (exclude *_codec_plugin.cpp -- that is the plugin's own source, already
  #  built into the .so; a client must not recompile it)
  for f in "${GEN_DIR}"/pyramid_services_"${comp}"_*.cpp; do
    [[ -f "${f}" ]] || continue
    case "${f}" in *_codec_plugin.cpp) continue ;; esac
    cp -f "${f}" "${dest}/src/"
  done

  # --- lib: framework + generated marshalling archives -----------------------
  for l in "${LINK_LIBS[@]}"; do [[ -f "${l}" ]] && cp -f "${l}" "${dest}/lib/"; done
  [[ -f "${PYRAMID_LIBDIR}/libpyramid_generated_marshal.a" ]] && \
    cp -f "${PYRAMID_LIBDIR}/libpyramid_generated_marshal.a" "${dest}/lib/"

  staged_modules=()
  for ml in "${PYRAMID_LIBDIR}"/libpyramid_marshal_*.a; do
    [[ -f "${ml}" ]] || continue
    cp -f "${ml}" "${dest}/lib/"
    m="$(basename "${ml}" .a)"
    staged_modules+=("${m#libpyramid_marshal_}")
  done
  echo "   marshalling modules: ${staged_modules[*]:-<none>}"

  # --- MANIFEST: absolute plugin paths (review/audit listing of every staged .so) ---
  ( cd "${dest}/plugins" && ls -1 ./*.so 2>/dev/null | sed "s|^\./|${dest}/plugins/|" ) > "${dest}/MANIFEST.txt" || true

  # --- codec_manifest.txt: codec plugin paths the runtime auto-loads ----------
  # Point PCL_CODEC_MANIFEST (or --codec-manifest) at this file so a component
  # runs without naming each codec plugin. Transport plugins are listed
  # separately because loading a transport needs runtime config (the executor).
  {
    echo "# Codec plugins for ${comp} -- auto-loaded via PCL_CODEC_MANIFEST."
    ( cd "${dest}/plugins" && ls -1 ./libpyramid_codec_*.so 2>/dev/null \
        | sed "s|^\./|${dest}/plugins/|" )
  } > "${dest}/codec_manifest.txt" || true

  # --- transport_manifest.txt: transport plugin path(s) for this component ----
  # Match both the pcl_core transport plugins (socket/shm/udp) and the coupled
  # ROS2 transport plugin, which does not follow the libpcl_transport_*_plugin
  # naming -- otherwise a staged ROS2 deployment never lists its transport.
  {
    echo "# Transport plugins for ${comp} (pass via --transport-plugin)."
    ( cd "${dest}/plugins" \
        && ls -1 ./libpcl_transport_*_plugin.so ./libpyramid_ros2_coupled_plugin.so 2>/dev/null \
        | sed "s|^\./|${dest}/plugins/|" )
  } > "${dest}/transport_manifest.txt" || true

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

## Build a client (plugin-only: no wire codec, no transport linked)
\`\`\`sh
g++ -std=c++17 my_client.cpp src/*.cpp \\
    -Iinclude -Iinclude/pyramid \\
    lib/libpcl_core.a lib/libpyramid_generated_marshal.a \\
    -lpthread -ldl -o my_client
\`\`\`
The client links only the framework + the generated native<->C-struct
marshalling archive (\`lib/libpyramid_generated_marshal.a\`) — no JSON, no
FlatBuffers, no codec, and **no transport**. Both the codec and the socket
transport arrive as \`.so\` plugins composed at run time.

## Run against the plugins
\`\`\`sh
# Explicit: name each plugin.
./my_client --codec-plugin "\$(grep -v '^#' codec_manifest.txt | head -1)" \\
            --transport-plugin "\$(grep -v '^#' transport_manifest.txt | head -1)"

# Config-driven default: the runtime auto-loads every codec in the manifest, so
# the client only needs the transport plugin path.
PCL_CODEC_MANIFEST="\$(pwd)/codec_manifest.txt" ./my_client \\
            --transport-plugin "\$(grep -v '^#' transport_manifest.txt | head -1)"
\`\`\`
The client calls the generated service with native types; the loaded codec
\`.so\` supplies the wire codec and the transport \`.so\` supplies the socket (or
shared-memory) transport. With no codec plugin loaded the C++ facade fails
closed.
EOF
done

echo
echo "Done. Layout:"
find "${OUT_DIR}" -maxdepth 2 -mindepth 1 | sort | sed "s|${OUT_DIR}|<out>|"
