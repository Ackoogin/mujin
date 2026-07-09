#!/usr/bin/env bash
# Build + run Phase 5 (plan-terminal) of
# doc/plans/PYRAMID/rpc_pubsub_interchangeability_plan.md: the interaction
# facade's RPC/pub-sub interchangeability claim, cross-process over a real
# libpcl_transport_shared_memory_plugin.so bus loaded via
# pcl_transport_routing_load, driving MaactionRequestPortProvider /
# MaactionRequestPortClient (not the raw publish*/subscribe* primitives
# build_agra_shm_comms_test.sh/build_agra_mixed_route_test.sh use).
#
# mission_autonomy (provided) and c2_station (consumed) are two distinct
# generated packages, each with its own JSON/FlatBuffers codec plugin
# exporting the identical extern "C" pcl_codec_plugin_entry symbol -- they
# cannot be statically linked into one binary, so each is built as its own
# loadable .so and dlopen'd per-process, same convention as
# build_agra_shm_comms_test.sh.
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_agra_seam_interchange"
scratch="$here/agra_seam_interchange_scratch"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
rm -rf "$scratch"; mkdir -p "$scratch"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/agra_example" "$gen" \
    --languages cpp --backends json,flatbuffers >/dev/null

libpcl=""
for candidate in $(find "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_transport_routing_load$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a with pcl_transport_routing_load not found under build*/"; exit 2; }

shm_plugin=""
for candidate in $(find "$root" -name 'libpcl_transport_shared_memory_plugin.so' -type f | sort); do
  shm_plugin="$candidate"
  break
done
[ -n "$shm_plugin" ] || { echo "libpcl_transport_shared_memory_plugin.so not found under build*/"; exit 2; }

flatc="$(find "$root" -path '*/_deps/flatbuffers-build/flatc' -type f -perm -u+x | head -1)"
[ -n "$flatc" ] || { echo "flatc not found under build*/_deps/flatbuffers-build/"; exit 2; }
flat_inc="$(find "$root" -path '*/_deps/flatbuffers-src/include/flatbuffers/flatbuffers.h' -type f | head -1)"
[ -n "$flat_inc" ] || { echo "FlatBuffers headers not found under build*/_deps/flatbuffers-src/include/"; exit 2; }
flat_inc_dir="$(dirname "$(dirname "$flat_inc")")"

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external" -I"$flat_inc_dir")

echo "== generating FlatBuffers headers =="
for pkg in pyramid_services_agra_mission_autonomy pyramid_services_agra_c2_station; do
  "$flatc" --cpp --gen-object-api -o "$gen/flatbuffers/cpp" "$gen/flatbuffers/cpp/$pkg.fbs"
done

pcl_alloc_obj="$gen/pcl_alloc.o"
cc -std=c11 -fPIC -I"$pyramid/../PCL/include" -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$pcl_alloc_obj"

common_pkgs=(
  pyramid_data_model_base
  pyramid_data_model_common
  pyramid_data_model_agra
)

build_role() {
  local role_label="$1"      # ma | c2
  local components_pkg="$2"  # e.g. pyramid_components_agra_mission_autonomy_services_provided
  local plugin_pkg="$3"      # e.g. pyramid_services_agra_mission_autonomy (json/flatbuffers plugin + .fbs basename)

  local pkgs=("${common_pkgs[@]}" "$components_pkg")

  local json_plug=("$gen/${plugin_pkg}_json_codec_plugin.cpp")
  for p in "${pkgs[@]}"; do
    for suffix in _codec.cpp _cabi_marshal.cpp; do
      [ -f "$gen/$p$suffix" ] && json_plug+=("$gen/$p$suffix")
    done
  done
  json_plug+=("$pcl_alloc_obj")
  echo "== building $role_label JSON codec plugin .so =="
  g++ -std=c++17 -fPIC -shared "${inc[@]}" "${json_plug[@]}" \
      -o "$here/lib${role_label}_agra_seam_json_codec.so"

  local flat_plug=("$gen/${plugin_pkg}_flatbuffers_codec_plugin.cpp"
                    "$gen/flatbuffers/cpp/${plugin_pkg}_flatbuffers_codec.cpp")
  for p in "${pkgs[@]}"; do
    for suffix in _codec.cpp _cabi_marshal.cpp; do
      [ -f "$gen/$p$suffix" ] && flat_plug+=("$gen/$p$suffix")
    done
  done
  flat_plug+=("$pcl_alloc_obj")
  echo "== building $role_label FlatBuffers codec plugin .so =="
  g++ -std=c++17 -fPIC -shared "${inc[@]}" "${flat_plug[@]}" \
      -o "$here/lib${role_label}_agra_seam_flatbuffers_codec.so"
}

build_role ma pyramid_components_agra_mission_autonomy_services_provided \
    pyramid_services_agra_mission_autonomy
build_role c2 pyramid_components_agra_c2_station_services_consumed \
    pyramid_services_agra_c2_station

main_pkgs=("${common_pkgs[@]}"
           pyramid_components_agra_mission_autonomy_services_provided
           pyramid_components_agra_c2_station_services_consumed)
srcs=("$here/agra_seam_interchange_test.cpp"
      "$gen/pyramid_services_agra_mission_autonomy_provided.cpp"
      "$gen/pyramid_services_agra_c2_station_consumed.cpp")
for p in "${main_pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [ -f "$gen/$p$suffix" ] && srcs+=("$gen/$p$suffix")
  done
done

echo "== compiling ($(echo "${srcs[@]}" | wc -w) sources) =="
g++ -std=c++17 "${inc[@]}" "${srcs[@]}" "$libpcl" -ldl -lpthread \
    -o "$here/agra_seam_interchange_test"

echo "== running =="
"$here/agra_seam_interchange_test" "$shm_plugin" \
    "$here/libma_agra_seam_json_codec.so" "$here/libc2_agra_seam_json_codec.so" \
    "$here/libma_agra_seam_flatbuffers_codec.so" "$here/libc2_agra_seam_flatbuffers_codec.so" \
    "$scratch"
