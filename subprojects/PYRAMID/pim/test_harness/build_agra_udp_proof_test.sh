#!/usr/bin/env bash
# Build + run Phase D of doc/plans/PYRAMID/agra_pubsub_shm_udp_proving_plan.md:
# the UDP proof (negative gate + the BEST_EFFORT information topic over real
# UDP datagrams), driving the A-GRA example contract's generated bindings.
# JSON codec only -- Phase D doesn't need the FlatBuffers witness (already
# covered by Phase C).
set -eu

here="$(cd "$(dirname "$0")" && pwd)"
pyramid="$(cd "$here/../.." && pwd)"
root="$(cd "$pyramid/../.." && pwd)"
gen="$here/generated_agra_udp_proof"
scratch="$here/agra_udp_proof_scratch"

echo "== generating bindings =="
rm -rf "$gen"; mkdir -p "$gen"
rm -rf "$scratch"; mkdir -p "$scratch"
python3 "$pyramid/pim/generate_bindings.py" "$pyramid/pim/agra_example" "$gen" \
    --languages cpp --backends json >/dev/null

libpcl=""
for candidate in $(find "$root" -path '*/libpcl_core.a' -type f | sort); do
  if nm -g "$candidate" 2>/dev/null | grep -q ' T pcl_transport_routing_load$'; then
    libpcl="$candidate"
    break
  fi
done
[ -n "$libpcl" ] || { echo "libpcl_core.a with pcl_transport_routing_load not found under build*/"; exit 2; }

udp_plugin=""
for candidate in $(find "$root" -name 'libpcl_transport_udp_plugin.so' -type f | sort); do
  udp_plugin="$candidate"
  break
done
[ -n "$udp_plugin" ] || { echo "libpcl_transport_udp_plugin.so not found under build*/"; exit 2; }

inc=(-I"$gen" -I"$pyramid/../PCL/include" -I"$pyramid/core/external")

pcl_alloc_obj="$gen/pcl_alloc.o"
cc -std=c11 -fPIC -I"$pyramid/../PCL/include" -c "$pyramid/../PCL/src/pcl_alloc.c" -o "$pcl_alloc_obj"

common_pkgs=(
  pyramid_data_model_base
  pyramid_data_model_common
  pyramid_data_model_agra
)

build_json_codec() {
  local role_label="$1"      # ma | c2
  local components_pkg="$2"
  local plugin_pkg="$3"

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
      -o "$here/lib${role_label}_agra_json_codec.so"
}

build_json_codec ma pyramid_components_agra_mission_autonomy_services_provided \
    pyramid_services_agra_mission_autonomy
build_json_codec c2 pyramid_components_agra_c2_station_services_consumed \
    pyramid_services_agra_c2_station

main_pkgs=("${common_pkgs[@]}"
           pyramid_components_agra_mission_autonomy_services_provided
           pyramid_components_agra_c2_station_services_consumed)
srcs=("$here/agra_udp_proof_test.cpp"
      "$gen/pyramid_services_agra_mission_autonomy_provided.cpp"
      "$gen/pyramid_services_agra_c2_station_consumed.cpp")
for p in "${main_pkgs[@]}"; do
  for suffix in _codec.cpp _cabi_marshal.cpp; do
    [ -f "$gen/$p$suffix" ] && srcs+=("$gen/$p$suffix")
  done
done

echo "== compiling ($(echo "${srcs[@]}" | wc -w) sources) =="
g++ -std=c++17 "${inc[@]}" "${srcs[@]}" "$libpcl" -ldl -lpthread \
    -o "$here/agra_udp_proof_test"

echo "== running =="
"$here/agra_udp_proof_test" "$udp_plugin" \
    "$here/libma_agra_json_codec.so" "$here/libc2_agra_json_codec.so" \
    "$scratch"
